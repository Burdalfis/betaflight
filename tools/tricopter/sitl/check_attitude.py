#!/usr/bin/env python3
"""Check real corrected SITL estimators on coherent tilted-yaw sensor inputs.

No Gazebo, actuator control, or physical serial connection is used. Native PID
gyro and estimated Euler attitude are read separately through MSP.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import time

from check_protocol import RawOutputs, Stimulus
from run import verified_binary
from runtime import (ARTIFACTS, Msp, RcFeed, SessionLock, fresh_run_directory,
                     provision, start_firmware, stop_process)


def read_sample(client, stimulus, label, origin):
    with stimulus.lock:
        now=time.monotonic();pose,rates=stimulus._pose(now)
        fields=stimulus.last_fdm
    attitude=struct.unpack('<3h',client.request(108))
    gyro=struct.unpack('<9h',client.request(102))[3:6]
    return {'label':label,'elapsed_wall_s':now-origin,
        'truth_roll_deg':math.degrees(pose[0]),'truth_pitch_deg':math.degrees(pose[1]),
        'truth_world_yaw_ccw_deg':math.degrees(pose[2]),
        'truth_euler_rates_dps':[math.degrees(v) for v in rates],
        'firmware_roll_deg':attitude[0]/10,'firmware_pitch_deg':attitude[1]/10,
        'firmware_heading_cw_deg':attitude[2],'raw_msp_gyro':list(gyro),
        'wire_gyro_frd_rad_s':list(fields[1:4]) if fields else None}


def run_variant(artifacts,directory,variant):
    directory.mkdir()
    binary=verified_binary(artifacts,variant)
    record={'pass':False,'variant':variant,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
        'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'samples':[],'segments':[]}
    process=client=stimulus=outputs=receiver=None
    origin=time.monotonic()
    try:
        provision(binary,directory,variant)
        outputs=RawOutputs();outputs.start()
        process=start_firmware(binary,directory)
        client=Msp.connect(port=5762);client.request(1);time.sleep(.5)
        stimulus=Stimulus();stimulus.start();receiver=RcFeed();receiver.start()

        def observe(duration,label):
            deadline=time.monotonic()+duration;rows=[]
            while time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError('SITL exited')
                row=read_sample(client,stimulus,label,origin)
                record['samples'].append(row);rows.append(row);time.sleep(.025)
            return rows

        observe(4.,'initial-level')
        # Both signs and both tilted axes exercise gyro/quaternion handedness.
        for roll,pitch in ((0,20),(0,-20),(20,0),(-20,0),(12,18)):
            stimulus.set_pose(roll=roll,pitch=pitch,yaw=0,duration=1.)
            observe(3.,'tilt-settle')
            for yaw in (-120,0,120,0):
                stimulus.set_pose(roll=roll,pitch=pitch,yaw=yaw,duration=2.)
                rows=observe(2.25,f'tilt-{roll}-{pitch}-yaw-{yaw}')
                # Allow 2 deg for native sensor/filter/UDP timing and 0.1-deg
                # attitude quantization, without hiding the old ~40-deg error.
                error=max(math.hypot(r['firmware_roll_deg']-roll,r['firmware_pitch_deg']-pitch) for r in rows)
                heading0=rows[0]['firmware_heading_cw_deg'];truth0=rows[0]['truth_world_yaw_ccw_deg']
                heading_error=max(abs(((r['firmware_heading_cw_deg']-heading0+
                    r['truth_world_yaw_ccw_deg']-truth0+180)%360)-180) for r in rows)
                result={'roll_deg':roll,'pitch_deg':pitch,'world_yaw_target_deg':yaw,
                    'maximum_roll_pitch_error_norm_deg':error,'maximum_relative_heading_error_deg':heading_error}
                record['segments'].append(result)
                if error>2. or heading_error>6.:raise AssertionError(f'Tilted-yaw attitude mismatch: {result}')
        record['final_status']=client.status()
        if record['final_status']['armed']:raise AssertionError('Synthetic attitude check unexpectedly armed')
        record['pass']=True
    except Exception as error:
        record['error']=repr(error);raise
    finally:
        if receiver:receiver.close()
        if stimulus:stimulus.close()
        if outputs:outputs.close()
        if client:client.close()
        stop_process(process)
        record['all_owned_processes_stopped']=process is None or process.poll() is not None
        (directory/'attitude-run.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts',type=Path,default=ARTIFACTS/'corrected-frame')
    parser.add_argument('--variant',choices=['stock','modified','both'],default='both')
    args=parser.parse_args()
    directory=fresh_run_directory(ARTIFACTS,args.variant,'corrected-attitude-check')
    report={'pass':False,'directory':str(directory),'variants':[],
        'boundary':'Unarmed native SITL sensor/estimator check; no Gazebo, physical serial or plant dynamics.'}
    print(f'Attitude check: {directory}',flush=True)
    try:
        with SessionLock():
            for variant in (('stock','modified') if args.variant=='both' else (args.variant,)):
                report['variants'].append(run_variant(args.artifacts.resolve(),directory/variant,variant))
                print(f'{variant}: tilted-yaw attitude checks passed',flush=True)
        report['pass']=True
    finally:(directory/'attitude-summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
