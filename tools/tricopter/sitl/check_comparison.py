#!/usr/bin/env python3
"""Matched free-flight yaw / collective experiments; only normal RC is written."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import threading
import time
import traceback
import xml.etree.ElementTree as ET

from check_flight import PILOT, Pilot, TraceReader, read_limits, setup_collective_feedforward
from compare_metrics import actual_yaw_rate, coupling_metrics, yaw_response
from run import build_plugin, materialize_world, native_environment, verified_binary, wait_for_plant
from runtime import (ARTIFACTS, HERE, Msp, RC_PACKET, RcFeed, SessionLock,
                     fresh_run_directory, provision, start_child, start_firmware, stop_process)


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def cases(hover):
    """Protocol fixed before running; PWM directions use native FRD yaw signs."""
    output=[]
    for name,kind,yaw,delta in [('sham','sham',1500,0),
        ('collective_up','collective',1500,25),('collective_down','collective',1500,-25),
        ('yaw_cw','yaw',1368,0),('yaw_ccw','yaw',1632,0),
        ('yaw_cw_collective_up','combined',1368,25),('yaw_ccw_collective_up','combined',1632,25)]:
        stages=[('baseline',1.,hover,1500)]
        if kind in ('yaw','combined'):
            stages += [('yaw_lead',.8,hover,yaw),('pulse',.4,hover+delta,yaw),
                       ('yaw_hold',.6,hover,yaw)]
        else: stages += [('pulse',.4,hover+delta,yaw)]
        stages += [('recovery',1.2,hover,1500)]
        output.append({'name':name,'kind':kind,'stages':[
            {'name':s,'duration_s':duration,'channels':[1500,1500,throttle,turn]}
            for s,duration,throttle,turn in stages]})
    return output


class ExperimentReceiver(RcFeed):
    """Atomically override the four pilot outputs, retaining native AUX/disarm."""
    def __init__(self):
        super().__init__();self.override=None;self.marker=('setup','setup')
        self.snapshot=lambda:None;self.trace=None;self.plan=None
        self.sent=[];self.transitions={};self.failed=None

    def select(self, case, stage, channels=None):
        with self.lock:
            self.plan=None
            self.marker=(case,stage)
            self.override=None if channels is None else list(channels)

    def program(self,case):
        with self.lock:
            self.plan={'case':case['name'],'stages':case['stages'],'epoch':None}

    def planned_input(self,sim):
        plan=self.plan
        if plan['epoch'] is None:plan['epoch']=sim
        elapsed=sim-plan['epoch'];boundary=0.
        for stage in plan['stages']:
            boundary+=stage['duration_s']
            if elapsed<boundary:
                return (plan['case'],stage['name']),stage['channels']
        # Hold neutral recovery inputs until the main thread takes over, but
        # timestamp the measurement end independently of MSP polling latency.
        return (plan['case'],'complete'),plan['stages'][-1]['channels']

    def _run(self):
        origin=time.monotonic()
        try:
            while not self.stop.is_set():
                state=self.trace.read() if self.trace is not None else self.snapshot()
                now=time.monotonic()
                with self.lock:
                    channels=self.channels.copy();marker=self.marker
                    sim=None if state is None else state['sim_time_s']
                    if self.plan is not None and sim is not None:
                        marker,override=self.planned_input(sim)
                        channels[:4]=override
                    elif self.override is not None:channels[:4]=self.override
                    self.sock.sendto(RC_PACKET.pack(now-origin,*channels),('127.0.0.1',9004))
                    record={'wall_monotonic':now,'sim_time_s':sim,'case':marker[0],
                            'stage':marker[1],'channels':channels}
                    self.sent.append(record)
                    if sim is not None:self.transitions.setdefault(marker,record.copy())
                self.stop.wait(1/self.rate)
        except Exception as error:self.failed=repr(error);self.stop.set()


class Trial:
    maximum_horizontal_distance_m = 5.0

    def __init__(self, directory, variant, plugin, world_source, protocol, extra_config=None, artifacts=None):
        self.directory,self.variant,self.protocol=directory,variant,protocol
        self.plugin,self.world_source=plugin,world_source
        self.extra_config=Path(extra_config) if extra_config is not None else None
        self.artifacts=Path(artifacts) if artifacts is not None else ARTIFACTS
        self.firmware=self.gazebo=self.receiver=self.client=self.pilot=None
        self.telemetry=[];self.next_msp=0.;self.record={'pass':False,'variant':variant,'cases':[]}

    def health(self):
        if self.pilot and self.pilot.fault:raise RuntimeError(self.pilot.fault)
        if self.receiver and self.receiver.failed:raise RuntimeError(self.receiver.failed)
        for process in (self.firmware,self.gazebo):
            if process and process.poll() is not None:raise RuntimeError('Simulation process exited')
        state=self.pilot.snapshot() if self.pilot else None
        if state and math.hypot(state['x_m']-self.pilot.origin[0],state['y_m']-self.pilot.origin[1])>self.maximum_horizontal_distance_m:
            raise RuntimeError(f'{self.maximum_horizontal_distance_m:g} metre horizontal envelope exceeded')
        if self.receiver and (self.receiver.override is not None or self.receiver.plan is not None) and state and state['z_m']<.35:
            raise RuntimeError('Measurement approached ground contact')
        if time.monotonic()>=self.next_msp:
            status=self.client.status();row={'wall_monotonic':time.monotonic(),
                'sim_time_s':None if state is None else state['sim_time_s'],**status}
            attitude=struct.unpack('<3h',self.client.request(108))
            row.update(firmware_roll_deg=attitude[0]/10,firmware_pitch_deg=attitude[1]/10,
                       firmware_yaw_deg=attitude[2])
            if self.variant=='modified':row['debug']=list(struct.unpack('<8h',self.client.request(254)))
            self.telemetry.append(row);self.next_msp=time.monotonic()+.1
            if self.armed:
                if not status['armed']:raise RuntimeError(f'Unexpected disarm: {status}')
                if status['arming_disable_flags']:raise RuntimeError(f'Arming fault during flight: {status}')
                if not status['mode_mask']&(1<<self.boxes.index(1)):raise RuntimeError('ANGLE mode inactive')
                if any(box in self.boxes and status['mode_mask']&(1<<self.boxes.index(box)) for box in (3,11,56)):
                    raise RuntimeError('Native altitude/position/autopilot mode active')
        return state

    def rate_for(self,pwm):return actual_yaw_rate(pwm,**self.yaw_settings)

    def heading_pwm(self, world_yaw):
        error=(-world_yaw+180)%360-180
        desired=-max(-30,min(30,2*error))
        best=min(range(1380,1621),key=lambda pwm:abs(self.rate_for(pwm)-desired))
        return best

    def recapture(self, first=False):
        self.receiver.select('setup','setup')
        self.pilot.integral=0.
        start=self.pilot.snapshot()['sim_time_s'];good_since=None
        deadline=time.monotonic()+50
        while time.monotonic()<deadline:
            state=self.health();sim=state['sim_time_s']
            self.pilot.command('setup',destination=self.target,yaw=self.heading_pwm(state['yaw_deg']))
            good=(abs(state['z_m']-self.target)<.10 and abs(state['vz_m_s'])<.12 and
                math.hypot(state['vx_m_s'],state['vy_m_s'])<.07 and
                math.hypot(state['x_m']-self.pilot.origin[0],state['y_m']-self.pilot.origin[1])<.35 and
                max(abs(state['roll_deg']),abs(state['pitch_deg']))<3 and
                max(abs(math.degrees(state[k])) for k in ('gyro_frd_x','gyro_frd_y'))<1 and
                abs(math.degrees(state['gyro_frd_z']))<2 and
                abs((state['yaw_deg']+180)%360-180)<1)
            if good:
                if good_since is None:good_since=sim
                if sim-good_since>=.5 and sim-start>=(6 if first else 2):return dict(state)
            else:good_since=None
            if sim-start>24:raise TimeoutError('Initial-state recapture gate not reached')
            time.sleep(.02)
        raise TimeoutError('Recapture wall timeout')

    def measurement(self,case):
        self.receiver.program(case)
        complete=(case['name'],'complete');deadline=time.monotonic()+25
        while complete not in self.receiver.transitions:
            self.health()
            if time.monotonic()>deadline:raise TimeoutError('Measurement plan wall timeout')
            time.sleep(.01)
        stages=[]
        for definition in case['stages']:
            marker=self.receiver.transitions[(case['name'],definition['name'])]
            stages.append({**definition,'actual_start_sim_time_s':marker['sim_time_s'],
                'first_send_wall_monotonic':marker['wall_monotonic'],
                'requested_yaw_rate_dps':self.rate_for(definition['channels'][3])})
        return stages,self.receiver.transitions[complete]['sim_time_s']

    def make_pilot(self, limits, mass, maximum, collective):
        return Pilot(self.receiver,self.directory/'pilot.csv',TraceReader(self.directory/'plant.csv'),
                     limits,mass,maximum,collective)

    def finish(self):
        self.receiver.select('landing','landing');self.pilot.command('landing',destination=self.pilot.origin[2],yaw=1500)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            state=self.health()
            if state['z_m']<self.pilot.origin[2]+.025 and abs(state['vz_m_s'])<.15:break
            time.sleep(.02)
        else:raise RuntimeError('Landing did not complete')
        self.receiver.set(4,1000);self.receiver.set(2,1000);self.pilot.command('disarmed',active=False)
        self.armed=False;time.sleep(.5);self.record['final_status']=self.client.status()
        assert not self.record['final_status']['armed']

    def execute(self):
        d=self.directory;d.mkdir();world=d/'world.sdf';materialize_world(self.world_source,world)
        model=ET.parse(world).find("./world/model[@name='tricopter']")
        bridge=model.find("plugin[@name='tricopter::TricopterBridge']")
        mass=sum(float(x.text) for x in model.findall('./link/inertial/mass'))
        maximum=sum(map(float,bridge.findtext('max_thrust_n').split()))
        binary=verified_binary(self.artifacts,self.variant)
        if self.extra_config is None:
            config=provision(binary,d,self.variant)
        else:
            overlay=d/'requested-overlay.cli'
            shutil.copyfile(self.extra_config,overlay)
            self.record.update(requested_overlay_path=overlay.name,
                requested_overlay_sha256=sha(overlay),
                requested_overlay_source=str(self.extra_config.resolve()))
            config=provision(binary,d,self.variant,extra_config=overlay)
        self.record.update(binary_sha256=sha(binary),plugin_sha256=sha(self.plugin),world_sha256=sha(world),
            firmware_config_sha256=sha(config),mass_kg=mass,total_maximum_thrust_n=maximum,
            common_config_sha256=sha(HERE/'configs/common.cli'),protocol_sha256=self.protocol['sha256'])
        self.armed=False
        try:
            self.firmware=start_firmware(binary,d);self.client=Msp.connect(port=5762);self.client.request(1)
            limits=read_limits(self.client);rates=self.client.request(111);deadband=self.client.request(125)
            self.yaw_settings={'center':rates[11]*10,'max':rates[4]*10,'expo':rates[10]/100,
                'deadband':deadband[1],'reversed':False,'limit':struct.unpack_from('<H',rates,20)[0]}
            self.record['yaw_settings']=self.yaw_settings
            self.record['common_msp_readback']={str(n):self.client.request(n).hex() for n in (44,90,94,111,112,125,131)}
            # MSP90's trailing debug mode/count intentionally differ between
            # stock and the measured-feedback debug overlay. Compare its motor
            # protocol/rate/idle prefix, preserving the full reply separately.
            self.record['msp_advanced_config_raw_hex']=self.record['common_msp_readback']['90']
            self.record['common_msp_readback']['90']=self.record['common_msp_readback']['90'][:16]
            self.record['common_msp_readback_byte_ranges']={'90':[0,8]}
            collective=setup_collective_feedforward(bridge,self.client)
            self.record['setup_collective_feedforward']=(collective.report() if collective else {'backend':'legacy_square_law'})
            self.boxes=self.client.request(119)
            env=native_environment();env['GZ_PARTITION']=f'tricopter-compare-{os.getpid()}-{d.name}'
            env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(self.plugin.parent);env['TRICOPTER_TRACE']=str(d/'plant.csv')
            command=['gz','sim','-s','-r','-v','2',str(world)]
            self.record['command']=command
            with (d/'gazebo.log').open('w') as stream:
                self.gazebo=start_child(command,cwd=d,env=env,stdout=stream,stderr=subprocess.STDOUT)
            self.receiver=ExperimentReceiver();self.receiver.set(5,1900);self.receiver.start()
            wait_for_plant(d,self.firmware,self.gazebo,self.client,self.variant)
            self.pilot=self.make_pilot(limits,mass,maximum,collective)
            self.receiver.trace=TraceReader(d/'plant.csv')
            self.receiver.snapshot=self.pilot.snapshot;self.pilot.start()
            deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                if self.client.status()['arming_disable_flags']==0 and self.pilot.snapshot():break
                time.sleep(.1)
            else:raise RuntimeError('Initial arming flags did not clear')
            self.target=self.pilot.origin[2]+1.5
            self.receiver.set(4,1900);deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                if self.client.status()['armed']:self.armed=True;break
                time.sleep(.05)
            if not self.armed:raise RuntimeError('Could not arm simulated flight')
            for i,case in enumerate(self.protocol['cases']):
                initial=self.recapture(first=i==0)
                result={k:v for k,v in case.items() if k!='stages'};result['initial_state']=initial;result['stages']=[]
                self.record['cases'].append(result)
                result['stages'],result['end_sim_time_s']=self.measurement(case)
                print(f"{d.name} {case['name']}: z={self.pilot.snapshot()['z_m']:.2f}m",flush=True)
            self.finish()
            self.record['pass']=True
        except Exception as error:
            self.record['error']=repr(error);self.record['traceback']=traceback.format_exc();raise
        finally:
            if self.receiver:self.receiver.select('disarmed','disarmed');self.receiver.set(4,1000);self.receiver.set(2,1000)
            if self.pilot:self.pilot.close()
            if self.receiver:
                self.receiver.close()
                if self.receiver.trace:self.receiver.trace.close()
                (d/'rc-packets.json').write_text(json.dumps(self.receiver.sent,separators=(',',':'))+'\n')
                gaps=[b['wall_monotonic']-a['wall_monotonic'] for a,b in zip(self.receiver.sent,self.receiver.sent[1:])]
                self.record['maximum_rc_send_gap_s']=max(gaps) if gaps else None
            if self.client:self.client.close()
            stop_process(self.gazebo);stop_process(self.firmware)
            self.record['all_owned_processes_stopped']=all(p is None or p.poll() is not None for p in (self.gazebo,self.firmware))
            (d/'telemetry.json').write_text(json.dumps(self.telemetry,separators=(',',':'))+'\n')
            (d/'comparison-run.json').write_text(json.dumps(self.record,indent=2)+'\n')
        return self.record


def analyze(directory):
    record=json.loads((directory/'comparison-run.json').read_text())
    packets=json.loads((directory/'rc-packets.json').read_text())
    telemetry=json.loads((directory/'telemetry.json').read_text())
    with (directory/'plant.csv').open() as stream:rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(stream)]
    for case in record['cases']:
        stages={s['name']:s for s in case['stages']}
        onset=stages.get('yaw_lead',stages['pulse'])['actual_start_sim_time_s']
        release=stages['recovery']['actual_start_sim_time_s'];end=case['end_sim_time_s']
        if case['kind'] in ('yaw','combined'):
            target=stages.get('yaw_lead',stages['pulse'])['requested_yaw_rate_dps']
            case['yaw']=yaw_response(rows,onset,release,end,target)
        coupling_onset=stages['pulse']['actual_start_sim_time_s']
        coupling_end=release if case['kind'] in ('yaw','combined') else end
        case['coupling']=coupling_metrics(rows,coupling_onset,coupling_end)
        selected=[r for r in rows if onset<=r['sim_time_s']<=end]
        case['motor_upper_saturation_fraction']=sum(any(r[f'pwm{i}_us']>=1999 for i in range(3)) for r in selected)/len(selected)
        case['servo_endpoint_fraction']=sum(r['servo_pwm_us']<=1101 or r['servo_pwm_us']>=1699 for r in selected)/len(selected)
        case['actuator_valid_fraction']=sum(r['actuator_valid']==1 for r in selected)/len(selected)
        received=[r for r in packets if r['sim_time_s'] is not None and onset<=r['sim_time_s']<=end]
        case['maximum_measurement_rc_send_gap_s']=max((b['wall_monotonic']-a['wall_monotonic']
            for a,b in zip(received,received[1:])),default=None)
        status=[r for r in telemetry if r['sim_time_s'] is not None and onset<=r['sim_time_s']<=end]
        case['firmware_status_samples']=len(status)
        case['all_status_samples_armed_without_fault']=bool(status) and all(r['armed'] and not r['arming_disable_flags'] for r in status)
        if record['variant']=='modified':
            case['sampled_feedback_valid_fraction']=sum(r['debug'][6]&3==3 for r in status)/len(status)
            case['sampled_yaw_allocator_saturation_fraction']=sum(bool(r['debug'][6]&4) for r in status)/len(status)
            case['sampled_vertical_allocator_saturation_fraction']=sum(bool(r['debug'][6]&8) for r in status)/len(status)
    (directory/'comparison-metrics.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repetitions',type=int,default=3)
    parser.add_argument('--label',default='controlled-comparison')
    args=parser.parse_args()
    if not 1<=args.repetitions<=5:parser.error('repetitions must be 1..5')
    # Setup/recapture only: its output is overridden throughout measurements.
    # The original flight smoke check retains its default five-degree bound.
    PILOT['max_angle_deg']=12.
    directory=fresh_run_directory(ARTIFACTS,'both',args.label)
    protocol={'cases':cases(1350),'repetitions':args.repetitions,'measurement_rc_rate_hz':50,
        'setup_pilot':PILOT.copy(),
        'initial_gate':{'altitude_error_m':.1,'vertical_speed_m_s':.12,'horizontal_speed_m_s':.07,
            'horizontal_position_error_m':.35,'absolute_roll_pitch_deg':3,
            'roll_pitch_rate_dps':1,'yaw_rate_dps':2,'heading_error_deg':1,'continuous_duration_s':.5},
        'measurement_minimum_body_height_m':.35,
        'matched_no_collective_controls':{'collective_up':'sham','collective_down':'sham',
            'yaw_cw_collective_up':'yaw_cw','yaw_ccw_collective_up':'yaw_ccw'},
        'execution_timing':'RC sender owns absolute simulation-time stage plan; independent 100 Hz plant trace clock',
        'maximum_stage_duration_error_s':.03,
        'plant_trace_rate_hz':100,'setup_uses_external_ground_truth_rc_pilot':True,
        'measurement_first_four_channels_override_all_external_pilot_outputs':True,
        'same_measurement_values_and_scheduled_durations_actual_send_times_retained':True,
        'variant_order':[variant for i in range(args.repetitions) for variant in
                         (['stock','modified'] if i%2==0 else ['modified','stock'])],
        'source_sha256':{str(p.relative_to(HERE)):sha(p) for p in [Path(__file__),HERE/'compare_metrics.py']}}
    protocol['sha256']=hashlib.sha256(json.dumps(protocol,sort_keys=True).encode()).hexdigest()
    (directory/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (directory/'source').mkdir()
    for name in ('check_comparison.py','compare_metrics.py','check_flight.py','runtime.py','run.py'):
        shutil.copy2(HERE/name,directory/'source'/name)
    summary={'pass':False,'directory':str(directory),'protocol':protocol,'runs':[]}
    print(f'Controlled comparison: {directory}',flush=True)
    try:
        with SessionLock():
            plugin=build_plugin(ARTIFACTS,4)
            for i,variant in enumerate(protocol['variant_order']):
                target=directory/f'{i+1:02d}-{variant}'
                Trial(target,variant,plugin,HERE/'gazebo/worlds/tricopter.sdf',protocol).execute()
                result=analyze(target)
                if summary['runs']:
                    reference=summary['runs'][0]
                    for name in ('world_sha256','common_config_sha256','common_msp_readback','yaw_settings'):
                        if result[name]!=reference[name]:raise AssertionError(f'Unmatched comparison setting: {name}')
                for case in result['cases']:
                    for j,stage in enumerate(case['stages']):
                        end=(case['stages'][j+1]['actual_start_sim_time_s'] if j+1<len(case['stages'])
                             else case['end_sim_time_s'])
                        if abs(end-stage['actual_start_sim_time_s']-stage['duration_s'])>.030001:
                            raise AssertionError('RC stage duration outside 30 ms execution tolerance')
                    if case['actuator_valid_fraction']!=1 or not case['all_status_samples_armed_without_fault']:
                        raise AssertionError('Unhealthy measurement window')
                    if result['variant']=='modified' and case['sampled_feedback_valid_fraction']!=1:
                        raise AssertionError('Invalid measured feedback during comparison')
                summary['runs'].append(result)
                (directory/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        summary['pass']=True
    finally:(directory/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('All controlled trials complete; processes stopped.',flush=True)


if __name__=='__main__':main()
