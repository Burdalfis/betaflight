#!/usr/bin/env python3
"""Actual headless Gazebo/SITL flight with an external, identical RC test pilot.

The pilot reads Gazebo ground truth and sends only normal RC channels. It is
not a Betaflight autopilot and never overrides motors, servo PWM or ADC data.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import threading
import time
import traceback
import xml.etree.ElementTree as ET

from run import build_plugin, materialize_world, verified_binary, wait_for_plant
from runtime import (ARTIFACTS, HERE, Msp, RC_PACKET, RcFeed, SessionLock,
                     fresh_run_directory, provision, start_child, start_firmware, stop_process)

G = 9.80665
PILOT = {'position_p_xy': .8, 'velocity_d_xy': 1.5,
         'position_p_z': 3.0, 'velocity_d_z': 2.5, 'integral_z': .4,
         'max_angle_deg': 5.0, 'max_vertical_acceleration_m_s2': 2.5,
         'ascent_m_s': .4, 'descent_m_s': .3, 'throttle_slew_us_s': 500.0,
         'height_above_initial_m': 1.0, 'rate_hz': 50.0}


def clamp(v, lo, hi): return max(lo, min(hi, v))


class RecordedReceiver(RcFeed):
    """Retain the exact send time and all channels of every RC datagram."""
    def __init__(self):
        super().__init__()
        self.sent = []

    def _run(self):
        origin = time.monotonic()
        while not self.stop.is_set():
            with self.lock: channels = self.channels.copy()
            now = time.monotonic()
            self.sock.sendto(RC_PACKET.pack(now-origin, *channels), ('127.0.0.1', 9004))
            self.sent.append({'monotonic': now, 'channels': channels})
            self.stop.wait(1/self.rate)


class TraceReader:
    def __init__(self, path):
        self.stream = path.open()
        self.header = None
        self.pending = ''
        self.latest = None
        self.updated = 0.0

    def read(self):
        self.pending += self.stream.read()
        lines = self.pending.split('\n'); self.pending = lines.pop()
        for line in lines:
            if not line: continue
            fields = next(csv.reader([line]))
            if self.header is None:
                self.header = fields; continue
            if len(fields) != len(self.header): raise ValueError('Malformed plant trace row')
            values = {name: float(value) for name, value in zip(self.header, fields)}
            if not all(math.isfinite(v) for v in values.values()): raise ValueError('Nonfinite plant state')
            self.latest = values; self.updated = time.monotonic()
        return self.latest

    def close(self): self.stream.close()


def actual_rate(deflection, center, maximum, expo):
    return center*deflection + max(0, maximum-center) * (
        expo*deflection**6 + (1-expo)*deflection**2)


def angle_channel(angle_deg, limits, axis):
    # ANGLE in this BF version normalizes the configured Acro rate curve.
    desired = abs(angle_deg)/limits['angle_limit_deg'] * limits['max_rate'][axis]
    low, high = 0.0, 1.0
    for _ in range(35):
        middle = (low+high)/2
        if actual_rate(middle, limits['center'][axis], limits['max_rate'][axis], limits['expo'][axis]) < desired:
            low = middle
        else: high = middle
    return int(round(1500 + math.copysign(500*(low+high)/2, angle_deg))) if angle_deg else 1500


def read_limits(client):
    rates = client.request(111); advanced = client.request(94)
    if len(rates) < 23 or rates[22] != 3:
        raise ValueError('External test pilot expects the common ACTUAL rate profile')
    limits = {'angle_limit_deg': advanced[17], 'center': [rates[0]*10, rates[12]*10],
              'max_rate': [rates[2]*10, rates[3]*10], 'expo': [rates[1]/100, rates[13]/100]}
    if limits['angle_limit_deg'] < 5: raise ValueError('ANGLE limit smaller than pilot bound')
    return limits


class Pilot:
    def __init__(self, receiver, path, trace, limits, mass, max_thrust):
        self.receiver, self.trace, self.limits = receiver, trace, limits
        self.mass, self.max_thrust = mass, max_thrust
        self.lock = threading.Lock(); self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.phase = 'idle'; self.active = False; self.destination = None; self.yaw = 1500
        self.latest = None; self.fault = None; self.integral = 0.0
        self.stream = path.open('w'); self.writer = None
        self.origin = None; self.reference_z = None; self.throttle = 1000.0

    def command(self, phase, active=True, destination=None, yaw=1500):
        with self.lock:
            self.phase, self.active, self.yaw = phase, active, yaw
            if destination is not None: self.destination = destination

    def snapshot(self):
        with self.lock: return self.latest.copy() if self.latest else None

    def start(self): self.thread.start()

    def _run(self):
        previous_sim = None
        try:
            while not self.stop.is_set():
                state = self.trace.read()
                if state is None:
                    self.stop.wait(.02); continue
                if time.monotonic()-self.trace.updated > .5: raise RuntimeError('Plant CSV stopped advancing')
                sim = state['sim_time_s']
                dt = clamp(sim-previous_sim, 0, .1) if previous_sim is not None else .02
                previous_sim = sim
                with self.lock:
                    phase, active, destination, yaw = self.phase, self.active, self.destination, self.yaw
                if self.origin is None:
                    self.origin = [state[k] for k in ['x_m', 'y_m', 'z_m']]
                    self.reference_z = self.origin[2]
                target_roll = target_pitch = 0.0
                if active:
                    if abs(state['roll_deg']) > 40 or abs(state['pitch_deg']) > 40:
                        raise RuntimeError('Vehicle exceeded 40 degree attitude envelope')
                    if state['z_m'] > self.origin[2]+3:
                        raise RuntimeError('Vehicle exceeded 3 metre altitude envelope')
                    destination = destination if destination is not None else self.origin[2]+1
                    delta = destination-self.reference_z
                    rate = PILOT['ascent_m_s'] if delta >= 0 else PILOT['descent_m_s']
                    step = clamp(delta, -rate*dt, rate*dt)
                    self.reference_z += step
                    desired_vz = step/dt if dt else 0.0
                    error_z = self.reference_z-state['z_m']
                    self.integral = clamp(self.integral+error_z*dt, -1.0, 1.0)
                    az = clamp(PILOT['position_p_z']*error_z +
                               PILOT['velocity_d_z']*(desired_vz-state['vz_m_s']) +
                               PILOT['integral_z']*self.integral,
                               -PILOT['max_vertical_acceleration_m_s2'],
                               PILOT['max_vertical_acceleration_m_s2'])
                    ax = -PILOT['position_p_xy']*(state['x_m']-self.origin[0])-PILOT['velocity_d_xy']*state['vx_m_s']
                    ay = -PILOT['position_p_xy']*(state['y_m']-self.origin[1])-PILOT['velocity_d_xy']*state['vy_m_s']
                    heading = math.radians(state['yaw_deg'])
                    bx = math.cos(heading)*ax+math.sin(heading)*ay
                    by = -math.sin(heading)*ax+math.cos(heading)*ay
                    target_roll = clamp(math.degrees(math.atan2(-by, G)), -5, 5)
                    target_pitch = clamp(math.degrees(math.atan2(bx, G)), -5, 5)
                    vertical_fraction = max(.5, math.cos(math.radians(state['roll_deg']))*math.cos(math.radians(state['pitch_deg'])))
                    effort = math.sqrt(max(0, self.mass*(G+az)/(self.max_thrust*vertical_fraction)))
                    requested = clamp(1000+1000*effort, 1250, 1700)
                    self.throttle += clamp(requested-self.throttle, -PILOT['throttle_slew_us_s']*dt, PILOT['throttle_slew_us_s']*dt)
                    roll = angle_channel(target_roll, self.limits, 0)
                    pitch = angle_channel(target_pitch, self.limits, 1)
                    throttle = int(round(self.throttle))
                else:
                    roll = pitch = yaw = 1500; throttle = 1000
                    self.throttle = 1000.0; self.integral = 0.0
                for channel, value in [(0, roll), (1, pitch), (2, throttle), (3, yaw)]:
                    self.receiver.set(channel, value)
                with self.receiver.lock: channels = self.receiver.channels.copy()
                row = {'wall_monotonic': time.monotonic(), 'phase': phase, **state,
                       'reference_z_m': self.reference_z,
                       'requested_roll_deg': target_roll, 'requested_pitch_deg': target_pitch,
                       **{f'rc{i}': value for i, value in enumerate(channels)}}
                with self.lock: self.latest = row
                if self.writer is None:
                    self.writer = csv.DictWriter(self.stream, fieldnames=list(row)); self.writer.writeheader()
                self.writer.writerow(row); self.stream.flush()
                self.stop.wait(1/PILOT['rate_hz'])
        except Exception as error:
            self.fault = f'{type(error).__name__}: {error}'
            self.receiver.set(4, 1000); self.receiver.set(2, 1000)

    def close(self):
        self.stop.set()
        if self.thread.is_alive(): self.thread.join(timeout=2)
        self.stream.close(); self.trace.close()


def map_three(x, y, value):
    value = clamp(value, min(x), max(x))
    i = 0 if ((x[1] > x[0]) == (value <= x[1])) else 1
    return y[i]+(value-x[i])/(x[i+1]-x[i])*(y[i+1]-y[i])


def rear_axis(roll, pitch, yaw, joint):
    r, p, y, j = (math.radians(v) for v in (roll, pitch, yaw, joint))
    # Rz(yaw) Ry(pitch) Rx(roll) times the rear axis Rx(-joint) * +Z.
    vx = 0.0; vy = math.sin(j); vz = math.cos(j)
    vy, vz = math.cos(r)*vy-math.sin(r)*vz, math.sin(r)*vy+math.cos(r)*vz
    vx, vz = math.cos(p)*vx+math.sin(p)*vz, -math.sin(p)*vx+math.cos(p)*vz
    return (math.cos(y)*vx-math.sin(y)*vy, math.sin(y)*vx+math.cos(y)*vy, vz)


def analyze(directory, phases, origin, parameters, telemetry):
    with (directory/'pilot.csv').open() as stream: raw = list(csv.DictReader(stream))
    rows = [{k: (v if k == 'phase' else float(v)) for k, v in row.items()} for row in raw]
    flight = [r for r in rows if r['phase'] not in ['idle', 'disarmed']]
    hover = [r for r in rows if r['phase'] == 'hover']
    assert len(hover) > 50, 'Insufficient hover samples'
    target = origin[2]+1
    result = {'actual_takeoff_height_m': max(r['z_m'] for r in flight)-origin[2],
              'hover_samples': len(hover),
              'hover_height_above_initial_m_range': [min(r['z_m'] for r in hover)-origin[2], max(r['z_m'] for r in hover)-origin[2]],
              'hover_max_abs_altitude_error_m': max(abs(r['z_m']-target) for r in hover),
              'hover_max_horizontal_error_m': max(math.hypot(r['x_m']-origin[0], r['y_m']-origin[1]) for r in hover),
              'hover_max_abs_roll_pitch_deg': max(max(abs(r['roll_deg']), abs(r['pitch_deg'])) for r in hover),
              'flight_max_abs_roll_pitch_deg': max(max(abs(r['roll_deg']), abs(r['pitch_deg'])) for r in flight)}
    assert result['actual_takeoff_height_m'] > .8, result
    assert result['hover_max_abs_altitude_error_m'] < .3, result
    assert result['hover_max_horizontal_error_m'] < 1.2, result
    assert result['hover_max_abs_roll_pitch_deg'] < 12, result
    assert result['flight_max_abs_roll_pitch_deg'] < 25, result
    adc_errors = []; axis_errors = []; reaction_errors = []
    for r in rows:
        expected_adc = map_three(parameters['angle_deg'], parameters['adc'], r['joint_angle_deg'])
        adc_errors.append(abs(r['pb0']-expected_adc))
        assert r['pc0'] == 0
        expected_axis = rear_axis(r['roll_deg'], r['pitch_deg'], r['yaw_deg'], r['joint_angle_deg'])
        actual_axis = [r['rear_axis_world_'+c] for c in 'xyz']
        axis_errors.append(math.sqrt(sum((a-b)**2 for a, b in zip(expected_axis, actual_axis))))
        for i in range(3):
            reaction_errors.append(abs(r[f'reaction{i}_nm'] + parameters['spin_sign'][i] *
                                       parameters['reaction_ratio_m'][i] * r[f'thrust{i}_n']))
    result['max_adc_quantization_error_counts'] = max(adc_errors)
    result['max_rear_thrust_reaction_axis_error'] = max(axis_errors)
    result['max_reaction_torque_formula_error_nm'] = max(reaction_errors)
    assert max(adc_errors) <= .501 and max(axis_errors) < 1e-5 and max(reaction_errors) < 1e-8, result
    result['joint_angle_deg_range'] = [min(r['joint_angle_deg'] for r in flight), max(r['joint_angle_deg'] for r in flight)]
    assert result['joint_angle_deg_range'][1]-result['joint_angle_deg_range'][0] > 2
    yaw_results = []
    for name in ['yaw-positive', 'yaw-negative']:
        selected = [r for r in rows if r['phase'] == name]
        start, end = selected[0]['yaw_deg'], selected[-1]['yaw_deg']
        change = (end-start+180) % 360-180
        mean_rate = sum(r['gyro_frd_z'] for r in selected)/len(selected)
        yaw_results.append({'phase': name, 'yaw_change_world_ccw_deg': change,
                            'mean_sensor_frd_yaw_rate_rad_s': mean_rate,
                            'servo_pwm_us_range': [min(r['servo_pwm_us'] for r in selected), max(r['servo_pwm_us'] for r in selected)]})
        assert abs(change) > 2, yaw_results[-1]
    assert yaw_results[0]['yaw_change_world_ccw_deg']*yaw_results[1]['yaw_change_world_ccw_deg'] < 0, yaw_results
    result['yaw_pulses'] = yaw_results
    return result


def run_variant(artifacts, variant, plugin, source_world, directory):
    directory.mkdir()
    world = directory/'world.sdf'; materialize_world(source_world, world)
    tree = ET.parse(world); model = tree.find("./world/model[@name='tricopter']")
    bridge = model.find("plugin[@name='tricopter::TricopterBridge']")
    parameters = {'mass_kg': sum(float(m.text) for m in model.findall('./link/inertial/mass')),
                  'max_thrust_n': [float(v) for v in bridge.find('max_thrust_n').text.split()],
                  'angle_deg': [float(v) for v in bridge.find('angle_knots_deg').text.split()],
                  'adc': [float(v) for v in bridge.find('adc_knots').text.split()],
                  'spin_sign': [float(v) for v in bridge.find('spin_sign').text.split()],
                  'reaction_ratio_m': [float(v) for v in bridge.find('reaction_ratio_m').text.split()]}
    parameters['theoretical_hover_pwm_us'] = 1000+1000*math.sqrt(parameters['mass_kg']*G/sum(parameters['max_thrust_n']))
    binary = verified_binary(artifacts, variant); config = provision(binary, directory, variant)
    record = {'variant': variant, 'pass': False, 'external_rc_pilot': PILOT,
              'pilot_reads_gazebo_ground_truth': True, 'firmware_autopilot_used': False,
              'only_rc_inputs_written': True, 'plant_parameters': parameters,
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'plugin_sha256': hashlib.sha256(plugin.read_bytes()).hexdigest(),
              'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
              'firmware_config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(),
              'phases': []}
    firmware = gazebo = receiver = client = pilot = None
    telemetry = []
    try:
        firmware = start_firmware(binary, directory); client = Msp.connect(port=5762); client.request(1)
        limits = read_limits(client); record['angle_rate_mapping'] = limits
        boxes = client.request(119)
        environment = os.environ.copy()
        environment['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(plugin.parent)+os.pathsep+environment.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        environment['TRICOPTER_TRACE'] = str(directory/'plant.csv'); environment['GZ_PARTITION'] = f'tricopter-flight-{os.getpid()}-{variant}'
        command = ['gz', 'sim', '-s', '-r', '-v', '2', str(world)]
        record['command'] = command
        with (directory/'gazebo.log').open('w') as stream:
            gazebo = start_child(command, cwd=directory, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        receiver = RecordedReceiver(); receiver.set(5, 1900); receiver.start()
        wait_for_plant(directory, firmware, gazebo, client, variant)
        trace = TraceReader(directory/'plant.csv')
        pilot = Pilot(receiver, directory/'pilot.csv', trace, limits, parameters['mass_kg'], sum(parameters['max_thrust_n'])); pilot.start()
        deadline = time.monotonic()+25
        while time.monotonic() < deadline:
            state = client.status()
            if state['arming_disable_flags'] == 0 and pilot.snapshot(): break
            time.sleep(.1)
        assert state['arming_disable_flags'] == 0, state
        origin = pilot.origin; record['initial_position_m'] = origin
        receiver.set(4, 1900)
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            state = client.status()
            if state['armed']: break
            time.sleep(.1)
        assert state['armed'], state
        target = origin[2]+1

        def phase(name, duration, destination=target, yaw=1500, active=True):
            start = pilot.snapshot()['sim_time_s']
            pilot.command(name, active, destination, yaw)
            detail = {'name': name, 'start_sim_time_s': start, 'duration_s': duration,
                      'destination_z_m': destination, 'yaw_rc_us': yaw}
            record['phases'].append(detail)
            next_msp = 0.0; wall_deadline = time.monotonic()+max(15, duration*4)
            while pilot.snapshot()['sim_time_s'] < start+duration:
                if pilot.fault: raise RuntimeError(pilot.fault)
                if firmware.poll() is not None or gazebo.poll() is not None: raise RuntimeError('SITL/Gazebo process exited')
                if time.monotonic() > wall_deadline: raise TimeoutError('Physics did not advance')
                if time.monotonic() >= next_msp:
                    before = pilot.snapshot(); status = client.status()
                    attitude = struct.unpack('<3h', client.request(108))
                    row = {'wall_monotonic': time.monotonic(), 'phase': name, **status,
                           'attitude': list(attitude), 'state_before': before, 'state_after': pilot.snapshot()}
                    if variant == 'modified': row['debug'] = list(struct.unpack('<8h', client.request(254)))
                    telemetry.append(row)
                    if active and not status['armed']: raise AssertionError(f'Disarmed during {name}: {status}')
                    forbidden = [box for box in [3, 11, 56]
                                 if box in boxes and status['mode_mask'] & (1 << boxes.index(box))]
                    assert not forbidden, f'Native altitude/position/autopilot mode unexpectedly active: {forbidden}'
                    next_msp = time.monotonic()+.15
                time.sleep(.02)
            detail['end_sim_time_s'] = pilot.snapshot()['sim_time_s']
            snapshot = pilot.snapshot()
            print(f"{variant} {name}: z={snapshot['z_m']:.3f}m xy=({snapshot['x_m']:.3f},{snapshot['y_m']:.3f}) "
                  f"RP=({snapshot['roll_deg']:.2f},{snapshot['pitch_deg']:.2f}) joint={snapshot['joint_angle_deg']:.2f}", flush=True)

        phase('takeoff', 6)
        phase('hover', 3)
        phase('yaw-positive', .7, yaw=1575)
        phase('yaw-positive-settle', 1.5)
        phase('yaw-negative', .7, yaw=1425)
        phase('yaw-negative-settle', 2)
        phase('landing', 4.5, destination=origin[2])
        receiver.set(4, 1000); receiver.set(2, 1000); pilot.command('disarmed', active=False)
        phase('disarmed', 1, active=False)
        final = client.status(); record['final_status'] = final
        assert not final['armed'], final
        assert all(pilot.snapshot()[f'pwm{i}_us'] == 1000 for i in range(3))
        pilot.close(); pilot = None
        record['metrics'] = analyze(directory, record['phases'], origin, parameters, telemetry)
        record['native_altitude_position_autopilot_modes_inactive'] = True
        if variant == 'modified':
            armed = [r for r in telemetry if r['armed']]
            good = [r for r in armed if r['debug'][6] & 2]
            record['modified_feedback_valid_fraction'] = len(good)/len(armed)
            assert record['modified_feedback_valid_fraction'] > .98
            hover_samples = [r for r in good if r['phase'] == 'hover']
            errors = [r['debug'][0]/10-r['state_after']['joint_angle_deg'] for r in hover_samples]
            record['firmware_angle_vs_actual_joint_hover_rms_deg'] = math.sqrt(sum(e*e for e in errors)/len(errors))
            record['firmware_angle_vs_actual_joint_hover_max_deg'] = max(abs(e) for e in errors)
            assert record['firmware_angle_vs_actual_joint_hover_max_deg'] < .5
        record['pass'] = True
    except Exception as error:
        record['error'] = f'{type(error).__name__}: {error}'; record['traceback'] = traceback.format_exc()
        raise
    finally:
        if receiver: receiver.set(4, 1000); receiver.set(2, 1000)
        if pilot: pilot.close()
        if receiver:
            receiver.close()
            intervals = [b['monotonic']-a['monotonic'] for a, b in zip(receiver.sent, receiver.sent[1:])]
            record['rc_packets'] = len(receiver.sent)
            record['rc_max_send_gap_seconds'] = max(intervals) if intervals else None
            (directory/'rc-packets.json').write_text(json.dumps(receiver.sent, indent=2)+'\n')
        if client: client.close()
        stop_process(gazebo); stop_process(firmware)
        record['all_owned_processes_stopped'] = all(p is None or p.poll() is not None for p in [gazebo, firmware])
        (directory/'telemetry.json').write_text(json.dumps(telemetry, indent=2)+'\n')
        (directory/'flight-check.json').write_text(json.dumps(record, indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['stock', 'modified', 'both'], default='both')
    parser.add_argument('--artifacts', type=Path, default=ARTIFACTS)
    parser.add_argument('--world', type=Path, default=HERE/'gazebo/worlds/tricopter.sdf')
    args = parser.parse_args(); artifacts = args.artifacts.resolve()
    with SessionLock():
        plugin = build_plugin(artifacts, 4)
        directory = fresh_run_directory(artifacts, args.variant, 'flight-validation')
        print(f'Actual Gazebo flight evidence: {directory}', flush=True)
        report = {'pass': False, 'variants': [], 'boundary': __doc__}
        try:
            for variant in (['stock', 'modified'] if args.variant == 'both' else [args.variant]):
                report['variants'].append(run_variant(artifacts, variant, plugin, args.world.resolve(), directory/variant))
                print(f'{variant}: actual physics flight check PASS', flush=True)
            report['pass'] = True
        finally: (directory/'flight-summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print('All flight-check processes stopped; shared ports released', flush=True)


if __name__ == '__main__': main()
