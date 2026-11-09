#!/usr/bin/env python3
"""Exercise real stock/modified SITL UDP, MSP and virtual ADC without Gazebo.

This is a transport/axis smoke check, not a vehicle dynamics or flight test.
Every process uses a fresh synthetic EEPROM under the artifact directory.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import socket
import struct
import threading
import time
import traceback

from runtime import (ADC_PACKET, ARTIFACTS, FDM_PACKET, Msp, RAW_PWM_PACKET,
                     RcFeed, SessionLock, fresh_run_directory, provision,
                     start_firmware, stop_process)
from run import verified_binary

GRAVITY = 9.80665
TRI_FEEDBACK = 1 << 30


def quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    cr, cp, cy = (math.cos(v / 2) for v in (roll, pitch, yaw))
    sr, sp, sy = (math.sin(v / 2) for v in (roll, pitch, yaw))
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)


def fdm_fields(timestamp: float, angles: tuple[float, ...],
               derivatives: tuple[float, ...]) -> tuple[float, ...]:
    """Match the plugin: FLU pose, sensor-FRD gyro and negated specific force."""
    roll, pitch, yaw = angles
    dr, dp, dy = derivatives
    omega_flu = (dr-dy*math.sin(pitch),
                 dp*math.cos(roll)+dy*math.sin(roll)*math.cos(pitch),
                 -dp*math.sin(roll)+dy*math.cos(roll)*math.cos(pitch))
    gyro_frd = (omega_flu[0], -omega_flu[1], -omega_flu[2])
    # Stationary world origin: a=0, so R^T(a-g) is body-specific force.
    specific_flu = (-GRAVITY*math.sin(pitch),
                    GRAVITY*math.sin(roll)*math.cos(pitch),
                    GRAVITY*math.cos(roll)*math.cos(pitch))
    acceleration_wire = tuple(-v for v in specific_flu)
    w, x, y, z = quaternion(roll, pitch, yaw)
    conjugated = (w, x, -y, -z)  # Rx(pi) * q_FLU_ENU * Rx(pi)
    return (timestamp, *gyro_frd, *acceleration_wire, *conjugated,
            0, 0, 0, 8.545594, 47.397742, 0, 101325)


class Stimulus:
    def __init__(self, rate: float = 400):
        self.rate = rate
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.thread = threading.Thread(target=self._run, daemon=True)
        now = time.monotonic()
        self.fdm_origin = self.adc_origin = now
        self.transition = (now, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.pb0, self.pc0 = 1490, 0
        self.epoch, self.sequence = 100, 0
        self.mode, self.frozen_sequence = 'valid', 0
        self.last_fdm = None
        self.counts: Counter[str] = Counter()

    def _pose(self, now):
        start, duration, source, target = self.transition
        t = min(1.0, max(0.0, (now-start)/duration)) if duration else 1.0
        pose = tuple(a+(b-a)*t for a, b in zip(source, target))
        velocity = tuple((b-a)/duration if t < 1 and duration else 0.0
                         for a, b in zip(source, target))
        return pose, velocity

    def set_pose(self, roll=0.0, pitch=0.0, yaw=0.0, duration=1.0):
        with self.lock:
            now = time.monotonic()
            source, _ = self._pose(now)
            target = tuple(math.radians(v) for v in (roll, pitch, yaw))
            self.transition = (now, duration, source, target)

    def set_adc(self, pb0=1490, pc0=0, mode='valid', reset_epoch=False):
        with self.lock:
            if mode == 'wrong-sequence':
                self.frozen_sequence = self.sequence
            if reset_epoch:
                self.epoch = (self.epoch+1) & 0xffffffff
                self.adc_origin = time.monotonic()
                self.sequence = 0
            self.pb0, self.pc0, self.mode = pb0, pc0, mode

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            now = time.monotonic()
            with self.lock:
                angles, derivatives = self._pose(now)
                fields = fdm_fields(now-self.fdm_origin, angles, derivatives)
                self.last_fdm = fields
                self.sequence = (self.sequence+1) & 0xffffffff
                sequence = self.frozen_sequence if self.mode == 'wrong-sequence' else self.sequence
                adc = ADC_PACKET.pack(0x31414454, 1, 32, now-self.adc_origin,
                                      self.epoch, sequence, self.pb0, self.pc0, 0)
                mode = self.mode
            if mode != 'withdrawn':
                self.socket.sendto(adc, ('127.0.0.1', 9005))
                self.counts['adc_'+mode] += 1
            self.socket.sendto(FDM_PACKET.pack(*fields), ('127.0.0.1', 9003))
            self.counts['fdm'] += 1
            self.stop.wait(1/self.rate)

    def close(self):
        self.stop.set()
        if self.thread.is_alive(): self.thread.join(timeout=2)
        self.socket.close()


class RawOutputs:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 9001))
        self.socket.settimeout(.1)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.latest = None
        self.counts: Counter[str] = Counter()

    def start(self): self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            try: data = self.socket.recv(512)
            except socket.timeout: continue
            except OSError: break
            self.counts['datagrams'] += 1
            self.counts['bytes_'+str(len(data))] += 1
            if len(data) == RAW_PWM_PACKET.size:
                values = RAW_PWM_PACKET.unpack(data)
                self.counts['motor_count_'+str(values[0])] += 1
                if values[0] == 3:
                    self.latest = {'received_monotonic': time.monotonic(),
                                   'motor_count': values[0], 'values': list(values[1:]),
                                   'hex': data.hex()}

    def close(self):
        self.stop.set()
        if self.thread.is_alive(): self.thread.join(timeout=2)
        self.socket.close()


def sample(msp: Msp, outputs: RawOutputs, label: str) -> dict:
    status = msp.status()
    angles = struct.unpack('<3h', msp.request(108))
    debug = struct.unpack('<8h', msp.request(254))
    return {'label': label, 'monotonic': time.monotonic(), **status,
            'attitude': {'roll_deg': angles[0]/10, 'pitch_deg': angles[1]/10,
                         'heading_deg': angles[2]},
            'debug': list(debug), 'raw_output': outputs.latest}


def wait_feedback(msp, outputs, observations, valid: bool, label: str, timeout=3):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        value = sample(msp, outputs, label)
        observations.append(value)
        feedback = bool(value['debug'][6] & 2)
        guard = bool(value['arming_disable_flags'] & TRI_FEEDBACK)
        if feedback == valid and guard != valid:
            return value
        time.sleep(.04)
    raise AssertionError(f'{label}: feedback/arming guard did not become {valid}: {value}')


def run_variant(artifacts: Path, directory: Path, variant: str, frame_hold: float) -> dict:
    binary = verified_binary(artifacts,variant)
    record = {'variant': variant, 'binary': str(binary),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'observations': [], 'pass': False}
    process = msp = outputs = stimulus = receiver = None
    directory.mkdir()
    try:
        provision(binary, directory, variant)
        outputs = RawOutputs(); outputs.start()
        process = start_firmware(binary, directory)
        msp = Msp.connect(port=5762)
        primary_api = msp.request(1)
        configurator = Msp.connect(port=5761)
        try:
            configurator_api = configurator.request(1)
            assert primary_api == configurator_api and len(primary_api) == 3
            assert msp.request(1) == primary_api  # Both clients remain connected.
            record['simultaneous_msp_ports'] = {'5761': configurator_api.hex(),
                                                '5762': primary_api.hex()}
        finally:
            configurator.close()
        # A completed MSP roundtrip proves initialization reached the main loop;
        # TCP listen alone can precede creation of the virtual gyro pointers.
        time.sleep(.5)
        stimulus = Stimulus(); stimulus.start()
        receiver = RcFeed(); receiver.start()
        time.sleep(3)
        initial = sample(msp, outputs, 'initial-disarmed')
        record['observations'].append(initial)
        assert not initial['armed'], initial
        raw = outputs.latest
        assert raw and time.monotonic()-raw['received_monotonic'] < .5, raw
        assert raw['motor_count'] == 3 and len(raw['values']) == 16, raw
        assert raw['values'][:3] == [1000.0]*3, raw
        assert 1100 <= raw['values'][3] <= 1700, raw
        logical_servo = struct.unpack('<8H', msp.request(103))[5]
        assert abs(raw['values'][3]-logical_servo) <= 5, (raw, logical_servo)
        record['raw_68byte_three_motors_servo0_after_motors'] = True

        frame_results = []
        for axis, degrees in [('roll', 10), ('roll', -10), ('pitch', 10), ('pitch', -10)]:
            stimulus.set_pose(duration=.5); time.sleep(.7)
            stimulus.set_pose(**{axis: degrees}, duration=1.0)
            time.sleep(1.0+frame_hold)
            rows = []
            for _ in range(5):
                value = sample(msp, outputs, f'frame-{axis}-{degrees}')
                record['observations'].append(value); rows.append(value['attitude'])
                time.sleep(.06)
            actual = sum(row[axis+'_deg'] for row in rows)/len(rows)
            cross = sum(row[('pitch' if axis == 'roll' else 'roll')+'_deg'] for row in rows)/len(rows)
            frame_results.append({'axis': axis, 'input_deg': degrees,
                                  'mean_output_deg': actual, 'cross_axis_deg': cross,
                                  'last_wire_fdm_fields': stimulus.last_fdm})
            assert abs(actual-degrees) < 1.2 and abs(cross) < 1.2, frame_results[-1]
        record['mahony_frame_checks'] = frame_results
        stimulus.set_pose(duration=1); time.sleep(1.5)

        if variant == 'modified':
            wait_feedback(msp, outputs, record['observations'], True, 'feedback-initial')
            angles = []
            for pb0, pc0, expected in [(1936, 0, -478), (1281, 0, 216),
                                       (1490, 0, 0), (1700, 210, 0)]:
                stimulus.set_adc(pb0, pc0); time.sleep(.12)
                row = wait_feedback(msp, outputs, record['observations'], True, f'adc-{pb0}-{pc0}')
                assert abs(row['debug'][0]-expected) <= 1, row
                assert 1100 <= row['debug'][2] <= 1700, row
                angles.append({'pb0': pb0, 'pc0': pc0, 'difference': pb0-pc0,
                               'expected_angle_deci_deg': expected,
                               'measured_angle_deci_deg': row['debug'][0]})
            record['canonical_adc_angle_checks'] = angles
            stimulus.set_adc(mode='withdrawn'); time.sleep(.15)
            stale = wait_feedback(msp, outputs, record['observations'], False, 'adc-withdrawn')
            assert stale['debug'][0] == -32768, stale
            stimulus.set_adc(); time.sleep(.1)
            wait_feedback(msp, outputs, record['observations'], True, 'adc-recovered')
            stimulus.set_adc(mode='wrong-sequence'); time.sleep(.15)
            replay = wait_feedback(msp, outputs, record['observations'], False, 'adc-wrong-sequence')
            assert replay['debug'][0] == -32768, replay
            stimulus.set_adc(reset_epoch=True); time.sleep(.1)
            wait_feedback(msp, outputs, record['observations'], True, 'adc-new-epoch-time-zero')
            record['withdrawal_replay_stale_guard_and_epoch_recovery'] = True

        # Software motors only: prove all three channels and the fourth float's
        # servo response while armed; the input FDM remains a stationary stub.
        receiver.set(4, 1900)
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            state = msp.status()
            if state['armed']: break
            time.sleep(.1)
        assert state['armed'], state
        receiver.set(2, 1200); receiver.set(3, 1575); time.sleep(.35)
        armed = sample(msp, outputs, 'armed-software-output-only')
        record['observations'].append(armed)
        assert armed['armed'], armed
        assert all(1000 < v <= 2000 for v in armed['raw_output']['values'][:3]), armed
        assert 1100 <= armed['raw_output']['values'][3] <= 1700, armed
        record['armed_three_motor_and_servo_output'] = armed['raw_output']['values'][:4]
        receiver.set(4, 1000); receiver.set(2, 1000); receiver.set(3, 1500); time.sleep(.25)
        final = sample(msp, outputs, 'final-disarmed')
        record['observations'].append(final)
        assert not final['armed'] and final['raw_output']['values'][:3] == [1000.0]*3, final
        record['pass'] = True
    except Exception as error:
        record['error'] = f'{type(error).__name__}: {error}'
        record['traceback'] = traceback.format_exc()
        raise
    finally:
        if receiver: receiver.close()
        if stimulus:
            stimulus.close(); record['stimulus_packets'] = dict(stimulus.counts)
        if msp: msp.close()
        stop_process(process)
        if outputs:
            outputs.close(); record['raw_output_packets'] = dict(outputs.counts)
        record['process_stopped'] = process is None or process.poll() is not None
        (directory/'protocol-check.json').write_text(json.dumps(record, indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, default=ARTIFACTS/'corrected-frame')
    parser.add_argument('--variant', choices=['stock', 'modified', 'both'], default='both')
    parser.add_argument('--frame-hold', type=float, default=2.0)
    args = parser.parse_args()
    if args.frame_hold < .5: parser.error('--frame-hold must be at least .5 seconds')
    artifacts = args.artifacts.resolve()
    report = {'boundary': 'Real SITL processes and UDP/MSP; stationary synthetic FDM, no Gazebo/physical aircraft.',
              'variants': [], 'pass': False}
    with SessionLock():
        run = fresh_run_directory(artifacts, args.variant, 'protocol')
        print(f'Protocol evidence: {run}', flush=True)
        try:
            for variant in (['stock', 'modified'] if args.variant == 'both' else [args.variant]):
                report['variants'].append(run_variant(artifacts, run/variant, variant, args.frame_hold))
                print(f'{variant}: protocol, frame and output checks passed', flush=True)
            report['pass'] = True
        finally:
            (run/'protocol-summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print('All owned SITL processes stopped; shared ports released', flush=True)


if __name__ == '__main__':
    main()
