"""Pinned TRI_GROUND inputs and complete-unit split; no actuator model in Python."""
from dataclasses import dataclass
from pathlib import Path
import csv
import hashlib
import json
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PROTOCOL = Path(__file__).with_name('servo_replay_protocol.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_csv(path):
    with Path(path).open() as f:
        rows = list(csv.DictReader(f, skipinitialspace=True))
    if not rows:
        raise ValueError('No decoded rows')
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def decode_inputs(decoder, output, protocol):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    tool_record = json.loads((ROOT/'docs/validation/tricopter-ground-2026-11-06/tools/blackbox-decoder-provenance.json').read_text())
    if sha(decoder) != tool_record['sha256']:
        raise ValueError('Decoder binary does not match pinned adapted official decoder; record/review a new input provenance explicitly')
    manifest = {'schema': 'TRI_GROUND', 'protocol_sha256': sha(PROTOCOL),
                'decoder': tool_record, 'datasets': {}}
    for name, config in protocol['datasets'].items():
        raw = ROOT/config['raw']
        if sha(raw) != config['raw_sha256']:
            raise ValueError(f'{name}: archived raw BBL hash mismatch')
        folder = output/name
        folder.mkdir(exist_ok=True)
        command = [str(Path(decoder).resolve()), '--unit-vbat', 'raw', '--unit-flags', 'raw',
                   '--save-headers', '--output-dir', str(folder), str(raw)]
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
        (folder/'decoder-stdout.log').write_text(completed.stdout)
        (folder/'decoder-stderr.log').write_text(completed.stderr)
        if completed.returncode:
            raise RuntimeError(f'Decoder failed for {name}: {completed.stderr}')
        csvpath = folder/(raw.stem+'.01.csv')
        headers = folder/(raw.stem+'.01.headers.csv')
        if not csvpath.exists() or not headers.exists():
            raise ValueError('Expected exactly the first archived recording')
        with headers.open() as f:
            header = {r['fieldname'].strip(): r[' fieldvalue'].strip() if ' fieldvalue' in r else r['fieldvalue'].strip()
                      for r in csv.DictReader(f)}
        if header.get('debug_mode') != '103':
            raise ValueError(f'{name}: expected captured TRI_GROUND debug mode103')
        manifest['datasets'][name] = {'raw': str(raw), 'raw_sha256': sha(raw),
                                      'csv': str(csvpath), 'csv_sha256': sha(csvpath),
                                      'headers': str(headers), 'headers_sha256': sha(headers),
                                      'events': str(ROOT/config['events']), 'events_sha256': sha(ROOT/config['events']),
                                      'decoder_command': command}
    path = output/'input-provenance.json'
    path.write_text(json.dumps(manifest, indent=2)+'\n')
    return path


@dataclass
class Unit:
    name: str
    split: str
    time_us: np.ndarray
    command_us: np.ndarray
    position_counts: np.ndarray
    valid: np.ndarray
    age_100us: np.ndarray
    transient: np.ndarray
    plateau: np.ndarray
    initial_command_us: float
    initial_direction: int
    metadata: dict


def validated_data(csvpath):
    d = read_csv(csvpath)
    required = ['time (us)']+[f'debug[{i}]' for i in range(8)]
    if any(k not in d for k in required):
        raise ValueError('TRI_GROUND CSV fields unavailable')
    t = d['time (us)']
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError('Non-finite or non-increasing timestamps')
    flags = d['debug[5]'].astype(np.int64)
    p, g, y = (d[f'debug[{i}]'] for i in (0, 2, 3))
    available = ((flags & 7) == 7) & (p >= 0) & (p <= 4095) & (g >= 0) & (g <= 4095) & (y != -32768)
    if np.any(available & (y != p-g)):
        raise ValueError('Signed corrected source identity failed; do not use old TRI_SERVO CSV')
    valid = available & ((flags & 8) != 0) & (p > 0) & (p < 4095) & (g < 4095) & np.isfinite(y)
    if not np.any(valid):
        raise ValueError('No usable corrected position')
    return d, valid, {'frames': len(t), 'valid_frames': int(valid.sum()),
                       'ground_floor_frames': int(np.count_nonzero(available & (g == 0))),
                       'ground_ceiling_frames': int(np.count_nonzero(available & (g == 4095))),
                       'wiper_rail_frames': int(np.count_nonzero(available & ((p == 0) | (p == 4095)))),
                       'max_pair_observation_age_100us': float(np.max(d['debug[4]'])),
                       'observation_age_is_conversion_age': False}


def load_units(provenance_path, csv_overrides=None):
    protocol = json.loads(PROTOCOL.read_text())
    provenance = json.loads(Path(provenance_path).read_text())
    if provenance.get('schema') != 'TRI_GROUND' or provenance.get('protocol_sha256') != sha(PROTOCOL):
        raise ValueError('Input provenance/schema/protocol mismatch')
    units, diagnostics = [], {}
    for name, config in protocol['datasets'].items():
        record = provenance['datasets'][name]
        csvpath = Path((csv_overrides or {}).get(name) or record['csv'])
        if record['raw_sha256'] != config['raw_sha256'] or sha(ROOT/config['raw']) != config['raw_sha256']:
            raise ValueError(f'{name}: raw BBL provenance mismatch')
        if sha(csvpath) != record['csv_sha256']:
            raise ValueError(f'{name}: decoded CSV provenance mismatch')
        if sha(ROOT/config['events']) != record['events_sha256']:
            raise ValueError(f'{name}: event provenance mismatch')
        d, valid, diagnostics[name] = validated_data(csvpath)
        events = json.loads((ROOT/config['events']).read_text())
        pwm, t = d['debug[1]'], d['time (us)']
        starts = np.r_[0, np.flatnonzero(np.diff(pwm) != 0)+1]
        targets = [int(pwm[k]) for k in starts]
        if targets != [int(e['target_us']) for e in events]:
            raise ValueError(f'{name}: recorded PWM/event sequence mismatch')
        ends = np.r_[starts[1:], len(t)]
        row_segment = np.empty(len(t), int)
        groups = {}
        for segment, (start, end, event) in enumerate(zip(starts, ends, events)):
            row_segment[start:end] = segment
            if name == 'ground' and event.get('cycle', -1) >= 0:
                key = ('cycle', event['cycle'])
                train = config['training_units']
                validation = config['validation_units']
            elif name == 'hysteresis' and event.get('phase') in ('approach-pre', 'approach-center'):
                key = ('approach', event['repetition'])
                train = config['approach_training_units']
                validation = config['approach_validation_units']
            elif name == 'hysteresis' and event.get('phase') == 'staircase':
                key = ('staircase', event['cycle'])
                train = config['staircase_training_units']
                validation = config['staircase_validation_units']
            else:
                continue
            split = 'training' if key[1] in train else 'validation' if key[1] in validation else None
            if split is None:
                raise ValueError('Undeclared split unit')
            groups.setdefault(key, {'segments': [], 'split': split})['segments'].append(segment)
        for key, group in groups.items():
            ss = group['segments']
            if ss != list(range(min(ss), max(ss)+1)):
                raise ValueError('Complete split unit is not contiguous')
            start, end = int(starts[min(ss)]), int(ends[max(ss)])
            possible = np.flatnonzero(valid[start:end])
            if not len(possible):
                raise ValueError('No valid unit initialization')
            initial = start+int(possible[0])
            if initial-start > 5:
                raise ValueError('Unit starts without a timely valid initialization; do not silently move boundary')
            indices = np.arange(initial, end)
            commands = pwm[indices].copy()
            segment_ids = row_segment[indices]
            age_in_command = (t[indices]-t[starts[segment_ids]])/1e6
            next_times = np.where(ends[segment_ids] < len(t), t[np.minimum(ends[segment_ids], len(t)-1)], t[-1]+np.median(np.diff(t)))
            to_end = (next_times-t[indices])/1e6
            prior_change = min(ss)-1
            prior_cmd = float(pwm[starts[prior_change]]) if prior_change >= 0 else float(commands[0])
            previous_cmd = float(pwm[starts[prior_change-1]]) if prior_change >= 1 else prior_cmd
            direction = int(np.sign(prior_cmd-previous_cmd))
            preceding_hold_us = float(t[start]-t[starts[prior_change]]) if prior_change >= 0 else 0.
            if preceding_hold_us < 150000:
                raise ValueError('Declared unit needs at least150ms preceding command hold for initial-history assumption')
            # Direction/history comes only from prior command metadata, not extra positions.
            units.append(Unit(f'{name}:{key[0]}:{key[1]}', group['split'],
                              np.ascontiguousarray(t[indices]-t[initial], dtype=np.float64),
                              np.ascontiguousarray(commands, dtype=np.float64),
                              np.ascontiguousarray(d['debug[3]'][indices], dtype=np.float64),
                              valid[indices].copy(), d['debug[4]'][indices].copy(), age_in_command < .150,
                              to_end <= .100, prior_cmd, direction,
                              {'dataset': name, 'kind': key[0], 'index': key[1],
                               'first_recorded_time_us': int(t[initial]), 'segments': ss,
                               'preceding_command_hold_us': preceding_hold_us,
                               'initial_play_memory_assumption': 'Prior command approached in last recorded PWM direction; no extra position history used',
                               'initialization_count': 1, 'skipped_initial_invalid_rows': initial-start}))
    return units, diagnostics, provenance
