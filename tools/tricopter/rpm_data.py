"""Pinned historical TRI_SERVO input and explicitly approximate cache metadata.

There is no recorded packet sequence or measured within-loop phase offset.
The interval-overlap adapter below is a conservative replay convention, never
an assertion of the actual observation timestamp or packet identity.
"""

from dataclasses import dataclass
import csv
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ARCHIVE = ROOT/'docs/validation/tricopter-rpm-2026-11-06'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_csv(path):
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream, skipinitialspace=True))
    if not rows:
        raise ValueError('Empty RPM recording')
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def decode_inputs(decoder, output):
    """Decode each immutable record and require its archived reproduction hash."""
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    decoder = Path(decoder).resolve()
    tool = json.loads((ARCHIVE/'tools/blackbox-decoder-provenance.json').read_text())
    expected = json.loads((ARCHIVE/'capture/archive-reproduction.json').read_text())
    if sha(decoder) != tool['sha256'] or expected['decoder_sha256'] != tool['sha256']:
        raise ValueError('RPM decoder does not match archived pinned adapted binary')
    result = {'schema': 'TRI_SERVO', 'decoder': tool, 'records': []}
    for record in expected['records']:
        raw = ARCHIVE/record['raw']
        if sha(raw) != record['raw_sha256']:
            raise ValueError(f'{raw.name}: raw evidence hash mismatch')
        command = [str(decoder), '--unit-vbat', 'raw', '--unit-flags', 'raw', '--save-headers',
                   '--output-dir', str(output), str(raw)]
        decoded = subprocess.run(command, text=True, capture_output=True, check=False)
        (output/(raw.stem+'.stdout.log')).write_text(decoded.stdout)
        (output/(raw.stem+'.stderr.log')).write_text(decoded.stderr)
        if decoded.returncode:
            raise RuntimeError(f'{raw.name}: decoder failed: {decoded.stderr}')
        csvpath = output/(raw.stem+'.01.csv')
        headers = output/(raw.stem+'.01.headers.csv')
        if sha(csvpath) != record['csv_sha256'] or sha(headers) != record['headers_sha256']:
            raise ValueError(f'{raw.name}: decoded reproduction hash mismatch')
        with headers.open() as stream:
            header = {row['fieldname'].strip(): row.get('fieldvalue', row.get(' fieldvalue', '')).strip()
                      for row in csv.DictReader(stream)}
        if header.get('debug_mode') != '102' or header.get('motor_poles') != '14':
            raise ValueError('Expected historical TRI_SERVO mode102 and 14 poles')
        data = read_csv(csvpath)
        validate_data(data)
        if len(data['time (us)']) != record['rows']:
            raise ValueError('RPM recording row count mismatch')
        result['records'].append({'name': raw.stem, 'raw': str(raw.relative_to(ROOT)),
                                  'raw_sha256': sha(raw), 'csv': str(csvpath), 'csv_sha256': sha(csvpath),
                                  'headers': str(headers), 'headers_sha256': sha(headers),
                                  'decoder_command': command})
    result['archive_manifest_sha256'] = sha(ARCHIVE/'capture/archive-manifest.json')
    (output/'input-provenance.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def validate_data(data):
    required = ['time (us)', 'debug[3]']+[f'debug[{i}]' for i in (4, 5, 6)]+[f'eRPM[{i}]' for i in range(3)]
    if any(key not in data for key in required):
        raise ValueError('Historical TRI_SERVO RPM fields missing')
    lengths = {len(data[key]) for key in required}
    if len(lengths) != 1 or not lengths or not next(iter(lengths)):
        raise ValueError('RPM fields must have matching nonempty lengths')
    times = data['time (us)']
    if (not np.all(np.isfinite(times)) or np.any(times != np.floor(times))
            or np.any(times < 0) or np.any(times > np.iinfo(np.uint32).max)
            or np.any(np.diff(times) <= 0)):
        raise ValueError('Expected strictly increasing exact uint32 Blackbox timestamps')
    flags = data['debug[3]']
    if not np.all(np.isfinite(flags)) or np.any(flags != np.floor(flags)) or np.any(flags < 0):
        raise ValueError('Invalid historical TRI_SERVO status flags')


@dataclass
class ProxyInput:
    now_us: np.ndarray
    observed_us: np.ndarray
    sequence: np.ndarray
    seen: np.ndarray
    mechanical_rpm: np.ndarray
    proxy_new: np.ndarray
    metadata_usable: np.ndarray
    native_age_upper_us: np.ndarray
    interval_lower_us: np.ndarray
    interval_upper_us: np.ndarray
    diagnostics: dict


def observation_proxy(data, phase_allowance_us):
    """Reconstruct a *proxy* source using only causal interval intersections.

    The nonnegative logger phase is assumed to lie in [0, allowance]. This is
    an unmeasured sensitivity assumption, not an established timing bound.
    Equal RPM values with disjoint newer intervals are new proxy observations,
    including zero; equal RPM in overlapping intervals preserves cached age.
    """
    validate_data(data)
    if isinstance(phase_allowance_us, bool) or not isinstance(phase_allowance_us, int) or not 0 <= phase_allowance_us <= 1000000:
        raise ValueError('Phase allowance must be a nonnegative integer microsecond sensitivity')
    times = data['time (us)'].astype(np.int64)
    if np.any(times+phase_allowance_us > np.iinfo(np.uint32).max):
        raise ValueError('Proxy query timestamp exceeds uint32 transport')
    count = len(times)
    now = np.ascontiguousarray(times+phase_allowance_us, dtype=np.uint32)
    observed = np.zeros((count, 3), np.uint32)
    sequence = np.zeros((count, 3), np.uint32)
    seen = np.zeros((count, 3), np.uint8)
    rpm = np.zeros((count, 3), np.float32)
    new = np.zeros((count, 3), bool)
    usable = np.zeros((count, 3), bool)
    age_upper = np.full((count, 3), np.iinfo(np.uint32).max, np.uint32)
    lower = np.full((count, 3), -1, np.int64)
    upper = np.full((count, 3), -1, np.int64)
    flags = data['debug[3]'].astype(np.int64)
    diagnostics = []
    for motor in range(3):
        previous_observed, previous_rpm, seq = 0, 0.0, 0
        cluster = None
        conflicts = 0
        for i, time in enumerate(times):
            age = data[f'debug[{4+motor}]'][i]
            electrical = data[f'eRPM[{motor}]'][i]
            metadata_ok = (bool(flags[i] & 16) and bool(flags[i] & (1 << (5+motor)))
                           and np.isfinite(age) and age == np.floor(age) and 0 <= age < 32767
                           and np.isfinite(electrical) and electrical >= 0)
            if metadata_ok:
                lo, hi = int(time-(age+1)*100+1), int(time-age*100+phase_allowance_us)
                metadata_ok = lo >= 0 and hi <= int(now[i]) and lo <= hi
            if metadata_ok:
                lower[i, motor], upper[i, motor] = lo, hi
                age_upper[i, motor] = (int(age)+1)*100+phase_allowance_us-1
                usable[i, motor] = True
                value = electrical*100.0/7.0
                overlap = cluster is not None and max(lo, cluster[0]) <= min(hi, cluster[1])
                if overlap and value == previous_rpm:
                    cluster = (max(lo, cluster[0]), min(hi, cluster[1]))
                else:
                    candidate = max(lo, previous_observed+1) if seq else lo
                    if candidate <= hi and (not seq or hi > previous_observed):
                        previous_observed, previous_rpm = candidate, value
                        seq += 1
                        cluster = (max(lo, candidate), hi)
                        new[i, motor] = True
                    else:
                        conflicts += 1
                seen[i, motor] = bool(seq)
            # Unseen/saturated/invalid metadata never claims a fresh input.
            observed[i, motor] = previous_observed
            sequence[i, motor] = seq
            rpm[i, motor] = previous_rpm
        diagnostics.append({'logical_motor': motor, 'proxy_observations': int(new[:, motor].sum()),
                            'ambiguous_cached_rows': int(usable[:, motor].sum()-new[:, motor].sum()-conflicts),
                            'metadata_unavailable_rows': int((~usable[:, motor]).sum()),
                            'timestamp_conflicts_skipped': conflicts,
                            'maximum_native_age_lower_us': float(max(0, np.nanmax(data[f'debug[{4+motor}]']))*100)})
    return ProxyInput(now, observed, sequence, seen, rpm, new, usable, age_upper, lower, upper,
                      {'phase_allowance_us': phase_allowance_us, 'identity_and_timing_are_approximate': True,
                       'motors': diagnostics})
