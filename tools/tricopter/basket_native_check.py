#!/usr/bin/env python3
"""Execute measured basket calibration in the real C map and offline allocator.

Motor/servo dynamics and effectiveness remain synthetic in the hybrid runs.
The physical recording is used only for settled count-to-angle validation.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from dynamic_native import NativeDynamic, dynamic_parameters, load_protocol, run_configuration, legacy
from dynamic_audit import validate_priority_every_row


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check(profile_path, validation_path, build):
    profile = json.loads(profile_path.read_text())
    validation = json.loads(validation_path.read_text())
    if validation['frozen_mean_map_sha256'] != sha(profile_path):
        raise ValueError('Validation report belongs to a different frozen profile')
    native = NativeDynamic(build)
    rows = [r for r in validation['points'] if r['qualified_for_scoring'] and r['in_common_count_domain']]
    counts = np.array([r['corrected_counts'] for r in rows])
    actual = np.array([r['measured_basket_roll_deg'] for r in rows])
    predicted = native.angles(profile, counts)
    if not predicted['valid'].all():
        raise ValueError('Native map rejected qualified in-domain count observations')
    native_deg = predicted['angle_rad'].astype(float) * 180 / np.pi
    reference = np.array([r['mean_map_prediction_deg'] for r in rows])
    parity = float(np.max(abs(native_deg - reference)))
    if parity > 5e-5:
        raise ValueError(f'Native float32/Python map mismatch: {parity} degrees')
    low, high = profile['count_bounds']
    if native.angles(profile, [low-1, high+1, np.nan])['valid'].any():
        raise ValueError('Out-of-domain/nonfinite counts must remain unavailable')

    protocol = load_protocol()
    base = legacy.load_protocol()
    d = dynamic_parameters(protocol)
    d[31] = 0  # Current + nominal candidates, with no legacy PWM grid assumption.
    simulations = []
    for frame in base['airframes']:
        _, c, plant, inputs = run_configuration(protocol, frame, protocol['scenarios'][0], protocol['experiments'][2])
        c[19:21] = [-.35, .30]  # Explicit subset of measured angle/predictor coverage.
        inputs = {k: v[:2501].copy() for k, v in inputs.items()}
        result = native.run(c, plant, d, angle_profile=profile, **inputs)
        expected = native.angles(profile, result['numeric'][:,35])
        available = result['angle_flags'][:,0].astype(bool)
        np.testing.assert_array_equal(available, expected['valid'])
        if not available.all() or not result['flags'][:,0].all():
            raise ValueError('Hybrid current angle or command validity is unavailable')
        np.testing.assert_array_equal(result['numeric'][available,34], expected['angle_rad'][available])
        used = result['dynamic_integer'][:,8].astype(bool)
        future = native.angles(profile, result['dynamic_numeric'][used,36])
        if not future['valid'].all() or not result['angle_flags'][used,1].all():
            raise ValueError('Selected horizon angles must be explicitly available')
        np.testing.assert_array_equal(result['dynamic_numeric'][used,1], future['angle_rad'])
        if used.sum() < 100:
            raise ValueError('Too few actual dynamic-allocator uses to verify integration')
        priority = validate_priority_every_row(result, d)
        simulations.append({'airframe': frame['name'], 'ticks': len(inputs['time_us']),
            'dynamic_used_rows': int(used.sum()), 'dynamic_fallback_rows': int((~used).sum()),
            'current_map_matches_C_batch_exactly': True, 'horizon_map_matches_C_batch_exactly': True,
            'global_priority_selection_pass': priority['global_staged_selection_pass'],
            'controller_parameters': c.tolist(), 'plant_parameters': plant.tolist(), 'dynamic_parameters': d.tolist(),
            'outputs_sha256': {k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in result.items()}})
    errors = native_deg - actual
    return {'schema_version':1, 'profile_sha256': sha(profile_path), 'held_out_validation_sha256': sha(validation_path),
        'script_sha256': sha(__file__), 'native_build':native.provenance,
        'settled_replay': {'qualified_rows':len(rows), 'excluded_unqualified_plateau_ids':validation['excluded_unqualified_plateau_ids'],
            'float32_C_vs_Python_max_abs_deg':parity, 'angle_error_rms_deg':float(np.sqrt(np.mean(errors**2))),
            'angle_error_max_abs_deg':float(max(abs(errors))), 'out_of_domain_and_nonfinite_queries_unavailable':True},
        'hybrid_simulations':simulations,
        'scope':'Real settled count-to-angle calibration; all motor/servo dynamics, effectiveness and reaction coefficients remain synthetic. Command-only prediction and flight outputs unchanged.',
        'angle_calibration_installed_in_flight_control':False, 'for_flight':False}


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('profile',type=Path);p.add_argument('validation',type=Path)
    p.add_argument('--build',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    result=check(a.profile,a.validation,a.build)
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'settled_replay':result['settled_replay'],
        'hybrid_simulations':[{k:r[k] for k in ('airframe','ticks','dynamic_used_rows','dynamic_fallback_rows','global_priority_selection_pass')} for r in result['hybrid_simulations']]},indent=2))
