#!/usr/bin/env python3
"""Run frozen synthetic reaction-torque cases through the actual C core."""

import argparse
import copy
import csv
import gzip
import json
import os
from pathlib import Path

import numpy as np

from reaction_cases import PROTOCOL, build_cases, evaluate_case, load_protocol
from reaction_native import NativeReaction, SOURCES
from rpm_data import ROOT, sha


def guard_checks(native, protocol):
    shape = (1, 3)
    baseline = dict(mechanical_rpm=np.full(shape, 1000.), acceleration_rpm_s=np.zeros(shape),
                    age_us=np.zeros(shape, np.uint32), seen=np.ones(shape, np.uint8), fresh=np.ones(shape, np.uint8),
                    valid=np.ones(shape, np.uint8), acceleration_valid=np.ones(shape, np.uint8))
    checks = []
    for name in ('seen', 'fresh', 'valid', 'acceleration_valid', 'stale_age', 'nan_rpm', 'negative_rpm',
                 'rpm_bound', 'acceleration_bound', 'torque_bound', 'unspecified_coefficients'):
        data = copy.deepcopy(baseline)
        parameters = copy.deepcopy(protocol['parameters'])
        source = protocol['coefficient_source']
        if name in ('seen', 'fresh', 'valid', 'acceleration_valid'):
            data[name][0, 1] = 0
        elif name == 'stale_age':
            data['age_us'][0, 1] = parameters['max_rpm_age_us']+1
        elif name == 'nan_rpm':
            data['mechanical_rpm'][0, 1] = np.nan
        elif name == 'negative_rpm':
            data['mechanical_rpm'][0, 1] = -1
        elif name == 'rpm_bound':
            data['mechanical_rpm'][0, 1] = parameters['max_mechanical_rpm']+1
        elif name == 'acceleration_bound':
            data['acceleration_rpm_s'][0, 1] = parameters['max_abs_acceleration_rpm_s']+1
        elif name == 'torque_bound':
            parameters['max_abs_torque_nm'] = 1e-9
        else:
            source = 'UNSPECIFIED'
        out = native.evaluate(parameters, coefficient_source=source, **data)
        numeric = ('omega_rad_s', 'alpha_rad_s2', 'aerodynamic_nm', 'inertial_nm', 'total_nm', 'summed_nm')
        cleared = all(np.all(out[key] == 0) for key in numeric)
        if out['valid'][0] or not cleared:
            raise ValueError(f'{name}: invalid input escaped all-output suppression')
        checks.append({'name': name, 'status': int(out['status'][0]), 'invalid_motor_mask': int(out['invalid_motor_mask'][0]),
                       'valid': bool(out['valid'][0]), 'all_numerical_outputs_zero': cleared})
    return checks


def evaluate(native, protocol, output):
    cases = build_cases(protocol)
    records, predictions = [], []
    with gzip.open(output/'synthetic-reaction-replay.csv.gz', 'wt') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case', 'time_s', 'logical_motor', 'mechanical_rpm', 'acceleration_rpm_s', 'omega_rad_s',
                         'alpha_rad_s2', 'aerodynamic_nm', 'inertial_nm', 'motor_total_nm', 'summed_aerodynamic_nm',
                         'summed_inertial_nm', 'summed_total_nm', 'valid', 'status', 'coefficients_calibrated', 'for_flight'])
        for case in cases:
            out = evaluate_case(native, protocol, case)
            if not out['valid'].all():
                raise ValueError(f'{case["name"]}: synthetic model history unexpectedly invalid')
            errors = {}
            for key, expected in case['oracle'].items():
                if not np.allclose(out[key], expected, rtol=3e-6, atol=2e-8):
                    raise ValueError(f'{case["name"]}: independent analytic oracle failed for {key}')
                errors[key] = float(np.max(abs(out[key]-expected)))
            record = {'name': case['name'], 'kind': case['kind'], 'rows': len(case['time_s']),
                      'maximum_absolute_oracle_errors': errors,
                      'maximum_absolute_summed_torque_nm': float(np.max(abs(out['summed_nm'][:, 2]))),
                      'all_evaluations_valid': bool(out['valid'].all())}
            if case['kind'] in ('ramp', 'free_coast'):
                impulse = np.trapezoid(out['inertial_nm'], x=case['time_s'], axis=0)
                delta = (case['rpm'][-1]-case['rpm'][0])*(2*np.pi)/60
                expected_impulse = np.asarray(protocol['parameters']['reaction_sign'])*np.asarray(protocol['parameters']['rotor_inertia_kg_m2'])*delta
                record['inertial_impulse_nm_s'] = impulse.tolist()
                record['signed_J_delta_omega_nm_s'] = expected_impulse.tolist()
                record['maximum_impulse_error_nm_s'] = float(np.max(abs(impulse-expected_impulse)))
                if not np.allclose(impulse, expected_impulse, rtol=2e-6, atol=1e-8):
                    raise ValueError('Independent angular-momentum impulse invariant failed')
            if case['kind'] == 'free_coast' and np.max(abs(out['summed_nm'][:, 2])) > 3e-9:
                raise ValueError('Free-coast aerodynamic/inertial shaft cancellation failed')
            records.append(record)
            predictions.append((case, out))
            for row, time in enumerate(case['time_s']):
                for motor in range(3):
                    writer.writerow([case['name'], float(time), motor, case['rpm'][row, motor], case['acceleration_rpm_s'][row, motor],
                                     *[float(out[key][row, motor]) for key in ('omega_rad_s', 'alpha_rad_s2', 'aerodynamic_nm', 'inertial_nm', 'total_nm')],
                                     *[float(value) for value in out['summed_nm'][row]], int(out['valid'][row]), int(out['status'][row]),
                                     int(out['calibrated'][row]), int(out['for_flight'][row])])
    plot_cases(predictions, output)
    return records


def plot_cases(predictions, output):
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(15, 8), layout='constrained')
    colors = ('#c58220', '#3275af', '#222222')
    for axis, (case, out) in zip(axes.flat, predictions):
        for component, (color, label) in enumerate(zip(colors, ('aerodynamic', 'inertial', 'total'))):
            axis.plot(case['time_s'], out['summed_nm'][:, component], color=color, label=label,
                      linewidth=1.3, marker='o' if len(case['time_s']) == 1 else None)
        axis.set_title(case['name']+(' (one instant)' if case['kind'] == 'cancellation_snapshot' else ''))
        if case['kind'] == 'cancellation_snapshot':
            scale = float(np.max(abs(out['aerodynamic_nm'])))
            axis.set_ylim(-1.1*scale, 1.1*scale)
            axis.text(.04, .94, f'max float32 residual {np.max(abs(out["summed_nm"])):.2g} N m',
                      transform=axis.transAxes, va='top', fontsize=8)
        axis.set_xlabel('Synthetic seconds')
        axis.set_ylabel('Body yaw reaction torque (N m)')
        axis.grid(alpha=.2)
    axes.flat[0].legend(fontsize=8)
    fig.suptitle('Synthetic placeholder coefficients only: signed aerodynamic + inertial body reaction; fixed near-vertical axes')
    fig.savefig(output/'synthetic-reaction-components.png', dpi=150, bbox_inches='tight')
    plt.close(fig)

    fig, axes = plt.subplots(3, 3, figsize=(15, 10), sharex='col', layout='constrained')
    selected = [pair for pair in predictions if pair[0]['name'] in ('steady_equal', 'rear_braking', 'free_coast')]
    for column, (case, out) in enumerate(selected):
        for row, key in enumerate(('aerodynamic_nm', 'inertial_nm', 'total_nm')):
            axis = axes[row, column]
            for motor, color in enumerate(('#3275af', '#c58220', '#57945a')):
                axis.plot(case['time_s'], out[key][:, motor], color=color, label=f'motor {motor}', linewidth=1)
            axis.plot(case['time_s'], out['summed_nm'][:, row], color='#222222', label='sum', linewidth=1.3, linestyle='--')
            if case['kind'] == 'free_coast' and key == 'total_nm':
                scale = float(np.max(abs(out['aerodynamic_nm'])))
                axis.set_ylim(-1.1*scale, 1.1*scale)
                axis.text(.04, .94, f'max float32 residual {np.max(abs(out["total_nm"])):.2g} N m',
                          transform=axis.transAxes, va='top', fontsize=8)
            axis.set_ylabel(key.replace('_nm', '')+' (N m)')
            axis.grid(alpha=.2)
            if row == 0:
                axis.set_title(case['name'])
            if row == 2:
                axis.set_xlabel('Synthetic seconds')
    axes[0, 0].legend(fontsize=8)
    fig.suptitle('Per-motor decomposition: illustrative synthetic coefficients, no aircraft torque calibration')
    fig.savefig(output/'synthetic-reaction-per-motor.png', dpi=150, bbox_inches='tight')
    plt.close(fig)


def report(summary):
    protocol = summary['protocol']
    parameters = protocol['parameters']
    lines = ['# Synthetic vertical-axis reaction torque', '',
             'The actual portable C core passes independent analytic checks of body reaction signs, SI units and software guards. Every coefficient here is an arbitrary **synthetic placeholder**, uncalibrated and unavailable for flight.', '',
             f'Logical motor order is rear/front-right/front-left, with body reaction signs {parameters["reaction_sign"]}. Positive body yaw is CCW/nose left. Mechanical RPM is a nonnegative rotation magnitude; acceleration is its signed RPM/s derivative. The core converts both by pi/30.', '',
             f'Placeholder kQ values are {parameters["k_q_nm_per_rad_s_squared"]} N m/(rad/s)^2 and rotor inertias J are {parameters["rotor_inertia_kg_m2"]} kg m². {protocol["coefficient_provenance"]}', '',
             'For each rotor, body reaction is sign*(kQ*omega² + J*alpha). The aerodynamic and inertial terms are separate components of the shaft reaction; the free-coast case alpha=-kQ*omega²/J cancels them. The inertial impulse independently matches sign*J*deltaOmega.', '',
             'This scalar formula approximates motor-mount/frame reaction when body angular motion is negligible. Equivalently, it supplies the simplified yaw-equation RHS when the base vehicle inertia includes the rotors\u2019 locked-rotation axial inertia. It is not the total external moment on the aircraft-plus-rotors system. A later full angular-momentum model must account for rotor spin momentum exactly once; do not add this inertial term again alongside -dH_spin/dt. Axes are near-vertical; there is no rear angle, tilted-axis projection, gyroscopic vector model, thrust calculation or allocator.', '',
             f'Software guards are {parameters["max_mechanical_rpm"]:g} RPM, {parameters["max_abs_acceleration_rpm_s"]:g} RPM/s, {parameters["max_rpm_age_us"]:g} us age, and {parameters["max_abs_torque_nm"]:g} N m for every component and sum. They are not measured physical limits. All motors must be seen/fresh/valid and independently acceleration-valid. Missing, stale or unknown-acceleration input clears every numerical output; no partial motor sum is returned.', '',
             '| Synthetic case | Rows | Maximum total torque magnitude (N m) | Maximum torque oracle error (N m) |',
             '|---|---:|---:|---:|']
    for case in summary['cases']:
        error = max(value for key, value in case['maximum_absolute_oracle_errors'].items() if key.endswith('_nm'))
        lines.append(f'| {case["name"]} | {case["rows"]} | {case["maximum_absolute_summed_torque_nm"]:.9g} | {error:.3g} |')
    lines += ['', 'The cancellation case is one algebraic instant, not a constant-speed trace with nonzero acceleration. Ramp, braking and free-coast trajectories have consistent speed/derivative inputs; the braking derivative changes at its stopping boundary.', '',
              f'All {len(summary["guard_checks"])} stored invalid-input/provenance/limit checks suppress the complete torque result. Independent Python/native tests also cover speed-squared scaling, acceleration reversal, full motor-tuple permutation, SI conversion, angular-momentum impulse and malformed transports.', '',
              'Only synthetic trajectories are evaluated. Historical RPM logs have no acceleration ground truth and are not converted into apparent aircraft torque. Servo PB0-PC0 remains the sole measured-position source elsewhere; this model consumes no servo position or angle.', '',
              'Reproduce from the repository root with `python3 tools/tricopter/reaction_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_reaction_replay.py -v`. `reaction-replay-summary.json`, `native-build.json` and `checksums.sha256` pin protocol, sources, compiler and artifacts. No hardware or active actuator path is used.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    protocol_hash = sha(PROTOCOL)
    protocol = load_protocol()
    native = NativeReaction(output/'build')
    cases = evaluate(native, protocol, output)
    summary = {'protocol_sha256': protocol_hash, 'protocol': protocol, 'native_build': native.provenance,
               'cases': cases, 'guard_checks': guard_checks(native, protocol), 'validation_boundary': protocol['validation_boundary'],
               'calibrated': False, 'for_flight': False}
    if sha(PROTOCOL) != protocol_hash or native.provenance['sources'] != {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}:
        raise RuntimeError('Protocol/native sources changed during replay; refusing mixed provenance')
    scripts = [path for path in Path(__file__).parent.glob('reaction_*') if path.is_file()]
    scripts += [Path(__file__).with_name('test_reaction_replay.py'), Path(__file__).with_name('rpm_data.py')]
    summary['script_sha256'] = {str(path.relative_to(ROOT)): sha(path) for path in scripts}
    (output/'reaction-replay-summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    (output/'native-build.json').write_text(json.dumps(native.provenance, indent=2)+'\n')
    (output/'README.md').write_text(report(summary))
    files = sorted(path for path in output.iterdir() if path.is_file() and path.name != 'checksums.sha256')
    (output/'checksums.sha256').write_text(''.join(f'{sha(path)}  {path.name}\n' for path in files))
    print(json.dumps({'output': str(output), 'cases': len(cases), 'guard_checks_passed': len(summary['guard_checks']),
                      'calibrated': False, 'for_flight': False}, indent=2))


if __name__ == '__main__':
    main()
