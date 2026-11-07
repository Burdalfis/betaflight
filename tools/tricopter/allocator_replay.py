#!/usr/bin/env python3
"""Offline direct-effectiveness allocation and optional synthetic SI adapter checks."""
import argparse
import copy
import csv
import gzip
import json
import os
from pathlib import Path
import numpy as np

from allocator_cases import (PROTOCOL, load_protocol, physical_problem, virtual_problem, physical_requests,
                             virtual_requests, controls_from_effort, check_output)
from allocator_native import NativeAllocator, SOURCES
from rpm_data import ROOT, sha


def identification_demo(native, config):
    """Noiseless algebraic identification of a synthetic virtual-tail basis."""
    rng = np.random.default_rng(20261107)
    p = virtual_problem(config)
    def excite(n):
        effort = rng.uniform(.1, .8, (n, 3))*p['maximum']
        direction = rng.uniform(p['minimum_angle']*.8, p['maximum_angle']*.8, n)
        return np.asarray([controls_from_effort(e, a) for e, a in zip(effort, direction)])
    training = excite(64)
    response = training@p['matrix'].T
    identified = np.linalg.lstsq(training, response, rcond=None)[0].T
    heldout = excite(24)
    targets = heldout@p['matrix'].T
    learned = copy.deepcopy(config); learned['effectiveness_matrix'] = identified.tolist()
    out = native.virtual_run(learned, requests=targets)
    error = float(np.max(abs(out['achieved']-targets)))
    if not out['converged'].all() or error > 1e-5:
        raise ValueError('Noiseless synthetic map identification/heldout allocation failed')
    return {'synthetic_only': True, 'noise': 0, 'seed': 20261107, 'training_rows': 64, 'heldout_rows': 24,
            'training_design_rank': int(np.linalg.matrix_rank(training)),
            'maximum_matrix_error': float(np.max(abs(identified-p['matrix']))),
            'maximum_heldout_response_error': error,
            'scope': 'Algebraic demo in the declared global virtual-tail basis; no aircraft logs or local motor/servo Jacobian fit'}


def evaluate_family(native, protocol, direct, output):
    config = protocol['direct_effectiveness'] if direct else protocol
    problem = virtual_problem(config) if direct else physical_problem(protocol)
    cases = config['cases']
    requests = virtual_requests(config) if direct else physical_requests(protocol)
    if direct:
        yaw = np.linspace(-1.5, 1.5, 81)
        sweep = np.column_stack((np.full(len(yaw), 1.2), np.zeros((len(yaw), 2)), yaw))
        reaction = np.zeros(len(requests)+len(sweep))
        out = native.virtual_run(config, requests=np.vstack((requests, sweep)))
    else:
        spec = protocol['sweep']; yaw = np.linspace(*spec['yaw_request_nm'], spec['points'])
        sweep = np.column_stack((np.full(len(yaw), spec['vertical_force_n']), np.zeros((len(yaw), 2)), yaw))
        reaction = np.r_[[case['reaction_yaw_nm'] for case in cases], np.full(len(yaw), spec['reaction_yaw_nm'])]
        out = native.run(protocol, requests=np.vstack((requests, sweep)), reaction_yaw=reaction,
                         use_reaction=np.ones(len(reaction)), reaction_valid=np.ones(len(reaction)))
    target = np.vstack((requests, sweep)).astype(np.float32)
    controls = out['controls'] if direct else out['controls_n']
    effort = out['effort'] if direct else out['thrust_n']
    direction = out['tail_direction_rad'] if direct else out['tail_angle_rad']
    records = []
    for row in range(len(target)):
        offset = None if direct else [0, 0, 0, float(np.float32(reaction[row]))]
        cert = check_output(problem, target[row], controls[row], out['achieved'][row], offset)
        passes = bool(out['valid'][row] and out['converged'][row]
                      and cert['global_objective_gap_upper_bound'] <= config['solver']['independent_gap_tolerance'])
        records.append({'name': cases[row]['name'] if row < len(cases) else f'sweep:{row-len(cases)}',
                        'requested': target[row].tolist(), 'achieved': out['achieved'][row].tolist(),
                        'residual_achieved_minus_requested': out['residual'][row].tolist(),
                        'controls': controls[row].tolist(), 'effort': effort[row].tolist(), 'tail_direction_rad': float(direction[row]),
                        'status': int(out['status'][row]), 'constraints': int(out['constraints'][row]),
                        'iterations': int(out['iterations'][row]), 'valid': bool(out['valid'][row]),
                        'solver_converged': bool(out['converged'][row]), 'exact': bool(out['exact'][row]),
                        'cost': float(out['cost'][row]), 'published_gradient_mapping_norm': float(out['projected_gradient_norm'][row]),
                        'independent_certificate': cert, 'passes_declared_checks': passes})
    label = 'direct-effectiveness' if direct else 'physical-adapter'
    with gzip.open(output/(label+'-replay.csv.gz'), 'wt') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case', 'coordinate_mode', *[f'request{i}' for i in range(4)], *[f'achieved{i}' for i in range(4)],
                         *[f'residual{i}' for i in range(4)], *[f'control{i}' for i in range(4)],
                         'tail_direction_rad', 'cost', 'global_gap_upper_bound', 'valid', 'converged', 'exact', 'iterations', 'constraints'])
        for record in records:
            writer.writerow([record['name'], 'proxy effort/response' if direct else 'SI thrust/wrench', *record['requested'],
                             *record['achieved'], *record['residual_achieved_minus_requested'], *record['controls'],
                             record['tail_direction_rad'], record['cost'], record['independent_certificate']['global_objective_gap_upper_bound'],
                             int(record['valid']), int(record['solver_converged']), int(record['exact']), record['iterations'], record['constraints']])
    plot_sweep(label, yaw, out, len(cases), problem, direct, output)
    return {'mode': label, 'units': config['units'], 'cases': records[:len(cases)],
            'sweep': records[len(cases):], 'all_declared_checks_pass': all(record['passes_declared_checks'] for record in records),
            'maximum_global_objective_gap_upper_bound': max(record['independent_certificate']['global_objective_gap_upper_bound'] for record in records),
            'maximum_response_publication_error_by_axis': np.max([record['independent_certificate']['published_response_error_by_axis'] for record in records], axis=0).tolist()}


def plot_sweep(label, yaw, out, start, problem, direct, output):
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    effort = out['effort'][start:] if direct else out['thrust_n'][start:]
    direction = out['tail_direction_rad'][start:] if direct else out['tail_angle_rad'][start:]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), sharex=True, layout='constrained')
    axes[0, 0].plot(yaw, yaw, '--', color='.5', label='requested')
    axes[0, 0].plot(yaw, out['achieved'][start:, 3], label='achieved')
    axes[0, 0].set_ylabel('Yaw response proxy' if direct else 'Yaw moment (N m)')
    for motor, color in enumerate(('#3275af', '#c58220', '#57945a')):
        axes[0, 1].plot(yaw, effort[:, motor], color=color, label=f'motor {motor}')
        axes[0, 1].axhline(problem['maximum'][motor], color=color, linestyle=':', linewidth=.8)
    axes[0, 1].set_ylabel('Normalized effort proxy' if direct else 'Thrust (N)')
    axes[1, 0].plot(yaw, direction)
    for bound in (problem['minimum_angle'], problem['maximum_angle']):
        axes[1, 0].axhline(bound, linestyle=':', color='.4')
    axes[1, 0].set_ylabel('Mathematical tail effort direction (rad)' if direct else 'Synthetic tail angle (rad)')
    for axis, name in enumerate(('collective', 'roll', 'pitch', 'yaw')):
        axes[1, 1].plot(yaw, out['residual'][start:, axis]/problem['scales'][axis], label=name)
    axes[1, 1].set_ylabel('Residual / declared response scale')
    for axis in axes.flat:
        axis.grid(alpha=.2)
        if axis is not axes[1, 0]: axis.legend(fontsize=8)
    for axis in axes[-1]: axis.set_xlabel('Static requested yaw response proxy' if direct else 'Static requested yaw moment (N m)')
    fig.suptitle(label+': synthetic static request sweep; no time evolution, servo dynamics or aircraft calibration')
    fig.savefig(output/(label+'-sweep.png'), dpi=150, bbox_inches='tight'); plt.close(fig)


def held_angle_demo(native, protocol):
    demo = protocol['held_angle_demo']; case = next(case for case in protocol['cases'] if case['name'] == demo['case'])
    target = native.run(protocol, thrust=[case['thrust_n']], angles=[case['tail_angle_rad']],
                        reaction_yaw=[case['reaction_yaw_nm']], use_reaction=[1], reaction_valid=[1])
    held = native.run(protocol, thrust=[case['thrust_n']], angles=[demo['held_tail_angle_rad']],
                      reaction_yaw=[case['reaction_yaw_nm']], use_reaction=[1], reaction_valid=[1])
    return {'requested_synthetic_angle_rad': case['tail_angle_rad'], 'held_synthetic_angle_rad': demo['held_tail_angle_rad'],
            'desired_wrench': target['achieved'][0].tolist(), 'actual_fixed_angle_wrench': held['achieved'][0].tolist(),
            'held_minus_desired': (held['achieved'][0]-target['achieved'][0]).tolist(),
            'scope': 'Same thrusts, different independently supplied angle. No angle inferred from PWM/ADC; no servo response simulated.'}


def failure_demos(native, protocol):
    config = copy.deepcopy(protocol['direct_effectiveness']); config['solver']['max_iterations'] = 1
    out = native.virtual_run(config, requests=[[1.5, 1, -.3, .6]])
    if not out['valid'][0] or out['converged'][0] or not out['status'][0]&64:
        raise ValueError('Fixed-iteration nonconvergence was hidden')
    nonconverged = {'valid': bool(out['valid'][0]), 'converged': bool(out['converged'][0]), 'status': int(out['status'][0]), 'iterations': int(out['iterations'][0])}
    config['effectiveness_matrix'] = np.zeros((4, 4)).tolist()
    invalid = native.virtual_run(config, requests=[[1, 0, 0, 0]])
    if invalid['valid'][0]: raise ValueError('Singular map was not rejected')
    reaction = native.run(protocol, requests=[[12, 0, 0, .2]], reaction_yaw=[.03], use_reaction=[1], reaction_valid=[0])
    if reaction['valid'][0]: raise ValueError('Unknown requested reaction yaw was not rejected')
    return {'one_iteration': nonconverged, 'singular_map_status': int(invalid['status'][0]), 'unknown_reaction_status': int(reaction['status'][0])}


def markdown(summary):
    lines = ['# Offline tricopter allocation', '',
             'The primary direct-effectiveness interface needs a response matrix, actuator effort bounds and explicit response priorities. It requires no rotor positions, CG, mass, inertia or kT. Its outputs are effort/response proxies, without thrust, force, moment or RPM claims. The tail direction is a mathematical effort coordinate until separately calibrated.', '',
             'The matrix acts on [front-right effort, front-left effort, tail effort*cos(direction), tail effort*sin(direction)]. It is a global virtual-tail basis, not an interchangeable local [delta motors, delta servo] Jacobian. A future characterization process must establish that basis and its usable range. The included noiseless least-squares demonstration is entirely synthetic; no aircraft map is identified.', '',
             'An optional geometry adapter checks SI thrust and r×F equations using arbitrary synthetic rotor positions, nonzero CG offsets and kT. Known reaction yaw is exogenous and applied once. The separate forward API demonstrates that a requested tail angle and a held actual angle produce different wrenches at the same thrusts. Servo angle is never inferred from ADC counts or PWM.', '',
             'Both interfaces use the actual portable C kernel. Exact inversion handles feasible requests; a bounded convex projected-gradient solve handles saturation. Residual means achieved minus requested. Priorities are the frozen response scales in the protocol. Nonzero saturated residual does not mean solver nonconvergence.', '',
             'Independent validation first checks feasibility, then forms the convex objective-gap bound g·x−min(g·z), with an independently computed linear support point over front intervals and the tail sector. The report states the dimensionless objective-gap cutoff separately from the C gradient-mapping tolerance. Tiny float32 publication rounding is projected to a feasible float64 certificate point; correction and response error are recorded. This is a numerical certificate, not an interval-arithmetic rounding proof.', '',
             '| Configuration | Static cases + sweep points | Maximum objective-gap bound | All declared checks pass |',
             '|---|---:|---:|---|']
    for family in summary['families']:
        lines.append(f'| {family["mode"]} | {len(family["cases"])+len(family["sweep"])} | {family["maximum_global_objective_gap_upper_bound"]:.6g} | {family["all_declared_checks_pass"]} |')
    lines += ['', summary['protocol']['numerical_tolerance_provenance'], '',
              'Plots are synthetic static request sweeps, not elapsed time or servo tracking. Every geometry, map, coefficient and limit is a placeholder, uncalibrated and unavailable for flight. Fixed-iteration nonconvergence and invalid maps/reaction metadata are stored explicitly. No PID, mixer, scheduler, actuator, hardware or flash path is connected.', '',
              'Reproduce with `python3 tools/tricopter/allocator_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_allocator_replay.py -v`. Source/build/protocol provenance and per-case certificates are in `allocator-replay-summary.json`; output hashes are in `checksums.sha256`.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--output', required=True)
    args = parser.parse_args(); output = Path(args.output).resolve(); output.mkdir(parents=True, exist_ok=True)
    protocol_hash = sha(PROTOCOL); protocol = load_protocol(); native = NativeAllocator(output/'build')
    summary = {'protocol_sha256': protocol_hash, 'protocol': protocol, 'native_build': native.provenance,
               'families': [evaluate_family(native, protocol, direct, output) for direct in (True, False)],
               'synthetic_identification_demo': identification_demo(native, protocol['direct_effectiveness']),
               'held_angle_demo': held_angle_demo(native, protocol), 'failure_demos': failure_demos(native, protocol),
               'calibrated': False, 'for_flight': False}
    if sha(PROTOCOL) != protocol_hash or native.provenance['sources'] != {str(path.relative_to(ROOT)): sha(path) for path in SOURCES}:
        raise RuntimeError('Frozen protocol/core/header/bridge changed during replay')
    scripts = [path for path in Path(__file__).parent.glob('allocator_*') if path.is_file()]
    scripts += [Path(__file__).with_name('test_allocator_replay.py'), Path(__file__).with_name('rpm_data.py')]
    summary['script_sha256'] = {str(path.relative_to(ROOT)): sha(path) for path in scripts}
    (output/'allocator-replay-summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    (output/'native-build.json').write_text(json.dumps(native.provenance, indent=2)+'\n')
    (output/'README.md').write_text(markdown(summary))
    files = sorted(path for path in output.iterdir() if path.is_file() and path.name != 'checksums.sha256')
    (output/'checksums.sha256').write_text(''.join(f'{sha(path)}  {path.name}\n' for path in files))
    if not all(family['all_declared_checks_pass'] for family in summary['families']):
        raise ValueError('At least one case failed convergence/certificate checks; report preserves the failure')
    print(json.dumps({'output': str(output), 'all_declared_checks_pass': True,
                      'families': [(family['mode'], family['maximum_global_objective_gap_upper_bound']) for family in summary['families']]}, indent=2))


if __name__ == '__main__': main()
