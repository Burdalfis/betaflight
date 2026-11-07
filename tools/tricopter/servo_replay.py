#!/usr/bin/env python3
"""Fit/replay the portable C count-domain servo predictor on pinned bench units."""
import argparse
import csv
import gzip
import json
import os
from pathlib import Path

import numpy as np
from servo_data import ROOT, PROTOCOL, decode_inputs, load_units, sha
from servo_native import NativeModel

SETTINGS = Path(__file__).with_name('servo_fit_settings.json')
PARAMETERS = ['map1100_counts', 'map1420_counts', 'map1700_counts', 'backlash_us', 'delay_us',
              'tau_increasing_pwm_s', 'tau_decreasing_pwm_s', 'rate_increasing_pwm_counts_s', 'rate_decreasing_pwm_counts_s']


def validate_frozen_settings(settings):
    if settings.get('map_knot_commands_us') != [1100, 1420, 1700] or settings.get('model_max_gap_us') != 5000:
        raise ValueError('Frozen map knots/max-gap metadata must match the fixed C bridge architecture')


def validate_replay_report(previous, frozen, native_sources):
    if previous.get('native', {}).get('sources') != native_sources:
        raise ValueError('Frozen parameter report uses different C sources')
    old = previous.get('frozen_protocol', {})
    for key in ('protocol_sha256', 'settings_sha256', 'protocol', 'settings'):
        if old.get(key) != frozen[key]:
            raise ValueError('Frozen parameter report uses a different split/protocol or fit settings; cannot relabel held-out evaluation')


def score_mask(unit):
    mask = unit.valid.copy()
    mask[0] = False  # The sole position measurement consumed by initialization is never scored.
    return mask


def initial_map(training):
    if any(u.split != 'training' for u in training):
        raise ValueError('Validation units cannot initialize fit parameters')
    values = []
    for pwm in (1100, 1420, 1700):
        groups = [u.position_counts[score_mask(u) & u.plateau & (u.command_us == pwm)] for u in training]
        values.append(float(np.median(np.concatenate([g for g in groups if len(g)]))))
    return np.asarray(values)


def specification(name, center, settings):
    relative = settings['map_bounds_counts_relative_training_median']
    low = list(center+relative[0])
    high = list(center+relative[1])
    start = list(center)
    prior, bound = settings['fixed_prior_start'], settings['bounds']
    if name == 'lag':
        low += [bound['tau_s'][0]]
        high += [bound['tau_s'][1]]
        start += [prior['tau_s']]
    elif name == 'full':
        for key, value in [('backlash_us', prior['backlash_us']), ('delay_us', prior['delay_us']),
                           ('tau_s', prior['tau_s']), ('tau_s', prior['tau_s']),
                           ('rate_counts_s', prior['rate_counts_s']), ('rate_counts_s', prior['rate_counts_s'])]:
            low.append(bound[key][0]); high.append(bound[key][1]); start.append(value)
    elif name != 'static':
        raise ValueError(name)
    return np.asarray(start), np.asarray(low), np.asarray(high)


def core_parameters(name, active):
    p = np.zeros(9)
    p[:3] = active[:3]
    if name == 'lag':
        p[5] = p[6] = active[3]
    elif name == 'full':
        p[:] = active
    return p


def nelder_mead(objective, start, low, high, budget):
    """Deterministic bounded optimizer only; dynamics are always evaluated by C."""
    if budget < len(start)+1:
        raise ValueError('Budget cannot initialize simplex')
    width = high-low
    x = np.clip((start-low)/width, 0, 1)
    evaluations, trace = 0, []
    best = float('inf')
    def evaluate(z):
        nonlocal evaluations, best
        z = np.clip(z, 0, 1)
        value = float(objective(low+z*width))
        evaluations += 1
        if value < best:
            best = value
            trace.append({'evaluation': evaluations, 'training_mse_counts2': value})
        return z, value
    simplex = [evaluate(x)]
    for j in range(len(x)):
        point = x.copy()
        point[j] += .07 if x[j] <= .90 else -.07
        simplex.append(evaluate(point))
    while evaluations < budget:
        simplex.sort(key=lambda a: a[1])
        centroid = np.mean([point for point, _ in simplex[:-1]], axis=0)
        worst = simplex[-1][0]
        reflected = evaluate(centroid+(centroid-worst))
        if reflected[1] < simplex[0][1] and evaluations < budget:
            expanded = evaluate(centroid+2*(reflected[0]-centroid))
            simplex[-1] = min((reflected, expanded), key=lambda a: a[1])
        elif reflected[1] < simplex[-2][1]:
            simplex[-1] = reflected
        elif evaluations < budget:
            contracted = evaluate(centroid+.5*((reflected[0] if reflected[1] < simplex[-1][1] else worst)-centroid))
            if contracted[1] < min(reflected[1], simplex[-1][1]):
                simplex[-1] = contracted
            else:
                first = simplex[0][0]
                for j in range(1, len(simplex)):
                    if evaluations >= budget:
                        break
                    simplex[j] = evaluate(first+.5*(simplex[j][0]-first))
        else:
            simplex[-1] = min((reflected, simplex[-1]), key=lambda a: a[1])
    simplex.sort(key=lambda a: a[1])
    return low+simplex[0][0]*width, {'evaluations': evaluations, 'best_training_mse_counts2': simplex[0][1], 'improvements': trace}


def fit_model(native, training, settings, name):
    validate_frozen_settings(settings)
    if not training or any(u.split != 'training' for u in training):
        raise ValueError('Fit objective accepts training units only')
    center = initial_map(training)
    start, low, high = specification(name, center, settings)
    def objective(active):
        p = core_parameters(name, active)
        losses = []
        for unit in training:
            try:
                predicted = native.replay(p, unit)
            except ValueError:
                return 1e15
            if not np.all(predicted['valid']):
                return 1e15
            mask = score_mask(unit)
            error = predicted['position'][mask]-unit.position_counts[mask]
            losses.append(float(np.mean(error**2)))
        return float(np.mean(losses))
    active, optimization = nelder_mead(objective, start, low, high, settings['evaluation_budget'][name])
    p = core_parameters(name, active)
    return {'name': name, 'parameters': dict(zip(PARAMETERS, [float(v) for v in p])),
            'active_start': start.tolist(), 'active_lower_bounds': low.tolist(), 'active_upper_bounds': high.tolist(),
            'training_units': [u.name for u in training], 'optimization': optimization,
            'angle_calibrated': False, 'position_units': 'corrected ADC counts', 'velocity_units': 'corrected ADC counts/s'}


def fit_models(native, units, settings, names=('static', 'lag', 'full')):
    training = [u for u in units if u.split == 'training']
    return [fit_model(native, training, settings, name) for name in names]


def errors(values):
    return ({'n': 0} if not len(values) else
            {'n': len(values), 'mae_counts': float(np.mean(abs(values))),
             'rmse_counts': float(np.sqrt(np.mean(values**2))),
             'p95_absolute_counts': float(np.percentile(abs(values), 95)),
             'mean_signed_counts': float(np.mean(values))})


def evaluate(native, models, units, output):
    results, predictions = [], {}
    columns = ['model', 'unit', 'split', 'time_us', 'command_us', 'measured_corrected_counts',
               'measurement_valid_fresh', 'pair_observation_age_100us', 'initialization_row',
               'predicted_counts', 'predicted_counts_s', 'model_valid', 'model_status']
    with gzip.open(output/'servo-replay.csv.gz', 'wt') as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        for model in models:
            p = np.asarray([model['parameters'][k] for k in PARAMETERS])
            rows = []
            for unit in units:
                predicted = native.replay(p, unit)
                predictions[(model['name'], unit.name)] = predicted
                if not np.all(predicted['valid']):
                    raise ValueError(f'{model["name"]}/{unit.name}: invalid model history; refusing to improve score by dropping invalid suffix')
                mask = score_mask(unit) & predicted['valid']
                residual = predicted['position']-unit.position_counts
                rows.append({'unit': unit.name, 'split': unit.split, 'metadata': unit.metadata,
                             'recorded_rows': len(mask), 'valid_measurement_rows': int(unit.valid.sum()),
                             'model_invalid_rows': int(np.count_nonzero(~predicted['valid'])),
                             'all': errors(residual[mask]), 'first150ms': errors(residual[mask & unit.transient]),
                             'final100ms': errors(residual[mask & unit.plateau])})
                for i in range(len(mask)):
                    writer.writerow([model['name'], unit.name, unit.split, int(unit.time_us[i]), unit.command_us[i],
                                     unit.position_counts[i], int(unit.valid[i]), unit.age_100us[i], int(i == 0),
                                     predicted['position'][i], predicted['velocity'][i], int(predicted['valid'][i]), int(predicted['status'][i])])
            aggregate = {}
            for split in ('training', 'validation'):
                rr = [r for r in rows if r['split'] == split]
                aggregate[split] = {'n_units': len(rr), 'equal_unit_rmse_counts': float(np.sqrt(np.mean([r['all']['rmse_counts']**2 for r in rr]))),
                                    'equal_unit_mae_counts': float(np.mean([r['all']['mae_counts'] for r in rr]))}
            results.append({'model': model['name'], 'aggregate': aggregate, 'units': rows})
    return results, predictions


def plot(units, predictions, output):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/tricopter-ground-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    heldout = [u for u in units if u.split == 'validation']
    fig, axes = plt.subplots(2, len(heldout), figsize=(16, 7), constrained_layout=True, squeeze=False)
    for j, unit in enumerate(heldout):
        t = unit.time_us/1e6
        measured = unit.position_counts.copy()
        measured[~unit.valid] = np.nan
        axes[0, j].plot(t, measured, color='black', alpha=.5, lw=.8, label='measured PB0−PC0')
        for name in ('static', 'lag', 'full'):
            predicted = predictions[(name, unit.name)]['position']
            axes[0, j].plot(t, predicted, lw=1, label=name)
            error = predicted-unit.position_counts
            error[~score_mask(unit)] = np.nan
            axes[1, j].plot(t, error, lw=.8, label=name)
        axes[0, j].set(title=unit.name+' (held out)', ylabel='Corrected counts')
        axes[1, j].set(xlabel='Time within complete unit (s)', ylabel='Prediction minus measurement (counts)')
        for ax in axes[:, j]:
            ax.grid(alpha=.2)
        axes[0, j].legend(fontsize=8)
    fig.suptitle('Open-loop replay of the same portable C servo model; one initialization per complete unit\nElectrical position proxy only; angle unavailable, unloaded fixture, previously explored recordings')
    fig.savefig(output/'servo-model-heldout.png', dpi=160)
    fig.savefig(output/'servo-model-heldout.pdf')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--prepare-inputs', action='store_true')
    parser.add_argument('--decoder', type=Path, default=ROOT/'.local-artifacts/tricopter/m1-bench-2026-11-06/tools/blackbox-tools/obj/blackbox_decode')
    parser.add_argument('--provenance', type=Path)
    parser.add_argument('--ground-csv', type=Path)
    parser.add_argument('--hysteresis-csv', type=Path)
    parser.add_argument('--replay-report', type=Path, help='Replay frozen parameters without refitting; requires matching C source hashes')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    protocol = json.loads(PROTOCOL.read_text())
    settings = json.loads(SETTINGS.read_text())
    validate_frozen_settings(settings)
    # Freeze exact settings/split and hashes in the output before any optimization.
    frozen = {'protocol': protocol, 'settings': settings, 'protocol_sha256': sha(PROTOCOL), 'settings_sha256': sha(SETTINGS)}
    (output/'frozen-fit-protocol.json').write_text(json.dumps(frozen, indent=2)+'\n')
    provenance_path = decode_inputs(args.decoder, output/'inputs', protocol) if args.prepare_inputs else args.provenance
    if provenance_path is None:
        parser.error('Use --prepare-inputs or provide --provenance (CSV overrides must match it)')
    units, diagnostics, provenance = load_units(provenance_path, {'ground': args.ground_csv, 'hysteresis': args.hysteresis_csv})
    native = NativeModel(output/'build')
    print('Fitting only:', ', '.join(u.name for u in units if u.split == 'training'), flush=True)
    if args.replay_report:
        previous = json.loads(args.replay_report.read_text())
        validate_replay_report(previous, frozen, native.provenance['sources'])
        models = previous['models']
        if [model['name'] for model in models] != ['static', 'lag', 'full']:
            raise ValueError('Expected all three predefined baselines')
    else:
        models = []
        for name in ('static', 'lag', 'full'):
            fit = fit_model(native, [u for u in units if u.split == 'training'], settings, name)
            models.append(fit)
            print(name, 'training RMSE', np.sqrt(fit['optimization']['best_training_mse_counts2']), flush=True)
    evaluation, predictions = evaluate(native, models, units, output)
    report = {'scope': 'Offline command-driven count predictor; no angle calibration or flight outputs',
              'frozen_protocol': frozen, 'input_provenance': provenance, 'input_diagnostics': diagnostics,
              'native': native.provenance, 'python_sources': {p.name: sha(p) for p in Path(__file__).parent.glob('servo_*.py')},
              'models': models, 'evaluation': evaluation,
              'limitations': [protocol['limits'], protocol['holdout_caveat'],
                              'Fitted lag, play and slew are descriptive joint parameters, not separately identified physical constants.',
                              'No measurement correction after unit initialization; future live observer would need a separately validated update law.']}
    if args.replay_report:
        report['frozen_parameter_report'] = {'path': str(args.replay_report.resolve()), 'sha256': sha(args.replay_report)}
    (output/'servo-model-report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    plot(units, predictions, output)
    print(json.dumps({r['model']: r['aggregate'] for r in evaluation}, indent=2))


if __name__ == '__main__':
    main()
