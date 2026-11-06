"""Offline paired-ADC electrical-response analysis; never accesses hardware."""
import argparse
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OLD_ANALYSIS = HERE / 'response_metrics_v4.py'
SPEC = importlib.util.spec_from_file_location('historical_servo_analysis', OLD_ANALYSIS)
HISTORICAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HISTORICAL)
SIGNALS = {'pb0': 'debug[0]', 'pc0': 'debug[2]', 'corrected': 'debug[3]'}
TRANSITIONS = [(1420, 1100), (1100, 1420), (1420, 1700), (1700, 1420)]
METRICS = ['raw_first_any_onset_ms', 'raw_first_toward_onset_ms',
           'raw_first_wrongway_onset_ms', 'established_toward_onset_ms',
           'raw_wrongway_peak_adc', 'raw_wrongway_peak_percent_step',
           'raw_wrongway_sustained5_peak_adc', 'raw_wrongway_longest_run_ms',
           'movement10_90_ms', 't10_ms', 't50_ms', 't90_ms',
           'peak_derivative_adc_per_s', 'maximum_absolute_derivative_adc_per_s',
           'peak_derivative_window_sensitivity_percent',
           'sustained_filtered_overshoot_percent', 'settling2_ms', 'settling5_ms',
           'noise_adjusted_settling2_ms', 'noise_adjusted_settling5_ms']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    v = np.asarray([v for v in values if v is not None and np.isfinite(v)], dtype=float)
    if not len(v):
        return {'n': 0}
    return dict(n=len(v), median=float(np.median(v)),
                q25=float(np.percentile(v, 25)), q75=float(np.percentile(v, 75)),
                min=float(v.min()), max=float(v.max()))


def load_csv(path):
    with Path(path).open() as f:
        rows = list(csv.DictReader(f, skipinitialspace=True))
    if not rows:
        raise ValueError('No decoded rows')
    return {name: np.asarray([float(row[name]) for row in rows]) for name in rows[0]}


def validity(data):
    flags = data['debug[5]'].astype(np.int64)
    pb0, pc0, corrected = (data[SIGNALS[key]] for key in SIGNALS)
    available = ((flags & 7) == 7) & (pb0 >= 0) & (pb0 <= 4095) & (pc0 >= 0) & (pc0 <= 4095) & (corrected != -32768)
    identity = corrected == pb0 - pc0
    fresh = (flags & 8) != 0
    # PC0=0 is accepted: negative servo-ground voltages are unobservable.
    usable = available & identity & fresh & (pb0 > 0) & (pb0 < 4095) & (pc0 < 4095)
    return flags, available, identity, fresh, usable


def baseline(y, mask):
    v = y[mask]
    if len(v) < 20:
        return None, None
    b = float(np.median(v))
    return b, float(1.4826 * np.median(abs(v - b)))


def pearson(a, b):
    return None if len(a) < 3 or np.std(a) == 0 or np.std(b) == 0 else float(np.corrcoef(a, b)[0, 1])


def transient_metrics(t, signals, usable):
    pre = (t >= -.200) & (t < 0) & usable
    early = (t >= 0) & (t < .020) & usable
    out = {'window_ms': [0, 20], 'sample_count': int(early.sum()),
           'expected_sample_count': int(np.count_nonzero((t >= 0) & (t < .020)))}
    baselines = {}
    for name, y in signals.items():
        b, sigma = baseline(y, pre)
        baselines[name] = b
        out[name] = {'baseline_adc': b, 'baseline_robust_sigma_adc': sigma}
        if b is None or not np.any(early):
            continue
        dy = y[early] - b
        tt = t[early]
        k = int(np.argmax(dy))
        out[name].update(positive_peak_adc=float(max(0, dy[k])),
                         positive_peak_at_ms=float(tt[k] * 1000),
                         negative_peak_adc=float(max(0, -np.min(dy))),
                         rms_displacement_adc=float(np.sqrt(np.mean(dy ** 2))),
                         mean_displacement_adc=float(np.mean(dy)))
    if all(value is not None for value in baselines.values()) and np.any(early):
        p = signals['pb0'][early] - baselines['pb0']
        g = signals['pc0'][early] - baselines['pc0']
        c = signals['corrected'][early] - baselines['corrected']
        at_peak = int(np.argmax(p))
        out['raw_peak_sample'] = dict(at_ms=float(t[early][at_peak]*1000),
                                     pb0_delta_adc=float(p[at_peak]),
                                     pc0_delta_adc=float(g[at_peak]),
                                     corrected_delta_adc=float(c[at_peak]))
        out['pb0_pc0_pearson_correlation'] = pearson(p, g)
        raw_peak = out['pb0']['positive_peak_adc']
        threshold = max(3 * out['pb0']['baseline_robust_sigma_adc'], 3)
        out['raw_positive_peak_above_noise'] = bool(raw_peak > threshold)
        out['positive_peak_reduction_percent'] = (100 * (1 - out['corrected']['positive_peak_adc']/raw_peak)
                                                    if raw_peak > threshold else None)
        out['ground_floor_fraction'] = float(np.mean(signals['pc0'][early] == 0))
    return out


def analyze_trial(data, time, change, end, index, usable, dt):
    start = int(np.searchsorted(time, time[change] - .200))
    sel = slice(start, end)
    t = time[sel] - time[change]
    valid = usable[sel]
    pwm = data['debug[1]']
    old, new = int(pwm[change-1]), int(pwm[change])
    signals = {name: data[field][sel] for name, field in SIGNALS.items()}
    result = dict(trial=index+1, from_pwm_us=old, to_pwm_us=new,
                  command_time_us=int(round(time[change]*1e6)),
                  observed_hold_ms=float(t[-1]*1000), sample_count=len(t),
                  usable_fraction=float(np.mean(valid)), invalid_count=int(np.count_nonzero(~valid)),
                  ground_floor_fraction=float(np.mean(signals['pc0'][valid] == 0)) if np.any(valid) else None,
                  max_sample_gap_us=float(np.max(np.diff(t))*1e6),
                  early=transient_metrics(t, signals, valid), signals={})
    early = (t >= 0) & (t < .020)
    early_flags = data['debug[5]'][sel][early].astype(np.int64)
    result['early']['status_masks'] = {str(int(v)): int(np.count_nonzero(early_flags == v)) for v in np.unique(early_flags)}
    result['early']['pair_observation_age_encoded_100us'] = stats(data['debug[4]'][sel][early])
    if 'servo[5]' in data:
        result['native_servo_pwm_mismatch_count'] = int(np.count_nonzero(data['servo[5]'][sel] != pwm[sel]))
    continuous = bool(np.all(valid) and np.max(np.diff(t)) <= 1.5 * dt and np.count_nonzero(t < 0) >= 100 and t[-1] >= .400)
    result['response_metrics_window_usable'] = continuous
    if not continuous:
        result['response_metrics_rejection'] = 'Requires all pair rows valid/fresh, no >1.5 nominal-frame gaps, >=100 pre-step rows, and >=400ms hold.'
    for name, y in signals.items():
        if continuous:
            result['signals'][name] = HISTORICAL.trial_metrics(t, y)
        else:
            b, sigma = baseline(y, (t < 0) & valid)
            e, esigma = baseline(y, (t >= t[-1]-.300) & valid)
            result['signals'][name] = dict(baseline_adc=b, baseline_robust_sigma_adc=sigma,
                                          endpoint_adc=e, endpoint_robust_sigma_adc=esigma,
                                          signed_step_adc=None if b is None or e is None else e-b)
    return result, {'t': t, 'signals': signals, 'usable': valid, 'trial': index+1, 'from': old, 'to': new}


def flatten(value, prefix=''):
    out = {}
    for key, item in value.items():
        key = prefix+key
        if isinstance(item, dict):
            out.update(flatten(item, key+'.'))
        elif not isinstance(item, (list, tuple)):
            out[key] = item
    return out


def write_csv(path, rows):
    names = sorted(set(k for row in rows for k in row))
    with Path(path).open('w') as f:
        writer = csv.DictWriter(f, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


def plot(traces, trials, output):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/tricopter-ground-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 4, figsize=(17, 8), constrained_layout=True)
    colors = {'pb0': '#2878b5', 'pc0': '#df8020', 'corrected': '#2b9a66'}
    for col, transition in enumerate(TRANSITIONS):
        matching = [(trace, result) for trace, result in zip(traces, trials) if (trace['from'], trace['to']) == transition]
        for trace, result in matching:
            t, ys, valid = trace['t'], trace['signals'], trace['usable']
            for name, y in ys.items():
                base = result['early'][name]['baseline_adc']
                if base is None:
                    continue
                ix = (t >= -.010) & (t <= .060)
                plotted = (y[ix]-base).copy()
                plotted[~valid[ix]] = np.nan
                axes[0, col].plot(t[ix]*1000, plotted, color=colors[name], alpha=.5, lw=1,
                                  label=name.upper() if result is matching[0][1] else None)
            metric = result['signals']['corrected']
            span = metric.get('signed_step_adc')
            if span and abs(span) > 10:
                ix = (t >= -.010) & (t <= .300)
                progress = 100*(ys['corrected'][ix]-metric['baseline_adc'])/span
                progress[~valid[ix]] = np.nan
                axes[1, col].plot(t[ix]*1000, progress, lw=1, alpha=.6)
        axes[0, col].set_title(f'{transition[0]} → {transition[1]} µs (n={len(matching)})')
        axes[0, col].set(xlabel='Time from logged PWM request (ms)', ylabel='Change from pre-step median (ADC counts)')
        axes[1, col].set(xlabel='Time from logged PWM request (ms)', ylabel='PB0−PC0 endpoint progress (%)')
        axes[1, col].axhline(100, color='gray', ls='--', lw=.7)
        for row in (0, 1):
            axes[row, col].axvline(0, color='gray', ls=':', lw=.7)
            axes[row, col].axhline(0, color='gray', lw=.7)
            axes[row, col].grid(alpha=.2)
        if matching:
            axes[0, col].legend(fontsize=8)
    fig.suptitle('Clamped, unloaded servo: paired wiper and PCB-ground electrical response\nSame completed ADC scan; channels sequential (~47 µs nominal skew). No shaft-angle calibration.')
    fig.savefig(output/'servo-ground-comparison.png', dpi=180)
    fig.savefig(output/'servo-ground-comparison.pdf')
    plt.close(fig)


def synthetic_validation():
    # Known common ground step must disappear under unit-gain subtraction.
    n = 100
    g = np.r_[np.zeros(20), np.full(10, 50), np.zeros(70)]
    mechanical = np.r_[np.full(40, 1000), np.linspace(1000, 700, 60)]
    p = mechanical+g
    assert np.array_equal(p-g, mechanical)
    d = {SIGNALS['pb0']: p, SIGNALS['pc0']: g, SIGNALS['corrected']: p-g,
         'debug[5]': np.full(n, 15.)}
    assert validity(d)[-1].all()
    d['debug[5]'][21] = 7
    assert not validity(d)[-1][21]  # Cached but not fresh.
    d['debug[5]'][21] = 15
    d['debug[0]'][0], d['debug[2]'][0], d['debug[3]'][0] = 100, 101, -1
    assert validity(d)[-1][0]  # Signed -1 is valid.
    d['debug[3]'][0] = -32768
    assert not validity(d)[-1][0]
    return {'passed': True, 'tests': ['unit-gain common-ground pulse rejection with known independent movement',
                                    'cached-but-stale rows rejected', 'signed -1 accepted', 'unavailable -32768 rejected'],
            'historical_metric_validation': HISTORICAL.synthetic_tests()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path)
    parser.add_argument('--bbl', type=Path)
    parser.add_argument('--run-json', type=Path, default=HERE/'servo-ground-run.json')
    parser.add_argument('--output-dir', type=Path, default=HERE)
    parser.add_argument('--expected-cycles', type=int, default=3)
    parser.add_argument('--synthetic-only', action='store_true')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    tests = synthetic_validation()
    (output/'ground-synthetic-validation.json').write_text(json.dumps(tests, indent=2)+'\n')
    if args.synthetic_only:
        print(json.dumps({'synthetic_validation_passed': True}))
        return
    if not args.csv:
        parser.error('--csv required unless --synthetic-only')
    data = load_csv(args.csv)
    time_name = 'time (us)' if 'time (us)' in data else 'time'
    time = data[time_name]/1e6
    gaps = np.diff(time)
    if np.any(gaps <= 0):
        raise ValueError('Non-increasing timestamps; no automatic sorting/deduplication')
    dt = float(np.median(gaps))
    flags, available, identity, fresh, usable = validity(data)
    pwm = data['debug[1]']
    changes = np.flatnonzero(abs(np.diff(pwm)) >= 100)+1
    changes = np.asarray([k for k in changes if (int(pwm[k-1]), int(pwm[k])) in TRANSITIONS])
    expected = args.expected_cycles*4
    if len(changes) != expected:
        raise ValueError(f'{len(changes)} recognized steps; expected exactly {expected}. Review log/schedule instead of silently choosing.')
    ordered = [(int(pwm[k-1]), int(pwm[k])) for k in changes]
    if ordered != TRANSITIONS*args.expected_cycles:
        raise ValueError(f'Unexpected transition order: {ordered}')
    trials, traces = [], []
    for i, change in enumerate(changes):
        end = int(changes[i+1]) if i+1 < len(changes) else int(np.searchsorted(time, time[change]+1.2))
        end = min(end, len(time))
        result, trace = analyze_trial(data, time, int(change), end, i, usable, dt)
        trials.append(result)
        traces.append(trace)
    groups = []
    for old, new in TRANSITIONS:
        rows = [r for r in trials if (r['from_pwm_us'], r['to_pwm_us']) == (old, new)]
        group = dict(from_pwm_us=old, to_pwm_us=new, n_trials=len(rows),
                     early={key: stats([r['early'].get(key) for r in rows]) for key in ('positive_peak_reduction_percent', 'pb0_pc0_pearson_correlation', 'ground_floor_fraction')},
                     signals={})
        group['early']['usable_sample_count'] = sum(r['early']['sample_count'] for r in rows)
        group['early']['expected_sample_count'] = sum(r['early']['expected_sample_count'] for r in rows)
        group['early']['status_masks'] = {key: sum(r['early']['status_masks'].get(key, 0) for r in rows)
                                           for key in sorted(set(key for r in rows for key in r['early']['status_masks']))}
        group['early']['maximum_observation_age_encoded_100us'] = max(r['early']['pair_observation_age_encoded_100us']['max'] for r in rows)
        for name in SIGNALS:
            group['signals'][name] = {key: stats([r['signals'][name].get(key) for r in rows]) for key in METRICS+['signed_step_adc', 'baseline_adc', 'endpoint_adc']}
            group['early'][name] = {key: stats([r['early'][name].get(key) for r in rows]) for key in ('positive_peak_adc', 'negative_peak_adc', 'mean_displacement_adc', 'baseline_robust_sigma_adc')}
        groups.append(group)
    age = data['debug[4]']
    quality = dict(frame_count=len(time), duration_s=float(time[-1]-time[0]),
                   sample_gap_us=stats(gaps*1e6), gap_over_1p5_nominal_count=int(np.count_nonzero(gaps > 1.5*dt)),
                   available_count=int(available.sum()), fresh_count=int(fresh.sum()), usable_count=int(usable.sum()),
                   usable_fraction=float(usable.mean()), exact_subtraction_mismatch_count=int(np.count_nonzero(available & ~identity)),
                   signed_corrected_negative_count=int(np.count_nonzero(available & (data['debug[3]'] < 0))),
                   signed_corrected_minus_one_count=int(np.count_nonzero(available & (data['debug[3]'] == -1))),
                   status_masks={str(int(v)): int(np.count_nonzero(flags == v)) for v in np.unique(flags)},
                   pair_observation_age_encoded_100us=stats(age[age >= 0]),
                   pair_observation_age_histogram={str(int(v)): int(np.count_nonzero(age == v)) for v in np.unique(age)},
                   wiper_rail_count=int(np.count_nonzero(available & ((data['debug[0]'] == 0) | (data['debug[0]'] == 4095)))),
                   ground_floor_count=int(np.count_nonzero(available & (data['debug[2]'] == 0))),
                   ground_ceiling_count=int(np.count_nonzero(available & (data['debug[2]'] == 4095))),
                   rpm_worst_observation_age_encoded_100us=stats(data['debug[6]'][data['debug[6]'] >= 0]),
                   invalid_packet_percentage_encoded_0p01percent=stats(data['debug[7]'][data['debug[7]'] >= 0]))
    inputs = [{'path': str(args.csv.resolve()), 'sha256': sha(args.csv)}]
    for path in (args.bbl, args.run_json):
        if path and path.exists():
            inputs.append({'path': str(path.resolve()), 'sha256': sha(path)})
    methods = json.loads((HERE/'ground-analysis-methods.json').read_text())
    result = dict(schema='TRI_GROUND', inputs=inputs, pipeline_sha256=sha(__file__),
                  historical_metric_pipeline_sha256=sha(OLD_ANALYSIS), methods=methods,
                  synthetic_validation_passed=True, quality=quality, groups=groups, trials=trials)
    (output/'servo-ground-analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    write_csv(output/'servo-ground-trials.csv', [flatten(r) for r in trials])
    early_rows = []
    for trace, result in zip(traces, trials):
        select = np.flatnonzero((trace['t'] >= -.020) & (trace['t'] <= .100))
        for i in select:
            row = dict(trial=trace['trial'], from_pwm_us=trace['from'], to_pwm_us=trace['to'],
                       t_ms=float(trace['t'][i]*1000), usable=bool(trace['usable'][i]))
            for name, values in trace['signals'].items():
                row[name+'_adc'] = float(values[i])
                b = result['early'][name]['baseline_adc']
                row[name+'_baseline_subtracted_adc'] = None if b is None else float(values[i]-b)
            early_rows.append(row)
    write_csv(output/'servo-ground-early.csv', early_rows)
    plot(traces, trials, output)
    print(json.dumps({'quality': quality, 'groups': groups}, indent=2))


if __name__ == '__main__':
    main()
