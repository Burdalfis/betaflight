"""Offline approach/staircase analysis using only corrected debug[3] as position."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    a = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    return ({'n': 0} if not len(a) else
            {'n': len(a), 'median': float(np.median(a)), 'min': float(a.min()),
             'max': float(a.max()), 'q25': float(np.percentile(a, 25)), 'q75': float(np.percentile(a, 75))})


def load_csv(path):
    with Path(path).open() as f:
        rows = list(csv.DictReader(f, skipinitialspace=True))
    if not rows:
        raise ValueError('No decoded rows')
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def masks(data):
    f = data['debug[5]'].astype(np.int64)
    p, g, c = (data[f'debug[{i}]'] for i in (0, 2, 3))
    available = ((f & 7) == 7) & (p >= 0) & (p <= 4095) & (g >= 0) & (g <= 4095) & (c != -32768)
    identity = c == p-g
    valid = available & identity & ((f & 8) != 0) & (p > 0) & (p < 4095) & (g < 4095)
    return f, available, identity, valid


def mad_sigma(a):
    return float(1.4826*np.median(abs(a-np.median(a))))


def window(t, y, valid, low, high, nominal_dt):
    take = (t >= low) & (t < high)
    indices = np.flatnonzero(take)
    good = take & valid
    n = len(indices)
    result = {'start_s': float(low), 'end_s': float(high), 'rows': n,
              'valid_rows': int(good.sum()), 'invalid_rows': int(np.count_nonzero(take & ~valid)),
              'accepted': False, 'median_adc': None, 'robust_sigma_adc': None}
    if not n:
        result['rejection'] = 'Empty metric window'
        return result
    result['max_frame_gap_us'] = float(np.max(np.diff(t[indices]))*1e6) if n > 1 else None
    # Freshness is local to this window. No stale samples are filled or interpolated.
    result['accepted'] = bool(np.all(valid[indices]) and n >= max(10, .85*(high-low)/nominal_dt)
                              and (n < 2 or np.max(np.diff(t[indices])) <= 1.5*nominal_dt))
    if not result['accepted']:
        result['rejection'] = 'Needs all local rows valid/fresh, sufficient rows and no >1.5 nominal-frame gap'
        if np.any(good):
            result['diagnostic_valid_row_median_adc'] = float(np.median(y[good]))
        return result
    v = y[indices]
    result.update(median_adc=float(np.median(v)), robust_sigma_adc=mad_sigma(v),
                  min_adc=float(v.min()), max_adc=float(v.max()))
    third = max(1, n//3)
    result['late_minus_early_third_adc'] = float(np.median(v[-third:])-np.median(v[:third]))
    # Descriptive block medians reveal low-frequency variation; no IID sample CI.
    blocks = []
    for begin in np.arange(low, high, .025):
        z = (t >= begin) & (t < min(begin+.025, high)) & take
        if z.sum() >= 10:
            blocks.append(float(np.median(y[z])))
    result['block25ms_medians_adc'] = blocks
    result['block25ms_median_range_adc'] = float(max(blocks)-min(blocks)) if blocks else None
    result['tail_quiet'] = bool(abs(result['late_minus_early_third_adc']) <= max(3., 3*result['robust_sigma_adc']))
    return result


def sustained_event(t, condition, valid, nominal_dt, n=5):
    candidate = np.flatnonzero(np.convolve((condition & valid).astype(int), np.ones(n, dtype=int), 'valid') == n)
    for k in candidate:
        if np.max(np.diff(t[k:k+n])) > 1.5*nominal_dt:
            continue
        # A first-event claim requires that every earlier row in its search prefix is usable.
        if not np.all(valid[:k+n]) or np.any(np.diff(t[:k+n]) > 1.5*nominal_dt):
            return None, 'Invalid/stale row or missing-frame gap in first-event search prefix'
        return float(t[k]), None
    return None, 'No five-row sustained crossing in search window'


def approach_speed(t, y, valid, baseline, endpoint, nominal_dt, methods):
    out = {'position_source': 'debug[3]', 'baseline_window': baseline,
           'endpoint_window': endpoint, 'credible_response': False}
    if not baseline['accepted'] or not endpoint['accepted']:
        out['rejection'] = 'Baseline or endpoint local window rejected'
        return out
    b, e = baseline['median_adc'], endpoint['median_adc']
    delta = e-b
    sigma = baseline['robust_sigma_adc']
    span = abs(delta)
    out.update(signed_span_adc=delta, span_adc=span)
    if span < max(methods['response_span_min_counts'], methods['response_span_min_sigma']*sigma):
        out['rejection'] = 'Displacement too small relative to raw baseline noise for timing metrics'
        return out
    out['credible_response'] = True
    post = (t >= 0) & (t < methods['speed_search_ms']/1000)
    tt, yy, vv = t[post], y[post], valid[post]
    signed = np.sign(delta)*(yy-b)
    threshold = max(3*sigma, .02*span, 3.)
    out['onset_threshold_adc'] = threshold
    for name, condition in [('first_away', signed <= -threshold),
                            ('first_toward', signed >= threshold),
                            ('t10', signed >= .1*span), ('t90', signed >= .9*span)]:
        crossing, reason = sustained_event(tt, condition, vv, nominal_dt)
        out[name+'_ms'] = None if crossing is None else crossing*1000
        if reason:
            out[name+'_rejection'] = reason
    # First-any requires a five-row excursion of consistent polarity, not alternating signs.
    found = [(out[name+'_ms'], direction) for name, direction in
             (('first_toward', 'toward'), ('first_away', 'away')) if out[name+'_ms'] is not None]
    out['first_any_ms'] = min(found)[0] if found else None
    out['first_any_direction'] = min(found)[1] if found else None
    t10, t90 = out.get('t10_ms'), out.get('t90_ms')
    if t10 is not None and t90 is not None and t90 > t10:
        interval = (tt >= t10/1000) & (tt <= t90/1000 + 4*nominal_dt + 1e-9)
        if np.all(vv[interval]):
            out['movement10_90_ms'] = t90-t10
            out['mean10_90_slew_adc_per_s'] = .8*span/((t90-t10)/1000)
    return out


def align_segments(time, pwm, events):
    starts = np.r_[0, np.flatnonzero(np.diff(pwm) != 0)+1]
    observed = [int(pwm[k]) for k in starts]
    planned = [int(event['target_us']) for event in events]
    if any(a == b for a, b in zip(planned, planned[1:])):
        raise ValueError('Consecutive identical event targets have no independent PWM edge; clarify labels before analysis')
    if observed != planned:
        mismatch = next((i for i, pair in enumerate(zip(observed, planned)) if pair[0] != pair[1]), None)
        raise ValueError(f'PWM/event mismatch at {mismatch}; observed {len(observed)} plateaus vs {len(planned)} planned; observed={observed}; planned={planned}')
    return [(int(s), int(starts[i+1]) if i+1 < len(starts) else len(time), events[i]) for i, s in enumerate(starts)]


def curve_analysis(plateaus):
    rows = [p for p in plateaus if p['event'].get('phase') == 'staircase']
    cycles = sorted(set(p['event']['cycle'] for p in rows))
    pairs, fits, reversals = [], [], []
    for cycle in cycles:
        rr = [p for p in rows if p['event']['cycle'] == cycle]
        branches = {direction: {p['target_us']: p for p in rr if p['event']['direction'] == direction}
                    for direction in ('up', 'down')}
        common = sorted(set(branches['up']) & set(branches['down']))
        for command in common:
            # Avoid the reversal endpoints in directional branch comparisons.
            if command in (1400, 1440):
                continue
            up, down = (branches[d][command]['endpoint'] for d in ('up', 'down'))
            if up['accepted'] and down['accepted']:
                pairs.append({'cycle': cycle, 'command_us': command,
                              'up_adc': up['median_adc'], 'down_adc': down['median_adc'],
                              'up_minus_down_adc': up['median_adc']-down['median_adc'],
                              'midpoint_adc': .5*(up['median_adc']+down['median_adc'])})
        cp = [p for p in pairs if p['cycle'] == cycle]
        if len(cp) >= 5:
            x = np.asarray([p['command_us'] for p in cp], dtype=float)
            y = np.asarray([p['midpoint_adc'] for p in cp])
            slope, intercept = np.polyfit(x, y, 1)
            residual = y-(slope*x+intercept)
            r2 = 1-float(np.sum(residual**2)/np.sum((y-y.mean())**2)) if np.var(y) else None
            fit = {'cycle': cycle, 'n_common_commands': len(cp), 'slope_adc_per_us': float(slope),
                   'intercept_adc': float(intercept), 'r_squared': r2,
                   'median_up_minus_down_adc': float(np.median([p['up_minus_down_adc'] for p in cp]))}
            fit['median_absolute_branch_gap_adc'] = float(np.median([abs(p['up_minus_down_adc']) for p in cp]))
            if r2 is not None and r2 >= .7 and abs(slope) > .05:
                fit['median_signed_input_equivalent_gap_us'] = fit['median_up_minus_down_adc']/slope
                fit['median_absolute_input_equivalent_gap_us'] = fit['median_absolute_branch_gap_adc']/abs(slope)
            fits.append(fit)
    # Evaluate actual reversals with the last plateau before direction changes as reference.
    for i, current in enumerate(plateaus):
        ev = current['event']
        if ev.get('phase') != 'staircase' or i == 0:
            continue
        previous = plateaus[i-1]
        old_ev = previous['event']
        if old_ev.get('phase') != 'staircase' or old_ev.get('direction') == ev.get('direction'):
            continue
        direction = ev['direction']
        reference = previous['endpoint']
        reversal = {'cycle': ev['cycle'], 'new_direction': direction,
                    'reference_command_us': previous['target_us'], 'reference_plateau': previous['plateau'],
                    'reference_endpoint': reference, 'detection': None, 'steps': []}
        follow = []
        for p in plateaus[i:]:
            if p['event'].get('phase') != 'staircase' or p['event'].get('direction') != direction or p['event'].get('cycle') != ev['cycle']:
                break
            follow.append(p)
        if reference['accepted']:
            adc_direction = -1 if direction == 'up' else 1  # Measured corrected-vs-PWM polarity.
            for p in follow:
                endpoint = p['endpoint']
                step = {'command_us': p['target_us'], 'cumulative_command_change_us': abs(p['target_us']-previous['target_us']),
                        'endpoint_accepted': endpoint['accepted'], 'detected': False}
                if endpoint['accepted']:
                    threshold = max(3., 3*max(reference['robust_sigma_adc'], endpoint['robust_sigma_adc']))
                    displacement = adc_direction*(endpoint['median_adc']-reference['median_adc'])
                    step.update(toward_displacement_adc=displacement, threshold_adc=threshold,
                                detected=bool(displacement > threshold))
                reversal['steps'].append(step)
            ss = reversal['steps']
            for j in range(len(ss)-1):
                if ss[j]['detected'] and ss[j+1]['detected']:
                    prior = ss[j-1] if j else None
                    # Missing endpoints before detection make its lower bound unresolved.
                    uninterrupted = all(s['endpoint_accepted'] for s in ss[:j+2])
                    reversal['detection'] = {'first_of_two_detected_command_change_us': ss[j]['cumulative_command_change_us'],
                                             'confirmation_command_change_us': ss[j+1]['cumulative_command_change_us'],
                                             'lower_grid_neighbor_us': (prior['cumulative_command_change_us'] if prior else 0) if uninterrupted else None,
                                             'all_prior_endpoint_windows_accepted': uninterrupted}
                    break
        else:
            reversal['rejection'] = 'Reference endpoint local window rejected'
        reversals.append(reversal)
    return {'pairs': pairs, 'per_cycle_fits': fits, 'reversals': reversals,
            'interpretation': 'Direction-dependent corrected voltage and thresholded reversal response, not isolated mechanical gear lash. Input-equivalent values use a fitted local command/ADC slope.'}


def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        k = prefix+k
        if isinstance(v, dict):
            out.update(flatten(v, k+'.'))
        elif not isinstance(v, list):
            out[k] = v
    return out


def write_csv(path, rows):
    names = sorted(set(k for row in rows for k in row))
    with Path(path).open('w') as f:
        w = csv.DictWriter(f, fieldnames=names)
        w.writeheader()
        w.writerows(rows)


def synthetic_tests():
    t = np.arange(0, .25, .001)
    y = np.full(len(t), 1500.)
    valid = np.ones(len(t), bool)
    # A late unrelated stale frame must not poison the earlier baseline window.
    valid[220] = False
    assert window(t, y, valid, 0, .1, .001)['accepted']
    assert not window(t, y, valid, .15, .25, .001)['accepted']
    tt = np.r_[np.arange(0, .010, .001), np.arange(.012, .050, .001)]
    crossing, reason = sustained_event(tt, tt >= .020, np.ones(len(tt), bool), .001)
    assert crossing is None and 'gap' in reason
    # A known100count/100ms ramp has an80ms10-90 interval; late stale data is irrelevant.
    tt = np.arange(-.100, .400, .001)
    yy = 1500-100*np.clip((tt-.020)/.100, 0, 1)
    vv = np.ones(len(tt), bool)
    vv[-1] = False
    speed = approach_speed(tt, yy, vv, {'accepted': True, 'median_adc': 1500., 'robust_sigma_adc': 1.},
                           {'accepted': True, 'median_adc': 1400.}, .001,
                           {'response_span_min_counts': 10, 'response_span_min_sigma': 10, 'speed_search_ms': 300})
    assert abs(speed['movement10_90_ms']-80) < 1.1
    assert speed['first_any_direction'] == 'toward' and 20 < speed['first_any_ms'] < 30
    pp = []
    for direction in ('up', 'down'):
        for command in range(1402, 1440, 2):
            median = 1500-1.1*(command-1420)+(2.2 if direction == 'up' else -2.2)
            pp.append({'event': {'phase': 'staircase', 'cycle': 0, 'direction': direction},
                       'target_us': command, 'plateau': len(pp),
                       'endpoint': {'accepted': True, 'median_adc': median, 'robust_sigma_adc': 1.}})
    c = curve_analysis(pp)
    fit = c['per_cycle_fits'][0]
    assert abs(fit['slope_adc_per_us']+1.1) < 1e-9
    assert abs(fit['median_absolute_input_equivalent_gap_us']-4) < 1e-9
    return {'passed': True, 'tests': ['metric-local freshness windows', 'earlier missing-frame gap rejects first-event claim',
                                     'known80ms10-90 ramp and toward polarity, with irrelevant late stale row',
                                     'known -1.1count/us directional slope',
                                     'known4us input-equivalent hysteresis recovered without raw channels'], 'known_hysteresis_fit': fit}


def plot(plateaus, curves, output):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/tricopter-ground-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
    approaches = [p for p in plateaus if p['event'].get('phase') == 'approach-center']
    for source in (1100, 1400, 1440, 1700):
        rr = [p for p in approaches if int(p['event']['approach_from_us']) == source and p['endpoint']['accepted']]
        axes[0].scatter([source]*len(rr), [p['endpoint']['median_adc'] for p in rr], alpha=.8)
    axes[0].set(xlabel='Previous command before 1420 µs (µs)', ylabel='Late corrected endpoint (ADC counts)', title='1420 µs center by approach')
    for cycle in sorted(set(p['event']['cycle'] for p in plateaus if p['event'].get('phase') == 'staircase')):
        for direction, style in (('up', '-'), ('down', '--')):
            rr = [p for p in plateaus if p['event'].get('phase') == 'staircase' and p['event']['cycle'] == cycle and p['event']['direction'] == direction]
            rr.sort(key=lambda p: p['target_us'])
            axes[1].plot([p['target_us'] for p in rr], [p['endpoint']['median_adc'] if p['endpoint']['accepted'] else np.nan for p in rr], style, marker='.', label=f'Cycle{cycle+1} {direction}')
        pp = [p for p in curves['pairs'] if p['cycle'] == cycle]
        lookup = {p['command_us']: p['up_minus_down_adc'] for p in pp}
        grid = range(1402, 1440, 2)
        axes[2].plot(list(grid), [lookup.get(command, np.nan) for command in grid], marker='.', label=f'Cycle{cycle+1}')
    axes[1].set(xlabel='Command (µs)', ylabel='Late corrected endpoint (ADC counts)', title='2 µs staircase branches')
    axes[2].set(xlabel='Command (µs)', ylabel='Up minus down endpoint (ADC counts)', title='Within-cycle directional difference')
    for ax in axes:
        ax.grid(alpha=.25)
    axes[1].legend(fontsize=8)
    axes[2].legend(fontsize=8)
    fig.suptitle('Clamped unloaded servo: corrected PB0−PC0 is the only position proxy\nFresh local plateau windows; descriptive repeatability, no calibrated angle or isolated gear-lash claim')
    fig.savefig(output/'servo-hysteresis.png', dpi=180)
    fig.savefig(output/'servo-hysteresis.pdf')
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--csv', type=Path)
    p.add_argument('--events', type=Path, default=HERE/'servo-hysteresis-events.json')
    p.add_argument('--run-json', type=Path, default=HERE/'servo-hysteresis-run.json')
    p.add_argument('--bbl', type=Path)
    p.add_argument('--methods', type=Path, default=HERE/'hysteresis-methods.json')
    p.add_argument('--output-dir', type=Path, default=HERE)
    p.add_argument('--synthetic-only', action='store_true')
    args = p.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    tests = synthetic_tests()
    (output/'hysteresis-synthetic-validation.json').write_text(json.dumps(tests, indent=2)+'\n')
    if args.synthetic_only:
        print(json.dumps({'synthetic_validation_passed': True}))
        return
    if not args.csv:
        p.error('--csv required')
    methods = json.loads(args.methods.read_text())
    events = json.loads(args.events.read_text())
    if isinstance(events, dict):
        events = events['events']
    data = load_csv(args.csv)
    time = data['time (us)']/1e6
    if np.any(np.diff(time) <= 0):
        raise ValueError('Non-increasing frame times')
    nominal_dt = float(np.median(np.diff(time)))
    flags, available, identity, valid = masks(data)
    segments = align_segments(time, data['debug[1]'], events)
    # No raw input enters y or any endpoint, motion, slope, curve or deadband metric.
    y = data['debug[3]']
    plateaus = []
    for i, (start, end, event) in enumerate(segments):
        finish = time[end] if end < len(time) else time[-1]+nominal_dt
        duration = finish-time[start]
        phase = event.get('phase')
        width = methods['center_tail_ms']/1000 if phase == 'approach-center' else methods['staircase_tail_ms']/1000
        if phase == 'staircase' and duration-width < methods['staircase_settling_exclusion_ms']/1000:
            width = max(0, duration-methods['staircase_settling_exclusion_ms']/1000)
        tail = window(time, y, valid, finish-width, finish, nominal_dt)
        row = {'plateau': i, 'target_us': int(event['target_us']), 'event': event,
               'start_time_us': int(round(time[start]*1e6)), 'duration_ms': duration*1000,
               'endpoint': tail,
               'diagnostic_raw': {'pb0_tail': stats(data['debug[0]'][(time >= finish-width) & (time < finish)]),
                                  'pc0_tail': stats(data['debug[2]'][(time >= finish-width) & (time < finish)]),
                                  'ground_floor_rows': int(np.count_nonzero((time >= finish-width) & (time < finish) & (data['debug[2]'] == 0)))}}
        if phase == 'approach-center':
            before = window(time-time[start], y, valid, -.100, 0, nominal_dt)
            row['speed'] = approach_speed(time-time[start], y, valid, before, tail, nominal_dt, methods)
        plateaus.append(row)
    centers = [r for r in plateaus if r['event'].get('phase') == 'approach-center']
    center_summary = []
    for source in sorted(set(int(r['event']['approach_from_us']) for r in centers)):
        rows = [r for r in centers if int(r['event']['approach_from_us']) == source]
        good = [r for r in rows if r['endpoint']['accepted']]
        center_summary.append({'approach_from_us': source, 'trials': len(rows), 'accepted_endpoints': len(good),
                               'center_endpoint_adc': stats([r['endpoint']['median_adc'] for r in good]),
                               'tail_drift_adc': stats([r['endpoint']['late_minus_early_third_adc'] for r in good]),
                               'speed': {key: stats([r['speed'].get(key) for r in rows]) for key in
                                         ('first_any_ms', 'first_toward_ms', 'movement10_90_ms', 'mean10_90_slew_adc_per_s')}})
    center_pairs = []
    for repeat in sorted(set(r['event']['repetition'] for r in centers)):
        lookup = {int(r['event']['approach_from_us']): r for r in centers if r['event']['repetition'] == repeat}
        for low, high in ((1100, 1700), (1400, 1440)):
            if low in lookup and high in lookup:
                a, b = lookup[low]['endpoint'], lookup[high]['endpoint']
                center_pairs.append({'repetition': repeat, 'low_approach_us': low, 'high_approach_us': high,
                                     'both_accepted': a['accepted'] and b['accepted'],
                                     'high_minus_low_center_adc': b['median_adc']-a['median_adc'] if a['accepted'] and b['accepted'] else None})
    curves = curve_analysis(plateaus)
    quality = {'frame_count': len(time), 'duration_s': float(time[-1]-time[0]),
               'frame_gap_us': stats(np.diff(time)*1e6), 'usable_rows': int(valid.sum()),
               'usable_fraction': float(valid.mean()),
               'exact_subtraction_mismatch_count': int(np.count_nonzero(available & ~identity)),
               'status_masks': {str(int(f)): int(np.count_nonzero(flags == f)) for f in np.unique(flags)},
               'pair_observation_age_100us': stats(data['debug[4]'][data['debug[4]'] >= 0]),
               'ground_floor_rows': int(np.count_nonzero(available & (data['debug[2]'] == 0))),
               'ground_ceiling_rows': int(np.count_nonzero(available & (data['debug[2]'] == 4095))),
               'accepted_plateau_windows': sum(r['endpoint']['accepted'] for r in plateaus),
               'total_plateaus': len(plateaus)}
    inputs = [{'path': str(path.resolve()), 'sha256': sha(path)} for path in
              (args.csv, args.events, args.run_json, args.bbl) if path and path.exists()]
    result = {'position_source': 'debug[3] only', 'pipeline_sha256': sha(__file__), 'inputs': inputs,
              'methods': methods, 'quality': quality, 'center_summary': center_summary,
              'center_pairs': center_pairs, 'curves': curves, 'plateaus': plateaus,
              'synthetic_validation_passed': True}
    (output/'hysteresis-analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    write_csv(output/'hysteresis-plateaus.csv', [flatten(r) for r in plateaus])
    write_csv(output/'hysteresis-branch-pairs.csv', curves['pairs'])
    write_csv(output/'hysteresis-center-pairs.csv', center_pairs)
    plot(plateaus, curves, output)
    print(json.dumps({'quality': quality, 'center_summary': center_summary, 'center_pairs': center_pairs,
                      'per_cycle_fits': curves['per_cycle_fits'], 'reversals': curves['reversals']}, indent=2))


if __name__ == '__main__':
    main()
