"""Focused response measurements for recorded, controlled Gazebo RC tests.

Rates use the plant's body-FRD gyro in rad/s; heading is world-CCW degrees.
Timing is relative to the recorded command-send boundary, not first motion.
Unreached crossings and finite-window non-settling are represented by None.
"""
from __future__ import annotations

import math
from bisect import bisect_left


def actual_yaw_rate(pwm, center, max, expo, deadband=0, reversed=False, limit=1998):
    """Unsmoothed native ACTUAL yaw setpoint, in body-FRD degrees/second.

    ``center``/``max`` are degrees/second, expo is 0..1, and PWM midrc is 1500.
    High RC requests negative FRD yaw with native yaw_control_reversed=OFF.
    Read actual profile/deadband settings; this does not model RC smoothing.
    """
    values = (pwm, center, max, expo, deadband, limit)
    if any(not math.isfinite(float(v)) for v in values):
        raise ValueError('Nonfinite ACTUAL rate setting')
    if not (1000 <= pwm <= 2000 and center >= 0 and max >= 0 and
            0 <= expo <= 1 and 0 <= deadband < 500 and limit >= 0):
        raise ValueError('Invalid ACTUAL rate setting')
    delta = pwm - 1500
    if not reversed:
        delta = -delta
    magnitude = abs(delta) - deadband
    if magnitude <= 0:
        return 0.0
    x = math.copysign(magnitude / (500 - deadband), delta)
    span = max - center if max > center else 0.0
    rate = math.copysign(center * abs(x) + span * (
        expo * abs(x)**6 + (1 - expo) * abs(x)**2), x)
    return math.copysign(min(abs(rate), limit), rate)


def _series(rows, field, scale=1):
    result = [(float(r['sim_time_s']), float(r[field]) * scale) for r in rows]
    if len(result) < 2 or any(not math.isfinite(t) or not math.isfinite(v) for t, v in result):
        raise ValueError(f'Insufficient or nonfinite {field} data')
    if any(b[0] <= a[0] for a, b in zip(result, result[1:])):
        raise ValueError('Plant timestamps must increase strictly')
    return result


def _at(series, stamp):
    if not series[0][0] <= stamp <= series[-1][0]:
        raise ValueError('Measurement window extends beyond recorded state')
    i = bisect_left([t for t, _ in series], stamp)
    if i == 0 or series[i][0] == stamp:
        return series[i][1]
    (ta, a), (tb, b) = series[i-1:i+1]
    return a + (b-a) * (stamp-ta) / (tb-ta)


def _window(series, start, end):
    if not math.isfinite(start) or not math.isfinite(end) or start >= end:
        raise ValueError('Measurement window must have finite positive duration')
    return [(start, _at(series, start))] + [(t, v) for t, v in series if start < t < end] + [(end, _at(series, end))]


def _mean(series):
    return sum((tb-ta) * (a+b) / 2 for (ta, a), (tb, b) in zip(series, series[1:])) / (series[-1][0] - series[0][0])


def _rms(series, offset=0):
    return math.sqrt(_mean([(t, (v-offset)**2) for t, v in series]))


def _cross(series, threshold, increasing=True):
    test = (lambda v: v >= threshold) if increasing else (lambda v: v <= threshold)
    if test(series[0][1]):
        return series[0][0]
    for (ta, a), (tb, b) in zip(series, series[1:]):
        if not test(a) and test(b):
            return ta + (threshold-a) * (tb-ta) / (b-a)
    return None


def _settle(series, target, band, minimum_remaining=.25):
    # A final point inside the band is not evidence of settling. Require an
    # observed suffix of at least 250 ms, with no state-sampling gaps >50 ms.
    last_outside = -1
    for i, (_, v) in enumerate(series):
        if abs(v-target) > band:
            last_outside = i
    i = last_outside + 1
    if i == len(series) or series[-1][0] - series[i][0] < minimum_remaining - 1e-9:
        return None
    if any(b[0]-a[0] > .05 + 1e-9 for a, b in zip(series[i:], series[i+1:])):
        return None
    return series[i][0] - series[0][0]


def _unwrap(series):
    result = [series[0]]
    for (t, raw), (_, previous_raw) in zip(series[1:], series):
        delta = (raw-previous_raw+180) % 360 - 180
        result.append((t, result[-1][1]+delta))
    return result


def yaw_response(rows, onset_s, release_s, recovery_end_s, target_rate_dps, baseline_s=.3):
    """Measure a rate pulse; yaw stick does not command a heading step.

    Rise thresholds go from the measured pre-step baseline to the requested
    rate, not each variant's observed plateau. Overshoot/tracking remain
    relative to the actual command, so DC tracking errors are not hidden.
    """
    if not math.isfinite(target_rate_dps) or target_rate_dps == 0 or baseline_s <= 0:
        raise ValueError('A finite nonzero yaw target and positive baseline are required')
    rates = _series(rows, 'gyro_frd_z', 180/math.pi)
    baseline = _window(rates, onset_s-baseline_s, onset_s)
    pulse = _window(rates, onset_s, release_s)
    recovery = _window(rates, release_s, recovery_end_s)
    base = _mean(baseline)
    step = target_rate_dps-base
    direction = math.copysign(1, target_rate_dps)
    directed = [(t, direction*v) for t, v in pulse]
    peak = max(v for _, v in directed)
    rise_direction = math.copysign(1, step) if step else 1
    rise = [(t, rise_direction*(v-base)) for t, v in pulse]
    t10 = _cross(rise, .1*abs(step)) if abs(step) > 1e-9 else None
    t90 = _cross(rise, .9*abs(step)) if abs(step) > 1e-9 else None
    band = max(2, .05*abs(target_rate_dps))
    tail = _window(rates, max(onset_s, release_s-.2), release_s)
    release_rate = _at(rates, release_s)
    decay = [(t, direction*v) for t, v in recovery]
    amplitude = direction*release_rate
    d90 = _cross(decay, .9*amplitude, False) if amplitude > 1 else None
    d10 = _cross(decay, .1*amplitude, False) if amplitude > 1 else None
    heading = _window(_unwrap(_series(rows, 'yaw_deg')), release_s, recovery_end_s)
    changes = [v-heading[0][1] for _, v in heading]
    return {
        'target_yaw_rate_frd_dps': target_rate_dps,
        'baseline_yaw_rate_frd_dps': base,
        'baseline_yaw_rate_std_dps': _rms(baseline, base),
        'rise_threshold_reference': 'measured baseline to requested command',
        'time_to_10_percent_s': None if t10 is None else t10-onset_s,
        'time_to_90_percent_s': None if t90 is None else t90-onset_s,
        'rise_time_10_90_s': None if t10 is None or t90 is None else t90-t10,
        'peak_yaw_rate_frd_dps': direction*peak,
        'overshoot_percent_of_command': max(0, peak-abs(target_rate_dps))/abs(target_rate_dps)*100,
        'last_200ms_mean_yaw_rate_frd_dps': _mean(tail),
        'last_200ms_tracking_error_dps': _mean(tail)-target_rate_dps,
        'rate_tracking_rms_error_dps': _rms(pulse, target_rate_dps),
        'settling_band_dps': band,
        'minimum_settled_suffix_s': .25,
        'settling_time_s': _settle(pulse, target_rate_dps, band),
        'release_yaw_rate_frd_dps': release_rate,
        'return_decay_reference': 'observed release rate toward zero',
        'return_decay_90_10_s': None if d90 is None or d10 is None else d10-d90,
        'return_settling_time_s': _settle(recovery, 0, band),
        'postrelease_opposite_rate_peak_dps': max(0, max(-direction*v for _, v in recovery)),
        'postrelease_extra_heading_in_command_direction_deg': max(0, max(-direction*v for v in changes)),
        'postrelease_heading_change_world_ccw_deg': changes[-1],
        'postrelease_heading_max_abs_excursion_deg': max(abs(v) for v in changes),
        'pulse_duration_s': release_s-onset_s,
        'recovery_duration_s': recovery_end_s-release_s,
        'maximum_state_sample_gap_s': max(b[0]-a[0] for a, b in zip(pulse+recovery[1:], (pulse+recovery[1:])[1:])),
    }


def coupling_metrics(rows, onset_s, end_s, baseline_s=.3):
    """Actual roll/pitch deviations and yaw disturbance during a fixed-RC window."""
    if baseline_s <= 0:
        raise ValueError('Positive baseline duration required')
    result = {'duration_s':end_s-onset_s}
    deviations = {}
    for axis in ('roll', 'pitch'):
        series = _series(rows, axis+'_deg')
        base = _mean(_window(series, onset_s-baseline_s, onset_s))
        measured = _window(series, onset_s, end_s)
        deviations[axis] = [(t, v-base) for t, v in measured]
        result.update({f'baseline_{axis}_deg':base,
                       f'{axis}_peak_abs_angle_deg':max(abs(v) for _, v in measured),
                       f'{axis}_peak_abs_deviation_deg':max(abs(v-base) for _, v in measured),
                       f'{axis}_rms_deviation_deg':_rms(measured,base),
                       f'{axis}_end_deviation_deg':measured[-1][1]-base})
    r, p = deviations['roll'], deviations['pitch']
    norm2 = [(a[0], a[1]**2+b[1]**2) for a, b in zip(r, p)]
    result['roll_pitch_peak_norm_deviation_deg'] = math.sqrt(max(v for _, v in norm2))
    result['roll_pitch_rms_norm_deviation_deg'] = math.sqrt(_mean(norm2))
    for axis, field in [('roll','gyro_frd_x'),('pitch','gyro_frd_y'),('yaw','gyro_frd_z')]:
        series = _series(rows,field,180/math.pi)
        base = _mean(_window(series,onset_s-baseline_s,onset_s))
        measured = _window(series,onset_s,end_s)
        result.update({f'baseline_{axis}_rate_frd_dps':base,
                       f'{axis}_peak_abs_rate_frd_dps':max(abs(v) for _,v in measured),
                       f'{axis}_peak_abs_rate_deviation_dps':max(abs(v-base) for _,v in measured),
                       f'{axis}_rms_rate_deviation_dps':_rms(measured,base)})
    heading = _window(_unwrap(_series(rows,'yaw_deg')),onset_s,end_s)
    result['heading_change_world_ccw_deg'] = heading[-1][1]-heading[0][1]
    result['heading_change_relative_baseline_rate_deg'] = result['heading_change_world_ccw_deg'] + result['baseline_yaw_rate_frd_dps']*(end_s-onset_s)
    return result
