"""Bounded objectives for fresh-hover, feedforward-zero yaw PID trials.

This scores recorded trials; it never launches Gazebo or changes controller
settings. Targets remain the requested native body-FRD yaw rate, including DC
tracking errors. Existing comparison measurements are reused unchanged.
"""
from __future__ import annotations

import math
import statistics
from numbers import Real

from compare_metrics import _at, _series, _window, coupling_metrics, yaw_response

SCORING_VERSION = 1
FAILURE_PENALTY = 1000.0
CENSORED_SETTLING_RATIO = 1.25
DIRECTION_ASYMMETRY_WEIGHT = .25
DEFAULT_WEIGHTS = {
    'pulse_iae': .35,
    'pulse_itae': .15,
    'release_iae': .25,
    'release_itae': .10,
    'settling': .05,
    'overshoot_squared': 2.0,
    'opposite_return_squared': 1.0,
    'tail_tracking': .05,
}
DEFAULT_LIMITS = {
    'baseline_s': .3,
    'maximum_state_sample_gap_s': .05,
    'maximum_rc_send_gap_s': .1,
    'maximum_stage_duration_error_s': .03,
    # Frozen common armed analog endpoints: 1000 + motorIdle(500)*.1 = 1050.
    'motor_lower_pwm_us': 1051.0,
    'motor_upper_pwm_us': 1999.0,
    'servo_lower_pwm_us': 1101.0,
    'servo_upper_pwm_us': 1699.0,
    'maximum_motor_saturation_fraction': .01,
    'maximum_servo_saturation_fraction': .01,
    'maximum_roll_pitch_deviation_deg': 5.0,
    'maximum_roll_pitch_rate_dps': 60.0,
    'maximum_yaw_peak_multiple': 2.0,
    'maximum_overshoot_ratio': .25,
    'maximum_opposite_return_ratio': .25,
    'oscillation_tail_s': .8,
    'oscillation_band_min_dps': 2.0,
    'oscillation_band_target_fraction': .05,
    'oscillation_reversals_limit': 4,
}


def _configuration(defaults, overrides):
    result = dict(defaults)
    if overrides:
        unknown = set(overrides)-set(result)
        if unknown:
            raise ValueError('Unknown scoring setting: '+', '.join(sorted(unknown)))
        result.update(overrides)
    if any(not math.isfinite(float(v)) or float(v) < 0 for v in result.values()):
        raise ValueError('Scoring settings must be finite and nonnegative')
    return result


def failed_trial(direction, reason):
    """Retain a failed/missing flight even when no valid trace was produced."""
    if direction not in ('cw', 'ccw', None):
        raise ValueError('Direction must be cw, ccw, or unknown (None)')
    return {'scoring_version': SCORING_VERSION, 'direction': direction,
            'feasible': False, 'score': FAILURE_PENALTY, 'objective': None,
            'violations': ['trial_failure'], 'failure': str(reason), 'metrics': None}


def _case_window(case):
    stages = {stage['name']: stage for stage in case['stages']}
    if len(stages) != len(case['stages']):
        raise ValueError('Duplicate stage names')
    command = stages.get('yaw_lead', stages['pulse'])
    onset = float(command['actual_start_sim_time_s'])
    release = float(stages['recovery']['actual_start_sim_time_s'])
    end = float(case['end_sim_time_s'])
    target = float(command['requested_yaw_rate_dps'])
    if not all(math.isfinite(v) for v in (onset, release, end, target)) or not onset < release < end or target == 0:
        raise ValueError('Invalid nonzero yaw-rate trial window/target')
    return onset, release, end, target


def _absolute_error_integrals(series, target, origin):
    """Exact IAE/ITAE for the piecewise-linear recorded rate interpolation.

    Split an error-zero crossing before integrating abs(error), so opposite
    errors cannot cancel and a trapezoid cannot bridge across the V shape.
    ITAE time starts at this command/release boundary.
    """
    iae = itae = 0.0
    for (ta, va), (tb, vb) in zip(series, series[1:]):
        a, b = target-va, target-vb
        segments = [(ta, a), (tb, b)]
        if a*b < 0:
            zero = ta+(tb-ta)*abs(a)/(abs(a)+abs(b))
            segments.insert(1, (zero, 0.0))
        for (left, first), (right, last) in zip(segments, segments[1:]):
            h = right-left; first, last = abs(first), abs(last)
            area = h*(first+last)/2
            iae += area
            itae += (left-origin)*area+h*h*(first+2*last)/6
    return iae, itae


def _oscillation(series, target, band, tail_s):
    start = max(series[0][0], series[-1][0]-tail_s)
    window = _window(series, start, series[-1][0])
    previous = 0; reversals = []
    for stamp, value in window:
        error = value-target
        state = 1 if error > band else -1 if error < -band else 0
        if state:
            if previous and state != previous:
                reversals.append(stamp)
            previous = state
    return {'tail_start_s': start, 'tail_end_s': window[-1][0], 'band_dps': band,
            'band_reversals': len(reversals), 'reversal_times_s': reversals,
            'definition': 'Schmitt-band reversals in final tail; in-band jitter does not reset sign'}


def _health_violations(health, limits):
    violations = []
    for key in ('pass', 'all_status_samples_armed_without_fault', 'all_owned_processes_stopped'):
        if key in health and not health[key]:
            violations.append('health.'+key)
    if health.get('error'):
        violations.append('health.error')
    for key in ('actuator_valid_fraction', 'sampled_feedback_valid_fraction'):
        if key in health and (health[key] is None or not math.isfinite(float(health[key])) or float(health[key]) < 1-1e-12):
            violations.append('health.'+key)
    for key, limit in [('maximum_measurement_rc_send_gap_s', 'maximum_rc_send_gap_s'),
                       ('maximum_stage_duration_error_s', 'maximum_stage_duration_error_s')]:
        if key in health and (health[key] is None or not math.isfinite(float(health[key])) or float(health[key]) > limits[limit]+1e-9):
            violations.append('health.'+key)
    return violations


def score_trial(rows, case, health=None, limits=None, weights=None):
    """Score one CW/CCW trial using the existing recorded case schema.

    ``case`` supplies stages with actual simulation timestamps, requested yaw
    rate and end_sim_time_s, as written by check_comparison.py. ``health`` may
    additionally supply run-level pass/cleanup/error fields. Malformed or
    missing data produces a retained penalized trial rather than an exception;
    invalid scoring configuration raises ValueError before trial processing.
    """
    bounds = _configuration(DEFAULT_LIMITS, limits)
    coefficients = _configuration(DEFAULT_WEIGHTS, weights)
    if bounds['baseline_s'] <= 0 or bounds['oscillation_tail_s'] <= 0 or bounds['oscillation_reversals_limit'] < 1:
        raise ValueError('Baseline/tail durations and reversal limit must be positive')
    name = case.get('name') if isinstance(case, dict) else None
    direction = 'cw' if name == 'yaw_cw' else 'ccw' if name == 'yaw_ccw' else None
    try:
        onset, release, end, target = _case_window(case)
        direction = 'cw' if target > 0 else 'ccw'
        amplitude = abs(target); pulse_s, return_s = release-onset, end-release
        rates = _series(rows, 'gyro_frd_z', 180/math.pi)
        pulse = _window(rates, onset, release); recovery = _window(rates, release, end)
        observed = _window(rates, onset-bounds['baseline_s'], end)
        yaw = yaw_response(rows, onset, release, end, target, bounds['baseline_s'])
        coupling = coupling_metrics(rows, onset, end, bounds['baseline_s'])
        selected = [row for row in rows if onset <= float(row['sim_time_s']) <= end]
        if len(selected) < 2:
            raise ValueError('Insufficient actuator observations in trial window')
        required = ('pwm0_us', 'pwm1_us', 'pwm2_us', 'servo_pwm_us', 'actuator_valid')
        if any(not math.isfinite(float(row[key])) for row in selected for key in required):
            raise ValueError('Nonfinite actuator observation')
        motor_fraction = sum(any(float(row[f'pwm{i}_us']) <= bounds['motor_lower_pwm_us'] or float(row[f'pwm{i}_us']) >= bounds['motor_upper_pwm_us'] for i in range(3)) for row in selected)/len(selected)
        servo_fraction = sum(float(row['servo_pwm_us']) <= bounds['servo_lower_pwm_us'] or float(row['servo_pwm_us']) >= bounds['servo_upper_pwm_us'] for row in selected)/len(selected)
        actuator_valid = sum(float(row['actuator_valid']) == 1 for row in selected)/len(selected)
        roll = _window(_series(rows, 'gyro_frd_x', 180/math.pi), onset, end)
        pitch = _window(_series(rows, 'gyro_frd_y', 180/math.pi), onset, end)
        rp_rate_peak = max(math.hypot(p, q) for (_, p), (_, q) in zip(roll, pitch))
        absolute_yaw_peak = max(abs(v) for _, v in pulse+recovery[1:])
        max_gap = max(b[0]-a[0] for a, b in zip(observed, observed[1:]))
        pulse_iae, pulse_itae = _absolute_error_integrals(pulse, target, onset)
        return_iae, return_itae = _absolute_error_integrals(recovery, 0, release)
        overshoot = yaw['overshoot_percent_of_command']/100
        opposite = yaw['postrelease_opposite_rate_peak_dps']/amplitude
        command_settling = yaw['settling_time_s']
        return_settling = yaw['return_settling_time_s']
        normalized = {
            'pulse_iae': pulse_iae/(amplitude*pulse_s),
            'pulse_itae': pulse_itae/(amplitude*pulse_s*pulse_s/2),
            'release_iae': return_iae/(amplitude*return_s),
            'release_itae': return_itae/(amplitude*return_s*return_s/2),
            'settling': ((CENSORED_SETTLING_RATIO if command_settling is None else command_settling/pulse_s)
                         +(CENSORED_SETTLING_RATIO if return_settling is None else return_settling/return_s))/2,
            'overshoot_squared': overshoot*overshoot,
            'opposite_return_squared': opposite*opposite,
            'tail_tracking': abs(yaw['last_200ms_tracking_error_dps'])/amplitude,
        }
        terms = {key: coefficients[key]*value for key, value in normalized.items()}
        objective = sum(terms.values())
        if not math.isfinite(objective):
            raise ValueError('Nonfinite objective')
        band = max(bounds['oscillation_band_min_dps'], amplitude*bounds['oscillation_band_target_fraction'])
        oscillation = {'pulse': _oscillation(pulse, target, band, bounds['oscillation_tail_s']),
                       'release': _oscillation(recovery, 0, band, bounds['oscillation_tail_s'])}
        trial_health = {**case, **(health or {})}
        violations = _health_violations(trial_health, bounds)
        comparisons = [
            ('motor_saturation', motor_fraction, bounds['maximum_motor_saturation_fraction']),
            ('servo_saturation', servo_fraction, bounds['maximum_servo_saturation_fraction']),
            ('roll_pitch_deviation', coupling['roll_pitch_peak_norm_deviation_deg'], bounds['maximum_roll_pitch_deviation_deg']),
            ('roll_pitch_rate', rp_rate_peak, bounds['maximum_roll_pitch_rate_dps']),
            ('yaw_peak', absolute_yaw_peak, amplitude*bounds['maximum_yaw_peak_multiple']),
            ('overshoot', overshoot, bounds['maximum_overshoot_ratio']),
            ('opposite_return', opposite, bounds['maximum_opposite_return_ratio']),
            ('state_sample_gap', max_gap, bounds['maximum_state_sample_gap_s']),
        ]
        violations += [name for name, value, limit in comparisons if value > limit+1e-9]
        if actuator_valid < 1-1e-12:
            violations.append('invalid_actuator_input')
        if any(not 1000 <= float(row[f'pwm{i}_us']) <= 2000 for row in selected for i in range(3)):
            violations.append('invalid_motor_pwm')
        violations += ['oscillation.'+name for name, result in oscillation.items()
                       if result['band_reversals'] >= bounds['oscillation_reversals_limit']]
        # Recorded durations are independently checked here; an orchestration
        # overrun cannot improve a candidate merely by extending its plateau.
        stages = case['stages']
        stage_errors = []
        for index, stage in enumerate(stages):
            stamp = float(stage['actual_start_sim_time_s']); duration = float(stage['duration_s'])
            stage_end = float(stages[index+1]['actual_start_sim_time_s']) if index+1 < len(stages) else end
            if not all(math.isfinite(v) for v in (stamp, duration, stage_end)) or duration <= 0 or stage_end <= stamp:
                raise ValueError('Nonfinite/nonpositive stage duration or ordering')
            stage_errors.append(abs(stage_end-stamp-duration))
        if stage_errors and max(stage_errors) > bounds['maximum_stage_duration_error_s']+1e-9:
            violations.append('stage_duration_error')
        feasible = not violations
        return {'scoring_version': SCORING_VERSION, 'direction': direction, 'case': case.get('name'),
                'feasible': feasible, 'objective': objective,
                'score': objective+(0 if feasible else FAILURE_PENALTY),
                'violations': sorted(set(violations)), 'weights': coefficients, 'limits': bounds,
                'objective_terms': terms, 'normalized': normalized,
                'metrics': {'yaw': yaw, 'coupling': coupling, 'oscillation': oscillation,
                            'pulse_iae_deg': pulse_iae, 'pulse_itae_deg_s': pulse_itae,
                            'release_iae_deg': return_iae, 'release_itae_deg_s': return_itae,
                            'overshoot_ratio': overshoot, 'opposite_return_ratio': opposite,
                            'motor_saturation_fraction': motor_fraction, 'servo_saturation_fraction': servo_fraction,
                            'actuator_valid_fraction': actuator_valid,
                            'roll_pitch_rate_peak_frd_dps': rp_rate_peak,
                            'absolute_yaw_rate_peak_dps': absolute_yaw_peak,
                            'maximum_state_sample_gap_s': max_gap,
                            'maximum_recorded_stage_duration_error_s': max(stage_errors, default=None)},
                'settling_censored': {'command': command_settling is None, 'release': return_settling is None},
                'allocator_flags_role': 'diagnostic only; common physical actuator bounds constrain both variants'}
    except (KeyError, ValueError, TypeError, IndexError, ZeroDivisionError, OverflowError) as error:
        return failed_trial(direction, f'{type(error).__name__}: {error}')


def score_candidate(trials, expected_repetitions=1):
    """Balance CW/CCW and retain every repeat, failure and missing counterpart.

    ``expected_repetitions`` is a minimum. Additional repeats are all included;
    both directions must have matching counts. Missing flights are explicitly
    inserted as failures. Any infeasible trial makes the candidate infeasible.
    """
    if not isinstance(expected_repetitions, int) or expected_repetitions < 1:
        raise ValueError('Expected repetitions must be a positive integer')
    retained = []
    for trial in trials:
        direction = trial.get('direction') if isinstance(trial,dict) else None
        direction = direction if direction in ('cw','ccw') else None
        value = trial.get('score') if isinstance(trial,dict) else None
        if (not isinstance(value,Real) or isinstance(value,bool) or
            not math.isfinite(float(value)) or value < 0 or
            not isinstance(trial.get('feasible'),bool)):
            invalid = failed_trial(direction,'Malformed/nonfinite/negative trial score or feasibility')
            invalid['invalid_score_repr'] = repr(value)
            retained.append(invalid)
        else:
            retained.append({**trial,'score':float(value) if trial['feasible'] else max(FAILURE_PENALTY,float(value))})
    recorded_count = len(retained)
    groups = {direction: [trial for trial in retained if trial.get('direction') == direction]
              for direction in ('cw', 'ccw')}
    unassigned = [trial for trial in retained if trial.get('direction') not in groups]
    required = max(expected_repetitions, *(len(values) for values in groups.values()))
    for direction, values in groups.items():
        while len(values) < required:
            missing = failed_trial(direction, 'Missing expected direction/repetition')
            missing['synthetic_missing_trial'] = True
            values.append(missing); retained.append(missing)
    directional = {}
    failures = len(unassigned)
    for direction, values in groups.items():
        scores = []
        for trial in values:
            value = trial.get('score')
            if not trial.get('feasible', False):
                failures += 1
            scores.append(float(value))
        directional[direction] = {'n': len(values), 'mean_score': statistics.mean(scores),
                                  'min_score': min(scores), 'max_score': max(scores)}
    means = [directional[direction]['mean_score'] for direction in ('cw', 'ccw')]
    mean = statistics.mean(means); gap = abs(means[0]-means[1])
    balanced = mean+DIRECTION_ASYMMETRY_WEIGHT*gap
    return {'scoring_version': SCORING_VERSION, 'feasible': failures == 0,
            'score': balanced+(FAILURE_PENALTY if failures else 0),
            'balanced_trial_score': balanced, 'direction_mean_score': mean,
            'direction_score_gap': gap, 'direction_asymmetry_weight': DIRECTION_ASYMMETRY_WEIGHT,
            'directions': directional, 'failed_or_missing_trials': failures,
            'recorded_trials': recorded_count, 'total_scored_trials': len(retained),
            'expected_minimum_repetitions': expected_repetitions, 'trials': retained,
            'boundary': 'All repeats/failures retained; objective tuning evidence is separate from held-out evaluation'}
