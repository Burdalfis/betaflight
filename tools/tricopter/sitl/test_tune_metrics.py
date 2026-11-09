"""Synthetic checks for bounded yaw tuning and retained failure penalties."""
import math
import unittest

from tune_metrics import (_absolute_error_integrals, failed_trial, score_candidate,
                          score_trial)


def case(sign=1, onset=.5, pulse=1, recovery=1.5):
    return {'name': 'yaw_cw' if sign > 0 else 'yaw_ccw', 'kind': 'yaw',
            'end_sim_time_s': onset+pulse+recovery,
            'all_status_samples_armed_without_fault': True,
            'stages': [
                {'name': 'baseline', 'actual_start_sim_time_s': 0, 'duration_s': onset,
                 'requested_yaw_rate_dps': 0},
                {'name': 'pulse', 'actual_start_sim_time_s': onset, 'duration_s': pulse,
                 'requested_yaw_rate_dps': sign*100},
                {'name': 'recovery', 'actual_start_sim_time_s': onset+pulse, 'duration_s': recovery,
                 'requested_yaw_rate_dps': 0}]}


def trace(rate, duration=3, roll=lambda t: 0, pitch=lambda t: 0):
    rows = []; heading = 170; previous = rate(0)
    for i in range(round(duration/.01)+1):
        stamp = i*.01; value = rate(stamp)
        if i:
            heading -= .01*(value+previous)/2
        previous = value
        rows.append({'sim_time_s': stamp, 'gyro_frd_z': math.radians(value),
                     'gyro_frd_x': 0, 'gyro_frd_y': 0,
                     'roll_deg': roll(stamp), 'pitch_deg': pitch(stamp),
                     'yaw_deg': (heading+180) % 360-180,
                     'pwm0_us': 1400, 'pwm1_us': 1400, 'pwm2_us': 1400,
                     'servo_pwm_us': 1509, 'actuator_valid': 1})
    return rows


def first_order(sign=1, gain=1, tau=.1, recovery_tau=.15):
    def rate(t):
        if t < .5:
            return 0
        if t <= 1.5:
            return sign*gain*100*(1-math.exp(-(t-.5)/tau))
        return sign*gain*100*(1-math.exp(-1/tau))*math.exp(-(t-1.5)/recovery_tau)
    return trace(rate)


class TrialScoreTests(unittest.TestCase):
    def test_error_zero_crossing_is_absolute_and_itae_origin_is_local(self):
        iae, itae = _absolute_error_integrals([(5, -1), (7, 1)], 0, 5)
        # |linear(-1,+1)|: two triangles, not a trapezoid or signed cancellation.
        self.assertAlmostEqual(iae, 1)
        self.assertAlmostEqual(itae, 1)
        iae, itae = _absolute_error_integrals([(5, 2), (7, 2)], 0, 5)
        self.assertAlmostEqual(iae, 4)
        self.assertAlmostEqual(itae, 4)

    def test_analytic_first_order_integrals_and_complete_objective(self):
        result = score_trial(first_order(), case())
        self.assertTrue(result['feasible'], result.get('violations'))
        metrics = result['metrics']
        expected_iae = 100*.1*(1-math.exp(-10))
        expected_itae = 100*.1**2*(1-11*math.exp(-10))
        self.assertAlmostEqual(metrics['pulse_iae_deg'], expected_iae, delta=.01)
        self.assertAlmostEqual(metrics['pulse_itae_deg_s'], expected_itae, delta=.002)
        self.assertAlmostEqual(result['normalized']['pulse_iae'], expected_iae/100, delta=.0001)
        self.assertAlmostEqual(result['objective'], sum(result['objective_terms'].values()))
        self.assertEqual(result['score'], result['objective'])
        self.assertFalse(result['settling_censored']['command'])
        self.assertFalse(result['settling_censored']['release'])

    def test_cw_ccw_scores_match_and_measured_bias_is_not_removed(self):
        cw = score_trial(first_order(), case())
        ccw = score_trial(first_order(-1), case(-1))
        self.assertAlmostEqual(cw['score'], ccw['score'])
        self.assertEqual((cw['direction'], ccw['direction']), ('cw', 'ccw'))
        offset = first_order()
        for row in offset:
            row['gyro_frd_z'] += math.radians(8)
        biased = score_trial(offset, case())
        self.assertAlmostEqual(biased['metrics']['yaw']['baseline_yaw_rate_frd_dps'], 8)
        self.assertGreater(biased['normalized']['release_iae'], cw['normalized']['release_iae'])
        self.assertGreater(biased['normalized']['tail_tracking'], .07)

    def test_half_gain_retains_dc_error_unreached_threshold_and_censoring(self):
        full = score_trial(first_order(), case())
        half = score_trial(first_order(gain=.5), case())
        self.assertTrue(half['feasible'])
        self.assertIsNone(half['metrics']['yaw']['rise_time_10_90_s'])
        self.assertIsNone(half['metrics']['yaw']['settling_time_s'])
        self.assertTrue(half['settling_censored']['command'])
        self.assertGreater(half['normalized']['tail_tracking'], .49)
        self.assertGreater(half['score'], full['score'])
        # Censored time is a score convention; the measured None is preserved.
        self.assertGreater(half['normalized']['settling'], .625)

    def test_overshoot_opposite_rate_and_endpoint_constraints_are_penalized(self):
        def rate(t):
            if t < .5: return 0
            if t < .6: return 140*(t-.5)/.1
            if t <= 1.5: return 140
            if t < 1.7: return -40
            return 0
        rows = trace(rate)
        for row in rows:
            if .8 < row['sim_time_s'] < 1:
                row['pwm0_us'] = 2000; row['servo_pwm_us'] = 1700
        result = score_trial(rows, case())
        self.assertFalse(result['feasible'])
        for name in ('overshoot', 'opposite_return', 'motor_saturation', 'servo_saturation'):
            self.assertIn(name, result['violations'])
        self.assertGreaterEqual(result['score'], 1000)
        self.assertAlmostEqual(result['objective_terms']['overshoot_squared'], 2*.4**2)
        self.assertAlmostEqual(result['objective_terms']['opposite_return_squared'], .4**2)

    def test_schmitt_tail_rejects_sustained_oscillation_but_ignores_small_jitter(self):
        def wave(amplitude):
            def rate(t):
                if t < .5: return 0
                if t <= 1.5: return 100+amplitude*math.sin(2*math.pi*6*(t-.5))
                return 0
            return trace(rate)
        noisy = score_trial(wave(1), case())
        self.assertNotIn('oscillation.pulse', noisy['violations'])
        self.assertEqual(noisy['metrics']['oscillation']['pulse']['band_reversals'], 0)
        ringing = score_trial(wave(10), case())
        self.assertIn('oscillation.pulse', ringing['violations'])
        self.assertGreaterEqual(ringing['metrics']['oscillation']['pulse']['band_reversals'], 4)

    def test_roll_pitch_state_rate_and_invalid_actuation_constraints(self):
        rows = first_order()
        for row in rows:
            if row['sim_time_s'] >= .7:
                row['roll_deg'] = 4; row['pitch_deg'] = 4
                row['gyro_frd_x'] = math.radians(50); row['gyro_frd_y'] = math.radians(50)
        result = score_trial(rows, case())
        self.assertIn('roll_pitch_deviation', result['violations'])
        self.assertIn('roll_pitch_rate', result['violations'])
        rows = first_order(); rows[100]['actuator_valid'] = 0
        self.assertIn('invalid_actuator_input', score_trial(rows, case())['violations'])
        for lower in (1000, 1050):
            rows = first_order()
            for row in rows:
                row['pwm0_us'] = lower
            result = score_trial(rows, case())
            self.assertIn('motor_saturation', result['violations'])
            self.assertEqual(result['metrics']['motor_saturation_fraction'], 1)

    def test_unhealthy_missing_nonfinite_gapped_and_overrun_trials_remain_failures(self):
        unhealthy = score_trial(first_order(), case(), health={'pass': False})
        self.assertFalse(unhealthy['feasible'])
        self.assertIn('health.pass', unhealthy['violations'])
        missing = score_trial([], case())
        self.assertFalse(missing['feasible']); self.assertEqual(missing['direction'], 'cw')
        self.assertGreaterEqual(missing['score'], 1000)
        rows = first_order(); rows[100]['gyro_frd_z'] = float('nan')
        self.assertFalse(score_trial(rows, case())['feasible'])
        rows = first_order(); rows = [r for r in rows if not .8 < r['sim_time_s'] < 1]
        self.assertIn('state_sample_gap', score_trial(rows, case())['violations'])
        definition = case(); definition['stages'][1]['duration_s'] = .8
        self.assertIn('stage_duration_error', score_trial(first_order(), definition)['violations'])
        for duration in (float('nan'), float('inf'), 0, -1):
            definition = case(); definition['stages'][1]['duration_s'] = duration
            invalid = score_trial(first_order(), definition)
            self.assertFalse(invalid['feasible'])
            self.assertGreaterEqual(invalid['score'], 1000)

    def test_bad_configuration_raises_instead_of_hiding_protocol_error(self):
        with self.assertRaises(ValueError): score_trial(first_order(), case(), weights={'unknown': 1})
        with self.assertRaises(ValueError): score_trial(first_order(), case(), limits={'baseline_s': 0})
        with self.assertRaises(ValueError): score_trial(first_order(), case(), weights={'settling': float('nan')})


class CandidateScoreTests(unittest.TestCase):
    @staticmethod
    def trial(direction, objective):
        return {'direction': direction, 'feasible': True, 'score': objective}

    def test_directional_balance_and_all_repeated_trials_are_averaged(self):
        trials = [self.trial('cw', 1), self.trial('cw', 3), self.trial('ccw', 2), self.trial('ccw', 6)]
        result = score_candidate(trials)
        self.assertTrue(result['feasible'])
        self.assertEqual(result['directions']['cw']['mean_score'], 2)
        self.assertEqual(result['directions']['ccw']['mean_score'], 4)
        self.assertEqual(result['score'], 3.5)
        self.assertEqual(result['total_scored_trials'], 4)

    def test_failed_and_missing_directions_are_never_discarded(self):
        failed = score_candidate([self.trial('cw', .1), failed_trial('ccw', 'crashed')])
        self.assertFalse(failed['feasible'])
        self.assertEqual(failed['failed_or_missing_trials'], 1)
        self.assertGreaterEqual(failed['score'], 1000)
        missing = score_candidate([self.trial('cw', .1)])
        self.assertFalse(missing['feasible'])
        self.assertTrue(missing['trials'][-1]['synthetic_missing_trial'])
        extra_cw = score_candidate([self.trial('cw', .1), self.trial('cw', .2), self.trial('ccw', .1)])
        self.assertFalse(extra_cw['feasible'])
        self.assertEqual(extra_cw['total_scored_trials'], 4)
        none = score_candidate([])
        self.assertFalse(none['feasible']); self.assertEqual(none['failed_or_missing_trials'], 2)

    def test_malformed_or_negative_scores_cannot_defeat_failure_penalties(self):
        for value in ('bad', None, float('nan'), float('inf'), -100000, True):
            result = score_candidate([self.trial('cw', value), self.trial('ccw', .1)])
            self.assertFalse(result['feasible'])
            self.assertGreaterEqual(result['score'], 1000)
            self.assertTrue(math.isfinite(result['score']))
        negative = self.trial('cw', -1e308)
        large = score_candidate([negative, self.trial('ccw', 1e308)])
        self.assertFalse(large['feasible']); self.assertTrue(math.isfinite(large['score']))
        failed = {'direction':'cw','score':0,'feasible':False}
        result = score_candidate([failed,self.trial('ccw',0)])
        self.assertFalse(result['feasible']);self.assertGreaterEqual(result['score'],1000)
        self.assertEqual(result['trials'][0]['score'],1000)


if __name__ == '__main__':
    unittest.main()
