"""Synthetic response tests for command-referenced comparison metrics."""
import math
import unittest

from compare_metrics import actual_yaw_rate, coupling_metrics, yaw_response


def trace(rate, duration=3, roll=lambda t:0, pitch=lambda t:0, heading0=170):
    rows=[]; heading=heading0; previous_rate=rate(0)
    for index in range(round(duration/.01)+1):
        t=index*.01; value=rate(t)
        if index:
            heading -= .01*(value+previous_rate)/2
        previous_rate=value
        rows.append({'sim_time_s':t,'gyro_frd_z':math.radians(value),
                     'gyro_frd_x':0,'gyro_frd_y':0,'roll_deg':roll(t),
                     'pitch_deg':pitch(t),'yaw_deg':(heading+180)%360-180})
    return rows


class ActualYawRateTests(unittest.TestCase):
    def test_native_profile_yaw_sign_endpoints_and_quantized_command(self):
        self.assertEqual(actual_yaw_rate(1500,70,670,0),0)
        self.assertAlmostEqual(actual_yaw_rate(1575,70,670,0),-24)
        self.assertAlmostEqual(actual_yaw_rate(1632,70,670,0),-60.2976)
        self.assertAlmostEqual(actual_yaw_rate(1368,70,670,0),60.2976)
        self.assertEqual(actual_yaw_rate(2000,70,670,0),-670)
        self.assertEqual(actual_yaw_rate(1000,70,670,0),670)

    def test_deadband_reversal_rate_limit_and_expo_are_independent(self):
        self.assertEqual(actual_yaw_rate(1505,70,670,0,deadband=5),0)
        expected=70*(70/495)+600*(70/495)**2
        self.assertAlmostEqual(actual_yaw_rate(1575,70,670,0,deadband=5),-expected)
        self.assertAlmostEqual(actual_yaw_rate(1575,70,670,0,reversed=True),24)
        self.assertEqual(actual_yaw_rate(2000,70,670,0,limit=100),-100)
        self.assertAlmostEqual(actual_yaw_rate(1750,70,670,1),-(35+600*.5**6))
        with self.assertRaises(ValueError):actual_yaw_rate(1500,70,670,float('nan'))


class ResponseTests(unittest.TestCase):
    def first_order(self, sign=1, gain=1):
        def rate(t):
            if t<.5:return 0
            if t<=1.5:return sign*gain*100*(1-math.exp(-(t-.5)/.1))
            return sign*gain*100*(1-math.exp(-10))*math.exp(-(t-1.5)/.15)
        return trace(rate)

    def test_analytic_first_order_rise_decay_and_finite_settling(self):
        result=yaw_response(self.first_order(),.5,1.5,3,100)
        self.assertAlmostEqual(result['rise_time_10_90_s'],math.log(9)*.1,delta=.001)
        self.assertAlmostEqual(result['return_decay_90_10_s'],math.log(9)*.15,delta=.001)
        self.assertEqual(result['overshoot_percent_of_command'],0)
        self.assertAlmostEqual(result['settling_time_s'],.1*math.log(20),delta=.011)
        self.assertAlmostEqual(result['return_settling_time_s'],.15*math.log(20),delta=.011)
        self.assertGreater(result['postrelease_extra_heading_in_command_direction_deg'],14)

    def test_negative_rate_step_has_same_response_times(self):
        positive=yaw_response(self.first_order(),.5,1.5,3,100)
        negative=yaw_response(self.first_order(-1),.5,1.5,3,-100)
        for key in ['rise_time_10_90_s','return_decay_90_10_s','settling_time_s',
                    'return_settling_time_s','postrelease_extra_heading_in_command_direction_deg']:
            self.assertAlmostEqual(positive[key],negative[key])
        self.assertAlmostEqual(positive['peak_yaw_rate_frd_dps'],-negative['peak_yaw_rate_frd_dps'])

    def test_unreached_target_does_not_become_success_by_plateau_normalization(self):
        result=yaw_response(self.first_order(gain=.5),.5,1.5,3,100)
        self.assertIsNone(result['time_to_90_percent_s'])
        self.assertIsNone(result['rise_time_10_90_s'])
        self.assertIsNone(result['settling_time_s'])
        self.assertLess(result['last_200ms_tracking_error_dps'],-49)

    def test_command_overshoot_opposite_return_rate_and_short_suffix(self):
        def rate(t):
            if t<.5:return 0
            if t<.7:return 100*(t-.5)/.2
            if t<.8:return 100+20*(t-.7)/.1
            if t<.9:return 120-20*(t-.8)/.1
            if t<1.5:return 100
            if t<1.7:return 100-112*(t-1.5)/.2
            if t<1.9:return -12+12*(t-1.7)/.2
            return 0
        result=yaw_response(trace(rate),.5,1.5,3,100)
        self.assertAlmostEqual(result['overshoot_percent_of_command'],20)
        self.assertAlmostEqual(result['postrelease_opposite_rate_peak_dps'],12)
        short=yaw_response(self.first_order(),.5,1.5,1.65,100)
        self.assertIsNone(short['return_settling_time_s'])

    def test_heading_wrap_and_baseline_relative_coupling(self):
        rows=trace(lambda t:-40,roll=lambda t:10 if t<.5 else 12,
                   pitch=lambda t:-3 if t<.5 else -4,heading0=170)
        result=coupling_metrics(rows,.8,1.8,baseline_s=.2)
        # This baseline already follows the angle step: no artificial DC error.
        self.assertAlmostEqual(result['roll_peak_abs_deviation_deg'],0)
        self.assertAlmostEqual(result['heading_change_world_ccw_deg'],40)
        self.assertAlmostEqual(result['heading_change_relative_baseline_rate_deg'],0)
        stepped=coupling_metrics(rows,.5,1.5,baseline_s=.3)
        self.assertAlmostEqual(stepped['roll_peak_abs_deviation_deg'],2,delta=.04)
        self.assertAlmostEqual(stepped['pitch_peak_abs_deviation_deg'],1,delta=.02)
        self.assertGreater(stepped['roll_peak_abs_angle_deg'],11.9)

    def test_rejects_missing_coverage_nonfinite_and_nonmonotonic_state(self):
        rows=self.first_order()
        with self.assertRaises(ValueError):yaw_response(rows,.1,1.5,3,100)
        rows[10]['gyro_frd_z']=float('nan')
        with self.assertRaises(ValueError):yaw_response(rows,.5,1.5,3,100)
        rows=self.first_order();rows[10]['sim_time_s']=rows[9]['sim_time_s']
        with self.assertRaises(ValueError):coupling_metrics(rows,.5,1.5)


if __name__=='__main__':unittest.main()
