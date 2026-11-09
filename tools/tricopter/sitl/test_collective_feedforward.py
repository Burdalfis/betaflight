"""Setup-only adaptation to the declared motor law and tilted-tail force."""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from check_flight import DcCollectiveFeedforward, tail_force_roll_trim


class CollectiveFeedforwardTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).parent/'gazebo'
        self.bridge = ET.parse(self.root/'worlds/tricopter.sdf').find(
            "./world/model/plugin[@name='tricopter::TricopterBridge']")
        self.feed = DcCollectiveFeedforward(self.bridge, 1050, .05, 1000, 2000)

    def test_current_model_hover_matches_independent_retained_calculation(self):
        report = json.loads((self.root/'model/motor_performance.json').read_text())
        point = report['ideal_equal_thrust_hover']
        self.assertAlmostEqual(self.feed(3*point['thrust_n']),
                               1000+1000*point['requested_duty'], places=8)

    def test_entire_current_equilibrium_curve_inverts_to_actual_pwm(self):
        report = json.loads((self.root/'model/motor_performance.json').read_text())
        for point in report['target_duty_sweep']:
            duty = point['requested_duty']
            if duty < .05: continue  # Ordinary armed idle cannot request zero.
            with self.subTest(duty=duty):
                self.assertAlmostEqual(self.feed(3*point['thrust_n']), 1000+1000*duty, places=7)

    def test_different_native_rx_and_idle_do_not_assume_rc_equals_motor_pwm(self):
        feed = DcCollectiveFeedforward(self.bridge, 1100, .10, 1000, 2000)
        force = 6.0
        physical_pwm = self.feed(force)
        expected = 1100+900*((physical_pwm-1100)/900)
        self.assertAlmostEqual(feed(force), expected)
        feed = DcCollectiveFeedforward(self.bridge, 1070, .10, 1000, 2000)
        self.assertAlmostEqual(feed(force), 1070+930*((physical_pwm-1100)/900))
        feed = DcCollectiveFeedforward(self.bridge, 1070, .10, 1000, 1800)
        self.assertAlmostEqual(feed(force), 1070+930*((physical_pwm-1100)/700))

    def test_invalid_force_is_rejected(self):
        for value in (-1, math.inf, math.nan):
            with self.assertRaises(ValueError): self.feed(value)

    def test_tail_trim_cancels_lateral_force_with_correct_sign(self):
        state = {'joint_angle_deg': -12, 'thrust0_n': 2, 'thrust1_n': 1.5, 'thrust2_n': 1.5}
        trim = tail_force_roll_trim(state)
        self.assertLess(trim, 0)
        fy = 2*math.sin(math.radians(-12))
        fz = 3+2*math.cos(math.radians(-12))
        self.assertAlmostEqual(math.cos(trim)*fy-math.sin(trim)*fz, 0, places=12)
        state['joint_angle_deg'] = 0
        self.assertEqual(tail_force_roll_trim(state), 0)


if __name__ == '__main__': unittest.main()
