"""Held-command ANGLE references and CLI evidence, without running firmware."""
import math
import struct
import unittest

from check_flight import angle_channel
from envelope_targets import angle_targets, read_angle_earth_ref


def record():
    return {'common_msp_readback': {
        '111': '07004343430032000000000707000064ce07ce07ce070332',
        '94': '00000000000000000000000000000000001e00000050000000000001010005002f003100000000282e00002300c80f64000001410f5a07000001414605',
        '125': '0000053200'},
        'yaw_settings': {'center': 70, 'max': 670, 'expo': 0., 'deadband': 0,
                         'reversed': False, 'limit': 1998}}


class AngleTargets(unittest.TestCase):
    def test_hover_preserves_native_yaw_sign_and_rate(self):
        result = angle_targets(record(), 1500, 1500, 1368)
        self.assertEqual(result['requested_roll_deg'], 0)
        self.assertEqual(result['requested_pitch_deg'], 0)
        self.assertAlmostEqual(result['requested_yaw_rate_dps'], 60.2976)
        self.assertEqual(result['angle_yaw_attenuation'], 1)
        opposite = angle_targets(record(), 1500, 1500, 1632)
        self.assertAlmostEqual(opposite['yaw_pid_target_frd_dps'], -60.2976)

    def test_integer_pwm_for_25_degrees_has_cosine_yaw_attenuation(self):
        limits = {'angle_limit_deg': 30, 'center': [70, 70],
                  'max_rate': [670, 670], 'expo': [0., 0.]}
        pwm = angle_channel(25, limits, 1)
        result = angle_targets(record(), 1500, pwm, 1368)
        x = (pwm-1500)/500
        expected_angle = 30*(70*x+600*x*x)/670
        self.assertAlmostEqual(result['requested_pitch_deg'], expected_angle)
        self.assertLess(abs(result['requested_pitch_deg']-25), .1)
        self.assertAlmostEqual(result['yaw_pid_target_frd_dps'],
                               60.2976*math.cos(math.radians(expected_angle)))
        negative = angle_targets(record(), 1500, 3000-pwm, 1368)
        self.assertAlmostEqual(negative['requested_pitch_deg'], -expected_angle)
        self.assertAlmostEqual(negative['yaw_pid_target_frd_dps'], result['yaw_pid_target_frd_dps'])

    def test_earth_reference_zero_and_half_gain(self):
        for earth in (0, 50, 100):
            result = angle_targets(record(), 2000, 1500, 1368, earth_ref=earth)
            self.assertAlmostEqual(result['requested_roll_deg'], 30)
            self.assertAlmostEqual(result['yaw_pid_target_frd_dps'],
                                   60.2976*math.cos(math.radians(30*earth/100)))

    def test_rate_limit_precedes_angle_normalization_and_deadband(self):
        r = record(); values = bytearray.fromhex(r['common_msp_readback']['111'])
        struct.pack_into('<H', values, 18, 335)
        r['common_msp_readback']['111'] = values.hex()
        result = angle_targets(r, 1500, 2000, 1500)
        self.assertAlmostEqual(result['requested_pitch_deg'], 15)
        r['common_msp_readback']['125'] = bytes([5, 0, 5, 50, 0]).hex()
        self.assertEqual(angle_targets(r, 1504, 1500, 1500)['requested_roll_deg'], 0)
        self.assertEqual(angle_targets(r, 1505, 1500, 1500)['requested_roll_deg'], 0)
        self.assertGreater(angle_targets(r, 1506, 1500, 1500)['requested_roll_deg'], 0)

    def test_center_above_maximum_uses_actual_full_stick_normalization(self):
        r = record(); values = bytearray.fromhex(r['common_msp_readback']['111'])
        values[0] = 100; values[2] = 60
        r['common_msp_readback']['111'] = values.hex()
        self.assertEqual(angle_targets(r, 2000, 1500, 1500)['requested_roll_deg'], 30)

    def test_reversed_yaw_and_receiver_midpoint(self):
        r = record(); r['yaw_settings']['reversed'] = True
        self.assertAlmostEqual(angle_targets(r, 1500, 1500, 1368)['yaw_pid_target_frd_dps'], -60.2976)
        self.assertEqual(angle_targets(r, 1510, 1510, 1510, midrc=1510)['yaw_pid_target_frd_dps'], 0)

    def test_readback_drift_bad_types_and_wrong_rate_mode_fail(self):
        for key, value in [('center', 71), ('expo', .5), ('limit', 100), ('reversed', 0)]:
            r = record(); r['yaw_settings'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): angle_targets(r, 1500, 1500, 1368)
        for earth in (-1, 101, True, 50.):
            with self.subTest(earth=earth), self.assertRaises(ValueError):
                angle_targets(record(), 1500, 1500, 1368, earth_ref=earth)
        for pwm in (999, 2001, float('nan'), True):
            with self.subTest(pwm=pwm), self.assertRaises(ValueError): angle_targets(record(), pwm, 1500, 1500)
        r = record(); values = bytearray.fromhex(r['common_msp_readback']['111']); values[22] = 0
        r['common_msp_readback']['111'] = values.hex()
        with self.assertRaises(ValueError): angle_targets(r, 1500, 1500, 1500)

    def test_native_cli_get_must_be_unambiguous_and_last_setting(self):
        log = '# get angle_earth_ref\r\nangle_earth_ref = 100\r\nprofile 0\r\nAllowed range: 0 - 100\r\n'
        self.assertEqual(read_angle_earth_ref(log), 100)
        self.assertEqual(read_angle_earth_ref(log+'# get angle_earth_ref\nangle_earth_ref = 50\n'), 50)
        for value in ('# set angle_earth_ref = 100\nangle_earth_ref set to 100\n',
                      '# get angle_earth_ref\nangle_earth_ref set to 100\n',
                      log+'# set angle_earth_ref = 50\nangle_earth_ref set to 50\n',
                      '# get angle_earth_ref\nangle_earth_ref = 100\nangle_earth_ref = 0\n'):
            with self.subTest(log=value), self.assertRaises(AssertionError): read_angle_earth_ref(value)


if __name__ == '__main__': unittest.main()
