"""Independent analytic invariants executed through the real C torque core."""

import copy
from pathlib import Path
import tempfile
import unittest

import numpy as np

from reaction_cases import analytic_oracle, build_cases, evaluate_case, load_protocol, validate_protocol
from reaction_native import NativeReaction


class ReactionReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='tricopter-reaction-native-')
        cls.native = NativeReaction(Path(cls.temporary.name)/'build')
        cls.protocol = load_protocol()
        cls.parameters = cls.protocol['parameters']

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def evaluate(self, rpm, acceleration=None, parameters=None, source='SYNTHETIC_PLACEHOLDER', **changes):
        rpm = np.asarray(rpm, np.float64).reshape(-1, 3)
        data = dict(mechanical_rpm=rpm, acceleration_rpm_s=np.zeros_like(rpm) if acceleration is None else np.asarray(acceleration).reshape(-1, 3),
                    age_us=np.zeros(rpm.shape, np.uint32), seen=np.ones(rpm.shape, np.uint8),
                    fresh=np.ones(rpm.shape, np.uint8), valid=np.ones(rpm.shape, np.uint8), acceleration_valid=np.ones(rpm.shape, np.uint8))
        data.update(changes)
        return self.native.evaluate(parameters or self.parameters, coefficient_source=source, **data)

    def test_si_conversion_and_body_reaction_signs(self):
        out = self.evaluate([[60, 60, 60]], [[60, 60, 60]])
        np.testing.assert_allclose(out['omega_rad_s'], 2*np.pi, rtol=1e-7)
        np.testing.assert_allclose(out['alpha_rad_s2'], 2*np.pi, rtol=1e-7)
        expected_aero = np.asarray([-1, 1, -1])*1e-7*(2*np.pi)**2
        expected_inertia = np.asarray([-1, 1, -1])*2e-5*(2*np.pi)
        np.testing.assert_allclose(out['aerodynamic_nm'][0], expected_aero, rtol=1e-7)
        np.testing.assert_allclose(out['inertial_nm'][0], expected_inertia, rtol=1e-7)
        self.assertFalse(out['calibrated'].any())
        self.assertFalse(out['for_flight'].any())

    def test_equal_rotors_leave_negative_residual_and_doubling_speed_quadruples_aero(self):
        out = self.evaluate([[1000, 1000, 1000], [2000, 2000, 2000]])
        one = 1e-7*(1000*2*np.pi/60)**2
        np.testing.assert_allclose(out['summed_nm'][:, 0], [-one, -4*one], rtol=2e-7)
        self.assertTrue((out['summed_nm'][:, 0] < 0).all())

    def test_deceleration_reverses_only_inertia_and_can_reverse_total(self):
        positive = self.evaluate([[1000, 0, 0]], [[30000, 0, 0]])
        negative = self.evaluate([[1000, 0, 0]], [[-30000, 0, 0]])
        np.testing.assert_array_equal(positive['aerodynamic_nm'], negative['aerodynamic_nm'])
        np.testing.assert_array_equal(positive['inertial_nm'], -negative['inertial_nm'])
        self.assertLess(positive['summed_nm'][0, 2], 0)
        self.assertGreater(negative['summed_nm'][0, 2], 0)

    def test_complete_motor_tuple_permutation_preserves_total(self):
        parameters = copy.deepcopy(self.parameters)
        parameters['k_q_nm_per_rad_s_squared'] = [1e-7, 2e-7, 3e-7]
        parameters['rotor_inertia_kg_m2'] = [2e-5, 4e-5, 1e-5]
        rpm = np.asarray([[700, 900, 1100]])
        acceleration = np.asarray([[1000, -2000, 3000]])
        before = self.evaluate(rpm, acceleration, parameters)
        permutation = [2, 0, 1]
        for key in ('k_q_nm_per_rad_s_squared', 'rotor_inertia_kg_m2', 'reaction_sign'):
            parameters[key] = [parameters[key][i] for i in permutation]
        after = self.evaluate(rpm[:, permutation], acceleration[:, permutation], parameters)
        np.testing.assert_allclose(before['summed_nm'], after['summed_nm'], rtol=0, atol=2e-9)
        np.testing.assert_array_equal(before['total_nm'][:, permutation], after['total_nm'])

    def test_frozen_cases_match_independent_analytic_oracles(self):
        for case in build_cases(self.protocol):
            with self.subTest(case=case['name']):
                out = evaluate_case(self.native, self.protocol, case)
                self.assertTrue(out['valid'].all())
                for key, expected in case['oracle'].items():
                    np.testing.assert_allclose(out[key], expected, rtol=3e-6, atol=2e-8)
                if case['kind'] == 'cancellation_snapshot':
                    self.assertEqual(len(case['time_s']), 1) # No constant-RPM/nonzero-alpha trajectory.
                    self.assertLess(abs(out['summed_nm'][0, 2]), 1e-9)

    def test_free_coast_frame_torque_cancels_and_inertial_impulse_matches_momentum(self):
        for case in build_cases(self.protocol):
            if case['kind'] not in ('ramp', 'free_coast'):
                continue
            out = evaluate_case(self.native, self.protocol, case)
            impulse = np.trapezoid(out['inertial_nm'], x=case['time_s'], axis=0)
            delta_omega = (case['rpm'][-1]-case['rpm'][0])*2*np.pi/60
            expected = np.asarray(self.parameters['reaction_sign'])*np.asarray(self.parameters['rotor_inertia_kg_m2'])*delta_omega
            np.testing.assert_allclose(impulse, expected, rtol=2e-6, atol=1e-8)
            if case['kind'] == 'free_coast':
                self.assertLess(np.max(abs(out['summed_nm'][:, 2])), 3e-9)
                self.assertGreater(np.max(abs(out['summed_nm'][:, 0])), .001)

    def test_one_bad_motor_suppresses_all_outputs_without_partial_sum(self):
        for field in ('seen', 'fresh', 'valid', 'acceleration_valid'):
            flags = np.ones((1, 3), np.uint8)
            flags[0, 1] = 0
            out = self.evaluate([[1000, 2000, 3000]], **{field: flags})
            self.assertFalse(out['valid'][0], field)
            self.assertEqual(out['invalid_motor_mask'][0], 2)
            for key in ('omega_rad_s', 'alpha_rad_s2', 'aerodynamic_nm', 'inertial_nm', 'total_nm', 'summed_nm'):
                self.assertTrue((out[key] == 0).all(), (field, key))
        age = np.zeros((1, 3), np.uint32); age[0, 2] = self.parameters['max_rpm_age_us']+1
        self.assertFalse(self.evaluate([[1000]*3], age_us=age)['valid'][0])
        for bad in (float('nan'), float('inf'), -1, 20001):
            out = self.evaluate([[1000, bad, 1000]])
            self.assertFalse(out['valid'][0])
            self.assertTrue((out['summed_nm'] == 0).all())
        self.assertFalse(self.evaluate([[0]*3], [[0, 100001, 0]])['valid'][0])

    def test_torque_guard_and_coefficient_provenance_cannot_be_hidden_by_cancellation(self):
        parameters = copy.deepcopy(self.parameters)
        parameters['max_abs_torque_nm'] = 1e-9
        out = self.evaluate([[1000, 1000, 0]], parameters=parameters)
        self.assertFalse(out['valid'][0])
        self.assertNotEqual(out['status'][0] & 256, 0)
        self.assertTrue((out['summed_nm'] == 0).all())
        out = self.evaluate([[1000]*3], source='UNSPECIFIED')
        self.assertFalse(out['valid'][0])
        self.assertNotEqual(out['status'][0] & 2, 0)
        for field, value in [('coefficient_source', 'CALIBRATED'), ('calibrated', True), ('for_flight', True),
                             ('logical_motor_order', ['front_right', 'rear', 'front_left'])]:
            changed = copy.deepcopy(self.protocol); changed[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_protocol(changed)
        changed = copy.deepcopy(self.protocol); changed['units']['mechanical_rpm'] = 'rad/s'
        with self.assertRaises(ValueError):
            validate_protocol(changed)

    def test_transport_rejects_malformed_inputs_before_native_access(self):
        for field, value in [('mechanical_rpm', np.zeros((1, 2))), ('age_us', np.full((1, 3), -1)),
                             ('seen', np.full((1, 3), 2)), ('acceleration_valid', np.ones((2, 3)))]:
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.evaluate([[1000]*3], **{field: value})
        changed = copy.deepcopy(self.parameters); changed['k_q_nm_per_rad_s_squared'] = [[1, 2, 3]]
        with self.assertRaises(ValueError):
            self.evaluate([[1000]*3], parameters=changed)
        changed = copy.deepcopy(self.parameters); changed['max_rpm_age_us'] = 2**40
        with self.assertRaises(ValueError):
            self.evaluate([[1000]*3], parameters=changed)


if __name__ == '__main__':
    unittest.main()
