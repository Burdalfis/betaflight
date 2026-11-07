"""Exact synthetic source identities test real C; historical proxies stay separate."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from rpm_data import observation_proxy
from rpm_native import NativeObserver

SETTINGS = Path(__file__).with_name('rpm_replay_settings.json')


def source(times, values=None):
    times = np.asarray(times, np.uint32)
    shape = (len(times), 3)
    return dict(now_us=times, observed_us=np.repeat(times[:, None], 3, axis=1),
                sequence=np.repeat(np.arange(1, len(times)+1, dtype=np.uint32)[:, None], 3, axis=1),
                seen=np.ones(shape, np.uint8),
                mechanical_rpm=np.zeros(shape, np.float32) if values is None else np.asarray(values, np.float32))


def historical(times, ages, erpm=None, flags=None):
    count = len(times)
    data = {'time (us)': np.asarray(times, np.float64),
            'debug[3]': np.full(count, 255.) if flags is None else np.asarray(flags, np.float64)}
    for motor in range(3):
        data[f'debug[{4+motor}]'] = np.asarray(ages, np.float64)
        data[f'eRPM[{motor}]'] = np.zeros(count) if erpm is None else np.asarray(erpm, np.float64)
    return data


class RpmReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='tricopter-rpm-native-')
        cls.native = NativeObserver(Path(cls.temporary.name)/'build')
        cls.params = json.loads(SETTINGS.read_text())['parameters']

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_source(self, data):
        return self.native.replay(self.params, **data)

    def test_equal_zero_receptions_refresh_by_identity_and_motors_are_independent(self):
        data = source(np.arange(0, 1400001, 1000))
        data['observed_us'][:, 0] = 0
        data['sequence'][:, 0] = 1
        out = self.run_source(data)
        self.assertEqual(out['accepted'][:, 0].sum(), 1)
        self.assertTrue(out['accepted'][:, 1:].all())
        self.assertEqual(out['age_us'][-1, 0], 1400000)
        self.assertFalse(out['fresh'][-1, 0])
        self.assertFalse(out['valid'][-1, 0])
        self.assertTrue(out['fresh'][:, 1:].all())
        self.assertTrue((out['rpm'] == 0).all())
        self.assertTrue((out['acceleration_rpm_s'] == 0).all())

    def test_known_ramp_tracks_speed_and_derivative_using_exact_synthetic_truth(self):
        times = np.arange(0, 500001, 1000, dtype=np.uint32)
        truth = 1000+2000*times.astype(float)/1e6
        data = source(times, np.repeat(truth[:, None], 3, axis=1))
        out = self.run_source(data)
        tail = times > 300000
        self.assertLess(np.max(abs(out['rpm'][tail, 0]-truth[tail])), .5)
        self.assertLess(np.max(abs(out['acceleration_rpm_s'][tail, 0]-2000)), 1.)
        self.assertTrue(out['valid'].all())

    def test_extra_cached_queries_do_not_double_integrate(self):
        sparse_time = np.arange(0, 300001, 5000, dtype=np.uint32)
        sparse = source(sparse_time, np.repeat((500+1500*sparse_time/1e6)[:, None], 3, axis=1))
        dense_time = np.arange(0, 300001, 1000, dtype=np.uint32)
        row = np.arange(len(dense_time))//5
        dense = {key: value[row].copy() for key, value in sparse.items()}
        dense['now_us'] = dense_time
        a, b = self.run_source(sparse), self.run_source(dense)
        np.testing.assert_allclose(a['rpm'], b['rpm'][::5], rtol=0, atol=0)
        np.testing.assert_allclose(a['acceleration_rpm_s'], b['acceleration_rpm_s'][::5], rtol=0, atol=0)

    def test_future_data_and_other_motors_cannot_change_prefix_or_motor_zero(self):
        data = source(np.arange(0, 100001, 1000))
        data['mechanical_rpm'][:] = 1000
        before = self.run_source(data)
        changed = copy.deepcopy(data)
        changed['mechanical_rpm'][51:, :] = 8000
        after = self.run_source(changed)
        np.testing.assert_array_equal(before['rpm'][:51], after['rpm'][:51])
        changed = copy.deepcopy(data)
        changed['mechanical_rpm'][:, 1] = np.arange(len(data['now_us']))*20
        after = self.run_source(changed)
        np.testing.assert_array_equal(before['rpm'][:, (0, 2)], after['rpm'][:, (0, 2)])
        np.testing.assert_array_equal(before['acceleration_rpm_s'][:, (0, 2)], after['acceleration_rpm_s'][:, (0, 2)])

    def test_invalid_future_nan_and_negative_observations_never_refresh_age(self):
        for bad in (float('nan'), float('inf'), -1., 20001.):
            data = source([1000, 2000, 3000])
            data['mechanical_rpm'][:] = 1000
            data['mechanical_rpm'][1:, 0] = bad
            out = self.run_source(data)
            self.assertEqual(out['accepted'][:, 0].tolist(), [True, False, False])
            self.assertEqual(out['age_us'][-1, 0], 2000)
            self.assertEqual(out['sequence'][-1, 0], 1)
        data = source([1000, 2000])
        data['observed_us'][1, 0] = 2001
        out = self.run_source(data)
        self.assertFalse(out['accepted'][1, 0])
        self.assertEqual(out['age_us'][1, 0], 1000)

    def test_identity_mutation_latches_and_same_timestamp_new_identity_coalesces(self):
        data = source([1000, 2000, 3000])
        data['mechanical_rpm'][:] = 500
        data['sequence'][1:, 0] = 1
        data['observed_us'][1:, 0] = 1000
        data['mechanical_rpm'][1:, 0] = 600
        out = self.run_source(data)
        self.assertTrue(np.all(~out['valid'][1:, 0]))
        self.assertTrue(np.all((out['status'][1:, 0] & 64) != 0))
        data = source([1000, 1000])
        data['mechanical_rpm'][1] = 700
        out = self.run_source(data)
        self.assertTrue(out['accepted'].all())
        self.assertTrue(np.all(out['rpm'][1] == 700))
        self.assertTrue(np.all(out['acceleration_rpm_s'][1] == 0))
        self.assertTrue(np.all((out['status'][1] & 2048) != 0))

    def test_proxy_deduplicates_quantized_cached_zero_but_accepts_new_equal_zero(self):
        data = historical([100000, 101000, 102000, 103000, 104000], [1, 11, 21, 1, 1])
        proxy = observation_proxy(data, 250)
        self.assertEqual(proxy.proxy_new[:, 0].tolist(), [True, False, False, True, True])
        self.assertEqual(proxy.sequence[:, 0].tolist(), [1, 1, 1, 2, 3])
        self.assertEqual(proxy.observed_us[0, 0], proxy.observed_us[2, 0])
        out = self.native.replay(self.params, proxy.now_us, proxy.observed_us, proxy.sequence, proxy.seen, proxy.mechanical_rpm)
        self.assertEqual(out['accepted'][:, 0].tolist(), [True, False, False, True, True])
        self.assertGreater(out['age_us'][2, 0], out['age_us'][0, 0])

    def test_proxy_guards_unseen_saturated_disabled_and_nonfinite_rpm(self):
        data = historical([100000, 101000, 102000, 103000, 104000, 105000], [1, -1, 32767, 1, 1, 1],
                          [0, 0, 0, 0, float('nan'), -1], [255, 255, 255, 239, 255, 255])
        proxy = observation_proxy(data, 250)
        self.assertEqual(proxy.metadata_usable[:, 0].tolist(), [True, False, False, False, False, False])
        self.assertEqual(proxy.seen[:, 0].tolist(), [1, 0, 0, 0, 0, 0])
        data = historical([100000], [1], flags=[223]) # Motor0 seen bit absent.
        self.assertFalse(observation_proxy(data, 250).metadata_usable[0, 0])
        self.assertTrue(observation_proxy(data, 250).metadata_usable[0, 1])

    def test_phase_sensitivity_widens_plausible_intervals_without_claiming_accuracy(self):
        data = historical([100000], [10], erpm=[70])
        for phase in (0, 250, 500):
            proxy = observation_proxy(data, phase)
            self.assertEqual(proxy.interval_lower_us[0, 0], 98901)
            self.assertEqual(proxy.interval_upper_us[0, 0], 99000+phase)
            self.assertEqual(proxy.native_age_upper_us[0, 0], 1099+phase)
            self.assertAlmostEqual(float(proxy.mechanical_rpm[0, 0]), 1000)
            self.assertTrue(proxy.diagnostics['identity_and_timing_are_approximate'])

    def test_transport_rejects_malformed_arrays_before_native_pointer_access(self):
        data = source([1000, 2000])
        for key, bad in [('now_us', np.array([])), ('observed_us', np.zeros((2, 2))),
                         ('sequence', np.full((2, 3), -1.)), ('seen', np.full((2, 3), 2)),
                         ('now_us', np.array([1., float('nan')])), ('now_us', np.array([1., 2.**40]))]:
            changed = copy.deepcopy(data)
            changed[key] = bad
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.run_source(changed)
        params = dict(self.params, max_prediction_us=2.**40)
        with self.assertRaises(ValueError):
            self.native.replay(params, **data)
        params = {key: [value, value] for key, value in self.params.items()}
        with self.assertRaises(ValueError):
            self.native.replay(params, **data)


if __name__ == '__main__':
    unittest.main()
