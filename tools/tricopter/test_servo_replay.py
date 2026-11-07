"""End-to-end C bridge, known-response fitting and leakage/quality checks."""
import copy
import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from servo_data import Unit, validated_data
from servo_native import NativeModel
from servo_replay import SETTINGS, PARAMETERS, evaluate, fit_models, score_mask, validate_frozen_settings, validate_replay_report


def synthetic_unit(name='train', split='training'):
    t = np.arange(0, 2.4, .001)
    cmd = np.asarray(([1100.]*400+[1700.]*400+[1420.]*400)*2)
    phase = np.arange(len(t)) % 400
    return Unit(name, split, np.ascontiguousarray(t*1e6), np.ascontiguousarray(cmd),
                np.full(len(t), 1585.), np.ones(len(t), bool), np.zeros(len(t)),
                phase < 150, phase >= 300, 1420., 0, {})


class ServoReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='tricopter-native-test-')
        cls.native = NativeModel(Path(cls.temporary.name)/'build')
        cls.parameters = np.asarray([1940., 1585., 1270., 0., 0., .040, .040, 0., 0.])

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def measured_unit(self):
        unit = synthetic_unit()
        unit.position_counts = self.native.replay(self.parameters, unit)['position'].copy()
        return unit

    def test_native_bridge_known_exponential_and_units(self):
        unit = synthetic_unit()
        prediction = self.native.replay(self.parameters, unit)
        expected = 1940+(1585-1940)*np.exp(-.100/.040)
        self.assertLess(abs(prediction['position'][100]-expected), .05)
        expected_velocity = (1940-expected)/.040
        self.assertLess(abs(prediction['velocity'][100]-expected_velocity), 2.)
        self.assertTrue(prediction['valid'].all())

    def test_only_first_measurement_initializes_and_stale_does_not_freeze_time(self):
        unit = self.measured_unit()
        first = self.native.replay(self.parameters, unit)
        unit.position_counts[1:] += 500
        unit.valid[20:200] = False
        second = self.native.replay(self.parameters, unit)
        np.testing.assert_array_equal(first['position'], second['position'])
        self.assertNotEqual(second['position'][20], second['position'][199])
        self.assertFalse(score_mask(unit)[0])

    def test_known_lag_recovered_with_real_core(self):
        unit = self.measured_unit()
        settings = json.loads(SETTINGS.read_text())
        settings['evaluation_budget']['lag'] = 700
        fit = fit_models(self.native, [unit], settings, names=('lag',))[0]
        self.assertLess(abs(fit['parameters']['tau_increasing_pwm_s']-.040), .002)
        self.assertLess(fit['optimization']['best_training_mse_counts2'], 1.)

    def test_validation_targets_never_change_fit(self):
        train = self.measured_unit()
        heldout = copy.deepcopy(train)
        heldout.name, heldout.split = 'heldout', 'validation'
        settings = json.loads(SETTINGS.read_text())
        settings['evaluation_budget']['static'] = 50
        before = fit_models(self.native, [train, heldout], settings, names=('static',))[0]
        heldout.position_counts[1:] = 4094-np.arange(len(heldout.time_us)-1) % 2000
        heldout.valid[1:] = False
        after = fit_models(self.native, [train, heldout], settings, names=('static',))[0]
        self.assertEqual(before, after)

    def test_malformed_native_inputs_rejected_before_pointer_access(self):
        changes = [('command_us', np.array([1420.], dtype=np.float64)),
                   ('time_us', np.zeros((2, 2))), ('time_us', np.array([], dtype=np.float64)),
                   ('time_us', np.full(2400, np.nan)), ('time_us', np.full(2400, -1.)),
                   ('time_us', np.full(2400, 2.**54)), ('position_counts', np.full(2400, np.inf))]
        for name, value in changes:
            unit = synthetic_unit()
            setattr(unit, name, value)
            with self.subTest(field=name), self.assertRaises(ValueError):
                self.native.replay(self.parameters, unit)
        for delay in (-1., float('inf'), 2.**40):
            params = self.parameters.copy(); params[4] = delay
            with self.subTest(delay=delay), self.assertRaises(ValueError):
                self.native.replay(params, synthetic_unit())

    def test_schema_identity_and_no_raw_fallback(self):
        folder = Path(self.temporary.name)
        path = folder/'quality.csv'
        names = ['time (us)']+[f'debug[{i}]' for i in range(8)]
        rows = [[0, 1585, 1420, 2, 1583, 0, 15, 0, 0],
                [1000, 1585, 1420, 2, -32768, 0, 15, 0, 0],
                [2000, 1585, 1420, 2, 1583, 17, 7, 0, 0]]
        def write():
            with path.open('w') as f:
                w = csv.writer(f); w.writerow(names); w.writerows(rows)
        write()
        data, valid, _ = validated_data(path)
        self.assertEqual(valid.tolist(), [True, False, False])
        self.assertEqual(data['debug[3]'][1], -32768)
        rows[0][1] += 20
        write()
        with self.assertRaisesRegex(ValueError, 'identity'):
            validated_data(path)

    def test_missing_command_history_cannot_improve_score_by_dropping_suffix(self):
        unit = self.measured_unit()
        unit.time_us[200:] += 20000
        prediction = self.native.replay(self.parameters, unit)
        self.assertTrue(prediction['valid'][:200].all())
        self.assertFalse(prediction['valid'][200:].any())
        model = {'name': 'full', 'parameters': dict(zip(PARAMETERS, self.parameters))}
        with self.assertRaisesRegex(ValueError, 'invalid model history'):
            evaluate(self.native, [model], [unit], Path(self.temporary.name))

    def test_replay_cannot_relabel_old_training_as_new_holdout(self):
        frozen = {'protocol_sha256': 'splitA', 'settings_sha256': 'settingsA',
                  'protocol': {'training_units': [0, 1], 'validation_units': [2]},
                  'settings': {'evaluation_budget': 500}}
        previous = {'native': {'sources': {'core': 'sameC'}}, 'frozen_protocol': copy.deepcopy(frozen)}
        validate_replay_report(previous, frozen, {'core': 'sameC'})
        for key, value in [('protocol_sha256', 'splitB'), ('settings_sha256', 'settingsB'),
                           ('protocol', {'training_units': [2], 'validation_units': [0, 1]}),
                           ('settings', {'evaluation_budget': 900})]:
            changed = copy.deepcopy(frozen)
            changed[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'relabel'):
                validate_replay_report(previous, changed, {'core': 'sameC'})

    def test_frozen_metadata_matches_fixed_bridge_architecture(self):
        settings = json.loads(SETTINGS.read_text())
        validate_frozen_settings(settings)
        for key, value in [('map_knot_commands_us', [1100, 1500, 1700]), ('model_max_gap_us', 10000)]:
            changed = copy.deepcopy(settings)
            changed[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'bridge architecture'):
                validate_frozen_settings(changed)


if __name__ == '__main__':
    unittest.main()
