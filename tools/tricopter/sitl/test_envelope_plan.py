"""Fixed operating-point excitation, bounds and simulator-only termination."""
import copy
import hashlib
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

from autotune_yaw import IdentityError, ProfileTrial
from check_comparison import ExperimentReceiver, Trial
from check_envelope import Campaign, EnvelopeTrial, cases
from check_flight import Pilot
from envelope_targets import angle_targets


def reference():
    return {'common_msp_readback': {
        '111': '07004343430032000000000707000064ce07ce07ce070332',
        '94': '00000000000000000000000000000000001e00000050000000000001010005002f003100000000282e00002300c80f64000001410f5a07000001414605',
        '125': '0000053200'},
        'yaw_settings': {'center': 70, 'max': 670, 'expo': 0., 'deadband': 0,
                         'reversed': False, 'limit': 1998}}


def trial(kind, directory=Path('/unused')):
    args = (directory, 'stock', Path('/plugin'), Path('/world'), {})
    return kind(*args, gains={'p': 31, 'i': 20, 'd': 10}) if kind is EnvelopeTrial else kind(*args)


def termination_fixture(obj, final_armed=False):
    obj.armed = True
    obj.pilot = Mock()
    obj.pilot.origin = [0., 0., 0.]
    obj.pilot.snapshot.return_value = {'sim_time_s': 20., 'z_m': 42., 'vz_m_s': 3.}
    channels = [1500, 1500, 1600, 1368, 1900, 1900]
    obj.receiver = Mock()
    obj.receiver.set.side_effect = lambda index, value: channels.__setitem__(index, value)
    obj.client = Mock()
    def status():
        # Successful simulated disarm is verified only after the normal arm
        # and throttle RC channels are low and the setup pilot is inactive.
        if not final_armed:
            assert channels[4] == 1000 and channels[2] == 1000
            obj.pilot.command.assert_any_call('disarmed', active=False)
        return {'armed': final_armed}
    obj.client.status.side_effect = status
    return channels


class EnvelopePlan(unittest.TestCase):
    def test_plugin_identity_uses_retained_per_variant_group_before_any_flight(self):
        class ReachedTrialFactory(Exception): pass
        for matching in (True, False):
            with tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                built = root/'build/plugin.so'; built.parent.mkdir(); built.write_bytes(b'frozen plugin')
                (built.parent/'build-manifest.json').write_text('{}\n')
                digest = hashlib.sha256(built.read_bytes()).hexdigest()
                campaign = Campaign.__new__(Campaign)
                campaign.directory = root/'campaign'; campaign.directory.mkdir()
                campaign.reference = {'runs': []}  # Historical summary has no top-level plugin hash.
                campaign.reference_records = {'stock': {'plugin_sha256': digest if matching else '0'*64}}
                campaign.gains = {v: {'p':31,'i':20,'d':10} for v in ('stock','modified')}
                campaign.protocol = {'variant_order':['stock'], 'cases':[{'name':'hover_control'}]}
                campaign.summary = {'pass':False,'runs':[]}
                campaign.guard = Mock()
                with patch('check_envelope.SessionLock'), \
                     patch('check_envelope.build_plugin', return_value=built), \
                     patch('check_envelope.EnvelopeTrial', side_effect=ReachedTrialFactory) as factory:
                    if matching:
                        with self.assertRaises(ReachedTrialFactory): campaign.run()
                        factory.assert_called_once()
                    else:
                        with self.assertRaisesRegex(AssertionError, 'Plugin differs from hover study'):
                            campaign.run()
                        factory.assert_not_called()
                self.assertFalse(campaign.summary['pass'])
                self.assertEqual(campaign.summary['plugin_sha256'], digest)

    def test_regimes_have_matched_controls_and_only_yaw_varies_at_test_boundary(self):
        r = reference(); original = copy.deepcopy(r)
        definitions = cases(r)
        self.assertEqual(r, original)
        self.assertEqual(len(definitions), 9)
        by_name = {case['name']: case for case in definitions}
        self.assertEqual(len(by_name), len(definitions))
        for point in ('hover', 'climb', 'forward'):
            control = by_name[point+'_control']
            self.assertEqual(control['direction'], 'control')
            for case in (by_name[point+'_cw'], by_name[point+'_ccw']):
                self.assertEqual(case['matched_control'], control['name'])
                self.assertEqual(case['operating_point'], point)
                self.assertEqual(case['stages'][:2], control['stages'][:2])
                self.assertEqual(case['stages'][-1], control['stages'][-1])
                pulse, neutral = case['stages'][2], control['stages'][2]
                self.assertEqual(pulse['channels'][:3], neutral['channels'][:3])
                self.assertEqual(pulse['duration_s'], neutral['duration_s'])
                self.assertNotEqual(pulse['channels'][3], neutral['channels'][3])
            operating = control['stages'][1]['channels']
            self.assertEqual(control['stages'][2]['channels'], operating)
            self.assertEqual(control['stages'][3]['channels'], operating)
            self.assertEqual(operating[3], 1500)
        self.assertGreater(by_name['climb_control']['stages'][1]['channels'][2], 1345)
        forward = by_name['forward_control']['stages'][1]['channels']
        target = angle_targets(r, forward[0], forward[1], forward[3])
        self.assertLess(abs(target['requested_pitch_deg']-20), .1)
        self.assertEqual(target['requested_roll_deg'], 0)
        self.assertEqual(forward[2], 1380)

    def test_stages_are_bounded_and_sender_owns_absolute_boundaries(self):
        for definition in cases(reference()):
            stages = definition['stages']
            self.assertEqual([stage['name'] for stage in stages],
                             ['baseline', 'operating_lead', 'pulse', 'recovery'])
            self.assertEqual(stages[0]['channels'], [1500, 1500, 1345, 1500])
            self.assertLess(sum(stage['duration_s'] for stage in stages), 25)
            for stage in stages:
                self.assertGreater(stage['duration_s'], 0)
                self.assertTrue(all(1000 <= value <= 2000 for value in stage['channels']))
                self.assertEqual(len(stage['channels']), 4)
            receiver = ExperimentReceiver.__new__(ExperimentReceiver)
            receiver.plan = {'case': definition['name'], 'stages': stages, 'epoch': 100.}
            age = 0.
            for stage in stages:
                marker, channels = receiver.planned_input(100+age)
                self.assertEqual(marker, (definition['name'], stage['name']))
                self.assertEqual(channels, stage['channels'])
                age += stage['duration_s']
            marker, channels = receiver.planned_input(100+age)
            self.assertEqual(marker, (definition['name'], 'complete'))
            self.assertEqual(channels, stages[-1]['channels'])

    def test_old_trial_bounds_still_reject_distance_and_measurement_ground_contact(self):
        self.assertEqual((Pilot.maximum_height_m, Pilot.maximum_attitude_deg), (3., 40.))
        for kind, distance, reject in ((Trial, 6., True), (EnvelopeTrial, 6., False),
                                      (EnvelopeTrial, 201., True)):
            obj = trial(kind)
            obj.next_msp = float('inf')
            obj.pilot = SimpleNamespace(fault=None, origin=[0.,0.,0.],
                snapshot=lambda: {'x_m':distance, 'y_m':0., 'z_m':1.})
            obj.receiver = SimpleNamespace(failed=None, override=None, plan=None)
            if reject:
                with self.subTest(kind=kind, distance=distance), self.assertRaisesRegex(RuntimeError, 'horizontal envelope'):
                    obj.health()
            else:
                self.assertEqual(obj.health()['x_m'], distance)
        for kind in (Trial, EnvelopeTrial):
            obj = trial(kind); obj.next_msp = float('inf')
            obj.pilot = SimpleNamespace(fault=None, origin=[0.,0.,0.],
                snapshot=lambda: {'x_m':0., 'y_m':0., 'z_m':.34})
            obj.receiver = SimpleNamespace(failed=None, override=[1500]*4, plan=None)
            with self.subTest(kind=kind), self.assertRaisesRegex(RuntimeError, 'ground contact'):
                obj.health()

    def test_envelope_pilot_extension_is_instance_local_and_keeps_attitude_bound(self):
        pilot = SimpleNamespace(maximum_height_m=3., maximum_attitude_deg=40.)
        obj = trial(EnvelopeTrial)
        with patch.object(ProfileTrial, 'make_pilot', return_value=pilot):
            self.assertIs(obj.make_pilot({}, .5, 30, None), pilot)
        self.assertEqual(pilot.maximum_height_m, 200.)
        self.assertEqual(pilot.maximum_attitude_deg, 40.)
        self.assertEqual(Pilot.maximum_height_m, 3.)

    def test_in_air_termination_sends_normal_disarm_and_retains_flight_state(self):
        obj = trial(EnvelopeTrial); channels = termination_fixture(obj)
        with patch('check_envelope.time.sleep'):
            obj.finish()
        self.assertFalse(obj.armed)
        self.assertFalse(obj.record['final_status']['armed'])
        self.assertEqual(channels[4], 1000)
        self.assertEqual(channels[2], 1000)
        self.assertEqual(obj.record['pretermination_state']['z_m'], 42.)
        self.assertEqual(obj.record['termination'], 'simulated_in_air_disarm_then_process_stop')
        obj.receiver.select.assert_called_once_with('disarmed', 'disarmed', [1500,1500,1000,1500])
        self.assertFalse(any(call.args and call.args[0] == 'landing' for call in obj.pilot.command.call_args_list))

    def test_failed_native_disarm_is_not_reported_as_success(self):
        obj = trial(EnvelopeTrial); termination_fixture(obj, final_armed=True)
        with patch('check_envelope.time.sleep'), self.assertRaisesRegex(AssertionError, 'did not disarm'):
            obj.finish()
        self.assertTrue(obj.record['final_status']['armed'])

    def test_default_finish_keeps_landing_before_disarm(self):
        obj = trial(Trial); termination_fixture(obj)
        obj.health = Mock(return_value={'z_m':.01, 'vz_m_s':0.})
        with patch('check_comparison.time.sleep'):
            obj.finish()
        self.assertFalse(obj.record['final_status']['armed'])
        obj.receiver.select.assert_called_once_with('landing', 'landing')
        obj.pilot.command.assert_any_call('landing', destination=0., yaw=1500)
        self.assertNotIn('termination', obj.record)

    def test_native_earth_reference_drift_fails_before_setup(self):
        for log in ('# get angle_earth_ref\nangle_earth_ref = 0\n',
                    '# set angle_earth_ref = 100\nangle_earth_ref set to 100\n'):
            with tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary); (path/'provision.log').write_text(log)
                obj = trial(EnvelopeTrial, path)
                with patch.object(ProfileTrial, 'recapture') as base:
                    with self.assertRaises(IdentityError): obj.recapture(first=True)
                base.assert_not_called()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)
            (path/'provision.log').write_text('# get angle_earth_ref\nangle_earth_ref = 100\n')
            obj = trial(EnvelopeTrial, path)
            with patch.object(ProfileTrial, 'recapture', return_value='healthy setup') as base:
                self.assertEqual(obj.recapture(first=True), 'healthy setup')
            base.assert_called_once_with(True)
            self.assertEqual(obj.record['angle_earth_ref'], 100)


if __name__ == '__main__': unittest.main()
