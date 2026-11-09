"""High-collective protocol and world changes without starting a simulator."""
import copy
import hashlib
import math
from pathlib import Path
import re
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from check_comparison import ExperimentReceiver, Trial
import check_yaw_stress as stress
from autotune_yaw import IdentityError
from envelope_targets import angle_targets
from run import materialize_world


def reference():
    advanced=bytearray.fromhex(
        '00000000000000000000000000000000001e00000050000000000001010005002f003100000000282e00002300c80f64000001410f5a07000001414605')
    advanced[17]=80
    return {'common_msp_readback': {
        '111':'07004343430032000000000707000064ce07ce07ce070332',
        '94':advanced.hex(),'125':'0000053200'},
        'yaw_settings':{'center':70,'max':670,'expo':0.,'deadband':0,'reversed':False,'limit':1998}}


class YawStressPlan(unittest.TestCase):
    def test_frozen_profiles_distinguish_factory_aircraft_and_hover_tune(self):
        self.assertEqual({name:{key:value for key,value in item.items() if key!='label'}
                          for name,item in stress.PROFILES.items()}, {
            'factory_yaw':{'gains':{'p':45,'i':80,'d':0},'feedforward':120,'hold_gain':15},
            'hover_tuned':{'gains':{'p':31,'i':20,'d':10},'feedforward':0,'hold_gain':0},
            'aircraft_original':{'gains':{'p':50,'i':10,'d':20},'feedforward':47,'hold_gain':15}})
        self.assertIn('not Betaflight factory',stress.PROFILES['aircraft_original']['label'])
        before=copy.deepcopy(stress.PROFILES);stress.stress_cases()
        self.assertEqual(stress.PROFILES,before)

    def test_unique_case_matrix_native_collective_and_bounded_programs(self):
        definitions=stress.stress_cases();names={case['name'] for case in definitions}
        expected={f'punch_{level}' for level in (70,85,100)}
        expected|={f'hold_{level}_{direction}' for level in (70,85,90,95,100) for direction in ('control','cw','ccw')}
        expected|={f'forward_70_{direction}' for direction in ('control','cw','ccw')}
        self.assertEqual(names,expected);self.assertEqual(len(definitions),len(expected))
        by_name={case['name']:case for case in definitions}
        for percent,throttle in ((70,1715),(85,1858),(100,2000)):
            case=by_name[f'punch_{percent}'];pulse=case['stages'][1]
            self.assertEqual(pulse['channels'][2],throttle)
            actual=((throttle-1050)*1000//950)/1000
            self.assertEqual(actual,percent/100)
            self.assertEqual(case['native_collective_requested'],actual)
            self.assertEqual([stage['name'] for stage in case['stages']],['baseline','pulse','recovery'])
            self.assertEqual(pulse['duration_s'],1.)
            self.assertEqual(case['stages'][-1]['channels'],[1500,1500,1345,1500])
        for case in definitions:
            stages=case['stages']
            self.assertEqual(len({stage['name'] for stage in stages}),len(stages))
            self.assertEqual(stages[0]['channels'],[1500,1500,1345,1500])
            for stage in stages:
                self.assertTrue(math.isfinite(stage['duration_s']) and stage['duration_s']>0)
                self.assertEqual(len(stage['channels']),4)
                self.assertTrue(all(isinstance(value,int) and 1000<=value<=2000 for value in stage['channels']))

    def test_high_collective_hold_controls_match_every_input_except_yaw(self):
        by_name={case['name']:case for case in stress.stress_cases()}
        for percent,throttle in ((70,1715),(85,1858),(90,1905),(95,1953),(100,2000)):
            control=by_name[f'hold_{percent}_control']
            hold_duration=4. if percent<=85 else 3.
            self.assertEqual([stage['duration_s'] for stage in control['stages']],[1.,.5,hold_duration,4.])
            self.assertEqual(control['stages'][1]['channels'][2],throttle)
            self.assertEqual(((throttle-1050)*1000//950)/1000,percent/100)
            for direction,yaw in (('cw',1368),('ccw',1632)):
                case=by_name[f'hold_{percent}_{direction}']
                self.assertEqual(case['stages'][:2],control['stages'][:2])
                self.assertEqual(case['stages'][-1],control['stages'][-1])
                pulse=case['stages'][2];neutral=control['stages'][2]
                self.assertEqual(pulse['channels'][:3],neutral['channels'][:3])
                self.assertEqual(pulse['duration_s'],neutral['duration_s'])
                self.assertEqual(pulse['channels'][3],yaw)

    def test_forward_ramp_and_steady_probe_inputs_are_shared_and_high_pitch(self):
        cases=[case for case in stress.stress_cases() if case['name'].startswith('forward_')]
        self.assertEqual({case['direction'] for case in cases},{'control','cw','ccw'})
        control=next(case for case in cases if case['direction']=='control')
        ramp=control['stages'][1:21]
        self.assertEqual(len(ramp),20)
        self.assertAlmostEqual(sum(stage['duration_s'] for stage in ramp),4.)
        self.assertTrue(all(b['channels'][2]>=a['channels'][2] for a,b in zip(ramp,ramp[1:])))
        self.assertTrue(all(b['channels'][1]>=a['channels'][1] for a,b in zip(ramp,ramp[1:])))
        self.assertTrue(all(b['channels'][0]<=a['channels'][0] for a,b in zip(ramp,ramp[1:])))
        for case in cases:
            self.assertTrue(case['steady_entry_required'])
            self.assertEqual(case['stages'][:-2],control['stages'][:-2])
            self.assertEqual(case['stages'][-1],control['stages'][-1])
            lead,pulse,recovery=case['stages'][-3:]
            self.assertEqual([stage['name'] for stage in (lead,pulse,recovery)],['operating_lead','pulse','recovery'])
            self.assertEqual([stage['duration_s'] for stage in (lead,pulse,recovery)],[25.,3.,4.])
            for stage in (lead,pulse,recovery):self.assertEqual(stage['channels'][:3],[1440,1986,1715])
            target=angle_targets(reference(),*pulse['channels'][:2],pulse['channels'][3])
            self.assertAlmostEqual(target['requested_roll_deg'],-2.03462686567)
            self.assertAlmostEqual(target['requested_pitch_deg'],75.8101970149)
            self.assertLess(abs(target['requested_yaw_rate_dps']),16)
            if case['direction']!='control':
                self.assertGreater(abs(target['stick_requested_yaw_rate_dps']),60)

    def test_extended_forward_lead_changes_only_the_declared_three_hold_durations(self):
        before=stress.stress_cases();after=stress.stress_cases(40.)
        self.assertEqual(len(before),len(after))
        changed=set()
        for old,new in zip(before,after):
            if not old['name'].startswith('forward_70_'):
                self.assertEqual(old,new);continue
            changed.add(new['direction'])
            self.assertEqual({k:v for k,v in old.items() if k!='stages'},
                             {k:v for k,v in new.items() if k!='stages'})
            for a,b in zip(old['stages'],new['stages']):
                if a['name']=='operating_lead':
                    self.assertEqual(a['duration_s'],25.)
                    self.assertEqual(b['duration_s'],40.)
                    self.assertEqual(a['channels'],b['channels'])
                else:self.assertEqual(a,b)
        self.assertEqual(changed,{'control','cw','ccw'})
        for invalid in (24.99,45.01,float('nan'),float('inf'),-float('inf')):
            with self.subTest(value=invalid),self.assertRaisesRegex(ValueError,'25..45'):
                stress.stress_cases(invalid)

    def test_native_integer_requires_the_last_unambiguous_get_and_no_later_set(self):
        good='\x1b[0m# get imu_dcm_kp\r\nimu_dcm_kp = 0\r\nAllowed range: 0 - 32000\r\n'
        self.assertEqual(stress.native_integer(good,'imu_dcm_kp'),0)
        self.assertEqual(stress.native_integer(good+'# get imu_dcm_kp\nimu_dcm_kp = 2500\n','imu_dcm_kp'),2500)
        self.assertEqual(stress.native_integer(good+'# set imu_dcm_ki = 0\nimu_dcm_ki set to 0\n','imu_dcm_kp'),0)
        for log in ('# set imu_dcm_kp = 0\nimu_dcm_kp set to 0\n',
                    '# get imu_dcm_kp\nimu_dcm_kp set to 0\n',
                    '# get imu_dcm_ki\nimu_dcm_ki = 0\n',
                    '# get imu_dcm_kp\nimu_dcm_kp = 0\nimu_dcm_kp = 1\n',
                    good+'# set imu_dcm_kp = 1\nimu_dcm_kp set to 1\n'):
            with self.subTest(log=log),self.assertRaises(IdentityError):
                stress.native_integer(log,'imu_dcm_kp')

    def test_absolute_receiver_boundaries_cover_the_complete_long_program(self):
        for case in stress.stress_cases():
            receiver=ExperimentReceiver.__new__(ExperimentReceiver)
            receiver.plan={'case':case['name'],'stages':case['stages'],'epoch':100.}
            elapsed=0.
            for stage in case['stages']:
                for age in (elapsed+1e-7,elapsed+stage['duration_s']-1e-7):
                    marker,channels=receiver.planned_input(100+age)
                    self.assertEqual(marker,(case['name'],stage['name']))
                    self.assertEqual(channels,stage['channels'])
                elapsed+=stage['duration_s']
            marker,channels=receiver.planned_input(100+elapsed+1e-7)
            self.assertEqual(marker,(case['name'],'complete'))
            self.assertEqual(channels,case['stages'][-1]['channels'])

    def test_partial_program_keeps_only_observed_stage_boundaries(self):
        case=next(case for case in stress.stress_cases() if case['name']=='forward_70_cw')
        trial=stress.StressTrial.__new__(stress.StressTrial);trial.record=reference()
        seen={}
        elapsed=10.
        for stage in case['stages'][:3]:
            seen[(case['name'],stage['name'])]={'sim_time_s':elapsed,'wall_monotonic':20+elapsed}
            elapsed+=stage['duration_s']
        trial.receiver=SimpleNamespace(transitions=seen)
        actual=trial.actual_stages(case)
        self.assertEqual([stage['name'] for stage in actual],[stage['name'] for stage in case['stages'][:3]])
        self.assertNotIn('pulse',{stage['name'] for stage in actual})
        self.assertEqual(actual[0]['actual_start_sim_time_s'],10.)
        self.assertEqual(actual[1]['actual_start_sim_time_s'],11.)
        self.assertTrue(all('requested_yaw_rate_dps' in stage for stage in actual))

    def test_ground_extension_preserves_complete_aircraft_and_other_world_fields(self):
        source=stress.HERE/'gazebo/worlds/tricopter.sdf';original=source.read_bytes()
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);normal=root/'normal.sdf';extended=root/'extended.sdf'
            materialize_world(source,normal)
            expected=ET.parse(normal);record=stress.extended_world(source,extended);actual=ET.parse(extended)
            aircraft=lambda tree:ET.tostring(tree.find("./world/model[@name='tricopter']"))
            self.assertEqual(aircraft(expected),aircraft(actual))
            ground=expected.find("./world/model[@name='ground']")
            collision_size=ground.find('./link/collision/geometry/plane/size')
            self.assertEqual(collision_size.text,'100 100');collision_size.text='20000 20000'
            actual_ground_link=actual.find("./world/model[@name='ground']/link")
            visuals=actual_ground_link.findall('visual')
            self.assertEqual(len(visuals),1)
            visual=visuals[0]
            self.assertFalse(visual.findall('.//box'))
            self.assertIsNone(visual.find('material'))
            self.assertEqual(visual.findtext('cast_shadows'),'false')
            uri=Path(visual.findtext('./geometry/mesh/uri'))
            self.assertTrue(uri.is_file());self.assertTrue(uri.with_suffix('.png').is_file())
            self.assertEqual(record['ground_mesh_sha256'],hashlib.sha256(uri.read_bytes()).hexdigest())
            # Restore only the declared visual changes before comparing every
            # remaining world field, including collisions, inertia and physics.
            expected_visual=ground.find('./link/visual')
            actual_ground_link.remove(visual)
            actual_ground_link.append(copy.deepcopy(expected_visual))
            actual.getroot().find('./world/scene').remove(actual.find('./world/scene/grid'))
            broadcaster=actual.find("./world/plugin[@name='gz::sim::systems::SceneBroadcaster']")
            for name in ('state_hertz','dynamic_pose_hertz'):
                setting=broadcaster.find(name);self.assertEqual(setting.text,'240');broadcaster.remove(setting)
            self.assertEqual(ET.tostring(expected.getroot()),ET.tostring(actual.getroot()))
            self.assertTrue(record['aircraft_subtree_unchanged'])
            self.assertEqual(record['source_sha256'],hashlib.sha256(original).hexdigest())
            self.assertEqual(record['extended_sha256'],hashlib.sha256(extended.read_bytes()).hexdigest())
        self.assertEqual(source.read_bytes(),original)

    def test_high_pitch_video_world_rate_uses_the_full_body_transform(self):
        roll,pitch=map(math.radians,(20,75));omega=math.radians(60)
        state={'roll_deg':20.,'pitch_deg':75.,'sim_time_s':20.,'z_m':50.,
            'vx_m_s':130.,'vy_m_s':0.,'vz_m_s':0.,'servo_pwm_us':1460.,'joint_angle_deg':-5.7,
            'gyro_frd_x':omega*math.sin(pitch),
            'gyro_frd_y':omega*math.sin(roll)*math.cos(pitch),
            'gyro_frd_z':omega*math.cos(roll)*math.cos(pitch)}
        trial=stress.StressTrial.__new__(stress.StressTrial)
        trial.video=Mock();trial.next_video=0.;trial.variant='stock'
        trial.record={'profile_name':'hover_tuned'};trial.protocol={'cases':[{'name':'forward_70_cw'}]}
        trial.receiver=SimpleNamespace(sent=[{'stage':'pulse','channels':[1440,1986,1715,1368]}])
        with patch.object(Trial,'health',return_value=state):self.assertIs(trial.health(),state)
        text=trial.video.update.call_args.args[0]
        rate=re.search(r'world[^:\n]*:\s*([+-]?[0-9.]+) deg/s',text)
        self.assertIsNotNone(rate,text)
        self.assertAlmostEqual(float(rate[1]),-60.,places=1)

    def test_native_angle_limit_must_match_declared_high_pitch_profile(self):
        for limit in (30,80):
            with tempfile.TemporaryDirectory() as temporary:
                trial=stress.StressTrial.__new__(stress.StressTrial)
                trial.record=reference();advanced=bytearray.fromhex(trial.record['common_msp_readback']['94'])
                advanced[17]=limit;trial.record['common_msp_readback']['94']=advanced.hex()
                trial.directory=Path(temporary);trial.profile=stress.PROFILES['hover_tuned']
                trial.protocol={}
                (trial.directory/'provision.log').write_text('# get angle_earth_ref\nangle_earth_ref = 100\n')
                with patch.object(stress,'verify_profile',return_value={'pass':True}), \
                     patch.object(Trial,'recapture',return_value='ready') as base:
                    if limit==80:
                        self.assertEqual(trial.recapture(first=True),'ready');base.assert_called_once_with(True)
                    else:
                        with self.assertRaisesRegex(IdentityError,'ANGLE limit'):trial.recapture(first=True)
                        base.assert_not_called()

    def test_gyro_only_diagnostic_requires_both_native_feedback_gains_zero(self):
        for ki in (0,1):
            with tempfile.TemporaryDirectory() as temporary:
                trial=stress.StressTrial.__new__(stress.StressTrial)
                trial.record=reference();trial.directory=Path(temporary)
                trial.profile=stress.PROFILES['hover_tuned'];trial.protocol={'gyro_only_diagnostic':True}
                (trial.directory/'provision.log').write_text(
                    '# get angle_earth_ref\nangle_earth_ref = 100\n'
                    '# get imu_dcm_kp\nimu_dcm_kp = 0\n'
                    f'# get imu_dcm_ki\nimu_dcm_ki = {ki}\n')
                with patch.object(stress,'verify_profile',return_value={'pass':True}), \
                     patch.object(Trial,'recapture',return_value='ready') as base:
                    if ki==0:
                        self.assertEqual(trial.recapture(first=True),'ready')
                        self.assertEqual(trial.record['ahrs_configuration']['native_readback'],{'imu_dcm_kp':0,'imu_dcm_ki':0})
                        base.assert_called_once_with(True)
                    else:
                        with self.assertRaisesRegex(IdentityError,'AHRS readback mismatch'):trial.recapture(first=True)
                        base.assert_not_called()

    def test_retained_plugin_mutation_aborts_the_campaign_guard(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);world=root/'world.sdf';world.write_text('unchanged world\n')
            plugin=root/'plugin.so';plugin.write_bytes(b'original plugin')
            campaign=stress.Campaign.__new__(stress.Campaign)
            campaign.args=SimpleNamespace(video=False)
            campaign.directory=root;campaign.identity=Mock();campaign.frozen={};campaign.world=world
            campaign.protocol={'world_change':{'extended_sha256':hashlib.sha256(world.read_bytes()).hexdigest()}}
            campaign.plugin=plugin;campaign.summary={'plugin_sha256':hashlib.sha256(plugin.read_bytes()).hexdigest()}
            with patch.object(stress,'verify_frozen_sources'):
                campaign.guard()
                plugin.write_bytes(b'changed plugin')
                with self.assertRaisesRegex(IdentityError,'Retained plugin changed'):campaign.guard()

    def test_optional_video_helper_is_frozen_only_for_video_campaigns(self):
        headless=stress.inputs(False);recorded=stress.inputs(True)
        self.assertNotIn('record_flight_video.py',headless)
        self.assertEqual(set(recorded)-set(headless),{'record_flight_video.py'})
        self.assertEqual({key:recorded[key] for key in headless},headless)
        self.assertEqual(recorded['record_flight_video.py'],stress.HERE/'record_flight_video.py')


if __name__=='__main__':unittest.main()
