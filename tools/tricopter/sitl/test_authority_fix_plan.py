"""New-plant authority comparison protocol and provenance without simulation."""
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import check_authority_fix as authority
from autotune_yaw import IdentityError
from build import ADAPTER_PATCH_SHA256,ADAPTER_PREIMAGES,ADAPTER_POSTIMAGES


def arguments(**changes):
    result=dict(forward_lead=25.,forward_roll=1332,forward_pitch=1903,
                cases=','.join(case['name'] for case in authority.stress_cases()))
    result.update(changes)
    return SimpleNamespace(**result)


@contextmanager
def catalog_fixture():
    with tempfile.TemporaryDirectory(prefix='tricopter-authority-catalog-') as temporary:
        base=Path(temporary);before=base/'before';after=base/'after';metadata={}
        adapter={'name':'corrected-frame','patch_sha256':ADAPTER_PATCH_SHA256,
                 'allowed_preimage_sha256':dict(ADAPTER_PREIMAGES),
                 'applied_postimage_sha256':dict(ADAPTER_POSTIMAGES),
                 'changed_paths':sorted(ADAPTER_PREIMAGES)}
        common={**ADAPTER_POSTIMAGES,
                **{name:'a'*64 for name in authority.ALLOWED_CONTROLLER_CHANGES},
                'src/main/flight/pid.h':'b'*64,
                'src/main/flight/tricopter_servo_calibration.c':'c'*64}
        for label,directory,variant in (('stock',before,'stock'),('before',before,'modified'),('modified',after,'modified')):
            source=dict(common)
            if label=='modified':source.update({name:'d'*64 for name in authority.ALLOWED_CONTROLLER_CHANGES})
            metadata[label]={'source_sha256':source,'provenance':{'sitl_adapter':copy.deepcopy(adapter)}}
            path=directory/variant;path.mkdir(parents=True)
            (path/'betaflight_SITL.elf').write_bytes(label.encode())
        def save():
            for label,directory,variant in (('stock',before,'stock'),('before',before,'modified'),('modified',after,'modified')):
                (directory/variant/'build-manifest.json').write_text(json.dumps(metadata[label])+'\n')
        save()
        data=SimpleNamespace(before=before,after=after,metadata=metadata,save=save)
        with patch.object(authority,'verified_binary',side_effect=lambda directory,variant:directory/variant/'betaflight_SITL.elf'):
            yield data


class AuthorityFixPlan(unittest.TestCase):
    def test_cli_reports_failed_execution_with_nonzero_exit_without_discarding_campaign(self):
        original=authority.PILOT.copy()
        try:
            for completed in (True,False):
                with self.subTest(completed=completed),tempfile.TemporaryDirectory() as temporary:
                    directory=Path(temporary)
                    with patch('sys.argv',['check_authority_fix.py','--before-build',str(directory/'before')]), \
                         patch.object(authority,'fresh_run_directory',return_value=directory), \
                         patch.object(authority,'Campaign') as campaign,patch('builtins.print'):
                        campaign.return_value.summary={'pass':completed}
                        if completed:authority.main()
                        else:
                            with self.assertRaises(SystemExit) as exit_status:authority.main()
                            self.assertEqual(exit_status.exception.code,1)
                        campaign.return_value.run.assert_called_once_with()
        finally:
            authority.PILOT.clear();authority.PILOT.update(original)

    def test_new_forward_trim_changes_only_forward_roll_pitch_channels(self):
        original=authority.stress_cases(25.)
        changed=authority.definitions(arguments())
        self.assertEqual([case['name'] for case in changed],[case['name'] for case in original])
        directions=set()
        for old,new in zip(original,changed):
            if old['operating_point']!='steady_forward_70':
                self.assertEqual(old,new)
                continue
            directions.add(new['direction'])
            self.assertEqual(new['forward_trim_rc_us'],[1332,1903])
            self.assertEqual({k:v for k,v in new.items() if k not in ('stages','forward_trim_rc_us')},
                             {k:v for k,v in old.items() if k!='stages'})
            for a,b in zip(old['stages'],new['stages']):
                self.assertEqual(a['name'],b['name'])
                self.assertEqual(a['duration_s'],b['duration_s'])
                self.assertEqual(a['channels'][2:],b['channels'][2:])
                if b['name']=='baseline':self.assertEqual(a,b)
            self.assertEqual(new['stages'][20]['channels'][:3],[1332,1903,1715])
            self.assertTrue(all(stage['channels'][:3]==[1332,1903,1715] for stage in new['stages'][-3:]))
        self.assertEqual(directions,{'control','cw','ccw'})

    def test_hover_probes_have_common_throttle_and_opposite_yaw_only(self):
        cases=authority.definitions(arguments(cases='hover_cw,hover_ccw'))
        self.assertEqual([case['direction'] for case in cases],['cw','ccw'])
        for case,yaw in zip(cases,(1368,1632)):
            self.assertEqual(case['operating_point'],'hover')
            self.assertEqual([stage['duration_s'] for stage in case['stages']],[1.,2.,2.])
            self.assertEqual(case['stages'][1]['channels'],[1500,1500,1345,yaw])
        self.assertEqual(cases[0]['stages'][0],cases[1]['stages'][0])
        self.assertEqual(cases[0]['stages'][2],cases[1]['stages'][2])

    def test_rejects_unknown_or_duplicate_case_selection(self):
        for selected in ('hold_95_control,hold_95_control','missing_case'):
            with self.subTest(selected=selected),self.assertRaises(ValueError):
                authority.definitions(arguments(cases=selected))

    def test_signed_trim_get_requires_a_native_final_unambiguous_readback(self):
        query='\x1b[0m# get tri_yaw_trim\r\ntri_yaw_trim = -61\r\nAllowed range: -478 - 216\r\n'
        self.assertEqual(authority.native_integer(query,'tri_yaw_trim'),-61)
        self.assertEqual(authority.native_integer('# get tri_yaw_trim\ntri_yaw_trim = 61\n','tri_yaw_trim'),61)
        self.assertEqual(authority.native_integer('# get tri_yaw_trim\ntri_yaw_trim = 0\n','tri_yaw_trim'),0)
        invalid=(query+'# set tri_yaw_trim = -60\ntri_yaw_trim set to -60\n',
                 '# set tri_yaw_trim = -61\ntri_yaw_trim set to -61\n',
                 '# get tri_yaw_trim\ntri_yaw_trim = -61\ntri_yaw_trim = 61\n',
                 '# get tri_yaw_trim\ntri_yaw_trim = -61.0\n')
        for log in invalid:
            with self.subTest(log=log),self.assertRaises(IdentityError):authority.native_integer(log,'tri_yaw_trim')

    def test_catalog_accepts_the_exact_common_adapter_and_declared_native_fix(self):
        with catalog_fixture() as data:
            catalog,changed=authority.controller_catalog(data.before,data.after)
            self.assertEqual(set(catalog),{'stock','before','modified'})
            self.assertEqual(set(changed),authority.ALLOWED_CONTROLLER_CHANGES)
            self.assertEqual(catalog['stock']['variant'],'stock')
            self.assertEqual(catalog['before']['variant'],'modified')
            self.assertEqual(catalog['modified']['variant'],'modified')
            self.assertEqual(catalog['before']['artifacts'],str(data.before))
            for label,item in catalog.items():
                self.assertEqual(item['metadata'],data.metadata[label])
                binary=Path(item['artifacts'])/item['variant']/'betaflight_SITL.elf'
                self.assertEqual(item['binary_sha256'],hashlib.sha256(binary.read_bytes()).hexdigest())

    def test_catalog_rejects_native_pid_or_servo_calibration_drift(self):
        for path in ('src/main/flight/pid.h','src/main/flight/tricopter_servo_calibration.c'):
            with self.subTest(path=path),catalog_fixture() as data:
                data.metadata['modified']['source_sha256'][path]='e'*64;data.save()
                with self.assertRaises(IdentityError):authority.controller_catalog(data.before,data.after)

    def test_catalog_rejects_no_native_fix(self):
        with catalog_fixture() as data:
            data.metadata['modified']['source_sha256']=dict(data.metadata['before']['source_sha256']);data.save()
            with self.assertRaises(IdentityError):authority.controller_catalog(data.before,data.after)

    def test_catalog_rejects_adapter_map_or_applied_source_drift_for_each_controller(self):
        mutations=(lambda a:a.update(name='other'),lambda a:a.update(patch_sha256='0'*64),
                   lambda a:a['allowed_preimage_sha256'].update({'src/main/flight/pid.c':'0'*64}),
                   lambda a:a['applied_postimage_sha256'].update({'src/main/flight/pid.c':'0'*64}),
                   lambda a:a.update(changed_paths=sorted(ADAPTER_PREIMAGES)+['src/main/flight/pid.h']))
        for label in ('stock','before','modified'):
            for mutation in mutations:
                with self.subTest(label=label,mutation=mutation),catalog_fixture() as data:
                    mutation(data.metadata[label]['provenance']['sitl_adapter']);data.save()
                    with self.assertRaises(IdentityError):authority.controller_catalog(data.before,data.after)
            with self.subTest(label=label,source='actual postimage'),catalog_fixture() as data:
                data.metadata[label]['source_sha256']['src/main/flight/pid.c']='0'*64;data.save()
                with self.assertRaises(IdentityError):authority.controller_catalog(data.before,data.after)

    def test_elevated_setup_applies_to_all_forward_controllers_and_restores_ascent(self):
        for controller,variant in (('stock','stock'),('before','modified'),('modified','modified')):
            for forward in (True,False):
                with self.subTest(controller=controller,forward=forward),tempfile.TemporaryDirectory() as temporary:
                    trial=authority.AuthorityTrial.__new__(authority.AuthorityTrial)
                    trial.variant=variant;trial.directory=Path(temporary);trial.record={};trial.target=1.5
                    trial.pilot=SimpleNamespace(origin=[1.,2.,.125])
                    trial.protocol={'trim_deci_deg':-61,'forward_start_height_m':20.,'cases':[{'operating_point':'steady_forward_70' if forward else 'high_collective_hold'}]}
                    (trial.directory/'provision.log').write_text(
                        '# get gyro_cal_on_first_arm\ngyro_cal_on_first_arm = ON\n'
                        '# get tri_yaw_trim\ntri_yaw_trim = -61\n')
                    original=authority.PILOT.copy();seen=[]
                    def recapture(instance,first=False):
                        seen.append((instance.target,authority.PILOT['ascent_m_s'],first));return 'ready'
                    with patch.object(authority.StressTrial,'recapture',new=recapture):self.assertEqual(trial.recapture(True),'ready')
                    self.assertEqual(seen,[(20.125 if forward else 1.5,2. if forward else original['ascent_m_s'],True)])
                    self.assertEqual(authority.PILOT,original)
                    self.assertEqual('forward_setup' in trial.record,forward)

    def test_setup_restores_ascent_after_gate_failure_and_does_not_move_later_recaptures(self):
        with tempfile.TemporaryDirectory() as temporary:
            trial=authority.AuthorityTrial.__new__(authority.AuthorityTrial)
            trial.variant='stock';trial.directory=Path(temporary);trial.record={};trial.target=1.5
            trial.pilot=SimpleNamespace(origin=[0.,0.,.125]);trial.protocol={'forward_start_height_m':20.,'cases':[{'operating_point':'steady_forward_70'}]}
            (trial.directory/'provision.log').write_text('# get gyro_cal_on_first_arm\ngyro_cal_on_first_arm = ON\n')
            original=authority.PILOT.copy()
            with patch.object(authority.StressTrial,'recapture',side_effect=TimeoutError('gate')):
                with self.assertRaises(TimeoutError):trial.recapture(True)
            self.assertEqual(authority.PILOT,original)
            trial.target=2.5
            with patch.object(authority.StressTrial,'recapture',return_value='later'):self.assertEqual(trial.recapture(False),'later')
            self.assertEqual(trial.target,2.5)
            self.assertEqual(authority.PILOT,original)

    def test_first_arm_gyro_calibration_requires_native_on_and_no_later_setting(self):
        on='# get gyro_cal_on_first_arm\ngyro_cal_on_first_arm = ON\n'
        trim='# get tri_yaw_trim\ntri_yaw_trim = -61\n'
        invalid=(
            '# get gyro_cal_on_first_arm\ngyro_cal_on_first_arm = OFF\n',
            '# set gyro_cal_on_first_arm = ON\ngyro_cal_on_first_arm set to ON\n',
            '',
            on+'# set gyro_cal_on_first_arm = OFF\ngyro_cal_on_first_arm set to OFF\n',
            '# get gyro_cal_on_first_arm\ngyro_cal_on_first_arm = ON\ngyro_cal_on_first_arm = ON\n')
        for variant in ('stock','modified'):
            for log in (on,*invalid):
                with self.subTest(variant=variant,log=log),tempfile.TemporaryDirectory() as temporary:
                    trial=authority.AuthorityTrial.__new__(authority.AuthorityTrial)
                    trial.variant=variant;trial.directory=Path(temporary);trial.record={};trial.target=1.5
                    trial.pilot=SimpleNamespace(origin=[0.,0.,.125])
                    trial.protocol={'trim_deci_deg':-61,'cases':[{'operating_point':'hover'}]}
                    retained=log+trim;(trial.directory/'provision.log').write_text(retained)
                    original=authority.PILOT.copy()
                    with patch.object(authority.StressTrial,'recapture',return_value='ready') as base:
                        if log==on:
                            self.assertEqual(trial.recapture(True),'ready')
                            self.assertIn('gyro_cal_on_first_arm=ON',trial.record['gyro_calibration_policy'])
                            base.assert_called_once_with(True)
                        else:
                            with self.assertRaises(IdentityError):trial.recapture(True)
                            base.assert_not_called()
                    self.assertEqual((trial.directory/'provision.log').read_text(),retained)
                    self.assertEqual(authority.PILOT,original)


if __name__=='__main__':unittest.main()
