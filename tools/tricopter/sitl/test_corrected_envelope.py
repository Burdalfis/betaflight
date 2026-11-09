"""A changed comparison binary requires the exact common frame adapter.

Only tiny manifests/binaries and physical-input placeholders are materialized.
Native manifest JSON, profile selection and hash-policy checks remain real;
source snapshotting and binary verification are mocked at their I/O boundary.
"""
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import check_envelope as envelope
from autotune_yaw import IdentityError
from build import ADAPTER_PATCH_SHA256, ADAPTER_POSTIMAGES, ADAPTER_PREIMAGES


def digest(data): return hashlib.sha256(data).hexdigest()
def write(path, value): path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value)+'\n')


@contextmanager
def fixture():
    with tempfile.TemporaryDirectory(prefix='tricopter-envelope-identity-') as temporary:
        base=Path(temporary); original=base/'original'; adapted=base/'adapted'; reference=base/'reference'
        output=base/'run';output.mkdir()
        gains={variant:{'p':31,'i':20,'d':10} for variant in ('stock','modified')}
        metadata={};old_metadata={};records=[]
        for variant in ('stock','modified'):
            old_bytes=('original-'+variant).encode();new_bytes=('adapted-'+variant).encode()
            sources=dict(ADAPTER_PREIMAGES)
            sources.update({'src/main/flight/mixer_tricopter.c':digest(('mixer-'+variant).encode()),
                            'src/main/config/config.c':digest(b'common config'),
                            'mk/source.mk':digest(b'common source list')})
            old_metadata[variant]={'binary_sha256':digest(old_bytes),'source_sha256':sources}
            adapter={'name':'corrected-frame','patch_sha256':ADAPTER_PATCH_SHA256,
                'allowed_preimage_sha256':dict(ADAPTER_PREIMAGES),
                'applied_postimage_sha256':dict(ADAPTER_POSTIMAGES),
                'changed_paths':sorted(ADAPTER_PREIMAGES),
                'unadapted_tree_sha256':digest(('before-'+variant).encode()),
                'adapted_tree_sha256':digest(('after-'+variant).encode())}
            metadata[variant]={'binary_sha256':digest(new_bytes),
                'source_sha256':{**sources,**ADAPTER_POSTIMAGES},
                'provenance':{'sitl_adapter':adapter}}
            for directory,data,manifest in ((original,old_bytes,old_metadata[variant]),
                                            (adapted,new_bytes,metadata[variant])):
                target=directory/variant;target.mkdir(parents=True)
                (target/'betaflight_SITL.elf').write_bytes(data)
                write(target/'build-manifest.json',manifest)
            records.append({'variant':variant,'binary_sha256':digest(old_bytes),
                'common_msp_readback':{
                    '111':'07004343430032000000000707000064ce07ce07ce070332',
                    '94':'00000000000000000000000000000000001e00000050000000000001010005002f003100000000282e00002300c80f64000001410f5a07000001414605'}})
        physical={}
        for name in ('gazebo/worlds/tricopter.sdf','configs/common.cli','cad/aircraft.step'):
            path=base/'physical'/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(name+' fixture\n')
            physical[name]=path
        selection={'pass':True,'selected_gains':gains}
        old_summary={'pass':True,'runs':records,
            'protocol':{'source_sha256':{name:digest(path.read_bytes()) for name,path in physical.items()},
                        'initial_gate':{'continuous_duration_s':.5}}}
        write(reference/'autotune-summary.json',selection)
        write(reference/'tuned-evaluation/comparison-summary.json',old_summary)
        result=SimpleNamespace(base=base,original=original,adapted=adapted,reference=reference,output=output,
            gains=gains,old_metadata=old_metadata,metadata=metadata,physical=physical,
            selection=selection,old_summary=old_summary)
        def save(variant): write(adapted/variant/'build-manifest.json',metadata[variant])
        result.save=save
        yield result


def construct(data, artifacts='adapted'):
    chosen=data.adapted if artifacts=='adapted' else None
    with patch.object(envelope,'ARTIFACTS',data.original), \
         patch.object(envelope,'inputs',return_value=data.physical), \
         patch.object(envelope,'freeze_sources',return_value={'fixture':'frozen'}) as freeze, \
         patch.object(envelope,'verified_binary',side_effect=lambda directory,variant:directory/variant/'betaflight_SITL.elf') as binary:
        campaign=envelope.Campaign(data.reference,data.output,3,None,artifacts=chosen)
    return campaign,freeze,binary


class CorrectedEnvelopeIdentity(unittest.TestCase):
    def test_longer_forward_lead_changes_only_forward_operating_duration(self):
        with fixture() as data:
            reference=data.old_summary['runs'][0]
            before=envelope.cases(reference)
            changed=envelope.cases(reference,forward_lead=6.)
            directions=set()
            self.assertEqual(len(before),len(changed))
            for original,revised in zip(before,changed):
                if original['operating_point']!='forward':
                    self.assertEqual(original,revised)
                    continue
                directions.add(revised['direction'])
                self.assertEqual({key:value for key,value in original.items() if key!='stages'},
                                 {key:value for key,value in revised.items() if key!='stages'})
                for a,b in zip(original['stages'],revised['stages']):
                    self.assertEqual(a['name'],b['name'])
                    self.assertEqual(a['channels'],b['channels'])
                    if a['name']=='operating_lead':
                        self.assertEqual(a['duration_s'],3.)
                        self.assertEqual(b['duration_s'],6.)
                    else:
                        self.assertEqual(a,b)
            self.assertEqual(directions,{'control','cw','ccw'})

    def test_forward_lead_requires_a_finite_bounded_number(self):
        with fixture() as data:
            reference=data.old_summary['runs'][0]
            for value in (float('nan'),float('inf'),-float('inf'),True,False,2.99,8.01,None,'6'):
                with self.subTest(value=value),self.assertRaisesRegex(ValueError,'finite and in 3..8'):
                    envelope.cases(reference,forward_lead=value)
            for value in (3,6,8.):
                definitions=envelope.cases(reference,forward_lead=value)
                for case in definitions:
                    if case['operating_point']=='forward':
                        self.assertEqual(case['stages'][1]['duration_s'],value)

    def test_higher_climb_preset_changes_only_operating_climb_collective(self):
        with fixture() as data:
            reference=data.old_summary['runs'][0]
            before=envelope.cases(reference)
            changed=envelope.cases(reference,climb_throttle=1450)
            self.assertEqual(len(before),len(changed))
            climb_directions=set()
            for original,revised in zip(before,changed):
                self.assertEqual(original['name'],revised['name'])
                if original['operating_point']!='climb':
                    self.assertEqual(original,revised)
                    continue
                climb_directions.add(revised['direction'])
                self.assertEqual(original['stages'][0],revised['stages'][0])
                self.assertEqual({key:value for key,value in original.items() if key!='stages'},
                                 {key:value for key,value in revised.items() if key!='stages'})
                for a,b in zip(original['stages'][1:],revised['stages'][1:]):
                    self.assertEqual(a['name'],b['name'])
                    self.assertEqual(a['duration_s'],b['duration_s'])
                    self.assertEqual(a['channels'][2],1400)
                    self.assertEqual(b['channels'][2],1450)
                    self.assertEqual([a['channels'][i] for i in (0,1,3)],
                                     [b['channels'][i] for i in (0,1,3)])
            self.assertEqual(climb_directions,{'control','cw','ccw'})

    def test_climb_collective_requires_a_bounded_integer(self):
        with fixture() as data:
            reference=data.old_summary['runs'][0]
            for value in (1345,1501,-1,1400.,1450.,True,False,None,'1450'):
                with self.subTest(value=value),self.assertRaisesRegex(ValueError,'integer in 1346..1500'):
                    envelope.cases(reference,climb_throttle=value)
            for value in (1346,1500):
                definitions=envelope.cases(reference,climb_throttle=value)
                for case in definitions:
                    if case['operating_point']=='climb':
                        self.assertTrue(all(stage['channels'][2]==value for stage in case['stages'][1:]))

    def test_accepts_identical_common_adapter_and_keeps_selected_hover_gains(self):
        with fixture() as data:
            original_manifests={variant:(data.original/variant/'build-manifest.json').read_bytes() for variant in ('stock','modified')}
            campaign,freeze,binary=construct(data)
            self.assertEqual(campaign.gains,data.gains)
            self.assertEqual(campaign.protocol['requested_yaw_gains'],data.gains)
            self.assertTrue(campaign.protocol['gains_frozen_no_retuning'])
            self.assertIn('common SITL coordinate correction',campaign.protocol['stock_label'])
            self.assertEqual([call.args for call in binary.call_args_list],[(data.adapted,'stock'),(data.adapted,'modified')])
            freeze.assert_called_once_with(data.output,data.physical)
            for variant in ('stock','modified'):
                identity=campaign.binary_identities[variant]
                self.assertEqual(identity['binary_sha256'],data.metadata[variant]['binary_sha256'])
                self.assertEqual(identity['original_hover_binary_sha256'],data.old_metadata[variant]['binary_sha256'])
                retained=data.output/'firmware'/variant
                self.assertEqual((retained/'betaflight_SITL.elf').read_bytes(),(data.adapted/variant/'betaflight_SITL.elf').read_bytes())
                self.assertEqual((retained/'build-manifest.json').read_bytes(),(data.adapted/variant/'build-manifest.json').read_bytes())
                self.assertEqual((data.original/variant/'build-manifest.json').read_bytes(),original_manifests[variant])

    def test_original_identical_binaries_need_no_adapter_declaration(self):
        with fixture() as data:
            campaign,_,_=construct(data,artifacts='original')
            self.assertEqual(campaign.protocol['stock_label'],'unmodified stock')
            self.assertTrue(all(value['sitl_adapter'] is None for value in campaign.binary_identities.values()))

    def test_rejects_unapproved_patch_preimage_postimage_or_path_map(self):
        mutations=(
            lambda a:a.update(patch_sha256='0'*64),
            lambda a:a['allowed_preimage_sha256'].update({'src/main/flight/pid.c':'0'*64}),
            lambda a:a['applied_postimage_sha256'].update({'src/main/flight/pid.c':'0'*64}),
            lambda a:a.update(changed_paths=sorted(ADAPTER_PREIMAGES)+['src/main/flight/mixer.c']))
        for mutate in mutations:
            with self.subTest(mutation=mutate),fixture() as data:
                mutate(data.metadata['modified']['provenance']['sitl_adapter']);data.save('modified')
                with self.assertRaisesRegex(IdentityError,'four approved changes'):construct(data)

    def test_rejects_unknown_adapter_name_and_one_sided_adapter(self):
        with fixture() as data:
            data.metadata['modified']['provenance']['sitl_adapter']['name']='another-adapter';data.save('modified')
            with self.assertRaisesRegex(IdentityError,'Unrecognized common SITL correction'):construct(data)
        with fixture() as data:
            data.metadata['modified']=copy.deepcopy(data.old_metadata['modified'])
            original=(data.original/'modified/betaflight_SITL.elf').read_bytes()
            (data.adapted/'modified/betaflight_SITL.elf').write_bytes(original);data.save('modified')
            with self.assertRaisesRegex(IdentityError,'same common SITL correction'):construct(data)

    def test_rejects_unrelated_flight_source_change(self):
        with fixture() as data:
            data.metadata['modified']['source_sha256']['src/main/flight/mixer_tricopter.c']='0'*64
            data.save('modified')
            with self.assertRaisesRegex(IdentityError,'outside the shared SITL correction'):construct(data)

    def test_rejects_critical_source_input_set_or_adapter_hash_drift(self):
        with fixture() as data:
            data.metadata['stock']['source_sha256']['src/main/flight/new_controller.c']='0'*64;data.save('stock')
            with self.assertRaisesRegex(IdentityError,'source input set changed'):construct(data)
        with fixture() as data:
            data.metadata['stock']['source_sha256']['src/main/flight/pid.c']='0'*64;data.save('stock')
            with self.assertRaisesRegex(IdentityError,'adapter pre/postimage'):construct(data)

    def test_rejects_undeclared_binary_change(self):
        with fixture() as data:
            data.metadata['stock']['provenance'].pop('sitl_adapter');data.save('stock')
            with self.assertRaisesRegex(IdentityError,'Undeclared firmware binary change'):construct(data)

    def test_rejects_changed_original_hover_binary_provenance(self):
        with fixture() as data:
            data.old_metadata['stock']['binary_sha256']='0'*64
            write(data.original/'stock/build-manifest.json',data.old_metadata['stock'])
            with self.assertRaisesRegex(IdentityError,'hover binary provenance changed'):construct(data)

    def test_rejects_changed_physical_inputs_and_missing_profile(self):
        with fixture() as data:
            data.physical['gazebo/worlds/tricopter.sdf'].write_text('different model\n')
            with self.assertRaisesRegex(AssertionError,'Hover plant/config changed'):construct(data)
        with fixture() as data:
            data.selection['selected_gains'].pop('modified');write(data.reference/'autotune-summary.json',data.selection)
            with self.assertRaisesRegex(ValueError,'Both selected profiles required'):construct(data)

    def test_manifest_mutation_after_protocol_freeze_is_fatal(self):
        with fixture() as data:
            campaign,_,_=construct(data)
            path=data.adapted/'modified/build-manifest.json';path.write_text(path.read_text()+'\n')
            with patch.object(envelope,'verify_frozen_sources'),patch.object(envelope,'inputs',return_value=data.physical):
                with self.assertRaisesRegex(IdentityError,'manifest changed during comparison'):campaign.guard()


if __name__=='__main__':unittest.main()
