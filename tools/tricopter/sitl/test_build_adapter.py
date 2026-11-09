"""Isolated source adapters must not silently replace pristine builds."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import build
import run


class AdapterChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.preimages = {name:subprocess.check_output(['git','-C',str(build.ROOT),'show',build.STOCK_REF+':'+name])
                         for name in build.ADAPTER_PREIMAGES}

    def source(self, directory):
        directory.mkdir()
        for name,data in self.preimages.items():
            path=directory/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
        other=directory/'src/main/flight/untouched.c';other.write_text('unchanged fixture\n');other.chmod(0o755)
        (directory/'relative-link').symlink_to('src/main/flight/untouched.c')
        return directory

    def test_real_patch_changes_exact_four_files_with_pinned_postimages(self):
        with tempfile.TemporaryDirectory() as temporary:
            source=self.source(Path(temporary)/'source');before=build.source_tree(source)
            result=build.apply_adapter(source,build.adapter_definition('corrected-frame'))
            self.assertEqual(result['changed_paths'],sorted(build.ADAPTER_PREIMAGES))
            self.assertEqual(result['applied_postimage_sha256'],build.ADAPTER_POSTIMAGES)
            after=build.source_tree(source)
            for name in set(before)-set(build.ADAPTER_PREIMAGES):self.assertEqual(before[name],after[name])

    def test_wrong_preimage_is_rejected_before_any_mutation(self):
        with tempfile.TemporaryDirectory() as temporary:
            source=self.source(Path(temporary)/'source');path=source/'src/main/flight/pid.c'
            path.write_bytes(path.read_bytes()+b'\n/* changed */\n');before=build.source_tree(source)
            with self.assertRaisesRegex(RuntimeError,'preimage mismatch'):
                build.apply_adapter(source,build.adapter_definition('corrected-frame'))
            self.assertEqual(before,build.source_tree(source))

    def test_git_apply_preserves_644_and_664_inputs_under_group_writable_umask(self):
        previous=os.umask(0o002)
        try:
            for mode in (0o644,0o664):
                with self.subTest(mode=oct(mode)),tempfile.TemporaryDirectory() as temporary:
                    source=self.source(Path(temporary)/'source')
                    for name in build.ADAPTER_PREIMAGES:(source/name).chmod(mode)
                    result=build.apply_adapter(source,build.adapter_definition('corrected-frame'))
                    self.assertEqual(result['applied_postimage_sha256'],build.ADAPTER_POSTIMAGES)
                    for name in build.ADAPTER_PREIMAGES:
                        self.assertEqual((source/name).stat().st_mode&0o777,mode)
        finally:
            os.umask(previous)

    def test_unapproved_patch_paths_and_content_rejected(self):
        original=Path(build.adapter_definition('corrected-frame')['patch_file']).read_text()
        with tempfile.TemporaryDirectory() as temporary:
            patch=Path(temporary)/'candidate.patch'
            patch.write_text(original.replace('src/main/flight/pid.c','src/main/flight/mixer.c'))
            with self.assertRaisesRegex(RuntimeError,'allowlisted'):
                build.adapter_definition('corrected-frame',patch)
            patch.write_text(original+'\n')
            with self.assertRaisesRegex(RuntimeError,'exact bytes'):
                build.adapter_definition('corrected-frame',patch)

    def test_patch_identity_cannot_change_after_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=self.source(root/'source')
            patch=root/'copy.patch';patch.write_bytes(Path(build.adapter_definition('corrected-frame')['patch_file']).read_bytes())
            definition=build.adapter_definition('corrected-frame',patch);patch.write_bytes(patch.read_bytes()+b'\n')
            before=build.source_tree(source)
            with self.assertRaisesRegex(RuntimeError,'changed after'):
                build.apply_adapter(source,definition)
            self.assertEqual(before,build.source_tree(source))

    def test_snapshots_preserve_inputs_and_detect_edited_cached_copy(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=self.source(root/'source');before=build.source_tree(source)
            definition=build.adapter_definition('corrected-frame')
            copied,result=build.snapshot_source(source,root/'artifacts','stock',definition)
            self.assertEqual(before,build.source_tree(source))
            self.assertNotEqual(source,copied)
            self.assertEqual((copied/'relative-link').readlink(),Path('src/main/flight/untouched.c'))
            self.assertEqual((copied/'src/main/flight/untouched.c').stat().st_mode&0o777,0o755)
            reused,again=build.snapshot_source(source,root/'artifacts','stock',definition)
            self.assertEqual(copied,reused);self.assertEqual(result,again)
            (copied/'src/main/flight/untouched.c').write_text('edited cache\n')
            with self.assertRaisesRegex(RuntimeError,'snapshot changed'):
                build.snapshot_source(source,root/'artifacts','stock',definition)

    def test_same_patch_has_same_identity_for_stock_and_modified(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=self.source(root/'source');definition=build.adapter_definition('corrected-frame')
            first,a=build.snapshot_source(source,root/'artifacts','stock',definition)
            second,b=build.snapshot_source(source,root/'artifacts','modified',definition)
            self.assertNotEqual(first,second);self.assertEqual(a,b)
            self.assertEqual(build.source_tree(first),build.source_tree(second))

    def test_existing_build_modes_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);output=root/'stock';output.mkdir()
            manifest=output/'build-manifest.json';manifest.write_text('{}')
            build.validate_output_mode(root,'stock','none')
            with self.assertRaisesRegex(RuntimeError,'Refusing to overwrite'):
                build.validate_output_mode(root,'stock','corrected-frame')
            manifest.write_text(json.dumps({'provenance':{'sitl_adapter':{'name':'corrected-frame'}}}))
            build.validate_output_mode(root,'stock','corrected-frame')
            with self.assertRaisesRegex(RuntimeError,'Refusing to overwrite'):
                build.validate_output_mode(root,'stock','none')
            manifest.unlink();(output/'sitl-adapter.json').write_text('{"name":"corrected-frame"}')
            with self.assertRaisesRegex(RuntimeError,'unfinished'):
                build.validate_output_mode(root,'stock','none')

    def test_plain_failed_builds_still_allow_normal_rebuild(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);(root/'stock').mkdir();(root/'stock/build.log').write_text('old failed build')
            build.validate_output_mode(root,'stock','none')
            with self.assertRaisesRegex(RuntimeError,'Unrecognized'):
                build.validate_output_mode(root,'stock','corrected-frame')
        with self.assertRaisesRegex(RuntimeError,'separate corrected-frame'):
            build.validate_output_mode(build.DEFAULT_ARTIFACTS,'nonexistent-fixture-variant','corrected-frame')

    def test_pristine_claim_describes_only_the_verified_input(self):
        pristine={'verified_git_blobs':4750,'source_files_unchanged':True}
        adapted=build.adapted_provenance('stock',Path('/pristine'),pristine,{'name':'corrected-frame'})
        self.assertNotIn('source_files_unchanged',adapted)
        self.assertEqual(adapted['pristine_input_verification'],pristine)
        self.assertFalse(adapted['built_source_is_pristine_stock'])
        self.assertTrue(adapted['built_source_has_common_sitl_adapter'])
        self.assertEqual(pristine,{'verified_git_blobs':4750,'source_files_unchanged':True})


class BinaryManifestChecks(unittest.TestCase):
    def fixture(self,root,adapter=True):
        output=root/'modified';output.mkdir();source=root/'snapshot';source.mkdir();live=root/'live';live.mkdir()
        (source/'pid.c').write_text('adapted');(live/'pid.c').write_text('original')
        binary=output/'firmware';binary.write_bytes(b'ELF fixture');patch=root/'patch';patch.write_bytes(b'patch fixture')
        proof=root/'snapshot-manifest.json';proof.write_text('{"proof":true}')
        report={'binary':str(binary),'binary_sha256':build.digest(binary),'source':str(source),
                'source_sha256':{'pid.c':build.digest(source/'pid.c')},'provenance':{}}
        if adapter:
            report.update(live_source_root=str(live),live_source_sha256={'pid.c':build.digest(live/'pid.c')},
                          source_snapshot_manifest=str(proof),source_snapshot_manifest_sha256=build.digest(proof))
            report['provenance']['sitl_adapter']={'name':'corrected-frame','patch_file':str(patch),'patch_sha256':build.digest(patch)}
        (output/'build-manifest.json').write_text(json.dumps(report))
        return binary,source,live,patch,proof

    def test_plain_manifests_remain_supported(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);binary,*_=self.fixture(root,False)
            self.assertEqual(run.verified_binary(root,'modified'),binary)

    def test_adapter_proof_and_live_source_are_checked_before_use(self):
        for field in ('binary','source','live','patch','proof'):
            with self.subTest(field=field),tempfile.TemporaryDirectory() as temporary:
                root=Path(temporary);binary,source,live,patch,proof=self.fixture(root)
                self.assertEqual(run.verified_binary(root,'modified'),binary)
                changed={'binary':binary,'source':source/'pid.c','live':live/'pid.c','patch':patch,'proof':proof}[field]
                changed.write_bytes(changed.read_bytes()+b'changed')
                with self.assertRaises(RuntimeError):run.verified_binary(root,'modified')


if __name__=='__main__':unittest.main()
