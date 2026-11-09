"""Exercise provisioning overlays before any SITL process or socket is opened."""
import hashlib
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import check_comparison


class ProvisionReached(Exception):
    pass


class TrialOverlay(unittest.TestCase):
    def fixture(self, directory):
        world=directory/'source.sdf'
        world.write_text('<sdf><world><model name="tricopter">'
            '<link><inertial><mass>0.5</mass></inertial></link>'
            '<plugin name="tricopter::TricopterBridge"><max_thrust_n>12 12 12</max_thrust_n></plugin>'
            '</model></world></sdf>')
        return world

    def test_default_retains_original_provision_call(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);world=self.fixture(root);binary=root/'firmware'
            trial=check_comparison.Trial(root/'trial','stock',root/'plugin',world,{})
            with patch.object(check_comparison,'materialize_world',side_effect=shutil.copyfile), \
                 patch.object(check_comparison,'verified_binary',return_value=binary), \
                 patch.object(check_comparison,'provision',side_effect=ProvisionReached) as provision:
                with self.assertRaises(ProvisionReached):trial.execute()
            provision.assert_called_once_with(binary,root/'trial','stock')
            self.assertFalse((root/'trial/requested-overlay.cli').exists())
            self.assertNotIn('requested_overlay_sha256',trial.record)

    def test_overlay_provisions_exact_retained_snapshot(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);world=self.fixture(root);binary=root/'firmware'
            source=root/'chosen yaw.cli'
            contents=b'profile 0\nset p_yaw = 67\nset f_yaw = 0\n'
            source.write_bytes(contents)
            trial=check_comparison.Trial(root/'trial','modified',root/'plugin',world,{},extra_config=source)
            def provision(binary_argument,directory,variant,*,extra_config):
                self.assertEqual((binary_argument,directory,variant),(binary,root/'trial','modified'))
                self.assertEqual(extra_config,root/'trial/requested-overlay.cli')
                source.write_text('changed after snapshot\n')
                self.assertEqual(extra_config.read_bytes(),contents)
                raise ProvisionReached
            with patch.object(check_comparison,'materialize_world',side_effect=shutil.copyfile), \
                 patch.object(check_comparison,'verified_binary',return_value=binary), \
                 patch.object(check_comparison,'provision',side_effect=provision):
                with self.assertRaises(ProvisionReached):trial.execute()
            self.assertEqual((root/'trial/requested-overlay.cli').read_bytes(),contents)
            self.assertEqual(trial.record['requested_overlay_path'],'requested-overlay.cli')
            self.assertEqual(trial.record['requested_overlay_source'],str(source.resolve()))
            self.assertEqual(trial.record['requested_overlay_sha256'],hashlib.sha256(contents).hexdigest())


if __name__=='__main__':unittest.main()
