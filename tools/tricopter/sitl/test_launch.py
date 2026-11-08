"""Regression checks for plant startup health and fair rotation comparisons."""
import copy
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import run


class LauncherChecks(unittest.TestCase):
    def test_reversal_changes_only_three_reaction_signs(self):
        with tempfile.TemporaryDirectory() as temporary:
            original = Path(temporary)/'original.sdf'
            reversed_world = Path(temporary)/'reversed.sdf'
            source = run.HERE/'gazebo/worlds/tricopter.sdf'
            run.materialize_world(source, original)
            run.materialize_world(source, reversed_world, True)
            first, second = ET.parse(original), ET.parse(reversed_world)
            first_sign = first.find('.//spin_sign')
            second_sign = second.find('.//spin_sign')
            self.assertEqual([int(x) for x in second_sign.text.split()], [-int(x) for x in first_sign.text.split()])
            second_sign.text = first_sign.text
            self.assertEqual(ET.tostring(first.getroot()), ET.tostring(second.getroot()))

    def test_wrong_model_rejected_before_launch(self):
        with tempfile.TemporaryDirectory() as temporary:
            source=Path(temporary)/'other.sdf';source.write_text('<sdf><world/></sdf>')
            with self.assertRaisesRegex(ValueError, 'no tricopter'):
                run.materialize_world(source, Path(temporary)/'out.sdf')

    def test_old_world_copy_receives_collisionless_camera_target(self):
        with tempfile.TemporaryDirectory() as temporary:
            tree = ET.parse(run.HERE/'gazebo/worlds/tricopter.sdf')
            world = tree.find('./world')
            world.remove(world.find("model[@name='tricopter_camera_heading']"))
            bridge = world.find("model/plugin[@name='tricopter::TricopterBridge']")
            bridge.remove(bridge.find('camera_heading_model'))
            source, output = Path(temporary)/'old.sdf', Path(temporary)/'new.sdf'
            tree.write(source)
            run.materialize_world(source, output)
            result = ET.parse(output)
            anchor = result.find("./world/model[@name='tricopter_camera_heading']")
            self.assertEqual(anchor.findtext('static'), 'false')
            self.assertEqual(anchor.findtext('link/kinematic'), 'true')
            self.assertEqual(anchor.findtext('link/gravity'), 'false')
            self.assertFalse(anchor.findall('.//collision'))
            self.assertEqual(result.findtext(
                "./world/model/plugin[@name='tricopter::TricopterBridge']/camera_heading_model"),
                'tricopter_camera_heading')

    def test_camera_target_cannot_alias_the_physical_aircraft(self):
        with tempfile.TemporaryDirectory() as temporary:
            tree = ET.parse(run.HERE/'gazebo/worlds/tricopter.sdf')
            tree.find("./world/model/plugin[@name='tricopter::TricopterBridge']/camera_heading_model").text='tricopter'
            source = Path(temporary)/'bad.sdf'; tree.write(source)
            with self.assertRaisesRegex(ValueError, 'separate empty'):
                run.materialize_world(source, Path(temporary)/'out.sdf')

    def test_no_plant_activity_fails_with_bounded_timeout(self):
        process=Mock();process.poll.return_value=None
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(RuntimeError, 'No valid actuator'):
                run.wait_for_plant(Path(temporary), process, process, Mock(), 'stock', timeout=0)

    def test_child_exit_is_not_mistaken_for_ready_plant(self):
        firmware=Mock();firmware.poll.return_value=None
        gazebo=Mock();gazebo.poll.return_value=1
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(RuntimeError, 'exited during startup'):
                run.wait_for_plant(Path(temporary), firmware, gazebo, Mock(), 'stock')

    def test_modified_start_requires_actual_feedback_in_addition_to_trace(self):
        process=Mock();process.poll.return_value=None
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)
            (directory/'plant.csv').write_text('sim_time_s,actuator_valid\n1.0,1\n')
            client=Mock()
            client.request.return_value=struct.pack('<8h',0,0,1509,0,0,0,1,0)
            with patch.object(run.time,'monotonic',side_effect=[0,0,1]), patch.object(run.time,'sleep'):
                with self.assertRaisesRegex(RuntimeError,'No valid actuator'):
                    run.wait_for_plant(directory,process,process,client,'modified',timeout=.5)
            client.request.return_value=struct.pack('<8h',0,0,1509,0,0,0,3,0)
            run.wait_for_plant(directory,process,process,client,'modified')


if __name__=='__main__':unittest.main()
