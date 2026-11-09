"""Protect matched controls and absolute RC scheduling; no sockets or physics."""
import argparse
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from check_comparison import ExperimentReceiver, cases
from check_comparison_fresh import (EXECUTION_HELPERS, freeze_sources, hover_throttle,
                                    source_inputs, verify_frozen_sources)
from run import materialize_world


class ComparisonPlanTests(unittest.TestCase):
    def test_delayed_observation_does_not_accumulate_stage_overrun(self):
        receiver=ExperimentReceiver.__new__(ExperimentReceiver)
        definition=next(case for case in cases(1350) if case['name']=='yaw_cw_collective_up')
        receiver.plan={'case':definition['name'],'stages':definition['stages'],'epoch':None}
        epoch=123.456
        expected=[(0,'baseline',1350,1500),(1.025,'yaw_lead',1350,1368),
                  (1.815,'pulse',1375,1368),(2.205,'yaw_hold',1350,1368),
                  (2.805,'recovery',1350,1500),(4.005,'complete',1350,1500)]
        for elapsed,name,throttle,yaw in expected:
            marker,channels=receiver.planned_input(epoch+elapsed)
            self.assertEqual(marker,(definition['name'],name))
            self.assertEqual(channels,[1500,1500,throttle,yaw])
            self.assertEqual(receiver.plan['epoch'],epoch)

    def test_matched_controls_differ_only_in_collective_pulse(self):
        matrix={case['name']:case for case in cases(1350)}
        pairs={'collective_up':'sham','collective_down':'sham',
               'yaw_cw_collective_up':'yaw_cw','yaw_ccw_collective_up':'yaw_ccw'}
        for name,control in pairs.items():
            target,reference=matrix[name]['stages'],matrix[control]['stages']
            self.assertEqual(len(target),len(reference))
            differences=0
            for a,b in zip(target,reference):
                self.assertEqual((a['name'],a['duration_s']),(b['name'],b['duration_s']))
                self.assertEqual(a['channels'][:2]+a['channels'][3:],b['channels'][:2]+b['channels'][3:])
                if a['channels'][2]!=b['channels'][2]:
                    self.assertEqual(a['name'],'pulse');differences+=1
            self.assertEqual(differences,1)

    def test_configurable_collective_preserves_matched_waveforms(self):
        shifted=cases(hover_throttle('1400'))
        original=cases(1350)
        for before,after in zip(original,shifted):
            for a,b in zip(before['stages'],after['stages']):
                self.assertEqual(b['channels'],a['channels'][:2]+[a['channels'][2]+50]+a['channels'][3:])
                self.assertEqual(b['duration_s'],a['duration_s'])
        for invalid in ('1024','1976'):
            with self.assertRaises(argparse.ArgumentTypeError):hover_throttle(invalid)


class FrozenPlantInputsTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)
        self.here=self.root/'sitl';self.directory=self.root/'campaign'
        for name in EXECUTION_HELPERS:
            path=self.here/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(name)
        files={'gazebo/MotorModel.hh':'motor model',
               'gazebo/model/motor_performance.json':'{"model":"dc_prop"}',
               'gazebo/meshes/frame.stl':'solid frame\nendsolid frame\n',
               'gazebo/__pycache__/ignored.pyc':'ignored',
               'gazebo/worlds/tricopter.sdf':'''<sdf version="1.10"><world name="test">
                 <model name="tricopter"><link name="base_link"><visual name="frame">
                 <geometry><mesh><uri>../meshes/frame.stl</uri></mesh></geometry></visual></link>
                 <plugin name="tricopter::TricopterBridge" filename="libTricopterBridge.so">
                 <spin_sign>1 -1 1</spin_sign></plugin></model></world></sdf>'''}
        for name,value in files.items():
            path=self.here/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(value)
        (self.root/'Tricopter_motors.step').write_text('optional original CAD')

    def inputs(self):return source_inputs(self.here,self.root)

    def test_frozen_world_resolves_its_frozen_meshes_after_relocation(self):
        inputs=self.inputs();frozen=freeze_sources(self.directory,inputs)
        self.assertIn('gazebo/MotorModel.hh',frozen)
        self.assertIn('gazebo/model/motor_performance.json',frozen)
        self.assertIn('cad/Tricopter_motors.step',frozen)
        self.assertFalse(any('__pycache__' in name for name in frozen))
        world=self.directory/'materialized.sdf'
        materialize_world(self.directory/'source/gazebo/worlds/tricopter.sdf',world)
        mesh=Path(ET.parse(world).findtext('.//mesh/uri'))
        self.assertEqual(mesh,self.directory/'source/gazebo/meshes/frame.stl')
        self.assertEqual(mesh.read_bytes(),inputs['gazebo/meshes/frame.stl'].read_bytes())

    def test_source_asset_change_and_snapshot_change_are_both_rejected(self):
        frozen=freeze_sources(self.directory,self.inputs())
        mesh=self.here/'gazebo/meshes/frame.stl';original=mesh.read_bytes()
        mesh.write_text('changed asset')
        with self.assertRaisesRegex(AssertionError,'changed after protocol freeze'):
            verify_frozen_sources(self.directory,frozen,self.inputs())
        mesh.write_bytes(original)
        (self.directory/'source/gazebo/MotorModel.hh').write_text('changed snapshot')
        with self.assertRaisesRegex(AssertionError,'changed after protocol freeze'):
            verify_frozen_sources(self.directory,frozen,self.inputs())

    def test_added_model_input_is_rejected(self):
        frozen=freeze_sources(self.directory,self.inputs())
        (self.here/'gazebo/model/new-coefficient.json').write_text('{}')
        with self.assertRaisesRegex(AssertionError,'input set changed'):
            verify_frozen_sources(self.directory,frozen,self.inputs())


if __name__=='__main__':unittest.main()
