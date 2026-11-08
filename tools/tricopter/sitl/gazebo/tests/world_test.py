#!/usr/bin/env python3
"""Check articulated model topology / calibration consistency without Gazebo."""
import math
import sys
import xml.etree.ElementTree as ET

world = ET.parse(sys.argv[1]).getroot().find('world')
assert world is not None
model = world.find("model[@name='tricopter']")
assert model is not None
joint = model.find("joint[@name='tail_joint']")
assert joint.attrib['type'] == 'revolute'
assert joint.findtext('parent') == 'base_link'
assert joint.findtext('child') == 'tail_motor'
assert list(map(float, joint.findtext('axis/xyz').split())) == [-1, 0, 0]
plugin = model.find("plugin[@name='tricopter::TricopterBridge']")
assert plugin is not None
camera_name = plugin.findtext('camera_heading_model')
camera = world.find(f"model[@name='{camera_name}']")
assert camera is not None and camera is not model
assert camera.findtext('static') == 'false'
assert camera.find('link') is not None
assert camera.findtext('link/kinematic') == 'true'
assert camera.findtext('link/gravity') == 'false'
assert not camera.findall('.//collision') and not camera.findall('.//visual')
assert not camera.findall('.//inertial') and not camera.findall('.//joint')
assert model.find(f"model[@name='{camera_name}']") is None, 'Camera target is outside aircraft physics'
angles = list(map(float, plugin.findtext('angle_knots_deg').split()))
assert math.isclose(float(joint.findtext('axis/limit/lower')), math.radians(angles[0]), abs_tol=1e-10)
assert math.isclose(float(joint.findtext('axis/limit/upper')), math.radians(angles[2]), abs_tol=1e-10)
assert list(map(int, plugin.findtext('adc_knots').split())) == [1936, 1490, 1281]
assert list(map(int, plugin.findtext('pwm_knots_us').split())) == [1100, 1509, 1700]
for i, name in enumerate(('tail_motor', 'right_motor', 'left_motor')):
    assert plugin.findtext(f'motor{i}_link') == name
    link = model.find(f"link[@name='{name}']")
    assert link is not None and float(link.findtext('inertial/mass')) > 0
assert model.find("joint[@name='right_fixed']").findtext('child') == 'right_motor'
assert model.find("joint[@name='left_fixed']").findtext('child') == 'left_motor'
assert not world.findall('.//include'), 'No Fuel / external network assets required'
assert math.isclose(sum(float(m.text) for m in model.findall('./link/inertial/mass')), .5), 'Aircraft total includes motor links'
assert list(map(float, plugin.findtext('max_thrust_n').split())) == [13.31255] * 3
for name in ('right_motor', 'left_motor', 'tail_motor'):
    radius = model.findtext(f"link[@name='{name}']/visual/geometry/cylinder/radius")
    assert math.isclose(float(radius), 5.1 * .0254 / 2), '5.1-inch prop diameter'
assert float(world.findtext('physics/max_step_size')) <= .001
assert list(map(float, world.findtext('gravity').split())) == [0, 0, -9.80665]
print('Passed articulated topology, physical mapping, limits and offline asset checks')
