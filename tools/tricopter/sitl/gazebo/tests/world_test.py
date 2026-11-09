#!/usr/bin/env python3
"""Check model topology, CAD mass properties and calibration without Gazebo."""
import json
import hashlib
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

source = Path(sys.argv[1]).resolve()
reference = json.loads((Path(__file__).resolve().parents[1] / 'model/mass_properties.json').read_text())
rotors = json.loads((Path(__file__).resolve().parents[1] / 'model/rotor_properties.json').read_text())
performance = json.loads((Path(__file__).resolve().parents[1] / 'model/motor_performance.json').read_text())
aerodynamics = json.loads((Path(__file__).resolve().parents[1] / 'model/airframe_aerodynamics.json').read_text())
world = ET.parse(source).getroot().find('world')
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


def close_vector(actual, expected, tolerance=2e-12):
    assert len(actual) == len(expected)
    assert all(math.isclose(a, b, rel_tol=1e-10, abs_tol=tolerance)
               for a, b in zip(actual, expected)), (actual, expected)


def pose_xyz(element):
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    assert len(values) == 6 and all(math.isfinite(v) for v in values)
    # All neutral link / inertial frames retain the CAD-derived FLU orientation.
    close_vector(values[3:], [0, 0, 0])
    return values[:3]


# Meshes are exported in metres in each physical link's local coordinates.
# Validate the actual binary assets, not just their SDF URIs or manifest names.
asset_root = Path(__file__).resolve().parents[1] / 'meshes'
manifest = json.loads((asset_root/'manifest.json').read_text())
assert manifest['units'] == 'metres'
assert manifest['source_step_sha256'] == reference['step_sha256']
mesh_properties = {}
for name, expected in manifest['meshes'].items():
    asset = asset_root/name
    data = asset.read_bytes()
    assert hashlib.sha256(data).hexdigest() == expected['sha256'], f'Changed mesh: {name}'
    assert len(data) >= 84, f'Truncated binary STL: {name}'
    facets = struct.unpack_from('<I', data, 80)[0]
    assert facets == expected['facets'] > 0
    assert len(data) == 84 + 50*facets, f'Invalid binary STL record count: {name}'
    minimum, maximum = [math.inf]*3, [-math.inf]*3
    radial_extent = 0.0
    for triangle in struct.iter_unpack('<12fH', memoryview(data)[84:]):
        assert all(math.isfinite(value) for value in triangle[:12]), f'Nonfinite STL: {name}'
        for start in (3, 6, 9):
            vertex = triangle[start:start+3]
            for axis, value in enumerate(vertex):
                minimum[axis] = min(minimum[axis], value)
                maximum[axis] = max(maximum[axis], value)
            if name.endswith('_prop.stl'):
                radial_extent = max(radial_extent, math.hypot(vertex[0], vertex[1]))
    bbox = minimum + maximum
    close_vector(bbox, expected['bbox_link_m'], tolerance=2e-8)
    assert all(b > a for a, b in zip(minimum, maximum)), f'Degenerate mesh: {name}'
    mesh_properties[name] = {'bbox': bbox, 'radial_extent': radial_extent}

referenced_meshes = set()
for link in model.findall('link'):
    for visual in link.findall('visual'):
        mesh = visual.find('geometry/mesh')
        if mesh is None:
            continue
        value = mesh.findtext('uri')
        assert value and '://' not in value, 'CAD visuals must use local mesh paths'
        asset = (source.parent/value).resolve()
        assert asset.is_relative_to(asset_root) and asset.is_file(), f'Missing local CAD visual: {asset}'
        name = asset.name
        assert name in manifest['meshes']
        assert link.get('name') == manifest['meshes'][name]['link'], f'Mesh on wrong articulated link: {name}'
        close_vector(pose_xyz(visual), [0, 0, 0])
        close_vector([float(v) for v in mesh.findtext('scale', '1 1 1').split()], [1, 1, 1])
        assert name not in referenced_meshes, f'Duplicated visual mesh: {name}'
        referenced_meshes.add(name)
blade_meshes = {'right_prop.stl', 'left_prop.stl', 'tail_prop.stl'}
assert referenced_meshes == set(manifest['meshes'])-blade_meshes, \
    'Only the three blade visuals are replaced; all other CAD assets remain used'
frame = mesh_properties['frame.stl']['bbox']
assert .18 < frame[3]-frame[0] < .25, 'Frame mesh must retain metre scale and full arm span'
assert .17 < frame[4]-frame[1] < .22
assert 'midplate_1' in manifest['meshes']['frame.stl']['cad_components']
assert {'Tri_Arm_Front_1', 'Tri_Arm_Front_2', 'Rear_Arm_1'} <= set(manifest['meshes']['frame.stl']['cad_components'])
assert {'source': 'midplate_1', 'translation_cad_mm': [0, -7, 0]} in manifest['meshes']['frame.stl']['derived_components'], 'Bottom plate duplicates the regular midplate beneath the arms'


def determinant(matrix):
    a, b, c = matrix
    return (a[0]*(b[1]*c[2]-b[2]*c[1]) -
            a[1]*(b[0]*c[2]-b[2]*c[0]) +
            a[2]*(b[0]*c[1]-b[1]*c[0]))


def positive_definite(matrix):
    assert matrix[0][0] > 0
    assert matrix[0][0]*matrix[1][1]-matrix[0][1]**2 > 0
    assert determinant(matrix) > 0


def tensor(element):
    i = {key: float(element.findtext('inertia/' + key))
         for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
    assert all(math.isfinite(v) for v in i.values())
    matrix = [[i['ixx'], i['ixy'], i['ixz']],
              [i['ixy'], i['iyy'], i['iyz']],
              [i['ixz'], i['iyz'], i['izz']]]
    positive_definite(matrix)
    # A positive-definite tensor alone can still violate the inertia triangle.
    # This second-moment matrix checks physical admissibility in any basis.
    half_trace = sum(matrix[k][k] for k in range(3))/2
    positive_definite([[(half_trace if row == col else 0)-matrix[row][col]
                        for col in range(3)] for row in range(3)])
    return matrix


links = {link.get('name'): link for link in model.findall('link')}
assert set(links) == set(reference['link_groups']), 'Each budget component belongs to one physical link'
properties = []
for name, expected in reference['link_groups'].items():
    link = links[name]
    origin = pose_xyz(link)
    close_vector(origin, reference['link_origins_flu_m'][name])
    if link.find('pose') is not None:
        assert link.find('pose').get('relative_to', '__model__') in ('__model__', 'base_link')
    inertial = link.find('inertial')
    mass = float(inertial.findtext('mass'))
    assert mass > 0 and math.isclose(mass, expected['mass_kg'], rel_tol=1e-10)
    local_com = pose_xyz(inertial)
    assert inertial.find('pose') is None or not inertial.find('pose').get('relative_to')
    com = [a+b for a, b in zip(origin, local_com)]
    close_vector(com, expected['com_flu_m'])
    inertia = tensor(inertial)
    for actual_row, expected_row in zip(inertia, expected['inertia_about_com_flu_kgm2']):
        close_vector(actual_row, expected_row, tolerance=5e-13)
    properties.append((mass, com, inertia))

# Independently assemble the SDF links about their combined CG. This catches
# duplicated front motor masses and applying a whole-aircraft tensor to base_link.
total_mass = sum(p[0] for p in properties)
assert math.isclose(total_mass, 0.5099032000886807, rel_tol=1e-10)
neutral = reference['neutral_aircraft']
assert math.isclose(total_mass, neutral['mass_kg'], rel_tol=1e-10)
combined_com = [sum(mass*com[k] for mass, com, _ in properties)/total_mass for k in range(3)]
close_vector(combined_com, neutral['com_flu_m'])
combined_inertia = [[0.0]*3 for _ in range(3)]
for mass, com, inertia in properties:
    dx, dy, dz = [a-b for a, b in zip(com, combined_com)]
    displacement = [[dy*dy+dz*dz, -dx*dy, -dx*dz],
                    [-dx*dy, dx*dx+dz*dz, -dy*dz],
                    [-dx*dz, -dy*dz, dx*dx+dy*dy]]
    for row in range(3):
        for col in range(3):
            combined_inertia[row][col] += inertia[row][col]+mass*displacement[row][col]
for actual_row, expected_row in zip(combined_inertia, neutral['inertia_about_com_flu_kgm2']):
    close_vector(actual_row, expected_row, tolerance=5e-13)
assert abs(combined_inertia[0][2]) > 6e-5, 'Retain the significant roll/yaw cross-axis inertia'

# Child origins lie on motor thrust axes; the rear rotates about a distinct hinge.
for label, name in [('right', 'right_motor'), ('left', 'left_motor'), ('rear', 'tail_motor')]:
    close_vector(pose_xyz(links[name]), reference['motor_mounts_neutral_flu_m'][label])
right, left = (pose_xyz(links[name]) for name in ('right_motor', 'left_motor'))
assert math.isclose(math.dist(right, left), .16503709925347088, abs_tol=2e-10)
assert joint.find('pose').get('relative_to') == 'base_link'
hinge = pose_xyz(joint)
close_vector(hinge, reference['hinge_neutral_flu_m'])
tail = pose_xyz(links['tail_motor'])
assert math.isclose(hinge[2]-tail[2], .0107, abs_tol=2e-8), 'Rear motor mount is below the actual pivot'
assert math.isclose(hinge[1], tail[1], abs_tol=2e-6), 'Rear thrust axis crosses the neutral hinge line'

# Battery contact geometry uses the operator's explicit orientation, not the
# nominal L/W/H ordering: 78 longitudinal, 42 transverse and 39 vertical.
base = links['base_link']
battery = base.find("collision[@name='battery_collision']")
close_vector(pose_xyz(battery), [0, 0, -.0265])
close_vector([float(v) for v in battery.findtext('geometry/box/size').split()], [.078, .042, .039])
# The default plant obtains speed, thrust and reaction from one coupled motor /
# prop model. The old lag coefficients remain compatibility fields only.
assert plugin.findtext('motor_model') == 'dc_prop'
target = performance['target_motor']
calibration = performance['calibration']
fields = {
    'motor_voltage_v': target['voltage_v'],
    'motor_kv_rpm_v': target['kv_rpm_v'],
    'motor_resistance_ohm': target['dc_equivalent_terminal_resistance_ohm'],
    'motor_no_load_current_a': target['no_load_current_a'],
    'motor_no_load_voltage_v': target['no_load_voltage_v'],
    'motor_current_limit_a': target['model_current_limit_a'],
    'motor_brake_current_limit_a': target['model_brake_current_limit_a'],
    'motor_friction_smoothing_rad_s': target['friction_smoothing_rad_s'],
    'prop_thrust_n_per_rad_s_squared': calibration['thrust_coefficient_n_per_rad_s_squared'],
    'prop_torque_nm_per_rad_s_squared': calibration['torque_coefficient_nm_per_rad_s_squared'],
}
for name, expected in fields.items():
    assert math.isclose(float(plugin.findtext(name)), expected, rel_tol=1e-10, abs_tol=1e-15), name
assert target['model_brake_current_limit_a'] > 0, 'Default DC plant enables the active-braking proxy'
assert 'Unmeasured' in target['model_brake_current_limit_status']
assert target['model_brake_on_stop'] is False, 'BrakeOnStop is separate from active nonzero-duty braking'
assert plugin.findtext('motor_brake_on_stop') == ('true' if target['model_brake_on_stop'] else 'false')
assert target['winding_connection'] == 'delta'
assert math.isclose(target['delta_branch_resistance_ohm'], 1.5*fields['motor_resistance_ohm'], rel_tol=1e-10)
assert math.isclose(fields['motor_resistance_ohm'], target['interphase_resistance_ohm'], rel_tol=1e-10)
full = performance['target_full_command']
close_vector([float(v) for v in plugin.findtext('max_thrust_n').split()], [full['thrust_n']]*3)
close_vector([float(v) for v in plugin.findtext('reaction_ratio_m').split()],
             [fields['prop_torque_nm_per_rad_s_squared']/fields['prop_thrust_n_per_rad_s_squared']]*3)
assert math.isclose(float(plugin.findtext('max_rotor_speed_rad_s')), full['omega_rad_s'], rel_tol=1e-10)

# The same polar inertia appears in the locked rigid-link model and as the
# relative-spin momentum coefficient, not as another added carrier mass/tensor.
assert rotors['motor_order'] == ['tail_motor', 'right_motor', 'left_motor']
assert rotors['source_step_sha256'] == reference['step_sha256']
relative_spin = [float(v) for v in plugin.findtext('rotor_inertia_kgm2').split()]
close_vector(relative_spin, reference['relative_rotor_spin_inertia_kgm2'], tolerance=1e-15)
close_vector(relative_spin, rotors['motor_plus_prop_polar_inertia_kgm2'], tolerance=1e-15)
for name, polar in zip(rotors['motor_order'], relative_spin):
    assert polar > 0
    assert tensor(links[name].find('inertial'))[2][2] >= polar

# Independent dimensional / energy checks on the calibration's steady point.
omega = full['omega_rad_s']
motor_constant = 60/(2*math.pi*target['kv_rpm_v'])
current = full['equivalent_motor_current_a']
no_load_current = target['no_load_current_a']
no_load_voltage = target['no_load_voltage_v']
assert no_load_current == 1.2 and no_load_voltage == 10, 'Retain the supplied no-load reference'
reference_speed = (no_load_voltage-fields['motor_resistance_ohm']*no_load_current)/motor_constant
assert reference_speed > 0
smooth_speed = target['friction_smoothing_rad_s']
loss_current_coefficient = no_load_current*(reference_speed+smooth_speed)/reference_speed
loss_torque = motor_constant*loss_current_coefficient*omega/(omega+smooth_speed)
# Smoothing is normalized at the measured no-load point, not at infinite RPM.
reference_loss_torque = motor_constant*loss_current_coefficient*reference_speed/(reference_speed+smooth_speed)
assert math.isclose(reference_loss_torque, motor_constant*no_load_current, rel_tol=1e-12)
assert math.isclose(full['thrust_n'], fields['prop_thrust_n_per_rad_s_squared']*omega**2, rel_tol=1e-10)
assert math.isclose(full['prop_torque_nm'], fields['prop_torque_nm_per_rad_s_squared']*omega**2, rel_tol=1e-10)
assert math.isclose(motor_constant*current-loss_torque, full['prop_torque_nm'], rel_tol=1e-10)
assert math.isclose(full['input_power_w'], target['voltage_v']*full['ideal_bus_current_a'], rel_tol=1e-10)
assert math.isclose(full['input_power_w'], full['shaft_power_w']+full['winding_loss_w']+full['no_load_loss_w'], rel_tol=1e-10)
assert 0 < current <= target['model_current_limit_a']
assert 0 < full['effective_duty'] <= 1+1e-12

for name in ('right_motor', 'left_motor', 'tail_motor'):
    prop = {'right_motor': 'right_prop', 'left_motor': 'left_prop', 'tail_motor': 'tail_prop'}[name]
    visual = model.find(f"link[@name='{name}']/visual[@name='{prop}']")
    actual = mesh_properties[prop+'.stl']
    assert visual.find('geometry/mesh') is None
    disc = visual.find('geometry/cylinder')
    assert disc is not None
    assert math.isclose(float(disc.findtext('radius')), 5.1*.0254/2, abs_tol=1e-12)
    assert math.isclose(float(disc.findtext('length')), .0008, abs_tol=1e-12)
    close_vector(pose_xyz(visual), [0, 0, (actual['bbox'][2]+actual['bbox'][5])/2])
    assert .5 < float(visual.findtext('transparency')) < .9
    assert visual.findtext('cast_shadows') == 'false'
    # The original blade assets remain intact as mass/inertia provenance.
    assert math.isclose(actual['radial_extent'], 5.1*.0254/2, rel_tol=.02), '5.1-inch prop radial envelope'
    assert .020 < actual['bbox'][2] < .022 and .028 < actual['bbox'][5] < .031
    collision = model.find(f"link[@name='{name}']/collision[@name='{name}_collision']")
    close_vector(pose_xyz(collision), [0, 0, .0105])
    assert math.isclose(float(collision.findtext('geometry/cylinder/radius')), .014, abs_tol=1e-12)
    assert math.isclose(float(collision.findtext('geometry/cylinder/length')), .021, abs_tol=1e-12)
    assert len(model.findall(f"link[@name='{name}']/collision")) == 1, 'Disc adds no collision'

assert plugin.findtext('airframe_drag_model') == aerodynamics['model'] == 'quadratic_body'
assert 'PLACEHOLDERS' in aerodynamics['status']
assert math.isclose(float(plugin.findtext('air_density_kg_m3')), aerodynamics['air_density_kg_m3'], abs_tol=1e-12)
close_vector(list(map(float, plugin.findtext('drag_cda_body_m2').split())), aerodynamics['cda_body_m2'])
assert math.isclose(float(plugin.findtext('linear_drag_n_s_m')), aerodynamics['legacy']['linear_drag_n_s_m'], abs_tol=1e-12)
assert math.isclose(float(plugin.findtext('angular_drag_nm_s_rad')), aerodynamics['angular_drag_nm_s_rad'], abs_tol=1e-12)
assert float(world.findtext('physics/max_step_size')) <= .001
assert list(map(float, world.findtext('gravity').split())) == [0, 0, -9.80665]
print('Passed articulated topology, CAD mass/CG/full inertia, hinge geometry, limits and offline asset checks')
