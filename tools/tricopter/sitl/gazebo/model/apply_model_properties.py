#!/usr/bin/env python3
"""Apply regenerated mass, rotor and motor load properties to the SDF.

Visuals, collisions and actuator calibration are kept.
Run calculate_mass.py and motor_performance.py first; --check checks values without changing the world.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent

def apply(world, check=False):
    reference=json.loads((HERE/'mass_properties.json').read_text())
    parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree=ET.parse(world,parser=parser)
    model=tree.find("./world/model[@name='tricopter']")
    differences=[]
    def assign(parent,path,values):
        element=parent.find(path)
        if element is None:raise ValueError(f'Missing {path}')
        actual=[float(v) for v in element.text.split()]
        if len(actual)!=len(values) or any(not math.isclose(a,b,rel_tol=2e-14,abs_tol=1e-13) for a,b in zip(actual,values)):
            differences.append(path)
        if not check:element.text=' '.join(f'{v:.15g}' for v in values)
    for name,p in reference['link_groups'].items():
        link=model.find(f"link[@name='{name}']")
        origin=reference['link_origins_flu_m'][name]
        assign(link,'pose',origin+[0,0,0])
        assign(link,'inertial/pose',[a-b for a,b in zip(p['com_flu_m'],origin)]+[0,0,0])
        assign(link,'inertial/mass',[p['mass_kg']])
        inertia=p['inertia_about_com_flu_kgm2']
        for tag,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
            assign(link,'inertial/inertia/'+tag,[inertia[i][j]])
    joint=model.find("joint[@name='tail_joint']")
    assign(joint,'pose',reference['hinge_neutral_flu_m']+[0,0,0])
    plugin=model.find("plugin[@name='tricopter::TricopterBridge']")
    assign(plugin,'rotor_inertia_kgm2',reference['relative_rotor_spin_inertia_kgm2'])
    performance=json.loads((HERE/'motor_performance.json').read_text())
    target=performance['target_motor'];coefficients=performance['calibration']
    full=performance['target_full_command']
    values={
        'motor_voltage_v':target['voltage_v'],
        'motor_kv_rpm_v':target['kv_rpm_v'],
        'motor_resistance_ohm':target['interphase_resistance_ohm'],
        'motor_no_load_current_a':target.get('no_load_current_a',target.get('no_load_current_a_assumed')),
        'motor_no_load_voltage_v':target.get('no_load_voltage_v',10),
        'motor_current_limit_a':target['model_current_limit_a'],
        'motor_brake_current_limit_a':target['model_brake_current_limit_a'],
        'motor_friction_smoothing_rad_s':target.get('friction_smoothing_rad_s',20),
        'prop_thrust_n_per_rad_s_squared':coefficients['thrust_coefficient_n_per_rad_s_squared'],
        'prop_torque_nm_per_rad_s_squared':coefficients['torque_coefficient_nm_per_rad_s_squared'],
        'max_rotor_speed_rad_s':full['omega_rad_s'],
    }
    for tag,value in values.items():
        if plugin.find(tag) is None and not check: ET.SubElement(plugin,tag)
        # Empty newly-created elements need an initial numeric value for comparison.
        if plugin.find(tag) is not None and not plugin.find(tag).text and not check: plugin.find(tag).text='0'
        assign(plugin,tag,[value])
    brake_on_stop=plugin.find('motor_brake_on_stop')
    if brake_on_stop is None:
        if check:raise ValueError('Missing motor_brake_on_stop')
        brake_on_stop=ET.SubElement(plugin,'motor_brake_on_stop')
    expected_brake_on_stop='true' if target['model_brake_on_stop'] else 'false'
    if brake_on_stop.text!=expected_brake_on_stop:differences.append('motor_brake_on_stop')
    if not check:brake_on_stop.text=expected_brake_on_stop
    assign(plugin,'max_thrust_n',[full['thrust_n']]*3)
    ratio=values['prop_torque_nm_per_rad_s_squared']/values['prop_thrust_n_per_rad_s_squared']
    assign(plugin,'reaction_ratio_m',[ratio]*3)
    kind=plugin.find('motor_model')
    if kind is None:
        if check: raise ValueError('Missing motor_model')
        kind=ET.SubElement(plugin,'motor_model')
    if check and kind.text!='dc_prop':raise ValueError('Default motor_model must be dc_prop')
    if not check:kind.text='dc_prop'
    if check and differences:raise ValueError('SDF physics differs from component budget: '+', '.join(differences))
    if not check:
        ET.indent(tree,space='  ');tree.write(world,encoding='utf-8',xml_declaration=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',type=Path,default=HERE.parent/'worlds/tricopter.sdf')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();apply(args.world,args.check)
    print(('Checked' if args.check else 'Applied')+' SDF mass, rotor and motor properties')
