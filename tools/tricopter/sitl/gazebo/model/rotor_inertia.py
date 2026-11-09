#!/usr/bin/env python3
"""Material estimate of rotor spin inertia, independent of Gazebo/FreeCAD.

The propellers use the frozen CAD moments normalized to the supplied 4.2 g mass.
The motor estimates 14 arc magnets, a steel back-iron annulus, a CAD-derived
6061 aluminum upper web, and the CAD shaft in hardenable stainless steel.
No CAD material densities are trusted. Motor and prop masses remain in the
locked SDF link tensors; this file's scalar J is additional relative angular
momentum, not an extra mass or another term to add to the locked tensor.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROT = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
MOTOR_DATUM_CAD_MM = np.array([-5.77, 96.19354, 70.739])

# Frozen FreeCAD volume moments in CAD millimetres. The shaft is the solid
# Part__Feature062, not an assumed solid 5 mm cylinder; its actual holes/steps
# are therefore retained. Generic motor CAD is a geometry proxy for XING-e.
SHAFT_GEOMETRY = {
    'volume_mm3': 406.5209485782585,
    'com_mm': [-5.769999999999979, 115.08730958596085, 70.73900000000005],
    'inertia_volume_mm5': [[27933.188116117148, 1.6592685504697418e-10, 1.8103815403172657e-11],
                          [1.6592685504697418e-10, 1520.4343442092027, 1.0519267109942998e-10],
                          [1.8103815403172657e-11, 1.0519267109942998e-10, 27933.188116117264]],
}
# Filled from a geometric intersection of the CAD bell with the upper web slab.
WEB_GEOMETRY = {'volume_mm3': 618.7273867996329, 'com_mm': [-5.770009908760383, 113.60357116360565, 70.73897974031662], 'inertia_volume_mm5': [[25331.953073408455, -0.011161865608301014, 0.02338075707666576], [-0.011161865608301014, 49183.101482332684, -0.017043220810592175], [0.02338075707666576, -0.017043220810592175, 25332.245338168927]]}

NOMINAL = {
    'magnet_count': 14,
    'stator_radius_mm': 11.0,
    'air_gap_mm': .2,
    'magnet_arc_length_at_mean_radius_mm': 4.75,
    'magnet_radial_thickness_mm': 1.2,
    'magnet_height_mm': 7.75,
    'magnet_density_g_cm3': 7.5,
    'magnet_bottom_above_mount_mm': 7.1,
    'back_iron_radial_thickness_mm': 1.2,
    'back_iron_extra_height_mm': .75,
    'back_iron_density_g_cm3': 7.85,
    'aluminum_web_density_g_cm3': 2.70,
    'shaft_density_g_cm3': 7.73,
    'supplied_motor_mass_g': 33.8,
    'supplied_prop_mass_g': 4.2,
    'polycarbonate_density_g_cm3': 1.20,
}
SOURCES = [
    {'url': 'https://shop.iflight.com/xing-e-pro-2207-2-6s-fpv-nextgen-motor-pro874',
     'use': 'Same-family XING-E Pro: 33.8 g with wires, 12N14P, N52H magnets, 5 mm hollow steel shaft; user independently confirmed 14 magnets.'},
    {'url': 'https://www.kjmagnetics.com/neodymium-magnet-specifications.asp',
     'use': 'NdFeB density 7.4–7.5 g/cm3.'},
    {'url': 'https://www.ssina.com/education/technical-resources/composition-properties/',
     'use': '420 martensitic stainless density 7.73 g/cm3; actual shaft alloy not identified.'},
    {'url': 'https://asm.matweb.com/search/SpecificMaterial.asp?bassnum=MA6061O',
     'use': 'Aluminum Association 6061 density 2.70 g/cm3.'},
    {'url': 'https://www.machinified.com/sites/machinified/files/2024-01/MaterialDatasheet_Machinified_Rev3.1_032023_3_7.pdf',
     'use': 'Typical 1045 steel density 7.85 g/cm3 and polycarbonate 1.20 g/cm3.'},
]


def component(name, mass_kg, com_m, inertia):
    return {'name': name, 'mass_kg': float(mass_kg),
            'com_motor_flu_m': np.asarray(com_m).tolist(),
            'inertia_about_com_motor_flu_kgm2': np.asarray(inertia).tolist()}


def cad_component(name, raw, density_g_cm3):
    mass = raw['volume_mm3']*density_g_cm3*1e-6
    com = ROT@(np.array(raw['com_mm'])-MOTOR_DATUM_CAD_MM)*1e-3
    inertia = ROT@np.array(raw['inertia_volume_mm5'])@ROT.T*density_g_cm3*1e-12
    transverse=(inertia[0,0]+inertia[1,1])/2
    inertia=np.diag([transverse,transverse,inertia[2,2]])
    com[:2]=0
    return component(name, mass, com, inertia)


def annulus(name, volume_mm3, inner_mm, outer_mm, height_mm, center_above_mount_mm, density):
    mass=volume_mm3*density*1e-6
    inner, outer, height=np.array([inner_mm,outer_mm,height_mm])*1e-3
    polar=mass*(inner*inner+outer*outer)/2
    transverse=mass*(3*(inner*inner+outer*outer)+height*height)/12
    return component(name,mass,[0,0,center_above_mount_mm/1000],np.diag([transverse,transverse,polar]))


def combine(parts):
    mass=sum(p['mass_kg'] for p in parts)
    com=sum((p['mass_kg']*np.array(p['com_motor_flu_m']) for p in parts),start=np.zeros(3))/mass
    inertia=np.zeros((3,3))
    for p in parts:
        d=np.array(p['com_motor_flu_m'])-com
        inertia+=np.array(p['inertia_about_com_motor_flu_kgm2'])+p['mass_kg']*((d@d)*np.eye(3)-np.outer(d,d))
    polar=inertia[2,2]+mass*(com[0]**2+com[1]**2)
    result=component('Complete rotating motor assembly',mass,com,inertia)
    result['polar_spin_inertia_about_shaft_kgm2']=float(polar)
    return result


def motor_model(parameters):
    p=parameters
    ri=p['stator_radius_mm']+p['air_gap_mm']
    ro=ri+p['magnet_radial_thickness_mm']
    mean=(ri+ro)/2
    theta=p['magnet_arc_length_at_mean_radius_mm']/mean
    volume=p['magnet_count']*theta/2*(ro*ro-ri*ri)*p['magnet_height_mm']
    magnets=annulus('14 NdFeB arc magnets',volume,ri,ro,p['magnet_height_mm'],
                   p['magnet_bottom_above_mount_mm']+p['magnet_height_mm']/2,p['magnet_density_g_cm3'])
    iron_outer=ro+p['back_iron_radial_thickness_mm']
    iron_height=p['magnet_height_mm']+p['back_iron_extra_height_mm']
    iron=annulus('Steel back-iron annulus',math.pi*(iron_outer**2-ro**2)*iron_height,
                 ro,iron_outer,iron_height,p['magnet_bottom_above_mount_mm']+iron_height/2,
                 p['back_iron_density_g_cm3'])
    assert WEB_GEOMETRY is not None, 'CAD upper-web volume moments have not been frozen'
    web=cad_component('6061 aluminum upper bell web',WEB_GEOMETRY,p['aluminum_web_density_g_cm3'])
    shaft=cad_component('Hardenable stainless shaft',SHAFT_GEOMETRY,p['shaft_density_g_cm3'])
    parts=[magnets,iron,web,shaft]
    result=combine(parts)
    result['components']=parts
    result['stationary_motor_residual_mass_kg']=p['supplied_motor_mass_g']/1000-result['mass_kg']
    assert result['stationary_motor_residual_mass_kg']>0
    return result


def calculate():
    frozen=json.loads((HERE/'cad_geometry.json').read_text())
    cad={r['name']:r for r in frozen['components']}
    props=[]
    for name in ['Part__Feature138','Part__Feature086','Part__Feature112']:
        raw=cad[name]['solid_properties'][0]
        volume=raw['volume_mm3']
        # CAD Y is the shaft axis. Use centroidal J; the mass model centres the
        # balanced prop and azimuth-averages its transverse tensor.
        inertia=np.array(raw['inertia_volume_mm5']).reshape(4,4)[:3,:3]
        polar=float(inertia[1,1]/volume*NOMINAL['supplied_prop_mass_g']*1e-9)
        props.append({'cad_component':name,'cad_label':cad[name]['label'],
                      'volume_mm3':volume,'mass_kg':NOMINAL['supplied_prop_mass_g']/1000,
                      'polycarbonate_volume_predicted_mass_g':volume*NOMINAL['polycarbonate_density_g_cm3']/1000,
                      'polar_spin_inertia_kgm2':polar})
    nominal=motor_model(NOMINAL)
    scenarios={}
    for name,changes in [('lower',{'magnet_height_mm':7.5,'magnet_density_g_cm3':7.4,
                                  'back_iron_extra_height_mm':.5,'back_iron_density_g_cm3':7.75}),
                         ('upper',{'magnet_height_mm':8.0,'magnet_density_g_cm3':7.5,
                                  'back_iron_extra_height_mm':1.0,'back_iron_density_g_cm3':7.90})]:
        parameters=dict(NOMINAL,**changes)
        motor=motor_model(parameters)
        scenarios[name]={'parameters':parameters,'rotating_motor_mass_kg':motor['mass_kg'],
                         'motor_polar_inertia_kgm2':motor['polar_spin_inertia_about_shaft_kgm2'],
                         'motor_plus_prop_inertia_kgm2':[motor['polar_spin_inertia_about_shaft_kgm2']+p['polar_spin_inertia_kgm2'] for p in props]}
    return {'status':'Material/CAD estimate; rotor inertia and RPM have not been measured',
            'source_step_sha256':frozen['source']['sha256'],
            'parameters':NOMINAL,'motor_order':['tail_motor','right_motor','left_motor'],
            'rotating_motor':nominal,'props':props,
            'motor_plus_prop_polar_inertia_kgm2':[nominal['polar_spin_inertia_about_shaft_kgm2']+p['polar_spin_inertia_kgm2'] for p in props],
            'sensitivity':scenarios,
            'assumptions':['Stator radius 11 mm from 2207 designation; generic CAD motor is not exact XING-e geometry.',
                           'Magnet arc length 4.75 mm is measured at mean magnet radius; magnets evenly spaced.',
                           'Magnet bottom is 7.1 mm above mount, aligned approximately to the generic CAD.',
                           'Back iron and magnets have coincident lower faces; extra iron height lies above magnets.',
                           'Steel back iron is a continuous annulus of nominal 1.2 mm radial thickness.',
                           '6061 web uses the frozen upper CAD cap above nominal back-iron height; unknown lower aluminum outer skirt/fillets are omitted.',
                           'The shaft uses generic CAD stepped/hollow geometry and assumed 420 stainless density.',
                           'All 4.2 g of each prop rotates. Supplied mass overrides nominal polycarbonate density.',
                           'Shaft/web transverse tensors are azimuth-averaged and tiny CAD radial centroid offsets removed.',
                           'Sensitivity varies supplied height ranges and selected density ranges with the web geometry fixed, not all model uncertainties.',
                           'Stator, base and motor wires are stationary. Bearing race rotation, retaining screw and prop nut are not separately resolved; their small spin inertia is omitted.',
                           'Do not add these inertias to locked carrier inertias again: the motor/prop mass model must already include them.',
                           'Rotor momentum also needs angular speed: the DC-equivalent motor/prop load model estimates RPM; actual loaded RPM remains unmeasured.'],
            'sources':SOURCES}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    data=calculate();text=json.dumps(data,indent=2)+'\n';path=HERE/'rotor_properties.json'
    if args.check:
        if not path.exists() or path.read_text()!=text:parser.exit(1,'Regenerate rotor_properties.json\n')
    else:path.write_text(text)
    print(f"Rotor estimate: motor={data['rotating_motor']['polar_spin_inertia_about_shaft_kgm2']:.8g}, "
          f"prop={data['props'][0]['polar_spin_inertia_kgm2']:.8g}, "
          f"combined={data['motor_plus_prop_polar_inertia_kgm2'][0]:.8g}kgm2")


if __name__=='__main__':main()
