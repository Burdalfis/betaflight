"""Preliminary, editable tricopter mass budget; no STEP material densities used.
Run with Python and numpy. Frozen geometric moments are in cad_geometry.json.
No FreeCAD installation or original STEP file is required to regenerate the budget.
Use --check to verify generated files; --verify-step PATH checks an optional raw STEP.

CAD input was extracted with FreeCAD1.1.4: transform each top-level Shape by
getGlobalPlacement()*Placement.inverse(), then use each solid's Volume,
CenterOfMass and MatrixOfInertia. The rear arm open shell was rebuilt by slicing
at mid-thickness along CADY and extruding the resulting closed planar profile.
Servo uses its bounding envelope; fastener surface placeholders are excluded.
Coordinates in outputs are FLU, about the regular midplate's CAD placement datum.
"""
import argparse
import csv
import io
import hashlib
import json
from collections import defaultdict
from pathlib import Path
import numpy as np

from rotor_inertia import calculate as calculate_rotor

HERE = Path(__file__).resolve().parent
INPUT = json.loads((HERE/'cad_geometry.json').read_text())
CAD = {r['name']: r for r in INPUT['components']}
DATUM = np.array([48.944242295288, 96.193540358645, 153.20714281222])
ROT = np.array([[-1.,0,0],[0,0,1],[0,1,0]]) # CAD -> FLU
CF_DENSITY_G_CM3 = 1.60
ROTOR_MODEL = calculate_rotor()
PARTS = []


def flu(cad_mm):
    return ROT @ (np.array(cad_mm)-DATUM) / 1000


def add(name, category, mass_g, com_m, inertia, evidence, moving=False):
    PARTS.append(dict(name=name, category=category, mass_kg=mass_g/1000,
                      com_m=np.asarray(com_m), inertia_kgm2=np.asarray(inertia),
                      evidence=evidence, moving=moving))


def box(name, category, mass_g, cad_com_mm, dims_flu_mm, evidence, moving=False):
    a,b,c=np.array(dims_flu_mm)/1000
    inertia=mass_g/1000/12*np.diag([b*b+c*c,a*a+c*c,a*a+b*b])
    add(name,category,mass_g,flu(cad_com_mm),inertia,evidence,moving)


def integrate_properties(props):
    vols=np.array([p['volume_mm3'] for p in props])
    assert all(p['valid'] for p in props) and np.all(vols>0)
    cs=np.array([p['com_mm'] for p in props])
    com=np.average(cs,axis=0,weights=vols)
    tensor=np.zeros((3,3))
    for p,v,c in zip(props,vols,cs):
        d=c-com
        tensor+=np.array(p['inertia_volume_mm5']).reshape(4,4)[:3,:3]+v*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    return vols.sum(),com,tensor


def geometry(name,category,density=None,mass_g=None,shift_mm=(0,0,0),moving=False):
    row=CAD[name]
    volume,com,tensor=integrate_properties(row['solid_properties'])
    if density is not None: mass_g=volume*density/1000
    # mm^5 / mm^3 = mm^2; g -> kg and mm^2 -> m^2.
    inertia=ROT@(tensor/volume*mass_g*1e-9)@ROT.T
    if category == 'props':
        # Three balanced blades rotate rapidly; retain polar inertia and average
        # the transverse tensor rather than lock an arbitrary CAD blade phase.
        transverse=(inertia[0,0]+inertia[1,1])/2
        inertia=np.diag([transverse,transverse,inertia[2,2]])
        motor_name={'Part__Feature086':'Front_lower_Z_Xing2207',
                    'Part__Feature112':'Front_upper_Z_Xing2207',
                    'Part__Feature138':'Rear_tilting_Xing2207'}[name]
        mount=np.array(CAD[motor_name]['global_placement']).reshape(4,4)[:3,3]
        com=np.array(com);com[[0,2]]=mount[[0,2]]
    add(row['label']+(' [bottom copy]' if any(shift_mm) else ''),category,mass_g,
        flu(com+np.array(shift_mm)),inertia,
        f'CAD volume/centroid/tensor; density={density} g/cm3' if density else 'CAD volume normalized to supplied mass; prop tensor azimuth-averaged',moving)


def motor_geometry(name, moving=False):
    row=CAD[name]
    frozen=row['solid_properties']
    # Frozen extraction order identifies the complete rotating bell and shaft
    # as the first2solids. They are REPLACED by the material decomposition;
    # the stator/base/bearings/internal screw are the stationary residual.
    assert abs(frozen[0]['volume_mm3']-2282.0777059917905)<1e-6
    assert abs(frozen[1]['volume_mm3']-406.5209485782585)<1e-6
    residual=frozen[2:]
    volume,stationary_com,stationary_tensor=integrate_properties(residual)
    rotating=ROTOR_MODEL['rotating_motor']
    stationary_mass=rotating['stationary_motor_residual_mass_kg']
    origin=np.array(row['global_placement']).reshape(4,4)[:3,3]
    pieces=[{'mass_kg':stationary_mass,'com_m':flu(stationary_com),
             'inertia':ROT@stationary_tensor@ROT.T*stationary_mass/volume*1e-6}]
    for p in rotating['components']:
        pieces.append({'mass_kg':p['mass_kg'],
                       'com_m':flu(origin)+np.array(p['com_motor_flu_m']),
                       'inertia':np.array(p['inertia_about_com_motor_flu_kgm2'])})
    mass=sum(p['mass_kg'] for p in pieces)
    com=sum((p['mass_kg']*p['com_m'] for p in pieces),start=np.zeros(3))/mass
    tensor=np.zeros((3,3))
    for p in pieces:
        d=p['com_m']-com
        tensor+=p['inertia']+p['mass_kg']*((d@d)*np.eye(3)-np.outer(d,d))
    assert abs(mass-.0338)<1e-12
    add(row['label'],'motors',mass*1000,com,tensor,
        'Material-derived magnets/back-iron/6061 web/stainless shaft; stationary CAD residual normalized to33.8g including wires',moving)


frame_names=['midplate_1','Tri_Arm_Front_1','Tri_Arm_Front_2','Rear_Motor_Plate_1',
'Rear_Motor_Bearing_Block_1','Rear_Motor_Bearing_Block_2','Rear_Motor_Swivel_Post_1',
'Rear_Motor_Swivel_Post_2','Rear_Arm_1','MotorPlate_1','Rear_Motor_Servo_Bracket_1',
'Rear_Motor_Servo_Bracket_2','ServoPlate_1','Midplate_Modded_1']
moving_names={'Rear_Motor_Plate_1','Rear_Motor_Bearing_Block_1','Rear_Motor_Bearing_Block_2'}
for name in frame_names:
    geometry(name,'carbon',density=CF_DENSITY_G_CM3,moving=name in moving_names)
geometry('midplate_1','carbon',density=CF_DENSITY_G_CM3,shift_mm=(0,-7,0))
# Top plate: confirmed 70% retained volume. Uniformly distributed slots are an assumption.
box('Missing slotted top plate','carbon',40*20*4*.70*CF_DENSITY_G_CM3/1000,
    DATUM+[0,29,0],(40,20,4),'Supplied envelope/retained fraction; symmetric distributed slots assumed')
# Motor mass includes motor wires. All motor internals counted only here.
for name in ['Front_lower_Z_Xing2207','Front_upper_Z_Xing2207','Rear_tilting_Xing2207']:
    motor_geometry(name,moving=name.startswith('Rear'))
for name in ['Part__Feature086','Part__Feature112','Part__Feature138']:
    geometry(name,'props',mass_g=4.2,moving=name=='Part__Feature138')
# Pivot bearings; internal motor bearings are already included in 33.8 g motor allocations.
for name in ['_57155K435_Flanged_Ball_Bearing_1','_57155K435_Flanged_Ball_Bearing_2']:
    geometry(name,'pivot_bearings',density=7.85,moving=True)
# Servo surface model is not a closed solid; use bbox with operator's measured mass.
r=CAD['Emax_ES08A_1']; bb=np.array(r['bbox_mm']); size=bb[3:]-bb[:3]
box('EMAX ES08MD II','servo',13,(bb[:3]+bb[3:])/2,(size[0],size[2],size[1]),'Supplied mass; approximate CAD ES08A envelope')
geometry('Micro_Servo_Horn_Arm_1','misc',density=1.14) # assumed nylon
geometry('M3_Standoff_11_5mm_1','hardware',density=2.70) # existing short servo support
box('Tattu 6S battery','battery',222,DATUM+[0,-26.5,0],(78,42,39),'Supplied mass/dimensions; flush below duplicated bottom plate')
# Strap uniform box approximation; small contribution to total inertia.
box('Battery strap','strap',11.2,DATUM+[0,-26.5,0],(20,46,43),'Supplied mass; approximate transverse wrap envelope')
box('20x20 FC','electronics',8,DATUM+[0,17,0],(30,30,5),'Assumed mass/envelope and stack height')
box('20x20 4-in-1 ESC','electronics',10,DATUM+[0,8,0],(30,30,5),'Assumed mass/envelope and stack height')
box('ELRS EP2','electronics',.44,DATUM+[0,21,0],(10,10,3),'Supplied mass; assumed position near stack')
box('VTX with whip','electronics',5,DATUM+[0,23,0],(20,15,4),'Supplied mass; assumed position near stack')
box('Camera','electronics',5.8,DATUM+[-17,15,0],(14,14,14),'Supplied mass; assumed position between front top standoffs')
box('Power lead capacitor mounts pads linkage allowance','misc',10,DATUM+[0,12,0],(30,30,15),'Operator-authorized allowance at stack; includes unmodeled small servo linkage')
# Four 6OD x M3ID x25mm Al standoffs, thread form replaced by3mm smooth bore.
# 34x14mm stand pattern is estimated to fit40x20mm top plate, NOT extracted CAD holes.
for i,(dx,dz) in enumerate([(-17,-7),(-17,7),(17,-7),(17,7)]):
    m=np.pi*(3**2-1.5**2)*25*2.70/1000
    iz=m/1000/2*((.003)**2+(.0015)**2)
    ix=m/1000/12*(3*((.003)**2+(.0015)**2)+(.025)**2)
    add(f'Top aluminum standoff {i+1}','hardware',m,flu(DATUM+[dx,14.5,dz]),np.diag([ix,ix,iz]),'Supplied dimensions/material; assumed stand pattern')
# Stationary hinge pins, CAD bearings define axis alongCAD X.
for i,x in enumerate([131.96,173.96]):
    m=np.pi*1.5**2*12*7.81/1000
    axial=m/1000*.0015**2/2
    radial=m/1000*(3*.0015**2+.012**2)/12
    add(f'52100 steel pivot pin {i+1}','hardware',m,flu([x,115.69354,153.351]),np.diag([axial,radial,radial]),'Supplied dimensions; assumed steel density7.81g/cm3, stationary hinge shafts')

# Supplied fastener inventory replaces all unusable imported screw/nut surfaces.
# Rounded catalogue-size mass proxies, not measured alloy-steel fastener masses.
SCREW_MASS={8:.55,10:.65,12:.70,20:1.05,30:1.41}

def screw(name,length,where,moving=False):
    box(name,'hardware',SCREW_MASS[length],where,(3,3,length+1.65),
        'Supplied count/length; rounded steel screw mass proxy; approximate placement/local inertia',moving)

def nut(name,where,moving=False,pem=False):
    box(name,'hardware',.4 if pem else .3,where,(5.5,5.5,2.4),
        'Supplied count; assumed PEM0.4g/hex0.3g and approximate placement',moving)

for motor in ['Front_lower_Z_Xing2207','Front_upper_Z_Xing2207','Rear_tilting_Xing2207']:
    mount=np.array(CAD[motor]['global_placement']).reshape(4,4)[:3,3]
    n=4 if motor.startswith('Rear') else 3
    for i in range(n):
        a=2*np.pi*i/n
        screw(f'{motor} mount M3x10 {i+1}',10,mount+[8*np.cos(a),-4,8*np.sin(a)],n==4)
for group,xs,y,moving in [('Basket',[133.96,171.96],102.994,True),('Uprights',[127.06,179.06],97.194,False)]:
    for i,x in enumerate(xs):
        screw(f'{group} M3x12 {i+1}',12,[x,y,153.35],moving)
        nut(f'{group} hex nut {i+1}',[x,y,153.35],moving)
for i,p in enumerate([[137.48,84.04,167.82],[137.54,84.04,138.82],[163.26,84.04,153.37]]):
    screw(f'Servo M3x20 {i+1}',20,p)
    nut(f'Servo PEM nut {i+1}',[p[0],94.194,p[2]],pem=True)
for i,(dx,dz) in enumerate([(x,z) for x in [-22,22] for z in [-10,10]]+[(x,z) for x in [-10,10] for z in [-18,18]]):
    screw(f'Frame M3x10 {i+1}',10,DATUM+[dx,-2,dz])
for i,(dx,dz) in enumerate([(x,z) for x in [-10,10] for z in [-10,10]]):
    screw(f'Stack stud M3x30 {i+1}',30,DATUM+[dx,8,dz])
for i,(dx,dz) in enumerate([(x,z) for x in [-17,17] for z in [-7,7]]):
    screw(f'Top plate M3x8 {i+1}',8,DATUM+[dx,27,dz])


def combine(parts):
    mass=sum(p['mass_kg'] for p in parts)
    com=sum((p['mass_kg']*p['com_m'] for p in parts),start=np.zeros(3))/mass
    inertia=np.zeros((3,3))
    for p in parts:
        d=p['com_m']-com
        inertia+=p['inertia_kgm2']+p['mass_kg']*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    eig=np.linalg.eigvalsh(inertia)
    assert np.all(eig>0) and eig[2]<=eig[0]+eig[1]+1e-12
    assert np.allclose(inertia,inertia.T)
    return {'mass_kg':mass,'com_flu_m':com.tolist(),'inertia_about_com_flu_kgm2':inertia.tolist(),'principal_moments_kgm2':eig.tolist()}

out={'status':'PRELIMINARY ESTIMATE; neutral basket; not measured/validated airframe properties',
     'step_sha256':INPUT['source']['sha256'],
     'datum_cad_mm':DATUM.tolist(),'cad_to_flu_rotation':ROT.tolist(),
     'assumptions':['No STEP material densities used. CF bulk density1.60g/cm3 assumed.',
                    'Rear arm reconstructed from mid-thickness section because STEP shell is open.',
                    'Motor rotor uses material decomposition from rotor_properties.json; stationary residual33.8g minusrotor normalized over remaining CAD stator/base/bearings including wires.',
                    'Rotating prop and motor web/shaft tensors are azimuth-averaged; material rotor spin inertia is estimated, not measured.',
                    'Missing bottom plate is regular midplate translated CADY=-7mm; modified midplate retained.',
                    'Battery78longitudinal/42transverse/39vertical, centered under regular plate, no gap.',
                    'Unspecified electronics and top standoff positions are estimates; top stand pattern34x14mm.',
                    'No unlisted frame nuts added. Verify any omitted arm clamping nuts/hardware.',
                    'Slot distribution of top plate and strap mass distribution approximated.',
                    'Fixed servo horn mass included; linkage small mass covered by10g allowance.'],
     'sources':['https://www.rockwestcomposites.com/on/demandware.static/Sites-RWC-Site/Sites-rwc-master-catalog/-/downloads/RWC_forgedblocks_tds.pdf',
                'https://www.speedybee.com/speedybee-f405-mini-bls-35a-20x20-stack/',
                'https://www.accu.co.uk/socket-button-screws/8104-SSB-M3-12-A2',
                'https://www.accu.co.uk/socket-button-screws/8108-SSB-M3-20-A2',
                'https://www.accu.co.uk/socket-button-screws/155060-SSB-M3-30-A2-BL'],
     'neutral_aircraft':combine(PARTS),'fixed_group':combine([p for p in PARTS if not p['moving']]),
     'moving_basket_group':combine([p for p in PARTS if p['moving']])}
mounts={}
for label,name in [('right','Front_lower_Z_Xing2207'),('left','Front_upper_Z_Xing2207'),('rear','Rear_tilting_Xing2207')]:
    point=np.array(CAD[name]['global_placement']).reshape(4,4)[:3,3]
    mounts[label]=flu(point)
out['motor_mounts_neutral_flu_m']={k:v.tolist() for k,v in mounts.items()}
out['motor_spacing_m']={f'{a}_to_{b}':float(np.linalg.norm(mounts[a]-mounts[b])) for a,b in [('right','left'),('rear','right'),('rear','left')]}
out['hinge_neutral_flu_m']=flu([152.96,115.69354,153.351]).tolist()
out['hinge_axis_flu']=[-1,0,0]
out['relative_rotor_spin_inertia_kgm2']=ROTOR_MODEL['motor_plus_prop_polar_inertia_kgm2']

# SDF child links retain the motor-mount origins. Their inertia poses locate
# actual group centres; front motor/prop masses are removed from base_link.
front_names = {
    'right_motor': {'Front_lower_Z_Xing2207', 'Front_lower_Z_Gemfan51466_CW'},
    'left_motor': {'Front_upper_Z_Xing2207', 'Front_upper_Z_Gemfan51466_CCW'},
}
front_components = front_names['right_motor'] | front_names['left_motor']
link_parts = {
    'base_link': [p for p in PARTS if not p['moving'] and p['name'] not in front_components],
    'right_motor': [p for p in PARTS if p['name'] in front_names['right_motor']],
    'left_motor': [p for p in PARTS if p['name'] in front_names['left_motor']],
    'tail_motor': [p for p in PARTS if p['moving']],
}
assert sum(len(parts) for parts in link_parts.values()) == len(PARTS)
out['link_groups'] = {name: combine(parts) for name, parts in link_parts.items()}
out['link_origins_flu_m'] = {
    'base_link': [0., 0., 0.], 'right_motor': mounts['right'].tolist(),
    'left_motor': mounts['left'].tolist(), 'tail_motor': mounts['rear'].tolist(),
}
summary=defaultdict(float)
for p in PARTS:summary[p['category']]+=1000*p['mass_kg']
out['category_mass_g']=dict(summary)
# Sensitivity envelopes, not statistical confidence intervals. Nominal inputs with supplied weights held fixed.
variants={}
for tag,cf_scale,hardware_scale,electronics_g,extras_g in [('lighter_assumptions',1.5/1.6,.8,13,5),('heavier_assumptions',1.7/1.6,1.2,22,15)]:
    altered=[]
    for p in PARTS:
        q=dict(p); scale=1.
        if p['category']=='carbon':scale=cf_scale
        elif p['category']=='hardware':scale=hardware_scale
        elif p['name'] in ['20x20 FC','20x20 4-in-1 ESC']:scale=electronics_g/18
        elif p['name'].startswith('Power lead'):scale=extras_g/10
        q['mass_kg']=p['mass_kg']*scale; q['inertia_kgm2']=p['inertia_kgm2']*scale
        altered.append(q)
    variants[tag]=combine(altered)
out['mass_assumption_sensitivity_only']=variants
def output_texts():
    stream = io.StringIO(newline='')
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(['component', 'category', 'moving_basket', 'mass_g',
                     'com_flu_x_m', 'com_flu_y_m', 'com_flu_z_m',
                     'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz', 'evidence'])
    for part in PARTS:
        tensor = part['inertia_kgm2']
        writer.writerow([part['name'], part['category'], part['moving'],
                         part['mass_kg']*1000, *part['com_m'],
                         tensor[0, 0], tensor[1, 1], tensor[2, 2],
                         tensor[0, 1], tensor[0, 2], tensor[1, 2], part['evidence']])
    return {'mass_properties.json': json.dumps(out, indent=2)+'\n',
            'component_budget.csv': stream.getvalue()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true',
                        help='Verify generated files without changing them')
    parser.add_argument('--verify-step', type=Path, metavar='PATH',
                        help='Additionally verify the SHA256 of the original STEP')
    args = parser.parse_args()
    if args.verify_step is not None:
        actual = hashlib.sha256(args.verify_step.read_bytes()).hexdigest()
        if actual != INPUT['source']['sha256']:
            parser.error('STEP hash differs from the source used for cad_geometry.json')
    mismatches = []
    for name, contents in output_texts().items():
        path = HERE/name
        if args.check:
            if not path.exists() or path.read_text() != contents:
                mismatches.append(name)
        else:
            path.write_text(contents)
    if mismatches:
        parser.exit(1, 'Regenerate mass assets: '+', '.join(mismatches)+'\n')
    print(('Checked' if args.check else 'Generated')+
          f" mass assets: {out['neutral_aircraft']['mass_kg']*1000:.4f} g, " +
          f"{len(PARTS)} components, {len(link_parts)} disjoint SDF link groups")


if __name__ == '__main__':
    main()
