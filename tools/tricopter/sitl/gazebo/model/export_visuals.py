#!/usr/bin/env python3
"""Export offline Gazebo visual meshes from the original STEP using FreeCAD.

Run in a FreeCAD Python environment with Part, Import and MeshPart available:
    python export_visuals.py /path/to/Tricopter_motors.step

Meshes have metre coordinates in their SDF link frames. Missing real parts keep
explicit primitive visuals in the SDF; CAD screw/nut surface placeholders are omitted.
The original STEP is not needed to use the generated Gazebo world.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import FreeCAD as App
import Import
import Mesh
import MeshPart
import Part

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('step', type=Path)
parser.add_argument('--output',type=Path,default=HERE.parent/'meshes')
parser.add_argument('--rotor-details',type=Path)
args = parser.parse_args()
source = json.loads((HERE/'mass_properties.json').read_text())
step = args.step.resolve()
actual_hash=hashlib.sha256(step.read_bytes()).hexdigest()
if actual_hash!=source['step_sha256']:
    parser.error('STEP differs from the geometry used for the mass budget')
App.ParamGet('User parameter:BaseApp/Preferences/Mod/Import').SetBool('ImportInvisible',False)
doc=App.newDocument('TricopterVisuals')
Import.insert(str(step),doc.Name)
doc.recompute()
objects={obj.Name:obj for obj in doc.Objects}

def world_shape(name):
    obj=objects[name]
    shape=obj.Shape.copy()
    shape.Placement=obj.getGlobalPlacement().multiply(obj.Placement.inverse()).multiply(shape.Placement)
    if name=='Rear_Arm_1':
        b=shape.BoundBox
        wires=shape.slice(App.Vector(0,1,0),(b.YMin+b.YMax)/2)
        face=Part.makeFace(wires,'Part::FaceMakerBullseye')
        face.translate(App.Vector(0,-(b.YMax-b.YMin)/2,0))
        shape=face.extrude(App.Vector(0,b.YMax-b.YMin,0))
    return shape

fixed=['midplate_1','Tri_Arm_Front_1','Tri_Arm_Front_2','Rear_Motor_Swivel_Post_1',
'Rear_Motor_Swivel_Post_2','Rear_Arm_1','MotorPlate_1','Rear_Motor_Servo_Bracket_1',
'Rear_Motor_Servo_Bracket_2','ServoPlate_1','Midplate_Modded_1']
groups={
'frame':('base_link',fixed),
'basket':('tail_motor',['Rear_Motor_Plate_1','Rear_Motor_Bearing_Block_1','Rear_Motor_Bearing_Block_2']),
'pivot_bearings':('tail_motor',['_57155K435_Flanged_Ball_Bearing_1','_57155K435_Flanged_Ball_Bearing_2']),
'servo':('base_link',['Emax_ES08A_1']),
'servo_horn':('base_link',['Micro_Servo_Horn_Arm_1']),
'servo_support':('base_link',['M3_Standoff_11_5mm_1']),
'right_motor':('right_motor',['Front_lower_Z_Xing2207']),
'left_motor':('left_motor',['Front_upper_Z_Xing2207']),
'tail_motor':('tail_motor',['Rear_tilting_Xing2207']),
'right_prop':('right_motor',['Part__Feature086']),
'left_prop':('left_motor',['Part__Feature112']),
'tail_prop':('tail_motor',['Part__Feature138']),
}
args.output.mkdir(parents=True,exist_ok=True)
manifest={'source_step_sha256':actual_hash,'units':'metres','linear_deflection_mm':.05,'angular_deflection_rad':.20,'meshes':{}}
for name,(link,names) in groups.items():
    mesh=Mesh.Mesh()
    for objname in names:
        shape=world_shape(objname)
        mesh.addMesh(MeshPart.meshFromShape(Shape=shape,LinearDeflection=.05,AngularDeflection=.20,Relative=False))
    if name=='frame':
        bottom=world_shape('midplate_1');bottom.translate(App.Vector(0,-7,0))
        mesh.addMesh(MeshPart.meshFromShape(Shape=bottom,LinearDeflection=.05,AngularDeflection=.20,Relative=False))
    datum=source['datum_cad_mm'];origin=source['link_origins_flu_m'][link]
    transform=App.Matrix()
    transform.A11=-.001;transform.A12=0;transform.A13=0;transform.A14=datum[0]*.001-origin[0]
    transform.A21=0;transform.A22=0;transform.A23=.001;transform.A24=-datum[2]*.001-origin[1]
    transform.A31=0;transform.A32=.001;transform.A33=0;transform.A34=-datum[1]*.001-origin[2]
    mesh.transform(transform)
    path=args.output/(name+'.stl');mesh.write(str(path))
    bb=mesh.BoundBox
    manifest['meshes'][path.name]={'link':link,'cad_components':names,'facets':mesh.CountFacets,
      'bbox_link_m':[bb.XMin,bb.YMin,bb.ZMin,bb.XMax,bb.YMax,bb.ZMax],
      'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    if name=='frame':
        manifest['meshes'][path.name]['derived_components']=[{'source':'midplate_1','translation_cad_mm':[0,-7,0]}]
    print(name,mesh.CountFacets,flush=True)
(args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
if args.rotor_details:
    details={}
    for name in ['Part__Feature061','Part__Feature062','Part__Feature063','Part__Feature084']:
        shape=world_shape(name)
        surfaces=[]
        for face in shape.Faces:
            s=face.Surface
            if isinstance(s,Part.Cylinder):
                b=face.BoundBox
                surfaces.append({'radius_mm':s.Radius,'center_mm':list(s.Center),'axis':list(s.Axis),'area_mm2':face.Area,
                 'bbox_mm':[b.XMin,b.YMin,b.ZMin,b.XMax,b.YMax,b.ZMax]})
        details[name]={'label':objects[name].Label,'cylinders':surfaces}
    args.rotor_details.write_text(json.dumps(details,indent=2)+'\n')
