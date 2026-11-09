#!/usr/bin/env python3
"""Generate a single textured visual ground quad for recording/replay only.

The material contains its grid, avoiding overlapping geometric line strips.
This asset is not a collision mesh and is not referenced by the physics world.
"""
from pathlib import Path
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
SIZE=1024
TILE_METRES=100
HALF_EXTENT_METRES=10000


def generate():
    image=Image.new('RGB',(SIZE,SIZE),(98,116,98))
    draw=ImageDraw.Draw(image)
    # Low-contrast 10m cells retain motion cues without bright near-field roads.
    for row in range(10):
        for column in range(10):
            if (row+column)%2:
                box=(round(column*SIZE/10),round(row*SIZE/10),
                     round((column+1)*SIZE/10),round((row+1)*SIZE/10))
                draw.rectangle(box,fill=(104,122,104))
    for i in range(1,10):
        pixel=round(i*SIZE/10)
        draw.line((pixel,0,pixel,SIZE),fill=(129,143,126),width=2)
        draw.line((0,pixel,SIZE,pixel),fill=(129,143,126),width=2)
    # Border halves meet seamlessly under repeat sampling: 0.59m major lines.
    for edge in (0,SIZE-1):
        draw.line((edge,0,edge,SIZE),fill=(164,164,142),width=6)
        draw.line((0,edge,SIZE,edge),fill=(164,164,142),width=6)
    # Asymmetric, unlabeled tile marks help judge rotation without asserting
    # a cardinal direction before the renderer's UV convention is inspected.
    def polygon(points,colour):
        draw.polygon([(round(x*SIZE),round(y*SIZE)) for x,y in points],fill=colour)
    polygon(((.47,.52),(.47,.46),(.50,.43),(.53,.46),(.53,.52),(.50,.49)),(153,157,132))
    draw.rectangle((round(.62*SIZE),round(.48*SIZE),round(.64*SIZE),round(.52*SIZE)),fill=(112,143,146))
    image.save(HERE/'review_ground.png',optimize=True)

    # Two upward-facing triangles, continuous UVs, one repeat every100m.
    h=HALF_EXTENT_METRES;repeat=2*h//TILE_METRES
    dae=f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><contributor><authoring_tool>generate_review_ground.py</authoring_tool></contributor><unit meter="1" name="metre"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="ground-image" name="ground-image"><init_from>review_ground.png</init_from></image></library_images>
  <library_effects><effect id="ground-effect"><profile_COMMON>
    <newparam sid="ground-surface"><surface type="2D"><init_from>ground-image</init_from></surface></newparam>
    <newparam sid="ground-sampler"><sampler2D><source>ground-surface</source><wrap_s>WRAP</wrap_s><wrap_t>WRAP</wrap_t><minfilter>LINEAR_MIPMAP_LINEAR</minfilter><magfilter>LINEAR</magfilter></sampler2D></newparam>
    <technique sid="common"><phong><emission><color>0 0 0 1</color></emission><ambient><color>0.6 0.6 0.6 1</color></ambient><diffuse><texture texture="ground-sampler" texcoord="UVMap"/></diffuse><specular><color>0 0 0 1</color></specular><shininess><float>0</float></shininess></phong></technique>
  </profile_COMMON></effect></library_effects>
  <library_materials><material id="ground-material" name="ground-material"><instance_effect url="#ground-effect"/></material></library_materials>
  <library_geometries><geometry id="ground-quad" name="ground-quad"><mesh>
    <source id="positions"><float_array id="positions-array" count="12">{-h} {-h} 0 {h} {-h} 0 {h} {h} 0 {-h} {h} 0</float_array><technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="normals"><float_array id="normals-array" count="3">0 0 1</float_array><technique_common><accessor source="#normals-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="uv"><float_array id="uv-array" count="8">0 0 {repeat} 0 {repeat} {repeat} 0 {repeat}</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
    <triangles material="ground-material-symbol" count="2"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="ground-scene"><node id="ground-node"><instance_geometry url="#ground-quad"><bind_material><technique_common><instance_material symbol="ground-material-symbol" target="#ground-material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#ground-scene"/></scene>
</COLLADA>
'''
    (HERE/'review_ground.dae').write_text(dae)


if __name__=='__main__':generate()
