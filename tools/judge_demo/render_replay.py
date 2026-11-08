"""Blender cutaway REPLAY of recorded execution, not a fabricated flight.

Run with Blender --background --python this_file -- --demo OUTPUT --scene ...
The autonomous decision/metrics were recorded before this visualization pass.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import bpy
from mathutils import Vector


def material(name,color):
    m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);return m


def cube(name,position,size,mat,parent=None):
    bpy.ops.mesh.primitive_cube_add(size=1,location=position)
    o=bpy.context.object;o.name=name;o.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    o.data.materials.append(mat)
    if parent:o.parent=parent
    bevel=o.modifiers.new('Manufactured edges','BEVEL');bevel.width=min(.015,min(size)/5);bevel.segments=2
    o.modifiers.new('Weighted normals','WEIGHTED_NORMAL')
    return o


def line(name,points,mat,radius=.012):
    c=bpy.data.curves.new(name,'CURVE');c.dimensions='3D';c.bevel_depth=radius;c.bevel_resolution=2
    s=c.splines.new('POLY');s.points.add(len(points)-1)
    for p,v in zip(s.points,points):p.co=(*v,1.)
    o=bpy.data.objects.new(name,c);bpy.context.collection.objects.link(o);o.data.materials.append(mat);return o


def drone(sdf):
    body=bpy.data.objects.new('Recorded vehicle pose',None);bpy.context.collection.objects.link(body)
    black=material('Carbon / avionics',(.045,.065,.08));green=material('Jetson PCB',(.07,.25,.18))
    orange=material('Flight hardware',(.88,.32,.07));silver=material('Motors',(.3,.38,.42));cyan=material('Optics',(.03,.7,.85))
    root=ET.parse(sdf).getroot().find('model')
    for link in root.findall('link'):
        pose=[float(x) for x in (link.findtext('pose') or '0 0 0 0 0 0').split()]
        link_empty=bpy.data.objects.new(link.get('name'),None);bpy.context.collection.objects.link(link_empty)
        link_empty.parent=body;link_empty.location=pose[:3];link_empty.rotation_euler=pose[3:]
        for visual in link.findall('visual'):
            name=visual.get('name');geo=visual.find('geometry');pv=[float(x) for x in (visual.findtext('pose') or '0 0 0 0 0 0').split()]
            mat=green if name=='jetson' else (orange if 'motor' in name else black)
            if geo.find('box') is not None:
                size=[float(x) for x in geo.findtext('box/size').split()]
                obj=cube(name,pv[:3],size,mat,link_empty)
            elif geo.find('cylinder') is not None:
                r=float(geo.findtext('cylinder/radius'));h=float(geo.findtext('cylinder/length'))
                bpy.ops.mesh.primitive_cylinder_add(vertices=32,radius=r,depth=h,location=pv[:3]);obj=bpy.context.object
                obj.name=name;obj.parent=link_empty;obj.data.materials.append(silver)
                if r>.06: obj.data.materials.clear();obj.data.materials.append(cyan)
            else:continue
            obj.rotation_euler=pv[3:]
    for y in (-.0375,.0375):
        bpy.ops.mesh.primitive_uv_sphere_add(segments=20,ring_count=10,radius=.011,location=(.104,y,.043))
        obj=bpy.context.object;obj.parent=body;obj.data.materials.append(cyan)
    return body


def point_camera(camera,position,target):
    camera.location=position;camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()


def render(demo,scene_name,stride):
    bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
    scene=bpy.context.scene;scene.render.engine='BLENDER_WORKBENCH'
    scene.display.shading.light='STUDIO';scene.display.shading.color_type='MATERIAL'
    scene.display.shading.show_shadows=True;scene.display.shading.show_cavity=True
    scene.display.shading.cavity_type='BOTH';scene.display.shading.curvature_ridge_factor=1.4
    scene.display.shading.background_type='WORLD';scene.world.color=(.035,.055,.075)
    scene.display.render_aa='16';scene.render.resolution_x=1280;scene.render.resolution_y=800
    scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG'
    white=material('Course / wall',(.72,.78,.79));floor=material('Orange competition floor',(.76,.26,.065))
    cyan=material('Planned path / cyan',(.025,.88,.85));blue=material('Blue square marker',(.03,.24,.86))
    dark=material('Base plate',(.06,.10,.15));trailmat=material('Recorded path',(.92,.95,.98))
    evidence=demo/'evidence'/scene_name;trace=json.loads((evidence/'trace.json').read_text())
    if scene_name=='course':
        asset=demo/'assets/course';meta=json.loads((asset/'course.json').read_text())
        for g in meta['groups']:
            name=g['name'].split('/')[-1];lower=name.lower()
            if 'net' in lower:continue # labelled cutaway; sensor run still includes the nets
            before=set(bpy.data.objects)
            bpy.ops.wm.ply_import(filepath=str(asset/g['file']))
            obj=(set(bpy.data.objects)-before).pop();obj.name=name
            mat=floor if 'floor' in lower else white
            if 'tube' in lower:mat=material(name,(.10,.28,.34))
            if 'landing pad' in lower:mat=floor
            if 'guideline' in lower:mat=cyan
            obj.data.materials.clear();obj.data.materials.append(mat)
            if 'landing pad 5' in lower:
                b=g['bounds'];center=[(b[0][i]+b[1][i])/2 for i in range(3)]
                cube('Blue square visual marker',(*center[:2],.009),(1.20,1.20,.012),blue)
        cube('Studio plinth',(0,0,-.12),(6.3,6.3,.22),dark)
    else:
        boxes=json.loads((evidence/'world.json').read_text())
        for i,(lo,hi) in enumerate(boxes['before']):
            size=[b-a for a,b in zip(lo,hi)];pos=[(a+b)/2 for a,b in zip(lo,hi)]
            if lo[2]>2.5: continue # ceiling removed only in the replay camera
            cube('Recorded world '+str(i),pos,size,floor if i==0 else white)
        lo,hi=boxes['after'][-1];size=[b-a for a,b in zip(lo,hi)];pos=[(a+b)/2 for a,b in zip(lo,hi)]
        obstacle=cube('Observed inserted obstacle',pos,size,material('New obstacle',(.94,.33,.12)))
    body=drone(demo/'assets/model.sdf')
    bpy.ops.object.camera_add();camera=bpy.context.object;scene.camera=camera;camera.data.type='ORTHO'
    if scene_name=='course':point_camera(camera,(-8,8,12),(0,0,.5));camera.data.ortho_scale=9.4
    else:point_camera(camera,(-7,-7,8),(-2.5,1.2,.5));camera.data.ortho_scale=7.0
    output=demo/'replay_frames'/scene_name;output.mkdir(parents=True,exist_ok=True)
    indexes=[0] if scene_name=='course' else list(range(0,len(trace),stride))
    pathobj=None;trailobj=None;safety=None
    for index in indexes:
        row=trace[index];body.location=row['position'];body.rotation_euler.z=row['yaw']
        for obj in (pathobj,trailobj,safety):
            if obj:bpy.data.objects.remove(obj,do_unlink=True)
        import numpy as np
        data=np.load(evidence/f'{index:04d}.npz');path=data['trajectory']
        pathobj=line('Actual current plan',path.tolist(),cyan,.014) if len(path)>1 else None
        history=[r['position'] for r in trace[:index+1]]
        trailobj=line('Recorded executed poses',history,trailmat,.012) if len(history)>1 else None
        p=row['position'];circle=[[p[0]+.46*math.cos(a),p[1]+.46*math.sin(a),p[2]-.12] for a in np.linspace(0,2*math.pi,65)]
        safety=line('Safety envelope illustration',circle,cyan,.008)
        if scene_name=='diagnostic':obstacle.hide_render=not row['dynamic_obstacle']
        scene.render.filepath=str(output/f'{index:04d}.png');bpy.ops.render.render(write_still=True)
    print('REPLAY_RENDERED',scene_name,len(indexes),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--demo',type=Path,required=True);p.add_argument('--scene',choices=['course','diagnostic'],required=True);p.add_argument('--stride',type=int,default=1)
    args=p.parse_args(sys.argv[sys.argv.index('--')+1:]);render(args.demo,args.scene,args.stride)
