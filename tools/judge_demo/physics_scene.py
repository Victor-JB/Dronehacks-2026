"""MuJoCo scene from simulator-only STEP meshes; never imported by navigation.

Concave window/tube retain original visual/depth mesh and CoACD convex collision
assemblies. The flight plant is a force/torque velocity servo, NOT PX4 SITL.
"""
import argparse,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh,coacd,mujoco
from scipy.spatial.transform import Rotation


def vector(a):return ' '.join(f'{float(x):.9g}' for x in a)


def build(assets,out,variant='baseline'):
    out.mkdir(parents=True,exist_ok=True);meta=json.loads((assets/'course.json').read_text())
    root=ET.Element('mujoco',model='forward_autonomy_course')
    ET.SubElement(root,'compiler',angle='radian',meshdir=str(out.resolve()))
    ET.SubElement(root,'option',timestep='.005',gravity='0 0 -9.81',integrator='implicitfast')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1920',offheight='1080')
    ET.SubElement(visual,'headlight',ambient='.4 .4 .4',diffuse='.6 .6 .6')
    asset=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 0 6',dir='0 0 -1',directional='true',diffuse='.6 .6 .6')
    groups=[];spawn=None;pad=None
    for i,g in enumerate(meta['groups']):
        name=g['name'].split('/')[-1].lower();bounds=np.array(g['bounds'])
        if 'launch mark' in name:spawn=np.r_[bounds.mean(axis=0)[:2],.85]
        if 'landing pad 5' in name:pad=bounds.mean(axis=0)[:2]
        if bounds[1,2]<.01 and 'floor' not in name:continue
        center=bounds.mean(axis=0);mesh=trimesh.load(assets/g['file'],force='mesh');mesh.process(validate=True)
        mesh.vertices-=center
        filename=f'visual_{i}.obj';mesh.export(out/filename)
        ET.SubElement(asset,'mesh',name=f'v{i}',file=filename)
        angle=0.
        if variant=='altered' and 'm1 maze' in name:angle=math.radians(-5)
        if variant=='altered' and 'c1 chicane' in name:angle=math.radians(5)
        body=ET.SubElement(world,'body',name=f'course_{i}',pos=vector(center),euler=f'0 0 {angle}')
        color='.86 .36 .09 1' if 'floor' in name else ('.12 .3 .36 1' if 'tube oval' in name else '.72 .77 .76 1')
        group='4' if 'net' in name else '2'
        ET.SubElement(body,'geom',name=f'visual_{i}',type='mesh',mesh=f'v{i}',rgba=color,contype='0',conaffinity='0',group=group)
        if 'floor' in name or 'net' in name:
            ET.SubElement(body,'geom',name=f'collision_{i}',type='box',size=vector((bounds[1]-bounds[0])/2),rgba='0 0 0 0',group='3')
            pieces=1
        else:
            if 'window' in name or 'tube oval' in name:
                cached=out/f'convex_{i}.npz'
                if cached.exists():
                    data=np.load(cached);hulls=[(data[f'v{k}'],data[f'f{k}']) for k in range(int(data['count']))]
                else:
                    coacd.set_log_level('warn')
                    hulls=coacd.run_coacd(coacd.Mesh(np.array(mesh.vertices),np.array(mesh.faces,dtype=np.int32)),
                        threshold=.01,max_convex_hull=64,mcts_iterations=80,mcts_nodes=15,seed=7)
                    data={'count':len(hulls)}
                    for k,(v,f) in enumerate(hulls):data[f'v{k}']=v;data[f'f{k}']=f
                    np.savez_compressed(cached,**data)
            else:
                hull=mesh.convex_hull;hulls=[(hull.vertices,hull.faces)]
            for k,(verts,faces) in enumerate(hulls):
                file=f'collision_{i}_{k}.obj';trimesh.Trimesh(verts,faces,process=False).export(out/file)
                ET.SubElement(asset,'mesh',name=f'c{i}_{k}',file=file)
                ET.SubElement(body,'geom',name=f'collision_{i}_{k}',type='mesh',mesh=f'c{i}_{k}',rgba='0 0 0 0',group='3')
            pieces=len(hulls)
        groups.append(dict(name=name,collision_parts=pieces,angle_radians=angle))
    # Simulator paint only. Detection receives its pixels and depth, not this coordinate.
    ET.SubElement(world,'geom',name='blue_square',type='box',pos=vector([*pad,.004]),size='.6 .6 .003',rgba='.025 .12 .95 1',contype='0',conaffinity='0',group='2')
    # Placement certificate is checked as a separate noncontact geom by the grader.
    ET.SubElement(world,'geom',name='placement_certificate',type='cylinder',pos=vector(spawn),size='.47 .27',rgba='0 0 0 0',contype='0',conaffinity='0',group='3')
    drone=ET.SubElement(world,'body',name='drone',pos=vector(spawn))
    ET.SubElement(drone,'freejoint',name='flight')
    ET.SubElement(drone,'inertial',pos='-.0164 0 .0136',mass='1.222',diaginertia='.00716 .00731 .01163')
    ET.SubElement(drone,'geom',name='drone_envelope',type='cylinder',size='.30 .12',rgba='0 0 0 0',group='3')
    ET.SubElement(drone,'geom',name='frame',type='box',size='.10 .07 .012',rgba='.035 .055 .065 1',contype='0',conaffinity='0')
    ET.SubElement(drone,'geom',name='battery',type='box',pos='-.0315 0 .046',size='.079 .023 .018',rgba='.06 .09 .1 1',contype='0',conaffinity='0')
    ET.SubElement(drone,'geom',name='oak',type='box',pos='.0891 0 .0426',size='.0105 .0485 .015',rgba='.03 .08 .11 1',contype='0',conaffinity='0')
    for k,(x,y) in enumerate([(.0831,-.1597),(-.1475,.135),(.0831,.1597),(-.1475,-.135)]):
        ET.SubElement(drone,'geom',name=f'arm{k}',type='capsule',fromto=vector([0,0,0,x,y,.01]),size='.012',rgba='.06 .10 .12 1',contype='0',conaffinity='0')
        ET.SubElement(drone,'geom',name=f'rotor{k}',type='cylinder',pos=vector([x,y,.03]),size='.0889 .002',rgba='.07 .55 .52 .8',contype='0',conaffinity='0')
    # OpenGL camera axes: right=-bodyY, up=bodyZ, back=-bodyX.
    R=np.array([[0,0,-1],[-1,0,0],[0,1,0.]])
    q=Rotation.from_matrix(R).as_quat();quat=[q[3],q[0],q[1],q[2]]
    ET.SubElement(drone,'camera',name='forward',pos='.103 -.0375 .043',quat=vector(quat),fovy='53.4')
    file=out/f'{variant}.xml';ET.ElementTree(root).write(file,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(file));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    result=dict(scene=str(file),variant=variant,spawn=spawn.tolist(),pad_visual_only=pad.tolist(),groups=groups,
        geoms=model.ngeom,convex_collision_decomposition=True,physics='MuJoCo4.0 force/torque servo, not PX4')
    (out/f'{variant}.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return file


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--assets',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--variant',default='baseline')
    a=p.parse_args();build(a.assets,a.out,a.variant)
