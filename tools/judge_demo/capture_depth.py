"""Blender BVH forward-depth sensor. Geometry stays in this producer only."""
import argparse,json,math,sys,time
from pathlib import Path
import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree


def capture(assets):
    start=time.perf_counter();meta=json.loads((assets/'course.json').read_text())
    vertices=[];faces=[];colors=[];position=None
    for group in meta['groups']:
        name=group['name'].split('/')[-1].lower()
        if 'launch mark' in name:position=np.r_[np.mean(group['bounds'],axis=0)[:2],.85]
        if group['bounds'][1][2]<.01 and 'floor' not in name:continue
        bpy.ops.wm.ply_import(filepath=str(assets/group['file']))
        obj=bpy.context.object;mesh=obj.data;mesh.calc_loop_triangles();offset=len(vertices)
        vertices.extend([tuple(v.co) for v in mesh.vertices])
        triangles=[tuple(offset+i for i in p.vertices) for p in mesh.loop_triangles];faces.extend(triangles)
        color=(.89,.43,.14) if 'floor' in name else ((.38,.48,.57) if 'net' in name else ((.12,.27,.32) if 'tube' in name else (.73,.78,.78)))
        colors.extend([color]*len(triangles));bpy.data.objects.remove(obj,do_unlink=True)
    bvh=BVHTree.FromPolygons(vertices,faces,all_triangles=True)
    K=np.array([[595.1951/3,0,487.3531/3],[0,595.3043/3,305.8477/3],[0,0,1.]])
    u,v=np.meshgrid(np.arange(320),np.arange(200));optical=np.stack([(u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],np.ones_like(u)],axis=-1).reshape(-1,3)
    norm=np.linalg.norm(optical,axis=1);R=np.array([[0,0,1.],[-1.,0,0],[0,-1.,0]])
    rays=optical@R.T/norm[:,None];origin=Vector(position+np.array([.1053,0,.0304]))
    depth=np.full(len(rays),np.nan,np.float32);rgb=np.tile([30,43,60],(len(rays),1)).astype(np.uint8)
    for i,direction in enumerate(rays):
        hit,normal,face,distance=bvh.ray_cast(origin,Vector(direction),8.)
        if hit is None or distance<.1:continue
        depth[i]=distance/norm[i]
        illumination=.65+.35*abs(np.dot(normal,[.3,-.4,.86]))
        rgb[i]=np.clip(np.array(colors[face])*illumination*255,0,255)
    np.savez_compressed(assets/'forward_capture.npz',depth=depth.reshape(200,320),bgr=rgb.reshape(200,320,3)[:,:,::-1],K=K,position=position)
    print('BVH_SENSOR',json.dumps(dict(triangles=len(faces),pixels=len(rays),valid=int(np.isfinite(depth).sum()),wall_s=time.perf_counter()-start)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--assets',type=Path,required=True)
    a=parser.parse_args(sys.argv[sys.argv.index('--')+1:]);capture(a.assets)
