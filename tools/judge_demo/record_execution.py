"""Record real policy execution using existing local SIL boundary and fixtures.

COURSE: STEP meshes belong to this sensor producer/grader only. The unchanged
stationary policy gets Image/CameraInfo/Odometry/VO messages, never a mesh/map.
DIAGNOSTIC: reuses the existing ideal panoramic fixture, explicitly NOT OAK-D.
No ROS DDS, Gazebo, PX4 or physical interface is started on this Windows host.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/sil'))
import test_local_policy as existing
from dh_core.online_geometry import nominal_transform_for_tests
from dh_core.camera_geom import quat_to_rot
F=existing.F


class CourseSensor:
    def __init__(self,assets):
        self.assets=assets; self.meta=json.loads((assets/'course.json').read_text())
        meshes=[]; colors=[]; self.floor=None; self.spawn=None
        for group in self.meta['groups']:
            name=group['name'].split('/')[-1].lower(); bounds=np.array(group['bounds'])
            if 'launch mark' in name: self.spawn=np.r_[bounds.mean(axis=0)[:2],.85]
            if 'floor' in name: self.floor=bounds; continue
            if bounds[1,2]<.01: continue # painted floor markings are not obstacles
            mesh=trimesh.load(assets/group['file'],force='mesh')
            meshes.append(mesh)
            color=(.38,.48,.57) if 'net' in name else ((.12,.27,.32) if 'tube' in name else (.73,.78,.78))
            colors.extend([color]*len(mesh.faces))
        self.mesh=trimesh.util.concatenate(meshes); self.colors=np.array(colors)
        self.width,self.height=320,200
        self.K=np.array([[595.1951/3,0,487.3531/3],[0,595.3043/3,305.8477/3],[0,0,1.]])

    def frame(self,position,yaw):
        cached=self.assets/'forward_capture.npz'
        if cached.exists() and abs(yaw)<1e-8:
            data=np.load(cached)
            if np.allclose(position,data['position']): return data['bgr'].copy(),data['depth'].copy()
        raise RuntimeError('Generate the Blender BVH capture first; unbounded bulk ray fallback disabled')

    def clearance(self,position):
        # Grader only; sampled cylinder surface, not exact continuous clearance.
        angle=np.linspace(0,2*math.pi,48,endpoint=False)
        pts=np.vstack([np.column_stack([.27*np.cos(angle),.27*np.sin(angle),np.full(len(angle),z)]) for z in (-.12,0,.12)])+position
        _,distance,_=trimesh.proximity.closest_point(self.mesh,pts)
        return float(distance.min())

    def sphere_clearance(self,position):
        _,distance,_=trimesh.proximity.closest_point(self.mesh,np.atleast_2d(position))
        return float(distance[0]-math.hypot(.27,.12))


def save_state(folder,index,position,yaw,command,state,stamp,model,plan=None,fpv=None,extra=None):
    folder.mkdir(parents=True,exist_ok=True)
    values=model.grid; k0=int((.85-.31-model.lo[2])/model.voxel); k1=int(math.ceil((.85+.31-model.lo[2])/model.voxel))
    if folder.name=='diagnostic': k0=int((1.-.31-model.lo[2])/model.voxel);k1=int(math.ceil((1.+.31-model.lo[2])/model.voxel))
    band=values[:,:,max(0,k0):min(k1,values.shape[2])]
    grid=np.full(values.shape[:2],-1,np.int8)
    fresh=(stamp-model.observed_at[:,:,max(0,k0):min(k1,values.shape[2])])<=.5
    grid[((band<0)&fresh).all(axis=2)]=0; grid[(band>model.l_occ).any(axis=2)]=1
    np.savez_compressed(folder/f'{index:04d}.npz',grid=grid,
        occupied=model.occupied()[::max(1,len(model.occupied())//2000)],
        trajectory=np.empty((0,3)) if plan is None else plan,
        position=position,origin=model.lo,resolution=model.voxel)
    if fpv is not None: cv2.imwrite(str(folder/f'{index:04d}_fpv.png'),fpv)
    return dict(index=index,time=stamp,position=list(map(float,position)),yaw=float(yaw),command=list(map(float,command)),state=state,**(extra or {}))


def record_course(assets,out):
    sensor=CourseSensor(assets); p=sensor.spawn.copy()
    F.BUS.t=1.;F.BUS.subs.clear();F.BUS.published.clear();F.BUS.service_calls.clear()
    F.Node.PARAM_OVERRIDES=dict(voxel_m=.2,size_xy_m=12.,depth_geometry_verified=True,
        below_wall_height_verified=True,legal_min_altitude=.35,legal_max_altitude=1.3,cruise_z=.85)
    node=existing.StationaryLocalPolicyNode()
    R,t=nominal_transform_for_tests(0.); rod,_=cv2.Rodrigues(R); angle=np.linalg.norm(rod); q=rod[:,0]/angle*np.sin(angle/2)
    import types
    F.TfBuffer.transforms={('base_link','demo_optical'):types.SimpleNamespace(transform=types.SimpleNamespace(
        rotation=types.SimpleNamespace(x=q[0],y=q[1],z=q[2],w=np.cos(angle/2)),
        translation=types.SimpleNamespace(x=t[0],y=t[1],z=t[2])))}
    color,depth=sensor.frame(p,0.) # static pose allows exact frame reuse
    rows=[]; cpu=time.process_time(); start=time.perf_counter()
    for tick in range(161):
        now=1+tick*.05; F.BUS.t=now
        odom=F.Odometry();odom.header=F.Header(stamp=F.make_time(now),frame_id='odom')
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,p)
        node._on_odom(odom); node.health.on_vo_status(now,1)
        if tick%4==0 and tick>0:
            capture=now-.05
            info=F.CameraInfo();info.header=F.Header(stamp=F.make_time(capture),frame_id='demo_optical')
            info.width=sensor.width;info.height=sensor.height;info.k=sensor.K.ravel().tolist();node._on_info(info)
            d=depth if now<7. else np.full_like(depth,np.nan)
            message=F.Image();message.header=info.header;message.width=sensor.width;message.height=sensor.height
            message.encoding='32FC1';message.is_bigendian=0;message.step=sensor.width*4;message.data=d.tobytes()
            node._on_depth(message);node._integrate()
        node.control_tick()
        if tick%20==0: node.on_start(None) # actual startup gate must reject, not bypass
        if tick%2==0:
            state=json.loads(F.BUS.published['/dh/local_policy/status'][-1][1].data)
            rows.append(save_state(out/'course',len(rows),p,0.,[0.,0.,0.],state['startup_reason'],now,node.map,fpv=color,
                extra=dict(replans=node.policy.replans,depth_fault=now>=7.,startup_ready=state['startup_ready'])))
    forbidden=[n for n in F.BUS.published if n.startswith(('/skybeta','/fmu'))]
    if forbidden: raise RuntimeError('Physical endpoint created: '+str(forbidden))
    if any(row['startup_ready'] for row in rows): raise RuntimeError('Unexpected startup bypass')
    if not any(row['state']=='WAIT_MEASURED_FULL_START_VOLUME' for row in rows): raise RuntimeError('Expected observed-volume hold was not exercised')
    if rows[-1]['state']!='WAIT_FRESH_MAP': raise RuntimeError('Injected depth loss did not hold')
    minimum=sensor.clearance(p)
    sphere=sensor.sphere_clearance(p)
    result=dict(name='actual_STEP_forward_depth',distance_m=0.,replans=node.policy.replans,
        collision_count=0 if sphere>0 else None,minimum_sampled_hull_clearance_m=minimum,
        conservative_bounding_sphere_clearance_m=sphere,
        collision_check='stationary enclosing-sphere clearance certificate; no dynamics claimed',
        mission_complete=False,status='BLOCKED_STARTUP_OBSERVABILITY',
        actual_policy_class='StationaryLocalPolicyNode',frames=len(rows),wall_s=time.perf_counter()-start,
        cpu_s=time.process_time()-cpu,ROS_DDS=False,PX4=False,Gazebo=False,
        sensor_model='Ideal forward depth at measured mono-intrinsic projection; not calibrated native-depth emulation',
        planner_geometry_access=False,physical_endpoints=forbidden)
    (out/'course/trace.json').write_text(json.dumps(rows,indent=2));(out/'course/result.json').write_text(json.dumps(result,indent=2))
    return result


def record_diagnostic(out):
    started=time.perf_counter();cpu=time.process_time()
    world=existing.UnknownWorld('open',0); m=existing.TimedTeamMap(existing.VoxelMap(.25,12.,-1.,3.)); policy=existing.LocalPolicy()
    p=np.array([-3.,0.,1.]);yaw=0.; rows=[];length=0.;minimum=math.inf;collisions=0; stopped=False
    sink=existing.DryRunBackend(F.Node('judge_diagnostic_sink'));sink.grant_authority()
    origin,dirs,_=world.observe(p,yaw,panoramic=False)
    source=np.column_stack([(np.tile(np.arange(0,960,24),25)-487.3531)/595.1951,
        (np.repeat(np.arange(0,600,24),40)-305.8477)/595.3043,np.ones(1000)])
    R,_=nominal_transform_for_tests(0.); fpv_dirs=source@R.T; fpv_dirs/=np.linalg.norm(fpv_dirs,axis=1)[:,None]
    boxes_before=[(a.tolist(),b.tolist()) for a,b in world.boxes]
    for step in range(130):
        now=1+step*.1;F.BUS.t=now
        if step==40:
            follower=policy.gate.follower; anchor=follower.traj.sample(min(follower.tau+2.5,follower.traj.duration))[0]
            if np.linalg.norm(anchor[:2]-p[:2])<=.7: raise RuntimeError('Obstacle fixture intersects current body')
            world.add_box(anchor+[-.12,-.12,-2.],anchor+[.12,.12,2.])
        origin,dirs,ranges=world.observe(p,yaw,panoramic=True);m.stamp=now;m.integrate(origin,dirs,ranges,4.)
        policy.update_map(existing.from_team_map(m,m.observed_at,now,now,1.,(.35,1.5),True,revision=m.frames))
        if policy.gate is None or policy.gate.follower.done: policy.plan(p,yaw,now)
        measured=float(world.raycast(p,np.array([[math.cos(yaw),math.sin(yaw),0.]]))[0])
        v,yr=policy.step(p,yaw,.1,existing.inputs(now,free_distance=measured),True)
        sink.send_velocity(*map(float,v),float(yr))
        if step>=40 and np.linalg.norm(v)<1e-9 and policy.state.startswith('HOLD'):stopped=True
        c,s=math.cos(yaw),math.sin(yaw); directions=fpv_dirs@np.array([[c,-s,0],[s,c,0],[0,0,1]]).T
        rr=world.raycast(p,directions); shade=np.clip(210-rr*20,40,205).reshape(25,40).astype(np.uint8)
        fpv=cv2.resize(cv2.applyColorMap(shade,cv2.COLORMAP_BONE),(320,200))
        plan=None if policy.gate is None else policy.gate.follower.traj.p
        rows.append(save_state(out/'diagnostic',step,p.copy(),yaw,v,policy.state,now,m,plan,fpv,
            dict(replans=policy.replans,dynamic_obstacle=step>=40)))
        length+=float(np.linalg.norm(v)*.1);p+=v*.1;yaw+=yr*.1
        collisions+=int(world.collision(p))
        for lo,hi in world.boxes:
            if p[2]+.12>lo[2] and p[2]-.12<hi[2]:minimum=min(minimum,float(np.linalg.norm(p[:2]-np.clip(p[:2],lo[:2],hi[:2]))-.27))
    result=dict(name='ideal_coverage_component_diagnostic',distance_m=length,replans=policy.replans,
        collision_count=collisions,minimum_horizontal_hull_clearance_m=minimum,
        obstacle_stop_observed=stopped,mission_complete=False,status='SHORT_LOCAL_NAVIGATION_ONLY',
        sensor_model='IDEAL PANORAMIC RANGE - NOT forward OAK-D',ROS_DDS=False,PX4=False,Gazebo=False,
        planner_geometry_access=False,frames=len(rows),wall_s=time.perf_counter()-started,cpu_s=time.process_time()-cpu,
        map_storage_bytes=m.grid.nbytes+m.observed_at.nbytes,
        final_position=p.tolist(),dry_run_command_count=len(rows),
        plant='kinematic Euler .1s, no modeled command delay or PX4 dynamics')
    if not stopped or collisions or length<.3:raise RuntimeError('Diagnostic acceptance failed: '+str(result))
    (out/'diagnostic/trace.json').write_text(json.dumps(rows,indent=2));(out/'diagnostic/result.json').write_text(json.dumps(result,indent=2))
    (out/'diagnostic/world.json').write_text(json.dumps(dict(before=boxes_before,after=[(a.tolist(),b.tolist()) for a,b in world.boxes]),indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--assets',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    results=[record_course(args.assets,args.out),record_diagnostic(args.out)]
    (args.out/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
