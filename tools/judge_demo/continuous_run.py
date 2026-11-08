"""Continuous MuJoCo forces -> noisy forward depth/odom -> real policy -> forces.

No scene geometry/finish coordinates enter the policy. Verified initial cylinder
is the ONLY launch assumption. ROS messages use the existing emulated boundary.
Not Gazebo/PX4, not hardware qualification. Original recordings are preserved.
"""
import argparse,collections,json,math,os,sys,time,types,hashlib
from pathlib import Path
import numpy as np,cv2,mujoco
cv2.setNumThreads(1)
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/sil'))
import test_local_policy as existing
F=existing.F


def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi


def run(scene,out,seconds=60.,shift=.15,yaw0=0.,record=False):
    out.mkdir(parents=True,exist_ok=True)
    sources=[Path(__file__),ROOT/'tools/judge_demo/live_video.py',
        ROOT/'ros2_ws/src/dh_core/dh_core/live_map_adapter.py',
        ROOT/'ros2_ws/src/dh_core/dh_core/depth_evidence.py',
        ROOT/'ros2_ws/src/dh_autonomy/dh_autonomy/local_policy_node.py']
    (out/'SOURCE.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2))
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model)
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'drone')
    envelope=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'drone_envelope')
    certificate=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'placement_certificate')
    start=data.qpos[:3].copy();start[1]+=shift;start[0]+=.05;data.qpos[:3]=start
    q=Rotation.from_euler('z',yaw0).as_quat();data.qpos[3:7]=[q[3],*q[:3]]
    # ±15mm PER AXIS means sqrt(2)*15mm horizontally, not just15mm.
    model.geom_size[certificate,:2]=[.552,.285]
    model.geom_pos[certificate]=start;mujoco.mj_forward(model,data)
    collision_ids=[i for i in range(model.ngeom) if model.geom_bodyid[i]!=body and model.geom_contype[i]]
    clearance=min(mujoco.mj_geomDistance(model,data,certificate,i,10.,None) for i in collision_ids)
    if clearance<0:
        result=dict(outcome='INITIAL_PLACEMENT_FAILURE',initial_certificate_clearance=clearance,sim_time=0.,distance=0.,collisions=0)
        (out/'result.json').write_text(json.dumps(result,indent=2));print(result);return result
    F.BUS.t=1.;F.BUS.subs.clear();F.BUS.published.clear();F.BUS.timers.clear()
    F.Node.PARAM_OVERRIDES=dict(voxel_m=.025,size_xy_m=8.,z_min_m=-.2,z_max_m=2.4,stride=4,
        depth_geometry_verified=True,below_wall_height_verified=True,legal_min_altitude=.35,legal_max_altitude=1.3,
        cruise_z=.85,robot_radius=.30,robot_half_height=.12,safety_margin=.10,pose_std_floor=.005,
        vertical_clearance_margin=.02,
        static_free_max_age_s=30.,map_confidence_decay_s=30.,minimum_free_logodds=.20,
        initial_placement_verified=True,initial_clearance_radius=.53,initial_clearance_half_height=.27,
        active_perception_enabled=True,dense_free_evidence_enabled=True,rgb_depth_registration_verified=True)
    node=existing.StationaryLocalPolicyNode()
    R=np.array([[0,0,1.],[-1.,0,0],[0,-1.,0]]);q=Rotation.from_matrix(R).as_quat()
    F.TfBuffer.transforms={('base_link','physics_optical'):types.SimpleNamespace(transform=types.SimpleNamespace(
        rotation=types.SimpleNamespace(x=q[0],y=q[1],z=q[2],w=q[3]),
        translation=types.SimpleNamespace(x=.103,y=-.0375,z=.043)))}
    width,height=320,200;f=height/2/math.tan(math.radians(53.4)/2)
    K=[f,0,width/2,0,f,height/2,0,0,1]
    renderer=mujoco.Renderer(model,height,width)
    from live_video import LiveVideo
    video=LiveVideo(model,out) if record else None
    option=mujoco.MjvOption();option.geomgroup[3]=0;option.geomgroup[4]=1
    sensor_queue=collections.deque();command_queue=collections.deque();velocity_command=np.zeros(3);yaw_command=0.
    random=np.random.default_rng(7);last_position=start.copy();distance=0.;contacts=0;rows=[];minimum=10.
    began=time.perf_counter();cpu=time.process_time();frames=[]
    progress_position=start.copy();last_progress=0.;deadlocked=False
    for tick in range(int(seconds/.05)):
        sim=tick*.05;now=1+sim;F.BUS.t=now
        position=data.qpos[:3].copy();quat=data.qpos[3:7].copy();xyzw=np.r_[quat[1:],quat[0]]
        noisy=position+np.clip(random.normal(0,.005,3),-.015,.015)
        odom=F.Odometry();odom.header=F.Header(stamp=F.make_time(now),frame_id='odom')
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,noisy)
        odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,xyzw)
        for j in (0,7,14):odom.pose.covariance[j]=.005**2
        node._on_odom(odom);node.health.on_vo_status(now,1)
        if tick%2==0:
            renderer.update_scene(data,camera='forward',scene_option=option)
            rgb=renderer.render().copy();renderer.enable_depth_rendering();depth=renderer.render().copy();renderer.disable_depth_rendering()
            depth+=np.clip(random.normal(0,.003,depth.shape),-.009,.009)
            depth[(depth<.35)|(depth>6.)|(random.random(depth.shape)<.01)]=np.nan
            sensor_queue.append((now+.05,now,rgb,depth.astype('<f4')))
        while sensor_queue and sensor_queue[0][0]<=now+1e-8:
            _,capture,rgb,depth=sensor_queue.popleft()
            info=F.CameraInfo();info.header=F.Header(stamp=F.make_time(capture),frame_id='physics_optical')
            info.width=width;info.height=height;info.k=K;node._on_info(info)
            color=F.Image();color.header=info.header;color.width=width;color.height=height
            color.encoding='rgb8';color.is_bigendian=0;color.step=width*3;color.data=rgb.tobytes();node.on_rgb(color)
            msg=F.Image();msg.header=info.header;msg.width=width;msg.height=height;msg.encoding='32FC1';msg.is_bigendian=0;msg.step=width*4;msg.data=depth.tobytes()
            node._on_depth(msg);node._integrate()
        node.control_tick()
        if not node.armed_start and node.startup_ready:node.on_start(None)
        command=node.backend.last_cmd if (hasattr(node.backend,'last_cmd') and node.armed_start
            and not node.policy.aborted and node.backend.offboard_active()) else (0.,0.,0.,0.)
        command_queue.append((sim+.15,np.array(command[:3]),float(command[3])))
        while command_queue and command_queue[0][0]<=sim+1e-8:_,velocity_command,yaw_command=command_queue.popleft()
        for step in range(10):
            v=data.qvel[:3];a=np.clip((velocity_command-v)/.20,-.8,.8)
            a[2]=np.clip(8*(.85-data.qpos[2])-4*v[2],-3.,3.)
            data.xfrc_applied[body,:3]=1.222*(a+np.array([0.,0.,9.81]))
            euler=Rotation.from_quat(np.r_[data.qpos[4:7],data.qpos[3]]).as_euler('xyz')
            # Applied torque is WORLD-frame; freejoint angular velocity is body-frame.
            # Euler/body-rate mixing becomes positive feedback after a half turn.
            rotation=data.xmat[body].reshape(3,3)
            omega=rotation@data.qvel[3:6]
            torque=.12*np.cross(rotation[:,2],[0.,0.,1.])-.03*omega
            torque[2]=.025*(yaw_command-omega[2])
            data.xfrc_applied[body,3:]=torque
            mujoco.mj_step(model,data)
            for contact in data.contact:
                if envelope in (contact.geom1,contact.geom2) and contact.dist<-.0001:contacts+=1
        distance+=float(np.linalg.norm(data.qpos[:3]-last_position));last_position=data.qpos[:3].copy()
        if np.linalg.norm(last_position-progress_position)>.05:
            progress_position=last_position.copy();last_progress=sim
        if tick%2==0:
            minimum=min(minimum,min(mujoco.mj_geomDistance(model,data,envelope,i,2.,None) for i in collision_ids))
            rows.append(dict(time=sim,position=position.tolist(),yaw=float(euler[2]),command=list(command),state=node.policy.state,
                startup=node.startup_reason,scan_remaining=node.scan_remaining,replans=node.policy.replans,distance=distance,
                capture_time=node.last_capture,valid_fraction=node.valid_fraction,
                measured_velocity=data.qvel[:3].tolist()))
            if video:video.frame(data,node,rgb,depth,position,float(euler[2]),sim,distance,minimum)
            if tick%20==0 and node.policy.view is not None:
                np.savez_compressed(out/f'sensors_{tick:06d}.npz',rgb=rgb,depth=depth,
                    states=node.policy.view.states,position=position,yaw=euler[2],time=sim)
        if tick%20==0:
            print(f'{sim:.1f}s {node.policy.state} start={node.startup_reason} scan={node.scan_remaining:.2f} p={position.round(2)} d={distance:.2f} plans={node.policy.replans} valid={node.valid_fraction:.2f} tf={node.tf_ok}',flush=True)
            if node.policy.view is not None:
                print('MAP',dict(free=int((node.policy.view.states==0).sum()),safe=int(node.policy.view.planner.free.sum())),flush=True)
            (out/'trace.json').write_text(json.dumps(rows))
        if contacts:break
        if sim>45 and sim-last_progress>25:
            deadlocked=True;break
    renderer.close()
    if video:video.close()
    outcome='COLLISION' if contacts else ('SENSOR_LOCALIZATION_FAILURE' if node.policy.aborted else
        ('NAVIGATION_DEADLOCK' if deadlocked or distance<.3 else 'TIMEOUT'))
    result=dict(outcome=outcome,sim_time=sim,distance=distance,replans=node.policy.replans,collisions=contacts,
        minimum_clearance=minimum,initial_certificate_clearance=clearance,wall_s=time.perf_counter()-began,cpu_s=time.process_time()-cpu,
        final_position=data.qpos[:3].tolist(),pose_noise_std=.005,depth_noise_std=.003,command_delay=.15,
        physics='MuJoCo4.0',ROS_DDS=False,PX4=False,course_geometry_given_to_planner=False,mission_complete=False)
    result.update(initial_position=start.tolist(),robot_radius=.30,robot_half_height=.12,
        horizontal_margin=.10,vertical_margin=.02,initial_certificate_radius=.53,
        placement_check_radius=.552,placement_check_half_height=.285,free_evidence_max_age=30.,live_depth_timeout=.3,
        simulated_vertical_error_bound=.02,full_start_scan=True,
        blue_pad_detections=node.pad_detections,finish_found=node.pad_track.count>=3)
    if node.policy.view is not None:
        result.update(free_columns=int((node.policy.view.states==0).sum()),
            occupied_columns=int((node.policy.view.states==1).sum()),
            observed_columns=int(((node.policy.view.states==0)|(node.policy.view.states==1)).sum()),
            planning_grid_columns=int(node.policy.view.states.size))
    (out/'result.json').write_text(json.dumps(result,indent=2));(out/'trace.json').write_text(json.dumps(rows));print(result,flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=60.);p.add_argument('--shift',type=float,default=.15);p.add_argument('--yaw',type=float,default=0.)
    p.add_argument('--record',action='store_true')
    a=p.parse_args();run(a.scene,a.out,a.seconds,a.shift,a.yaw,a.record)
