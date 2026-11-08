"""Focused existing-fixture test: real team mapper + existing planner/filter/follower.

Uses fake ROS and the EXISTING UnknownWorld sensor/grader; no Gazebo or PX4.
DH_TEAM_SOURCE must name a git-exported team-main source or installed checkout.
"""
import json
import math
import os
from pathlib import Path
import sys
import types
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[os.environ['DH_TEAM_SOURCE'],str(ROOT/'ros2_ws/src/dh_core'),str(ROOT/'ros2_ws/src/dh_autonomy')]
import fake_ros as F
F.install()
sys.modules['sensor_msgs.msg'].PointCloud2=F._msg_class('PointCloud2')
pc=types.ModuleType('sensor_msgs_py'); pc.point_cloud2=types.SimpleNamespace(create_cloud_xyz32=lambda h,p:types.SimpleNamespace(header=h,points=p))
sys.modules['sensor_msgs_py']=pc
original=F.Node.declare_parameter
def declare(self,name,default): original(self,name,default); return self.get_parameter(name)
F.Node.declare_parameter=declare
from dronehacks_nav.mapping import VoxelMap
from dh_autonomy.local_policy_node import TimedTeamMap,StationaryLocalPolicyNode,LocalPolicyNode
from dh_autonomy.backends import DryRunBackend,verify_isolation
from dh_core.live_map_adapter import from_team_map,LocalPolicy
from dh_core.safety_filter import SensorInputs
from unknown_environment import UnknownWorld


def view_fixture(now=1.,unknown=False):
    m=VoxelMap(.1,6.,-.2,2.8); m.grid.fill(-2.)
    times=np.full(m.shape,now)
    if unknown: m.grid[:,:,12]=0.
    return from_team_map(m,times,now,now,1.,(.35,1.5),True)


def inputs(now=1.,**changes):
    kw=dict(now=now,free_distance=3.,depth_stamp=now,depth_rx=now,
            odom_stamp=now,odom_rx=now,pos_std=.03,transform_valid=True,loc_mode='OK')
    kw.update(changes); return SensorInputs(**kw)


def test_missing_unknown_height_and_legal_limits_fail_closed():
    p=LocalPolicy(); p.update_map(view_fixture(unknown=True))
    assert not p.plan(np.array([0.,0.,1.]),0,1.)
    m=VoxelMap(.1,4.,-.2,2.8)
    try:
        from_team_map(m,np.ones(m.shape),1.,1.,1.); raise AssertionError('Unverified legal height accepted')
    except ValueError: pass


def test_sensor_failure_and_takeover_gates():
    for bad in (dict(free_distance=None),dict(free_distance=float('nan')),dict(depth_stamp=.2),
                dict(odom_stamp=.2),dict(transform_valid=False),dict(loc_mode='HOVER'),dict(depth_valid_fraction=.1)):
        p=LocalPolicy(); p.update_map(view_fixture()); assert p.plan(np.array([0.,0.,1.]),0,1.)
        tau=p.gate.follower.tau
        v,_=p.step(np.array([0.,0.,1.]),0,.05,inputs(**bad),True)
        assert np.allclose(v,0) and p.gate.follower.tau==tau,(bad,p.state)
    p=LocalPolicy(); p.update_map(view_fixture()); assert p.plan(np.array([0.,0.,1.]),0,1.)
    p.step(np.array([0.,0.,1.]),0,.05,inputs(),False); assert p.gate.follower.tau==0
    p.step(np.array([0.,0.,1.]),0,.05,inputs(),True)
    v,_=p.step(np.array([0.,0.,1.]),0,.05,inputs(),False)
    assert p.aborted and np.allclose(v,0)
    assert np.allclose(p.step(np.array([0.,0.,1.]),0,.05,inputs(),True)[0],0)


def test_stale_map_cannot_leave_queued_motion():
    p=LocalPolicy(); p.update_map(view_fixture()); assert p.plan(np.array([0.,0.,1.]),0,1.)
    assert np.allclose(p.step(np.array([0.,0.,1.]),0,.05,inputs(1.6),True)[0],0)
    assert p.state=='HOLD_MAP_STALE'


def test_stationary_executable_isolated_despite_false_parameter():
    F.Node.PARAM_OVERRIDES=dict(dry_run=False,voxel_m=.2,size_xy_m=6.)
    node=StationaryLocalPolicyNode()
    assert isinstance(node.backend,DryRunBackend)
    verify_isolation(node,True)
    assert not node.clients


def test_bounded_commands_and_new_obstacle_veto_without_a_plant():
    p=LocalPolicy(); view=view_fixture(); p.update_map(view)
    assert p.plan(np.array([0.,0.,1.]),0,1.)
    commands=[]
    for index in range(1,9):
        t=1+index*.05
        command,_=p.step(np.array([0.,0.,1.]),0,.05,inputs(t),True)
        assert np.linalg.norm(command)<=.3+1e-9 and command[2]==0
        if commands: assert np.linalg.norm(command-commands[-1])<=.4*.05+1e-9
        commands.append(command)
    assert any(np.linalg.norm(v)>0 for v in commands),p.state
    # A changed occupied/unknown mask vetoes the remaining path immediately.
    view.planner.free[:]=False
    assert np.allclose(p.step(np.array([0.,0.,1.]),0,.05,inputs(1.4),True)[0],0)
    assert p.state=='HOLD_PATH_CHANGED'
    assert np.allclose(p.step(np.array([0.,0.,1.]),0,float('nan'),inputs(1.4),True)[0],0)


def test_real_team_depth_tf_adapter_rejects_uncertified_start_volume():
    import cv2
    from pad_scene import render_pad
    from dh_core.online_geometry import nominal_transform_for_tests
    F.BUS.t=1.
    F.Node.PARAM_OVERRIDES=dict(voxel_m=.2,size_xy_m=6.,depth_geometry_verified=True)
    node=StationaryLocalPolicyNode()
    R,t=nominal_transform_for_tests(0.); rod,_=cv2.Rodrigues(R)
    angle=np.linalg.norm(rod); xyz=rod[:,0]/angle*np.sin(angle/2)
    F.TfBuffer.transforms={('base_link','depth_optical'):types.SimpleNamespace(transform=types.SimpleNamespace(
        rotation=types.SimpleNamespace(x=xyz[0],y=xyz[1],z=xyz[2],w=np.cos(angle/2)),
        translation=types.SimpleNamespace(x=t[0],y=t[1],z=t[2])))}
    image,depth,K,_,_=render_pad()
    depth=cv2.resize(depth,(320,200),interpolation=cv2.INTER_NEAREST).astype('<f4'); K=K.copy(); K[:2]/=3
    info=F.CameraInfo(); info.header=F.Header(frame_id='depth_optical'); info.width=320; info.height=200; info.k=K.ravel().tolist()
    node._on_info(info)
    odom=F.Odometry(); odom.header.stamp=F.make_time(1.); odom.header.frame_id='odom'; odom.pose.pose.position.z=.8
    node._on_odom(odom); node.health.on_vo_status(1.,1)
    m=F.Image(); m.header=F.Header(frame_id='depth_optical',stamp=F.make_time(1.)); m.encoding='32FC1'
    m.width=320; m.height=200; m.step=1280; m.is_bigendian=0; m.data=depth.tobytes()
    node._on_depth(m); node._integrate()
    assert node.map.frames==1 and node.last_capture==1. and node.valid_fraction>0
    assert node.policy.view is None  # actual legal limits unconfirmed, no invented free space
    verify_isolation(node,True)
    # A newer malformed image must invalidate current coverage, not refresh it.
    F.BUS.t=1.1; odom.header.stamp=F.make_time(1.1); node._on_odom(odom)
    m.header.stamp=F.make_time(1.1); m.data=b'bad'; node._on_depth(m); node._integrate()
    assert not node.tf_ok and node.valid_fraction==0


def test_actual_node_skybeta_handshake_and_takeover_fake_bus_only():
    F.BUS.t=1.; F.BUS.subs.clear(); F.BUS.published.clear(); F.BUS.service_calls.clear()
    F.Node.PARAM_OVERRIDES=dict(dry_run=False,voxel_m=.2,size_xy_m=6.,physical_configuration_reviewed=True)
    node=LocalPolicyNode(); node.pose=(np.array([0.,0.,1.]),0.)
    node.get_node_names=lambda:['flight_interface','dh_local_policy']
    node.get_name=lambda:'dh_local_policy'; node.get_namespace=lambda:'/'
    node.get_publishers_info_by_topic=lambda topic:[types.SimpleNamespace(node_name='dh_local_policy',node_namespace='/')]
    node.geometry_verified=True; node.rules_verified=True; node.cruise=1.
    node.policy.update_map(view_fixture()); node.K=(100.,100.,50.,50.)
    node.current_msg=types.SimpleNamespace(width=100); node.clearance_points=np.tile([3.,0.,0.],(10,1))
    node.last_capture=1.; node.tf_ok=True; node.valid_fraction=1.
    node.clearance_origin=node.pose[0].copy(); node.capture_yaw=0.
    def update(active,t):
        F.BUS.t=t; node.health.on_odom(t,t,node.pose[0],.03,yaw=0.); node.health.on_vo_status(t,1)
        node.odom_valid=True
        node.backend._on_state(F.String(data=json.dumps(dict(stamp=t,offboard_active=active,armed=True,px4_link=True,frozen=False))))
    update(False,1.); node.control_tick()
    cmds=F.BUS.published['/skybeta/setpoint/twist']
    assert cmds and all(m.twist.linear.x==m.twist.linear.y==m.twist.linear.z==0 for _,m in cmds)
    node.on_start(None); node.control_tick(); assert node.policy.gate.follower.tau==0
    update(True,1.05); node.control_tick(); assert node.policy.gate.follower.tau>0
    count=len(cmds); update(False,1.1); node.control_tick()
    assert node.policy.aborted and len(cmds)==count
    update(True,1.15); node.control_tick(); assert len(cmds)==count
    assert not F.BUS.service_calls and not any(t.startswith('/fmu/in') for t in F.BUS.published)
    node.policy.aborted=False; node.policy.had_authority=False
    node.get_publishers_info_by_topic=lambda topic:[types.SimpleNamespace(node_name='another_mission',node_namespace='/')]
    node.control_tick()
    assert not node.startup_ready and 'EXCLUSIVE' in node.startup_reason and len(cmds)==count


def test_capture_heading_and_translation_are_not_current_heading():
    from dh_core.live_map_adapter import clearance_at_current_pose
    from refnav import depth as D
    # Obstacle was2m east. Vehicle translated .5m east and turned north.
    current=clearance_at_current_pose(np.array([[2.,0.,0.]]),np.array([0.,0.,1.]),0.,
        np.array([.5,0.,1.]),math.pi/2)
    assert np.allclose(current,[[0.,1.5,0.]],atol=1e-8)
    assert abs(D.clearance(current,bearing_rad=math.pi/2,half_width_m=.3,band_m=.3,min_hits=1)-1.5)<1e-8


def test_velocity_is_enu_world_not_body_at_ninety_degree_yaw():
    p=LocalPolicy(); p.update_map(view_fixture()); position=np.array([0.,0.,1.])
    assert p.plan(position,math.pi/2,1.)
    end=p.gate.follower.traj.p[-1]
    assert end[1]>abs(end[0])  # selected destination is north in odom world
    # Isolate the command-frame contract from the planner's first cell-centre leg.
    from dh_core.timing import time_parameterize
    from dh_core.tracker import TrajectoryFollower
    p.gate.follower=TrajectoryFollower(time_parameterize(np.array([[0.,0.,1.],[0.,.5,1.],[0.,1.,1.]]),.3,.4,.4))
    commands=[p.step(position,math.pi/2,.05,inputs(1+i*.05),True)[0] for i in range(1,9)]
    velocity=commands[-1]
    assert velocity[1]>abs(velocity[0]) and velocity[2]==0,velocity
    sink=DryRunBackend(F.Node('enu_contract')); sink.send_velocity(*map(float,velocity),0.)
    message=F.BUS.published['/dh/dry_run/cmd_vel'][-1][1]
    assert message.header.frame_id=='odom' and message.twist.linear.y==velocity[1]


def test_startup_is_passive_and_refuses_unknown_volume():
    F.Node.PARAM_OVERRIDES=dict(voxel_m=.2,size_xy_m=6.)
    node=StationaryLocalPolicyNode(); node.geometry_verified=True; node.rules_verified=True; node.cruise=1.
    node.policy.update_map(view_fixture(unknown=True))
    before=node.map.grid.copy()
    ready,reason=node.startup_check(np.array([0.,0.,1.]),inputs())
    assert not ready and reason=='WAIT_MEASURED_FULL_START_VOLUME'
    node.startup_ready=ready; node.startup_reason=reason; node.on_start(None)
    assert not node.armed_start and not node.backend.offboard_active()
    assert np.array_equal(before,node.map.grid)


def test_physical_constructor_requires_review_before_command_endpoints():
    names=[]; original=F.Node.create_publisher
    def record(self,kind,topic,qos):
        names.append(topic); return original(self,kind,topic,qos)
    F.Node.create_publisher=record
    F.Node.PARAM_OVERRIDES=dict(dry_run=False,voxel_m=.2,size_xy_m=6.)
    try:
        try: LocalPolicyNode(); raise AssertionError('Physical review gate missing')
        except ValueError as e: assert 'reviewed configuration' in str(e)
    finally: F.Node.create_publisher=original
    assert not any(name.startswith(('/skybeta','/fmu')) for name in names)


def test_short_obstacle_scenario():
    world=UnknownWorld('open',0)  # existing sensor producer/grader ONLY
    m=TimedTeamMap(VoxelMap(.25,12.,-1.,3.)); policy=LocalPolicy()
    p=np.array([-3.,0.,1.]); yaw=0.; length=0.; collisions=0; states={}; moving=0; stopped=False
    sink=DryRunBackend(F.Node('test_command_sink')); sink.grant_authority()
    for step in range(130):
        now=1.+step*.1; F.BUS.t=now
        if step==40:
            assert policy.gate is not None
            follower=policy.gate.follower
            anchor=follower.traj.sample(min(follower.tau+2.5,follower.traj.duration))[0]
            assert np.linalg.norm(anchor[:2]-p[:2])>.7,'Fixture must not spawn a collision inside the drone'
            world.add_box(anchor+[-.12,-.12,-2.],anchor+[.12,.12,2.])
            # Prove this obstacle actually intersects the currently commanded route.
            assert np.any(np.linalg.norm(follower.traj.p[:,:2]-anchor[:2],axis=1)<.15)
            assert not world.collision(p)
        origin,dirs,ranges=world.observe(p,yaw,panoramic=True)
        m.stamp=now; m.integrate(origin,dirs,ranges,4.)
        policy.update_map(from_team_map(m,m.observed_at,now,now,1.,(.35,1.5),True,revision=m.frames))
        if policy.gate is None or policy.gate.follower.done: policy.plan(p,yaw,now)
        measured=float(world.raycast(p,np.array([[math.cos(yaw),math.sin(yaw),0.]]))[0])
        v,yr=policy.step(p,yaw,.1,inputs(now,free_distance=measured),True)
        if step>=40 and np.linalg.norm(v)<1e-9 and policy.state.startswith('HOLD'): stopped=True
        if np.linalg.norm(v)>1e-5: moving+=1
        sink.send_velocity(*map(float,v),float(yr))
        length+=np.linalg.norm(v)*.1; p+=v*.1; yaw+=yr*.1
        collisions+=int(world.collision(p)); states[policy.state]=states.get(policy.state,0)+1
        if collisions: break
    result=dict(steps=step+1,path_length_m=float(length),moving_commands=moving,replans=policy.replans,
        obstacle_stop_observed=stopped,collisions=collisions,state_counts=states,
        input_model='existing ideal panoramic ray/odometry fixture; not physical forward camera',
        output='real DryRunBackend on fake ROS',ROS=False,Gazebo=False,hardware=False)
    print('SCENARIO_RESULT',json.dumps(result),flush=True)
    assert moving>5 and length>.3 and stopped and collisions==0,result
    return result


if __name__=='__main__':
    # User paused simulation runs. Only lightweight component checks by default.
    names=[n for n in globals() if n.startswith('test_') and n!='test_short_obstacle_scenario']
    if '--scenario' in sys.argv: names.append('test_short_obstacle_scenario')
    for name in names:
        globals()[name](); print('PASS',name,flush=True)
