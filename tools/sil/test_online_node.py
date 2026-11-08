"""Real online node/OpenCV pathways on fake ROS ONLY; no DDS or hardware."""
import sys
import types
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/dh_core'),str(ROOT/'ros2_ws/src/dh_autonomy')]
import fake_ros as F
F.install()
for module,names in (('sensor_msgs.msg',('PointCloud2','PointField')),('geometry_msgs.msg',('PolygonStamped','Point32'))):
    for name in names: setattr(sys.modules[module],name,F._msg_class(name))
sys.modules['sensor_msgs.msg'].PointField.FLOAT32=7
original=F.Node.declare_parameter
def declare(self,name,default): original(self,name,default); return self.get_parameter(name)
F.Node.declare_parameter=declare
from dh_autonomy.online_node import OnlineStationaryNode,OnlineMissionNode,decode_depth,decode_color
from dh_autonomy.backends import DryRunBackend,verify_isolation
from dh_core.online_geometry import nominal_transform_for_tests
from pad_scene import render_pad


def fresh(**params):
    F.BUS.t=1.; F.BUS.subs.clear(); F.BUS.published.clear(); F.BUS.timers.clear(); F.BUS.service_calls.clear()
    F.TfBuffer.transforms={}; F.Node.PARAM_OVERRIDES=dict(map_size=8.,**params)
    return OnlineStationaryNode()


def add_pose(node,t):
    F.BUS.t=t; m=F.Odometry(); m.header.stamp=F.make_time(t); m.header.frame_id='odom'
    m.pose.pose.position.z=.8
    node.on_odom(m); node.nav.tracking(t,1,t)


def transform(tilt):
    R,t=nominal_transform_for_tests(tilt); rod,_=cv2.Rodrigues(R); angle=np.linalg.norm(rod); xyz=rod[:,0]/angle*np.sin(angle/2)
    return types.SimpleNamespace(transform=types.SimpleNamespace(
        rotation=types.SimpleNamespace(x=xyz[0],y=xyz[1],z=xyz[2],w=np.cos(angle/2)),
        translation=types.SimpleNamespace(x=t[0],y=t[1],z=t[2])))


def feed(node,t):
    image,depth,K,_,_=render_pad()
    add_pose(node,t)
    for typ,frame,data,encoding in (('depth','depth_optical',depth.astype('<f4'),'32FC1'),('color','color_optical',image,'bgr8')):
        m=F.Image(); m.header.stamp=F.make_time(t); m.header.frame_id=frame; m.encoding=encoding; m.is_bigendian=0
        m.width=data.shape[1]; m.height=data.shape[0]; m.step=data.strides[0]; m.data=data.tobytes()
        info=F.CameraInfo(); info.header=F.Header(frame_id=frame,stamp=F.make_time(t)); info.width=m.width; info.height=m.height; info.k=K.ravel().tolist()
        if typ=='depth': node.depth_info=info; node.depths.append(m)
        else: node.color_info=info; node.colors.append(m)
    node.process_depth()


def test_stationary_cannot_select_physical_backend():
    n=fresh(dry_run=False,backend='px4')
    assert isinstance(n.backend,DryRunBackend) and not n.clients
    verify_isolation(n,True)
    n.executor.shutdown()


def test_unverified_geometry_does_not_refresh_depth():
    n=fresh(); feed(n,1.)
    assert n.nav.map.last_stamp is None
    n.executor.shutdown()


def test_wrong_tf_tilt_rejected():
    n=fresh(depth_geometry_verified=True)
    F.TfBuffer.transforms[('base_link','depth_optical')]=transform(15.)
    feed(n,1.)
    assert n.nav.map.last_stamp is None and n.rejected
    n.executor.shutdown()


def test_real_hsv_registration_and_tf_to_tracked_goal():
    n=fresh(depth_geometry_verified=True)
    for frame in ('depth_optical','color_optical'): F.TfBuffer.transforms[('base_link',frame)]=transform(0.)
    for t in (1.,1.1,1.2): feed(n,t)
    assert n.nav.map.last_stamp==1.2,n.rejected
    assert n.pad.count==3,(n.pad_state,n.rejected)
    assert n.nav.goal is not None and np.linalg.norm(n.nav.goal.position-[2.8,0,.65])<.08
    assert n.colors[-1].header.frame_id=='color_optical'  # no input-header mutation
    verify_isolation(n,True)
    n.executor.shutdown()


def test_depth_decode_rejects_bad_buffers_and_color_mono():
    m=F.Image(); m.encoding='32FC1'; m.width=3; m.height=2; m.step=12; m.is_bigendian=0; m.data=b'bad'
    try: decode_depth(m); raise AssertionError('accepted broken depth')
    except ValueError: pass
    m.encoding='mono8'
    try: decode_color(m); raise AssertionError('accepted mono as color')
    except ValueError: pass


def test_future_skybeta_readiness_authority_and_takeover_contract():
    import json
    from dh_core.timing import time_parameterize
    from dh_core.tracker import TrajectoryFollower
    n=fresh(); n.executor.shutdown()  # reset fake graph/parameters before normal entrypoint
    F.BUS.subs.clear(); F.BUS.published.clear()
    F.Node.PARAM_OVERRIDES=dict(dry_run=False,backend='skybeta',map_size=8.,
        legal_altitude_verified=True,legal_min_altitude=.35,legal_max_altitude=2.3,
        floor_height_verified=True,floor_z=0.)
    n=OnlineMissionNode(); add_pose(n,1.)
    n.nav.map.model.grid.fill(-1.); n.nav.map.seen.fill(1.); n.nav.map.last_stamp=1.
    def state(active,t):
        n.backend._on_state(F.String(data=json.dumps(dict(stamp=t,offboard_active=active,armed=True,px4_link=True,frozen=False))))
    state(False,1.); n.tick()
    commands=F.BUS.published['/skybeta/setpoint/twist']
    assert all(m.twist.linear.x==m.twist.linear.y==m.twist.linear.z==0 for _,m in commands)
    assert not F.BUS.service_calls and not any(t.startswith('/fmu/in') for t in F.BUS.published)
    n.start(None)
    n.nav.follower=TrajectoryFollower(time_parameterize(np.array([[0,0,.8],[.5,0,.8],[1,0,.8]]),.5,.6,.6))
    n.tick(); assert n.nav.follower.tau==0  # no Offboard, no clock advancement
    add_pose(n,1.05); state(True,1.05); n.tick()
    assert n.nav.follower.tau>0
    add_pose(n,1.1); state(False,1.1); count=len(commands); n.tick()
    assert n.nav.aborted and len(commands)==count
    add_pose(n,1.15); state(True,1.15); n.tick()
    assert len(commands)==count  # permanent takeover latch, no automatic resume
    n.executor.shutdown()


def test_online_normal_cannot_select_raw_px4():
    F.Node.PARAM_OVERRIDES=dict(dry_run=False,backend='px4',map_size=4.)
    try:
        OnlineMissionNode(); raise AssertionError('Raw PX4 accepted')
    except ValueError: pass
    assert not any(t.startswith('/fmu/in') for t in F.BUS.published)


if __name__=='__main__':
    tests=[value for name,value in list(globals().items()) if name.startswith('test_')]
    for test in tests: test(); print('PASS',test.__name__)
    print('online fake-ROS processing:',len(tests),'passed')
