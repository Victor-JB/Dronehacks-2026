"""Unknown/free/occupied, 3D planning and health invariants; no ROS/truth inputs."""
import math
import numpy as np
import pytest
from dh_core.online_map import OnlineMap,MapView,FREE,UNKNOWN,OCCUPIED
from dh_core.online_planning import astar3d,plan_observed,exploration_goal
from dh_core.online_navigation import OnlineNavigator,GoalObservation
from dh_core.online_geometry import RigidHistory,validate_mount,coverage,nominal_transform_for_tests


def known_view():
    safe=np.ones((24,24,12),bool)
    safe[[0,-1]]=False; safe[:,[0,-1]]=False; safe[:,:,[0,-1]]=False
    return MapView(np.array([-2.4,-2.4,-.2]),.2,np.where(safe,FREE,UNKNOWN).astype(np.int8),safe,1)


def test_unknown_not_free_and_bad_ranges_do_not_carve():
    m=OnlineMap(size=4.)
    for ranges in ([np.nan],[np.inf],[0.],[-1.]):
        assert not m.observe([0,0,1],[[1,0,0]],ranges,1,1)
    assert (m.snapshot(1).states==UNKNOWN).all()


def test_epoch_precision_replay_and_expiration():
    m=OnlineMap(size=4.,free_ttl=.5); t=1794104000.123
    assert m.observe([0,0,1],[[1,0,0]],[1.],t,t)
    assert not m.observe([0,0,1],[[1,0,0]],[1.],t,t+.1)
    assert (m.snapshot(t+.1).states==FREE).any()
    assert not (m.snapshot(t+.6).states==FREE).any()
    assert (m.snapshot(t+.6).states==OCCUPIED).any()


def test_unobserved_height_blocks_flight_volume():
    m=OnlineMap(res=.2,size=4.)
    m.model.grid[:,:,7]=-1.; m.seen[:,:,7]=1.
    assert not m.snapshot(1).safe.any()  # One clear scan plane cannot certify a vehicle.


def test_vertical_opening_and_no_teleport_to_nearest_free():
    v=known_view(); v.safe[12,:,:]=False; v.safe[12,10:15,7:10]=True
    p=astar3d(v,[-1,0,.6],[1,0,.6])
    assert p is not None and max(p[:,2])>=1.3
    assert v.path_safe(p)
    v.safe[7,12,4]=False
    assert astar3d(v,[-.9,.1,.7],[1,0,.6]) is None


@pytest.mark.parametrize('planner',['clothoid','sfc','mpl'])
def test_three_planners_use_certified_altitude_stages(planner):
    v=known_view(); v.safe[12,:,:]=False; v.safe[12,10:15,7:10]=True
    t,stats=plan_observed(v,[-1,0,.6],[1,0,.6],planner,.5)
    assert t is not None,stats
    assert stats['shared_3d_stages']>0 and v.path_safe(t.p)
    directions=np.diff(t.p,axis=0); norm=np.linalg.norm(directions,axis=1)
    unit=directions/np.maximum(norm[:,None],1e-9)
    for i in range(1,len(t.p)-1):
        if abs(unit[i-1,2]-unit[i,2])>.3:
            assert t.v[i]<1e-6


@pytest.mark.parametrize('planner',['clothoid','sfc','mpl'])
def test_all_planners_share_observed_space(planner):
    v=known_view()
    t,stats=plan_observed(v,[-1.,.1,1.1],[1.,.1,1.1],planner,.5)
    assert t is not None,stats
    assert stats['native_horizontal_calls']>0 and v.path_safe(t.p)
    assert np.linalg.norm(t.p[-1]-[1,.1,1.1])<1e-6


def test_dynamic_obstacle_invalidates_route():
    v=known_view(); p=astar3d(v,[-1,0,1],[1,0,1]); assert v.path_safe(p)
    v.safe[12,:,:]=False
    assert not v.path_safe(p)


def test_first_hit_overrides_old_free_confidence():
    m=OnlineMap(size=4.)
    for stamp in (1.,1.1,1.2,1.3,1.4): m.observe([0,0,1],[[1,0,0]],[2.],stamp,stamp)
    assert m.observe([0,0,1],[[1,0,0]],[.8],1.5,1.5)
    cell=tuple(np.floor((np.array([.8,0,1])-m.model.lo)/m.model.voxel).astype(int))
    assert m.snapshot(1.5).states[cell]==OCCUPIED


def test_no_frontier_behind_fully_observed_walls():
    v=known_view(); v.states[:]=OCCUPIED; v.states[v.safe]=FREE
    assert exploration_goal(v,[0,0,1]) is None


def test_health_failure_and_authority_freeze_commands():
    n=OnlineNavigator(map_options=dict(size=4.,legal_band=(.35,2.3),floor_z=0.)); n.active=True
    n.odometry([0,0,1],0,1,1); n.tracking(1,1,1)
    assert np.allclose(n.tick(1,.03)[0],0) and 'DEPTH' in n.state
    n.had_authority=True
    n.tick(1,.03,authority=False)
    assert n.aborted and np.allclose(n.tick(1.1,.03,True)[0],0)


def test_pose_jump_discards_map_and_goal():
    n=OnlineNavigator(map_options=dict(size=4.)); n.odometry([0,0,1],0,1,1)
    n.map.observe([0,0,1],[[1,0,0]],[1.],1,1)
    n.set_goal(GoalObservation(np.array([1,0,1]),1,.9,'blue_pad'),1)
    n.odometry([0,0,2],0,1.03,1.03)
    assert n.epoch==1 and n.map.last_stamp is None and n.goal is None


def test_rigid_interpolation_and_no_future_pose():
    h=RigidHistory(); h.add(1,[0,0,0],[0,0,0,1]); h.add(1.2,[.2,0,0],[0,0,math.sin(.1),math.cos(.1)])
    p,R=h.at(1.1); assert np.allclose(p,[.1,0,0])
    assert abs(math.atan2(R[1,0],R[0,0])-.1)<1e-6
    assert h.at(1.21) is None


def test_roll_correction_invalidates_map_even_without_yaw_change():
    n=OnlineNavigator(map_options=dict(size=4.))
    assert n.odometry([0,0,1],0,1,1,orientation=[0,0,0,1])
    n.map.observe([0,0,1],[[1,0,0]],[1.],1,1)
    assert n.odometry([0,0,1],0,1.03,1.03,orientation=[math.sin(.3),0,0,math.cos(.3)])
    assert n.epoch==1 and n.map.last_stamp is None and n.health.jump_since is not None


def test_stale_odometry_and_stale_goal_stop_without_discarding_unknown():
    n=OnlineNavigator(map_options=dict(size=4.,legal_band=(.35,2.3),floor_z=0.),required_passages=()); n.active=True
    n.odometry([0,0,1],0,1,1); n.tracking(1,1,1)
    n.map.observe([0,0,1],[[1,0,0]],[1.],1,1)
    assert np.allclose(n.tick(1.41,.1)[0],0) and 'odometry stale' in n.state
    assert not n.odometry([0,0,1],0,1,1.41)
    n.set_goal(GoalObservation(np.array([1,0,1]),1,.9,'blue_pad'),1)
    n.odometry([0,0,1],0,3.1,3.1); n.tracking(3.1,1,3.1)
    n.map.observe([0,0,1],[[1,0,0]],[1.],3.1,3.1)
    assert np.allclose(n.tick(3.1,.1)[0],0) and n.state=='HOLD_GOAL_UNCERTAIN'


def test_forward_mount_is_measured_and_coverage_does_not_include_nadir():
    R,_=nominal_transform_for_tests(0.)
    assert validate_mount(R)
    old,_=nominal_transform_for_tests(15.); assert not validate_mount(old)
    K=[595.1951,0,487.3531,0,595.3043,305.8477,0,0,1]
    c=coverage(K,960,600,0.,1.)
    assert 25<c['upward_degrees']<29 and 2<c['nearest_floor_forward_m']<2.2
    assert not c['nadir_visible']
