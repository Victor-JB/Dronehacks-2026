"""Regression checks for startup evidence; no ROS runtime, physics or hardware."""
import test_local_policy as T
import numpy as np
from dh_core.live_map_adapter import InitialVolume,from_team_map
from dh_core.depth_evidence import carve_visible_band


def test_retention_does_not_refresh_live_capture():
    m=T.VoxelMap(.1,4.,-.2,2.8);m.grid.fill(-2.);times=np.ones(m.shape)
    args=(m,times,10.,10.,1.,(.35,1.5),True)
    assert from_team_map(*args).planner.free.sum()==0
    assert from_team_map(*args,free_max_age_s=20.,min_free_logodds=.2).safe([[0,0,1]])[0]
    assert not from_team_map(*args,free_max_age_s=5.).safe([[0,0,1]])[0]
    assert not from_team_map(*args,free_max_age_s=20.,decay_time_s=1.,min_free_logodds=.2).safe([[0,0,1]])[0]
    try:
        from_team_map(m,times,1.,10.,1.,(.35,1.5),True,free_max_age_s=20.)
        raise AssertionError('Retained map bypassed live capture deadline')
    except ValueError:pass


def test_placement_is_bounded_expiring_and_never_overrides_hits():
    m=T.VoxelMap(.05,4.,0.,2.);times=np.full(m.shape,-np.inf)
    cert=InitialVolume(np.array([0.,0.,1.]),.6,.4,1.)
    args=(m,times,1.,1.,1.,(.35,1.5),True)
    view=from_team_map(*args,initial_volume=cert)
    assert view.safe([[0,0,1]])[0] and not view.safe([[.5,0,1]])[0]
    assert not cert.mask(m,32.).any()
    mask=cert.mask(m,1.);centres=m.centre(np.flatnonzero(mask))
    assert (np.linalg.norm(centres[:,:2],axis=1)+np.sqrt(2)*.025<=.6+1e-9).all()
    m.grid[mask]=1.
    assert not from_team_map(*args,initial_volume=cert).safe([[0,0,1]])[0]


def test_dense_projection_cannot_carve_occlusion_invalid_or_occupied_cells():
    m=T.VoxelMap(.05,3.,-1.,2.);times=np.full(m.shape,-np.inf)
    depth=np.full((120,160),2.,np.float32);K=(100.,100.,80.,60.)
    def cell(p):return tuple(np.floor((np.array(p)-m.lo)/m.voxel).astype(int))
    origin=np.array([0.,0.,0.]);R=np.eye(3)
    count=carve_visible_band(m,times,depth,K,R,origin,1.,.5,1.5,max_range=2.)
    assert count>0 and m.grid[cell([0,0,1])]<0
    assert m.grid[cell([1.,0.,1.])]==0 # outside field of view
    m.grid.fill(0);times.fill(-np.inf);depth[40:80,60:100]=np.nan
    carve_visible_band(m,times,depth,K,R,origin,2.,.5,1.5,max_range=2.)
    assert m.grid[cell([0,0,1])]==0 # invalid rectangle never treated as a clear ray
    depth[:]=.7;m.grid.fill(0)
    carve_visible_band(m,times,depth,K,R,origin,3.,.5,1.5,max_range=2.)
    assert m.grid[cell([0,0,1])]==0 # behind measured obstacle
    depth[:]=2.;m.grid[cell([0,0,1])]=1.
    carve_visible_band(m,times,depth,K,R,origin,4.,.5,1.5,max_range=2.)
    assert m.grid[cell([0,0,1])]==1. # independent immediate-hit veto


def test_stationary_scan_requires_live_evidence_and_aborts_on_takeover():
    F=T.F;F.BUS.t=1.;F.Node.PARAM_OVERRIDES=dict(voxel_m=.2,size_xy_m=6.,active_perception_enabled=True)
    node=T.StationaryLocalPolicyNode();node.pose=(np.array([0.,0.,1.]),0.)
    node.policy.update_map(T.view_fixture());node.geometry_verified=True;node.rules_verified=True;node.cruise=1.
    node.K=(100.,100.,50.,50.);node.current_msg=T.types.SimpleNamespace(width=100)
    node.clearance_points=np.tile([3.,0.,0.],(10,1));node.clearance_origin=node.pose[0].copy();node.capture_yaw=0.
    node.last_capture=1.;node.tf_ok=True;node.valid_fraction=1.;node.odom_valid=True
    node.health.on_odom(1.,1.,node.pose[0],.03,yaw=0.);node.health.on_vo_status(1.,1)
    node.control_tick();assert node.startup_ready
    node.on_start(None);F.BUS.t=1.05;node.control_tick()
    assert node.policy.state=='OBSERVING_YAW' and node.backend.last_cmd[:3]==(0.,0.,0.)
    assert node.backend.last_cmd[3]>0
    F.BUS.t=1.4;node.control_tick()
    assert node.policy.state=='HOLD_SCAN_EVIDENCE' and node.backend.last_cmd==(0.,0.,0.,0.)
    node.backend.authority=False;F.BUS.t=1.45;node.control_tick()
    assert node.policy.aborted and node.policy.abort_reason=='TAKEOVER'


def test_depth_registration_is_required_for_color_goal_input():
    T.F.Node.PARAM_OVERRIDES=dict(voxel_m=.2,size_xy_m=6.)
    node=T.StationaryLocalPolicyNode();msg=T.F.Image();msg.header=T.F.Header(stamp=T.F.make_time(1.))
    node.on_rgb(msg);assert not node.rgb_frames and node.policy.goal_hint is None


def test_swept_cylinder_preserves_collision_boundary_without_corner_double_count():
    from dh_core.live_map_adapter import swept_cylinder_clearance as clearance
    r=.415
    assert abs(clearance([[1.,0.,0.]],0.,r,.2)-1.)<1e-9
    assert clearance([[.1,0.,0.]],0.,r,.2)<r # already inside envelope: stop
    a,b=.4,.4
    contact=clearance([[a,b,0.]],0.,r,.2)-r
    assert contact>0 and abs(np.hypot(a-contact,b)-r)<1e-9
    # Just before contact is clear, just after intersects the same full envelope.
    assert np.hypot(a-(contact-.001),b)>r
    assert np.hypot(a-(contact+.001),b)<r
    assert np.isinf(clearance([[.1,.5,0.]],0.,r,.2))
    assert np.isnan(clearance([[np.nan,0.,0.]],0.,r,.2))


def test_observed_turn_target_requires_explicit_active_perception():
    from dh_core.live_map_adapter import plan_local
    view=T.view_fixture();grid=view.planner.grid
    yy,xx=np.mgrid[:grid.shape[0],:grid.shape[1]]
    x=grid.origin[0]+(xx+.5)*grid.res;y=grid.origin[1]+(yy+.5)*grid.res
    view.planner.free&=(x>-.2)&(abs(y)<.2)
    p=np.array([0.,0.,1.])
    assert plan_local(view,p,np.pi/2) is None
    path=plan_local(view,p,np.pi/2,allow_turn=True)
    assert path is not None and path.p[-1,0]>.6
    assert view.path_safe(path.p)


def test_route_heading_ignores_tiny_start_cell_stub():
    from dh_core.timing import time_parameterize
    from dh_core.tracker import TrajectoryFollower
    from dh_core.safety_filter import SafetyGate
    F=T.F;F.BUS.t=1.;F.Node.PARAM_OVERRIDES=dict(voxel_m=.2,size_xy_m=6.,active_perception_enabled=True)
    node=T.StationaryLocalPolicyNode();node.pose=(np.array([0.,0.,1.]),0.)
    node.policy.update_map(T.view_fixture());node.geometry_verified=True;node.rules_verified=True;node.cruise=1.
    node.K=(100.,100.,50.,50.);node.current_msg=T.types.SimpleNamespace(width=100)
    node.clearance_points=np.tile([3.,0.,0.],(10,1));node.clearance_origin=node.pose[0].copy();node.capture_yaw=0.
    node.last_capture=1.;node.tf_ok=True;node.valid_fraction=1.;node.odom_valid=True
    node.health.on_odom(1.,1.,node.pose[0],.03,yaw=0.);node.health.on_vo_status(1.,1)
    trajectory=time_parameterize(np.array([[0.,0.,1.],[-.005,0.,1.],[.3,0.,1.],[1.,0.,1.]]),.3,.4,.4)
    node.policy.gate=SafetyGate(TrajectoryFollower(trajectory),node.policy.ev)
    node.backend.grant_authority();node.armed_start=True;node.last_plan=1.
    F.BUS.t=1.05;node.control_tick()
    assert node.policy.state!='OBSERVING_ROUTE_HEADING',node.policy.state
    assert node.policy.gate.follower.tau>0


def test_astar_adapter_does_not_insert_a_backward_start_cell_leg():
    from dh_core.live_map_adapter import plan_local
    view=T.view_fixture();position=np.array([-.008,.008,1.])
    trajectory=plan_local(view,position,0.)
    assert trajectory is not None and view.path_safe(trajectory.p)
    forward=trajectory.p[-1,:2]-position[:2]
    nonzero=[trajectory.sample(t)[1][:2] for t in np.linspace(.01,.5,20)]
    assert all(v@forward>=-1e-8 for v in nonzero)


def test_stale_occupied_confidence_becomes_unknown_without_opening_a_route():
    m=T.VoxelMap(.1,4.,-.2,2.8);m.grid.fill(-2.);times=np.full(m.shape,1.)
    m.grid[25:27,18:22,:]=2.
    old=from_team_map(m,times,1.,1.,1.,(.35,1.5),True,free_max_age_s=30.)
    aged=from_team_map(m,times,29.,29.,1.,(.35,1.5),True,free_max_age_s=30.)
    assert (old.states==1).any() and not (aged.states==1).any()
    assert np.array_equal(old.planner.free,aged.planner.free)
    assert np.array_equal(old.exploration_unknown,aged.exploration_unknown)


if __name__=='__main__':
    for name in list(globals()):
        if name.startswith('test_'):globals()[name]();print('PASS',name,flush=True)
