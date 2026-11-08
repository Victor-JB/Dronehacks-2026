"""Minimum adapter: existing team voxels -> existing A*/clothoid -> existing follower.

No mapper, SLAM, flight controller or exploration framework is defined here.
The ROS producer must supply per-voxel observation times; /map/occupied alone
is insufficient. Fixed-height local motion only, never an over-wall climb.
"""
from dataclasses import dataclass
import math
import numpy as np
from .course import Grid2D
from .planner_astar import ClearancePlanner
from .timing import fillet_path,time_parameterize
from .tracker import TrajectoryFollower,FollowerParams
from .safety_filter import SafetyGate,SafetyEvaluator,SafetyParams


@dataclass
class InitialVolume:
    """Explicit verified placement condition, not sensor evidence or a route.

    Only cells wholly inside the original cylinder may be certified. Positive
    obstacle evidence always wins. Defaults never create such a certificate.
    """
    centre: np.ndarray
    radius: float
    half_height: float
    stamp: float
    max_age: float=30.

    def mask(self,voxels,now):
        if not 0<=now-self.stamp<=self.max_age:return np.zeros(voxels.shape,bool)
        x=voxels.lo[0]+(np.arange(voxels.shape[0])+.5)*voxels.voxel
        y=voxels.lo[1]+(np.arange(voxels.shape[1])+.5)*voxels.voxel
        z=voxels.lo[2]+(np.arange(voxels.shape[2])+.5)*voxels.voxel
        xy=np.hypot(x[:,None]-self.centre[0],y[None,:]-self.centre[1])+math.sqrt(2)*voxels.voxel/2<=self.radius
        zz=abs(z-self.centre[2])+voxels.voxel/2<=self.half_height
        return xy[:,:,None]&zz[None,None,:]


def clearance_at_current_pose(points,capture_position,capture_yaw,position,yaw):
    """Captured level-FRD points -> current level-FRD, through odom ENU.

    The policy's trajectory/velocity stays in ENU world axes. This transform
    is only for querying a depth corridor after vehicle translation/yaw.
    """
    def basis(a):
        c,s=math.cos(a),math.sin(a)
        return np.array([[c,s,0.],[s,-c,0.],[0.,0.,-1.]])
    return (np.asarray(points)@basis(capture_yaw).T+capture_position-position)@basis(yaw)


def swept_cylinder_clearance(points,bearing,radius,half_height):
    """Equivalent centreline range for the independent filter's circular envelope.

    A rectangular corridor's nearest along-range double-counts the whole radius
    at its lateral edges. Solve the circle/point intersection instead. Returning
    travel-to-contact + radius preserves SafetyEvaluator's existing subtraction.
    Unknown/invalid evidence remains the caller's freshness/map responsibility.
    """
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():return math.nan
    if not np.isfinite([bearing,radius,half_height]).all() or min(radius,half_height)<=0:return math.nan
    c,s=math.cos(bearing),math.sin(bearing)
    a=p[:,0]*c+p[:,1]*s;b=-p[:,0]*s+p[:,1]*c
    band=(abs(b)<radius)&(abs(p[:,2])<half_height)
    extent=np.sqrt(np.maximum(0.,radius**2-b**2))
    relevant=band&((a>0)|(abs(a)<extent))
    if not relevant.any():return math.inf
    return float(max(0.,np.min(a[relevant]-extent[relevant]+radius)))


@dataclass
class PlanningSlice:
    planner: ClearancePlanner
    altitude: float
    stamp: float
    revision: int
    states: np.ndarray=None
    exploration_unknown: np.ndarray=None

    def safe(self,points):
        p=np.atleast_2d(points); grid=self.planner.grid
        cells=np.floor((p[:,:2]-grid.origin)/grid.res).astype(int)
        inside=((cells>=0)&(cells<np.array(grid.shape)[::-1])).all(axis=1)
        out=np.zeros(len(p),bool)
        out[inside]=self.planner.free[cells[inside,1],cells[inside,0]]
        return out & (abs(p[:,2]-self.altitude)<.03)

    def path_safe(self,path):
        for a,b in zip(path[:-1],path[1:]):
            n=max(2,int(np.ceil(np.linalg.norm(b-a)/(self.planner.grid.res/3)))+1)
            if not self.safe(np.linspace(a,b,n)).all(): return False
        return len(path)>1


def from_team_map(voxels,observed_at,capture_stamp,now,altitude,legal_band=None,
                  height_below_walls_verified=False,radius=.27,half_height=.12,
                  margin=.10,pos_std=.03,revision=0,free_max_age_s=.5,
                  decay_time_s=30.,min_free_logodds=0.,initial_volume=None,vertical_margin=None):
    """Consume VoxelMap directly. Timestamp/geometry contract fails closed.

    All voxels through the *whole* vertical aircraft band need valid free evidence.
    Rows are +y, columns +x: transpose, NOT the display slice's flipud().
    No nearest-free relocation or out-of-bounds clipping is permitted.
    """
    if (not height_below_walls_verified or legal_band is None
            or len(legal_band)!=2 or not np.isfinite(legal_band).all()
            or not legal_band[0]<=altitude<=legal_band[1]): raise ValueError('Legal below-wall altitude unverified')
    if (not np.isfinite([capture_stamp,now,altitude,radius,half_height,margin,pos_std]).all()
            or capture_stamp<=0 or not 0<=now-capture_stamp<=.5
            or min(radius,half_height)<=0 or min(margin,pos_std)<0): raise ValueError('Invalid/stale planning inputs')
    times=np.asarray(observed_at,float)
    if times.shape!=voxels.grid.shape: raise ValueError('Missing per-voxel observation timestamps')
    vertical_margin=margin if vertical_margin is None else vertical_margin
    if not np.isfinite(vertical_margin) or vertical_margin<0:raise ValueError('Invalid vertical margin')
    hh=half_height+vertical_margin+3*pos_std
    low=math.floor((altitude-hh-voxels.lo[2])/voxels.voxel)
    high=math.ceil((altitude+hh-voxels.lo[2])/voxels.voxel)
    if low<0 or high>voxels.shape[2] or high<=low: raise ValueError('Unobserved height band')
    t=times[:,:,low:high]; values=voxels.grid[:,:,low:high]
    if not 0<free_max_age_s<=60 or decay_time_s<=0 or min_free_logodds<0:raise ValueError('Invalid evidence retention')
    age=now-t
    retained=(values*np.exp(-np.clip(age,0,1e4)/decay_time_s)<-min_free_logodds)&np.isfinite(t)&(t>0)&(age<=free_max_age_s)&(t<=capture_stamp)&(t<=now)
    sensor_free=retained.all(axis=2)
    if initial_volume is not None:
        certificate=initial_volume.mask(voxels,now)[:,:,low:high]
        retained |= certificate&(values<=0) # never override a measured hit/uncertain positive
    free=retained.all(axis=2)
    states=np.full(free.shape,-1,np.int8);states[free]=0;states[(values>voxels.l_occ).any(axis=2)]=1
    states[free&~sensor_free]=2 # explicit launch certificate, never labelled sensor free
    exploration_unknown=states==-1
    # Stale occupied evidence loses its semantic confidence too. It becomes
    # UNKNOWN/BLOCKED, never free. Preserve the exploration ranking so ageing a
    # wall does not create an attractive fictitious frontier at that wall.
    occupied_confident=((values*np.exp(-np.clip(age,0,1e4)/decay_time_s)>voxels.l_occ)
        &np.isfinite(t)&(t>0)&(age<=free_max_age_s)&(t<=capture_stamp)&(t<=now)).any(axis=2)
    states[(states==1)&~occupied_confident]=-1
    grid=Grid2D(voxels.lo[:2].copy(),voxels.voxel,(~free).T.copy())
    grid.occ[[0,-1],:]=True; grid.occ[:,[0,-1]]=True
    body=radius+margin+3*pos_std+math.sqrt(2)*voxels.voxel/2
    planner=ClearancePlanner(grid,body,preferred_clearance=body+.20)
    return PlanningSlice(planner,float(altitude),float(capture_stamp),int(revision),states,exploration_unknown)


def plan_local(view,start,yaw,max_distance=1.5,speed=.3,recent=(),goal_hint=None,allow_turn=False):
    """A short observed goal, not maze completion. Reuse existing A* search.

    Rank at most12 forward candidates and ask the existing planner for reachability.
    This deliberately postpones advanced exploration and passage objectives.
    """
    start=np.asarray(start,float)
    if not view.safe([start])[0] or not 0<speed<=.3: return None
    rc=np.argwhere(view.planner.free)
    if not len(rc): return None
    xy=view.planner.grid.origin+(rc[:,::-1]+.5)*view.planner.grid.res
    delta=xy-start[:2]; distance=np.linalg.norm(delta,axis=1)
    forward=delta@np.array([math.cos(yaw),math.sin(yaw)])
    eligible=(distance>=.6)&(distance<=max_distance)&(forward>.6*distance)
    if allow_turn:eligible=(distance>=.6)&(distance<=max_distance)
    candidates=xy[eligible]; score=forward[eligible]-.15*distance[eligible]
    if allow_turn:score=.85*distance[eligible]-.20*(distance[eligible]-forward[eligible])
    if goal_hint is not None:
        goal_hint=np.asarray(goal_hint,float)
        if goal_hint.shape==(3,) and np.isfinite(goal_hint).all():
            score+=3*(np.linalg.norm(start[:2]-goal_hint[:2])-np.linalg.norm(candidates-goal_hint[:2],axis=1))
    for old in recent: score-=1.2*np.exp(-np.linalg.norm(candidates-old,axis=1)**2/.4)
    if view.states is not None:
        import cv2
        width=max(3,int(1.8/view.planner.grid.res))
        unknown=view.states==-1 if view.exploration_unknown is None else view.exploration_unknown
        gain=cv2.blur(unknown.astype(np.float32),(width,width))
        rows=rc[eligible];score+=3*gain[rows[:,1],rows[:,0]]
    for goal in candidates[np.argsort(score)[::-1][::max(1,len(score)//12)][:12]]:
        if not view.safe([[*goal,view.altitude]])[0]: continue
        path=view.planner.astar(start[:2],goal)
        if path is None: continue
        path=view.planner.shortcut(path,min_clearance=view.planner.r)
        # A* begins at the centre of the cell containing the measured start.
        # That redundant sub-cell stub can point backwards and deadlock the
        # directional live-depth gate. Connect the actual start to its next
        # waypoint; the final dense path check still validates the whole segment.
        if len(path)>1 and np.linalg.norm(path[0]-start[:2])<=view.planner.grid.res/math.sqrt(2)+1e-8:
            path=path[1:]
        path=np.vstack([start[:2],path,goal])
        path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-7]]
        points=np.column_stack([path,np.full(len(path),view.altitude)])
        points=fillet_path(points,.25,transition='clothoid',
            clearance_ok=lambda p:bool(view.safe([[*p,view.altitude]])[0]))
        if not view.path_safe(points): continue
        if len(points)<3: points=np.linspace(points[0],points[-1],3)
        return time_parameterize(points,speed,.4,.4,a_total=.4)
    return None


class LocalPolicy:
    """Small command gate around the existing follower, independent safety filter.

    Caller publishes through existing DryRunBackend/SkybetaBackend; this pure
    class creates no publishers and cannot arm, land or select a flight mode.
    SafetyInputs.free_distance must cover the actual swept travel corridor.
    """
    def __init__(self):
        self.view=None; self.gate=None; self.had_authority=False; self.aborted=False
        self.state='WAIT_MAP'; self.replans=0
        self.last_velocity=np.zeros(3)
        self.abort_reason=''; self.recent=[]
        self.visited=[]
        self.goal_hint=None
        self.allow_turn=False
        self.ev=SafetyEvaluator(SafetyParams(v_max=.3,latency_s=.8,brake_decel=.6,
            odom_timeout_s=.4,depth_timeout_s=.3,vehicle_radius=.27,uncertainty_margin=.10,k_pos_std=3.))

    def update_map(self,view): self.view=view

    def plan(self,position,yaw,now):
        if self.aborted or self.view is None or not 0<=now-self.view.stamp<=.5: return False
        if self.gate is not None and self.gate.follower.done:
            old=self.gate.follower.traj.p[-1,:2]
            if not self.recent or np.linalg.norm(self.recent[-1]-old)>.1: self.recent=(self.recent+[old.copy()])[-8:]
        trajectory=plan_local(self.view,position,yaw,recent=self.recent+self.visited[-20:],goal_hint=self.goal_hint,allow_turn=self.allow_turn)
        if trajectory is None: self.state='HOLD_NO_OBSERVED_PATH'; return False
        follower=TrajectoryFollower(trajectory,FollowerParams(v_cap=.3,vz_cap=.1,v_lead=0.))
        self.gate=SafetyGate(follower,self.ev); self.replans+=1; self.state='READY'; return True

    def step(self,position,yaw,dt,inputs,authority=False):
        zero=np.zeros(3)
        previous=self.last_velocity.copy(); self.last_velocity=zero.copy()
        if (np.asarray(position).shape!=(3,) or not np.isfinite(position).all()
                or not np.isfinite([yaw,dt]).all() or not 0<dt<=.2):
            self.state='HOLD_INVALID_CONTROL_INPUT'; return zero,0.
        if self.had_authority and not authority: self.aborted=True; self.abort_reason='TAKEOVER'
        if self.aborted: self.state='ABORTED_'+(self.abort_reason or 'LOCALIZATION'); return zero,0.
        if not authority: self.state='WAIT_OFFBOARD'; return zero,0.
        self.had_authority=True
        if self.view is None or not 0<=inputs.now-self.view.stamp<=.5:
            self.state='HOLD_MAP_STALE'; return zero,0.
        if self.gate is None: self.state='HOLD_NO_PLAN'; return zero,0.
        follower=self.gate.follower
        if not self.visited or np.linalg.norm(self.visited[-1]-np.asarray(position)[:2])>.3:self.visited.append(np.asarray(position)[:2].copy())
        path=np.vstack([position,follower.traj.p[follower.traj.nearest_index(position):]])
        if not self.view.path_safe(path):
            self.gate=None; self.state='HOLD_PATH_CHANGED'; return zero,0.
        # Judge the configured ceiling, not v_ref(t=0)==0: asking the filter
        # for zero gives a zero cap and would freeze a rest-to-rest start.
        verdict=self.gate.update(inputs,self.ev.prm.v_max)
        before=follower.tau
        v,yr,_,_=follower.step(position,yaw,min(max(dt,0),.1))
        v,yr=self.gate.clamp(v,yr)
        self.state='TRACK' if verdict.allows_motion else 'HOLD_'+verdict.reason
        # Fixed-height prototype cannot climb over any obstruction.
        v[2]=0.
        if verdict.allows_motion:
            change=v-previous; norm=np.linalg.norm(change); limit=.4*min(max(dt,0),.1)
            if norm>limit: v=previous+change*(limit/norm)
            v,yr=self.gate.clamp(v,yr)
            factor=1.
            while factor>=.03125:
                candidate=v*factor;horizon=self.ev.prm.latency_s+np.linalg.norm(candidate)/self.ev.prm.brake_decel
                if self.view.path_safe(np.array([position,np.asarray(position)+candidate*horizon])):break
                factor*=.5
            if factor<.03125:
                follower.tau=before;self.gate=None
                self.state='HOLD_STOPPING_SWEEP_UNKNOWN'; return zero,0.
            follower.tau=before+(follower.tau-before)*factor;v*=factor
            self.last_velocity=v.copy()
        return v,yr
