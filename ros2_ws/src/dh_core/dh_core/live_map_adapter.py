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
class PlanningSlice:
    planner: ClearancePlanner
    altitude: float
    stamp: float
    revision: int

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
                  margin=.10,pos_std=.03,revision=0):
    """Consume VoxelMap directly. Timestamp/geometry contract fails closed.

    All voxels through the *whole* vertical aircraft band must be freshly free.
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
    hh=half_height+margin+3*pos_std
    low=math.floor((altitude-hh-voxels.lo[2])/voxels.voxel)
    high=math.ceil((altitude+hh-voxels.lo[2])/voxels.voxel)
    if low<0 or high>voxels.shape[2] or high<=low: raise ValueError('Unobserved height band')
    t=times[:,:,low:high]; values=voxels.grid[:,:,low:high]
    free=((values<0)&np.isfinite(t)&(t>0)&(now-t<=.5)&(t<=capture_stamp)&(t<=now)).all(axis=2)
    grid=Grid2D(voxels.lo[:2].copy(),voxels.voxel,(~free).T.copy())
    grid.occ[[0,-1],:]=True; grid.occ[:,[0,-1]]=True
    body=radius+margin+3*pos_std+math.sqrt(2)*voxels.voxel/2
    planner=ClearancePlanner(grid,body,preferred_clearance=body+.20)
    return PlanningSlice(planner,float(altitude),float(capture_stamp),int(revision))


def plan_local(view,start,yaw,max_distance=1.5,speed=.3,recent=()):
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
    candidates=xy[eligible]; score=forward[eligible]-.15*distance[eligible]
    for old in recent: score-=.3*np.exp(-np.linalg.norm(candidates-old,axis=1)**2/.25)
    for goal in candidates[np.argsort(score)[::-1][::max(1,len(score)//12)][:12]]:
        if not view.safe([[*goal,view.altitude]])[0]: continue
        path=view.planner.astar(start[:2],goal)
        if path is None: continue
        path=view.planner.shortcut(path,min_clearance=view.planner.r)
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
        self.ev=SafetyEvaluator(SafetyParams(v_max=.3,latency_s=.8,brake_decel=.6,
            odom_timeout_s=.4,depth_timeout_s=.3,vehicle_radius=.27,uncertainty_margin=.10,k_pos_std=3.))

    def update_map(self,view): self.view=view

    def plan(self,position,yaw,now):
        if self.aborted or self.view is None or not 0<=now-self.view.stamp<=.5: return False
        if self.gate is not None and self.gate.follower.done:
            old=self.gate.follower.traj.p[-1,:2]
            if not self.recent or np.linalg.norm(self.recent[-1]-old)>.1: self.recent=(self.recent+[old.copy()])[-8:]
        trajectory=plan_local(self.view,position,yaw,recent=self.recent)
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
        path=np.vstack([position,follower.traj.p[follower.traj.nearest_index(position):]])
        if not self.view.path_safe(path):
            self.gate=None; self.state='HOLD_PATH_CHANGED'; return zero,0.
        # Judge the configured ceiling, not v_ref(t=0)==0: asking the filter
        # for zero gives a zero cap and would freeze a rest-to-rest start.
        verdict=self.gate.update(inputs,self.ev.prm.v_max)
        v,yr,_,_=follower.step(position,yaw,min(max(dt,0),.1))
        v,yr=self.gate.clamp(v,yr)
        self.state='TRACK' if verdict.allows_motion else 'HOLD_'+verdict.reason
        # Fixed-height prototype cannot climb over any obstruction.
        v[2]=0.
        if verdict.allows_motion:
            change=v-previous; norm=np.linalg.norm(change); limit=.4*min(max(dt,0),.1)
            if norm>limit: v=previous+change*(limit/norm)
            v,yr=self.gate.clamp(v,yr)
            horizon=self.ev.prm.latency_s+np.linalg.norm(v)/self.ev.prm.brake_decel
            if not self.view.path_safe(np.array([position,np.asarray(position)+v*horizon])):
                follower.tau=max(0.,follower.tau-min(max(dt,0),.1))
                self.state='HOLD_STOPPING_SWEEP_UNKNOWN'; return zero,0.
            self.last_velocity=v.copy()
        return v,yr
