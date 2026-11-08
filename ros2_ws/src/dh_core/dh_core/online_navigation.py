"""Shared CAD-free exploration/replanning/tracking. No ROS or truth access.

Planning is separable from tick() so the ROS shell can run bounded work on one
worker while odometry/health callbacks remain responsive.
"""
from dataclasses import dataclass
import math
import numpy as np
from .health import LocalizationHealth,LocHealthParams,HOVER,ABORT
from .online_map import OnlineMap
from .online_planning import exploration_goal,plan_observed
from .tracker import TrajectoryFollower,FollowerParams
from .passages import PassageProgress


@dataclass
class GoalObservation:
    position: np.ndarray
    stamp: float
    confidence: float
    source: str


class OnlineNavigator:
    def __init__(self,planner='clothoid',map_options=None,speed=.5,required_passages=('tunnel','window')):
        if planner not in ('clothoid','sfc','mpl'): raise ValueError('Invalid planner')
        if not math.isfinite(speed) or not 0<speed<=1.: raise ValueError('Unsafe online speed')
        self.map=OnlineMap(**(map_options or {})); self.planner=planner; self.speed=speed
        self.health=LocalizationHealth(LocHealthParams(require_vo_status=True))
        self.position=None; self.yaw=0.; self.pos_std=0.; self.epoch=0
        self.active=False; self.had_authority=False; self.aborted=False
        self.follower=None; self.plan_stats={}; self.replans=0; self.visits={}
        self.goal=None; self.goal_seen=False; self.exploration_target=None
        self.state='WAIT_START'; self.distance=0.; self._last_position=None
        self.next_plan_after=0.
        self.last_orientation=None
        self.passages=PassageProgress(required_passages)

    def odometry(self,position,yaw,stamp,now,pos_std=0.,orientation=None):
        p=np.asarray(position,float)
        q=None if orientation is None else np.asarray(orientation,float)
        if (p.shape!=(3,) or not np.isfinite(p).all() or not math.isfinite(yaw)
                or not math.isfinite(pos_std) or pos_std<0 or not -.02<=now-stamp<=.4
                or (q is not None and (q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.01))
                or (self.health.last_odom_t is not None and stamp<=self.health.last_odom_t)):
            return False
        old_jump=self.health.jump_since
        if q is not None and self.last_orientation is not None and self.health.last_odom_t is not None:
            angle=2*math.acos(float(np.clip(abs(q@self.last_orientation),0.,1.)))
            if angle>1.5*(stamp-self.health.last_odom_t)+self.health.prm.jump_yaw_rad:
                if self.health.jump_since is None: self.health.jump_since=now
                self.health.pause_until=max(self.health.pause_until,now+self.health.prm.jump_pause_s)
        self.health.on_odom(now,stamp,p,pos_std=pos_std,yaw=yaw)
        if q is not None: self.last_orientation=q/np.linalg.norm(q)
        if old_jump is None and self.health.jump_since is not None:
            self.map.reset(); self.follower=None; self.goal=None; self.epoch+=1
            self.passages=PassageProgress(self.passages.required)
        if self._last_position is not None: self.distance+=float(np.linalg.norm(p-self._last_position))
        self._last_position=p.copy(); self.position=p.copy(); self.yaw=yaw; self.pos_std=pos_std
        return True

    def tracking(self,stamp,state,now):
        if stamp>0 and -.02<=now-stamp<=1. and (self.health.vo_rx is None or stamp>self.health.vo_rx):
            self.health.on_vo_status(stamp,state)

    def set_goal(self,goal,now):
        if np.asarray(goal.position).shape!=(3,) or not math.isfinite(goal.confidence): return False
        if (goal.source!='blue_pad' or goal.confidence<.7 or not -.02<=now-goal.stamp<=.5
                or not np.isfinite(goal.position).all()): return False
        if self.goal is not None and np.linalg.norm(goal.position-self.goal.position)>.3: return False
        if self.goal is not None and goal.stamp<=self.goal.stamp: return False
        if self.goal is None or np.linalg.norm(goal.position-self.goal.position)>.1:
            self.follower=None; self.exploration_target=None
        self.goal=GoalObservation(np.array(goal.position,copy=True),goal.stamp,goal.confidence,goal.source)
        self.goal_seen=True; return True

    def health_ok(self,now):
        mode,scale,reason=self.health.evaluate(now)
        if mode==ABORT: self.aborted=True
        if self.aborted: self.state='ABORTED'; return False,0.
        if self.map.legal_band is None: self.state='HOLD_LEGAL_ALTITUDE_UNCONFIRMED'; return False,0.
        if self.map.floor_z is None: self.state='HOLD_FLOOR_REFERENCE_UNCONFIRMED'; return False,0.
        if mode==HOVER: self.state='HOLD_LOCALIZATION:'+reason; return False,0.
        if self.map.last_stamp is None or not 0<=now-self.map.last_stamp<=.5:
            self.state='HOLD_DEPTH_STALE_OR_UNKNOWN'; return False,0.
        if self.passages.done and self.goal_seen and (self.goal is None or now-self.goal.stamp>2.):
            self.state='HOLD_GOAL_UNCERTAIN'; return False,0.
        return True,scale

    def request(self,now):
        ok,_=self.health_ok(now)
        if not self.active or not ok or self.position is None or now<self.next_plan_after: return None
        view=self.map.snapshot(now,self.pos_std)
        self.passages.update(view,self.position)
        if not bool(view.points_safe([self.position])[0]):
            self.state='SCAN_REQUIRED_VOLUME_UNKNOWN'; return None
        if self.follower is not None and not self.follower.done:
            remaining=self.follower.traj.p[self.follower.traj.nearest_index(self.position):]
            if len(remaining)>1 and view.path_safe(np.vstack([self.position,remaining])): return None
            self.follower=None
        if self.exploration_target is not None and np.linalg.norm(self.exploration_target-self.position)<.25:
            cell=view.cell(self.exploration_target)
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for dz in (-1,0,1):
                        c=(cell[0]+dx,cell[1]+dy,cell[2]+dz); self.visits[c]=self.visits.get(c,0)+1
        passage_goal=self.passages.target(view,self.position)
        confined=passage_goal is not None or any(np.linalg.norm(p.centre-self.position)<2 for p in view.passages)
        return dict(view=view,start=self.position.copy(),yaw=self.yaw,epoch=self.epoch,
                    goal=None if self.goal is None or not self.passages.done else self.goal.position.copy(),
                    passage_goal=passage_goal,visits=dict(self.visits),planner=self.planner,speed=min(self.speed,.2) if confined else self.speed)

    @staticmethod
    def compute(request):
        goal=request.get('passage_goal')
        if goal is None: goal=request['goal']
        if goal is None:
            goal=exploration_goal(request['view'],request['start'],request['visits'],request['yaw'])
        if goal is None: return None,dict(reason='NO_REACHABLE_FRONTIER'),None,request['epoch']
        if np.linalg.norm(goal-request['start'])<.12: return None,dict(reason='AT_GOAL'),goal,request['epoch']
        trajectory,stats=plan_observed(request['view'],request['start'],goal,request['planner'],request['speed'])
        if trajectory is None and (request['goal'] is not None or request.get('passage_goal') is not None):
            # Seeing a pad through an opening does not reveal a traversable route.
            # Continue mapping reachable frontiers rather than crossing unknown.
            frontier=exploration_goal(request['view'],request['start'],request['visits'],request['yaw'])
            if frontier is not None:
                trajectory,stats=plan_observed(request['view'],request['start'],frontier,request['planner'],request['speed'])
                stats['explore_for_goal']=True; goal=frontier
        stats['requested_speed']=request['speed']
        stats['objective']='passage' if request.get('passage_goal') is not None else ('pad' if request['goal'] is not None else 'exploration')
        return trajectory,stats,goal,request['epoch']

    def accept(self,result,now):
        trajectory,stats,goal,epoch=result
        if epoch!=self.epoch or not self.health_ok(now)[0]: return False
        if trajectory is None:
            self.state='PAD_APPROACH_COMPLETE' if stats['reason']=='AT_GOAL' and self.goal_seen and self.passages.done else stats['reason']
            self.next_plan_after=now+1.
            if goal is not None and stats['reason']!='AT_GOAL':
                cell=self.map.snapshot(now,self.pos_std).cell(goal); self.visits[cell]=self.visits.get(cell,0)+2
            return False
        view=self.map.snapshot(now,self.pos_std)
        if np.linalg.norm(self.position-trajectory.p[0])>.15:
            self.state='HOLD_PLAN_START_CHANGED'; return False
        if not view.path_safe(np.vstack([self.position,trajectory.p])):
            self.state='HOLD_PLAN_OBSOLETE'; return False
        cap=min(self.speed,stats.get('requested_speed',self.speed))
        self.follower=TrajectoryFollower(trajectory,FollowerParams(v_cap=cap,vz_cap=min(.25,cap),v_lead=0.))
        self.plan_stats=stats; self.replans+=1; self.exploration_target=goal
        self.state='EXPLORE_REACHABLE_ROUTE' if stats.get('explore_for_goal') else (
            'APPROACH_OR_TRAVERSE_PASSAGE' if stats.get('objective')=='passage' else (
            'APPROACH_PAD' if stats.get('objective')=='pad' else 'EXPLORE'))
        return True

    def tick(self,now,dt,authority=True):
        zero=np.zeros(3)
        if self.active and self.had_authority and not authority:
            self.aborted=True; self.state='ABORTED_PILOT_TAKEOVER'
        if self.aborted: return zero,0.
        if not self.active: self.state='WAIT_START'; return zero,0.
        if not authority: self.state='WAIT_AUTHORITY'; return zero,0.
        self.had_authority=True
        ok,scale=self.health_ok(now)
        if not ok: return zero,0.
        if self.follower is None:
            if self.state in ('WAIT_START','WAIT_AUTHORITY','FRONTIER_REACHED','HOLD_PATH_CHANGED'):
                self.state='WAIT_PLAN'
            return zero,0.
        view=self.map.snapshot(now,self.pos_std)
        idx=self.follower.traj.nearest_index(self.position)
        remaining=np.vstack([self.position,self.follower.traj.p[idx:]])
        if not view.path_safe(remaining):
            self.follower=None; self.state='HOLD_PATH_CHANGED'; return zero,0.
        ref=self.follower.traj.sample(self.follower.tau)[0]
        err=np.linalg.norm(ref-self.position)
        self.follower.speed_scale=scale*float(np.clip((.35-err)/.2,0,1))
        before=self.follower.tau
        velocity,yaw_rate,_,_=self.follower.step(self.position,self.yaw,min(max(dt,0),.1))
        # A collision-free curved reference can still produce a straight velocity
        # sweep outside it. Reduce both commanded speed AND reference progress;
        # freezing at the original high speed would deadlock at safe corners.
        factor=1.
        while factor>=.03125:
            candidate=velocity*factor; horizon=.5+np.linalg.norm(candidate)/.6
            if view.segment_safe(self.position,self.position+candidate*horizon): break
            factor*=.5
        if factor<.03125:
            self.follower.tau=before; self.follower=None
            self.state='HOLD_STOPPING_VOLUME_UNKNOWN'; return zero,0.
        self.follower.tau=before+(self.follower.tau-before)*factor
        velocity*=factor; yaw_rate*=factor
        if self.follower.done:
            self.state='PAD_APPROACH_COMPLETE' if self.goal is not None and self.passages.done and np.linalg.norm(self.position-self.goal.position)<.15 else 'FRONTIER_REACHED'
        return velocity,yaw_rate
