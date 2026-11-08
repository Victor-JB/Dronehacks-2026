"""Shared observed-free 3D search and existing planner kernels; no CAD input."""
from collections import deque
import heapq
import itertools
import math
import numpy as np
from .timing import fillet_path,time_parameterize
from .planner_sfc import plan_sfc,SFCParams
from .planner_mpl import MotionPrimitivePlanner,MPLParams

NEIGH=tuple(tuple(s if j==axis else 0 for j in range(3)) for axis in range(3) for s in (-1,1))


def reachable(view,start,limit=30000):
    s=view.cell(start)
    if not view.inside(s) or not view.safe[s]: return []
    q=deque([s]); seen={s}
    while q and len(seen)<limit:
        c=q.popleft()
        for d in NEIGH:
            n=tuple(a+b for a,b in zip(c,d))
            if n not in seen and view.inside(n) and view.safe[n]: seen.add(n); q.append(n)
    return sorted(seen)


def astar3d(view,start,goal,limit=30000):
    s,t=view.cell(start),view.cell(goal)
    if not all(view.inside(c) and view.safe[c] for c in (s,t)): return None
    heap=[(0.,0.,s)]; costs={s:0.}; parent={}
    h=lambda c: sum(abs(a-b) for a,b in zip(c,t))
    expanded=0
    while heap and expanded<limit:
        _,cost,c=heapq.heappop(heap)
        if cost!=costs[c]: continue
        expanded+=1
        if c==t:
            cells=[c]
            while c!=s: c=parent[c]; cells.append(c)
            path=np.vstack([start,*[view.centre(c) for c in cells[::-1]],goal])
            # Shortcut only within altitude layers; native SFC/MPL remain planar.
            out=[path[0]]; i=0
            while i<len(path)-1:
                candidates=[j for j in range(i+1,len(path)) if abs(path[j,2]-path[i,2])<1e-6]
                j=next((j for j in reversed(candidates) if view.segment_safe(path[i],path[j])),i+1)
                if np.linalg.norm(path[j]-out[-1])>1e-7: out.append(path[j])
                i=j
            # Collapse collinear altitude steps; keep true 3D turns explicit.
            compact=[]
            for point in out:
                while len(compact)>=2:
                    a,b=compact[-2:]; u=b-a; v=point-b
                    if np.linalg.norm(np.cross(u,v))<1e-8 and u@v>=0: compact.pop()
                    else: break
                compact.append(point)
            return np.asarray(compact)
        for delta in NEIGH:
            n=tuple(a+b for a,b in zip(c,delta))
            if not view.inside(n) or not view.safe[n]: continue
            nc=cost+1
            if nc<costs.get(n,math.inf):
                costs[n]=nc; parent[n]=c; heapq.heappush(heap,(nc+h(n),nc,n))
    return None


def exploration_goal(view,position,visits=None,heading=0.,limit=30000):
    cells=reachable(view,position,limit)
    if not cells: return None
    visits=visits or {}
    # Unknown gain estimate is a heuristic over sensor-derived voxels, not truth.
    unknown=(view.states==-1).astype(np.int32)
    prefix=np.pad(unknown,((1,0),)*3).cumsum(0).cumsum(1).cumsum(2)
    radius=max(2,int(1.5/view.res))
    best=None
    for c in cells[::max(1,len(cells)//600)]:
        point=view.centre(c); delta=point-np.asarray(position); distance=np.linalg.norm(delta)
        if distance<.45: continue
        visible_frontier=False
        for direction in NEIGH:
            for step in range(1,radius+1):
                look=tuple(c[k]+direction[k]*step for k in range(3))
                if not view.inside(look): break
                state=view.states[look]
                if state==1: break
                if state==-1: visible_frontier=True; break
            if visible_frontier: break
        if not visible_frontier: continue  # Do not reward unseen space behind a known wall.
        lo=np.maximum(np.array(c)-radius,0); hi=np.minimum(np.array(c)+radius+1,view.safe.shape)
        gain=0
        for bits in itertools.product((0,1),repeat=3):
            idx=tuple(hi[k] if b else lo[k] for k,b in enumerate(bits))
            gain+=(-1)**(3-sum(bits))*int(prefix[idx])
        if gain<=0: continue
        score=gain/(2*radius+1)**3-0.035*distance-0.5*visits.get(c,0)
        score-=.08*max(0.,-float(delta[:2]@np.array([math.cos(heading),math.sin(heading)]))/distance)
        if best is None or score>best[0]: best=(score,point)
    return None if best is None else best[1]


def plan_observed(view,start,goal,planner='clothoid',speed=.5):
    if planner not in ('clothoid','sfc','mpl'): raise ValueError('Unknown planner')
    path=astar3d(view,start,goal)
    if path is None or len(path)<2: return None,dict(reason='NO_OBSERVED_FREE_PATH',planner=planner)
    pieces=[]; stops=[]; calls=0; common=0; i=0
    while i<len(path)-1:
        j=i+1
        if abs(path[j,2]-path[i,2])<1e-6:
            while j+1<len(path) and abs(path[j+1,2]-path[i,2])<1e-6: j+=1
            section=path[i:j+1]; z=section[0,2]
            if planner=='clothoid':
                points=fillet_path(section,.35,clearance_ok=lambda xy:bool(view.points_safe([[*xy,z]])[0]),transition='clothoid')
            elif planner=='sfc':
                k=view.cell(section[0])[2]
                trajectory,_,stats=plan_sfc(section[:,:2],~view.safe[:,:,k].T,view.lo[:2],view.res,
                    SFCParams(robot_r=0.,v_max=speed,time_iters=3,admm_iters=1500))
                if not stats['corridor_ok'] or stats['corridor_violation_m']>.01: return None,dict(reason='SFC_INFEASIBLE',planner=planner)
                xy=trajectory.eval(np.linspace(0,trajectory.duration,max(3,int(trajectory.duration/.04))))
                points=np.column_stack([xy,np.full(len(xy),z)])
                if max(np.linalg.norm(points[0]-section[0]),np.linalg.norm(points[-1]-section[-1]))>.01:
                    return None,dict(reason='SFC_BOUNDARY_ERROR',planner=planner)
                points[0]=section[0]; points[-1]=section[-1]
            else:
                free=lambda xy:view.points_safe(np.column_stack([xy,np.full(len(xy),z)]))
                # At confined speed .2m/s the old .225m/s primitive quantum
                # admitted no nonzero control. Scale the existing lattice;
                # retain the same MPL search and common collision checker.
                trajectory,stats=MotionPrimitivePlanner(free,MPLParams(v_max=speed,
                    a_max=min(1.5,2*speed/.3),pos_res=min(.1,speed*.3),max_expansions=3000)).plan_through(section,z)
                if trajectory is None: return None,dict(reason='MPL_INFEASIBLE',planner=planner,**stats)
                points=trajectory.p
                # MPL's goal tolerance is not permission to miss the requested end.
                points=np.vstack([points,section[-1]])
            calls+=1
        else:
            points=np.linspace(path[i],path[j],max(3,int(np.linalg.norm(path[j]-path[i])/.04)+1)); common+=1
        if not view.path_safe(points): return None,dict(reason='SMOOTHING_3D_COLLISION',planner=planner)
        stops.extend([points[0].copy(),points[-1].copy()])
        pieces.extend(points if not pieces else points[1:]); i=j
    points=np.asarray(pieces)
    if len(points)<3: points=np.linspace(points[0],points[-1],3)
    # Native SFC/MPL kernels are planar. Every altitude-stage boundary must be
    # rest-to-rest; XY-only curvature is insufficient for a vertical 90deg turn.
    stops=np.asarray(stops)
    limit=lambda p:0. if np.linalg.norm(stops-p,axis=1).min()<1e-6 else speed
    trajectory=time_parameterize(points,speed,.6,.6,a_total=.6,v_limit_fn=limit)
    if not view.path_safe(trajectory.p): return None,dict(reason='TRAJECTORY_UNSAFE',planner=planner)
    return trajectory,dict(planner=planner,native_horizontal_calls=calls,shared_3d_stages=common,
                            length=trajectory.length,duration=trajectory.duration,revision=view.revision)
