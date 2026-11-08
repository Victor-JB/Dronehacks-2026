"""Deterministic sensor-only navigation benchmark. NOT ROS/Gazebo/hardware.
World truth is used ONLY by the sensor producer and collision grader.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import tracemalloc
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/dh_core'))
from dh_core.online_navigation import OnlineNavigator
from unknown_environment import UnknownWorld


def run(layout='wall',seed=0,planner='clothoid',panoramic=True,fault=None,steps=600):
    world=UnknownWorld(layout,seed)
    # Explicit synthetic centre-height rules, not inferred from PX4 clamps.
    # This matrix isolates exploration; separate passage tests require both.
    nav=OnlineNavigator(planner,dict(res=.25,size=12.,z_min=-1.,z_max=3.,legal_band=(.35,2.3),floor_z=0.),speed=.45,required_passages=())
    p=np.array([-3.,0.,1.]); yaw=0.; nav.active=True
    collisions=0; path_length=0.; replanning=[]; states={}; seen=set(); retraced=0.; previous=None
    found=False; approached=False; inserted=False; post_fault_motion=0.
    start=time.perf_counter(); cpu=time.process_time(); tracemalloc.start()
    for step in range(steps):
        now=1.+step*.1
        nav.odometry(p,yaw,now,now,.01 if fault!='uncertainty' or step<30 else .5)
        nav.tracking(now,2 if fault=='tracking' and step>=30 else 1,now)
        if step%2==0:
            origin,dirs,ranges=world.observe(p,yaw,panoramic)
            if fault=='depth' and step>=30: ranges[:]=np.nan
            nav.map.observe(origin,dirs,ranges,now,now)
        if fault=='dynamic' and not inserted and step>=35 and nav.follower is not None:
            anchor=nav.follower.traj.sample(nav.follower.tau+2.)[0]
            delta=anchor-p
            if np.linalg.norm(delta)>=.8:
                world.add_box(anchor+[-.15,-.4,-5.],anchor+[.15,.4,5.]); inserted=True
        observation=world.goal_sensor(p,now)
        if observation is not None: found=nav.set_goal(observation,now) or found
        request=nav.request(now)
        if request is not None:
            tick=time.perf_counter(); result=nav.compute(request); replanning.append(time.perf_counter()-tick); nav.accept(result,now)
        v,yr=nav.tick(now,.1)
        next_p=p+v*.1; yaw+=yr*.1
        length=float(np.linalg.norm(next_p-p)); path_length+=length
        if fault in ('depth','tracking','uncertainty') and step>=36: post_fault_motion+=length
        cell=nav.map.model._index(np.atleast_2d(next_p))
        key=int(cell[0]) if len(cell) else -1
        if key!=previous:
            if key in seen: retraced+=length
            seen.add(key); previous=key
        p=next_p
        if world.collision(p): collisions+=1; break
        states[nav.state]=states.get(nav.state,0)+1
        if found and nav.goal is not None and np.linalg.norm(p-nav.goal.position)<.15:
            approached=True; break
        if fault in ('depth','tracking','uncertainty') and step>=75: break
        if not panoramic and step>=30: break  # Deliberately report incomplete-view safe hold.
    _,peak=tracemalloc.get_traced_memory(); tracemalloc.stop()
    view=nav.map.snapshot(now,nav.pos_std)
    return dict(layout=layout,seed=seed,planner=planner,panoramic_sensor_model=panoramic,fault=fault,
        steps=step+1,collisions=collisions,goal_discovered=found,pad_approach_complete=approached,
        dynamic_obstacle_inserted=inserted,post_fault_motion_after_600ms_m=post_fault_motion,
        landing_complete=False,landing_reason='NO_VERIFIED_ORGANIZER_LANDING_AUTHORITY',
        path_length_m=path_length,revisited_cell_entry_length_m=retraced,replans=nav.replans,
        planning_calls=len(replanning),max_plan_wall_s=max(replanning,default=0.),
        observed_voxels=int(np.sum(view.states!=-1)),known_fraction=float(np.mean(view.states!=-1)),
        final_state=nav.state,state_counts=states,final_position=p.tolist(),
        wall_seconds=time.perf_counter()-start,cpu_seconds=time.process_time()-cpu,
        peak_traced_bytes=peak,map_storage_bytes=nav.map.memory_bytes,
        ROS=False,Gazebo=False,hardware=False,geometry_access='sensor producer and grader only')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',required=True); p.add_argument('--quick',action='store_true')
    a=p.parse_args()
    cases=[('open',0,'clothoid',True,None),('wall',11,'clothoid',True,None),('opening',23,'clothoid',True,None),
           ('dead_end',7,'clothoid',True,None),('narrow',2,'clothoid',True,None),
           ('open',3,'sfc',True,None),('open',4,'mpl',True,None),
           ('open',6,'clothoid',True,'dynamic'),('open',1,'clothoid',True,'depth'),
           ('open',2,'clothoid',True,'tracking'),('open',5,'clothoid',True,'uncertainty'),
           ('open',0,'clothoid',False,None)]
    if a.quick: cases=cases[:1]
    results=[]
    for args in cases:
        result=run(*args)
        expected_approach=args[3] and args[0]!='narrow' and args[4] not in ('depth','tracking','uncertainty')
        checks=dict(collision_free=result['collisions']==0,
                    approach_expected_or_safe_incomplete=(result['pad_approach_complete'] if expected_approach else not result['pad_approach_complete']),
                    dynamic_exercised=args[4]!='dynamic' or result['dynamic_obstacle_inserted'],
                    sensor_fault_stopped=args[4] not in ('depth','tracking','uncertainty') or result['post_fault_motion_after_600ms_m']<1e-9)
        result['checks']=checks; result['expectation']='PAD_APPROACH' if expected_approach else 'SAFE_INCOMPLETE'
        result['checks_passed']=all(checks.values()); results.append(result)
        print(result['layout'],result['planner'],result['fault'],'approach',result['pad_approach_complete'],
              'collisions',result['collisions'],'state',result['final_state'],flush=True)
        Path(a.output).write_text(json.dumps(results,indent=2),encoding='utf-8')
    if any(not r['checks_passed'] for r in results): raise SystemExit('FAILED EXPECTATION: inspect retained evidence')


if __name__=='__main__': main()
