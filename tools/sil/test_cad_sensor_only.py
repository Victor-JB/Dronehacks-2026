"""Original CAD used ONLY by the representative sensor producer/grader.

One forward-camera/map smoke test, not a completed CAD-free course run. No CAD
object, waypoint, obstacle list, wall height or finish coordinate enters nav.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/dh_core'))
from dh_core.course import Course
from dh_core.online_navigation import OnlineNavigator
from dh_core.online_geometry import nominal_transform_for_tests
from scene import VoxelScene


def run(path):
    course=Course.from_obj(str(path)); scene=VoxelScene.from_course(course,resolution=.1)
    simulated_start=np.array([*course.launch[:2],.9])
    u,v=np.meshgrid(np.arange(0,960,12),np.arange(0,600,12))
    rays=np.column_stack([(u.ravel()-487.3531)/595.1951,(v.ravel()-305.8477)/595.3043,np.ones(u.size)])
    R,offset=nominal_transform_for_tests(0.); rays=rays@R.T; rays/=np.linalg.norm(rays,axis=1)[:,None]
    origin=simulated_start+offset
    hits,valid,_=scene.raycast(origin,rays)
    ranges=np.linalg.norm(hits-origin,axis=1); ranges[~valid]=np.nan
    # No course object or derived rule/goal is provided to the navigator.
    nav=OnlineNavigator(map_options=dict(size=16.)); nav.active=True
    nav.odometry([0,0,.9],0,1.,1.); nav.tracking(1.,1,1.)
    assert nav.map.observe([offset[0],offset[1],.9+offset[2]],rays,ranges,1.,1.)
    view=nav.map.snapshot(1.)
    assert (view.states==0).any() and (view.states==-1).any()
    assert nav.request(1.) is None and nav.state=='HOLD_LEGAL_ALTITUDE_UNCONFIRMED'
    return dict(course_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        rays=len(rays),finite_returns=int(np.isfinite(ranges).sum()),
        observed_free_voxels=int((view.states==0).sum()),occupied_voxels=int((view.states==1).sum()),
        unknown_voxels=int((view.states==-1).sum()),final_state=nav.state,
        complete_course_run=False,geometry_access='sensor producer/grader only',ROS=False,hardware=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--course',type=Path,required=True)
    args=parser.parse_args(); print(json.dumps(run(args.course),indent=2))
