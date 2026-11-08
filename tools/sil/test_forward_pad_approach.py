"""Forward COLOR/depth approach model, NOT a flight dynamics/ROS test.

Shows the actual visibility failure rather than declaring an unseen pad landed.
The mono K is used as a projection model; no actual RGB K/feed was observed.
"""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/dh_core'))
from pad_scene import render_pad
from dh_core.landing_pad import detect_blue_pad,PadTrack


def run():
    p=np.array([0.,0.,.8]); track=PadTrack(); goal=np.array([2.8,0.,0.])
    first_error=None; frames=0; last_visual=None; hold=None; max_memory_error=0.
    for step in range(200):
        now=1+step*.1
        # Explicit slowly changing odometry-frame error after takeoff.
        drift=np.array([.002*step,0.,0.])
        if step%3==0:
            image,depth,K,R,t=render_pad(height=p[2],position_xy=p[:2])
            observation=detect_blue_pad(image,depth,K,R,t+drift,now,registered=True)
            if track.update(observation,now):
                frames+=1; last_visual=now
                error=float(np.linalg.norm(observation.centre-(goal+drift)))
                if first_error is None: first_error=error
        estimate=track.estimate(now,pos_std=.01+.002*step)
        if estimate is None:
            if frames>=3: hold=now; break
            continue
        # Report relative-target error as drift accumulates through visual loss.
        max_memory_error=max(max_memory_error,float(np.linalg.norm((estimate.centre-(p+drift))-(goal-p))))
        delta=estimate.centre+np.array([0,0,.65])-(p+drift)
        p+=delta/max(np.linalg.norm(delta),1e-9)*min(.02,np.linalg.norm(delta))
    assert frames>=3 and first_error<.06
    assert hold is not None and np.linalg.norm(p[:2]-goal[:2])>.2
    assert max_memory_error<.15
    return dict(confirmed_frames=frames,first_pad_localization_error_m=first_error,
        maximum_relative_target_error_during_memory_m=max_memory_error,
        final_horizontal_distance_to_pad_m=float(np.linalg.norm(p[:2]-goal[:2])),
        last_visual_time=last_visual,held_uncertain_time=hold,
        aligned=False,landed=False,conclusion='Forward pad leaves FOV; bounded uncertainty/age causes safe hold before alignment.',
        ground_truth_use='renderer and grader only',ROS=False,hardware=False)


if __name__=='__main__': print(json.dumps(run(),indent=2))
