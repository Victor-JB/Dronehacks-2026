"""Bounded offline check of waypoint tracking under lagged velocity/yaw response.

This is a diagnostic plant approximation, not PX4 or hardware validation.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from learning.sim import Course, clip_norm
from mission.airframe import (check_scene_path, evaluate_window_crossing,
                              load_x500_envelope, quaternion_xyzw_from_rpy)
from mission.core import CourseController, wrap
from mission.heading import route_yaw


def simulate(plan, response_s, yaw_response_s=.35, lookahead_m=1.8):
    controller = CourseController(dict(plan, waypoints=plan['waypoints'][1:]))
    position = np.array(plan['waypoints'][0]['position'], float)
    velocity = np.zeros(3)
    yaw = command_yaw = math.pi/2
    dt, time_s = .04, 0.
    positions, attitudes, phases = [position.copy()], [quaternion_xyzw_from_rpy(0, 0, yaw)], []
    done, fault = False, None
    while time_s < 240:
        command = controller.update(position, velocity, time_s)
        if command.fault or command.done:
            done, fault = command.done, command.fault
            break
        index = controller.waypoint_index
        following = controller.waypoints[index+1]['position'] if index+1 < len(controller.waypoints) else None
        desired_yaw = route_yaw(position, command.target, following, command.phase, lookahead_m)
        if desired_yaw is not None:
            step = controller.max_yaw_rate*dt
            command_yaw = wrap(command_yaw + np.clip(wrap(desired_yaw-command_yaw), -step, step))
        yaw = wrap(yaw + np.clip(wrap(command_yaw-yaw)/yaw_response_s, -1.6, 1.6)*dt)
        acceleration = clip_norm((np.array(command.velocity)-velocity)/response_s, .8)
        next_velocity = velocity + dt*acceleration
        position = position + dt*(velocity+next_velocity)/2
        velocity = next_velocity
        # Approximate the thrust tilt needed for this horizontal acceleration.
        forward = math.cos(yaw)*acceleration[0] + math.sin(yaw)*acceleration[1]
        left = -math.sin(yaw)*acceleration[0] + math.cos(yaw)*acceleration[1]
        roll, pitch = -math.atan2(left, 9.81), math.atan2(forward, 9.81)
        positions.append(position.copy())
        attitudes.append(quaternion_xyzw_from_rpy(roll, pitch, yaw))
        phases.append(command.phase)
        time_s += dt
    scene = Course().scene(np.random.default_rng(0), 0.)
    envelope = load_x500_envelope(margin_m=.015)
    geometry = check_scene_path(positions, attitudes, scene, envelope)
    if geometry['first_unresolved']:
        index = geometry['first_unresolved']['segment']
        geometry['first_unresolved'].update(phase=phases[index], position_enu=positions[index].tolist())
    window = evaluate_window_crossing(positions, attitudes, plan['window'], envelope)
    return dict(response_s=response_s, yaw_response_s=yaw_response_s, heading_lookahead_m=lookahead_m,
                completed=done, fault=fault, elapsed_sim_s=time_s, waypoints_reached=controller.waypoint_index,
                geometry=geometry, window=window,
                limitation='Approximate velocity/yaw dynamics with measured-state feedback, not a PX4 flight.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--course', type=Path, default=Path('mission/course-window-x500.json'))
    parser.add_argument('--response', type=float, nargs='+', default=[.5, .66, .8])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in args.response):
        parser.error('response times must be positive and finite')
    plan = json.loads(args.course.read_text())
    results = [simulate(plan, value) for value in args.response]
    text = json.dumps(results, indent=2, allow_nan=False)+'\n'
    if args.output:
        with args.output.open('x') as output:
            output.write(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
