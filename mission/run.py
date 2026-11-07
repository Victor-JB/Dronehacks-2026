"""Fly the waypoint route using Gazebo odometry and body-frame Twist commands."""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import signal
from threading import Lock
import time
from uuid import uuid4

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

from .core import CourseController, ZERO, norm, sub
from .crossing import evaluate_window_crossing
from .frames import rotate_body_velocity

ROOT = Path(__file__).resolve().parents[1]
CLOCK_TOPIC = '/world/dronehacks/clock'
ODOM_TOPIC = '/model/course_quad/odometry'
TWIST_TOPIC = '/course_quad/gazebo/command/twist'
CONTACT_TOPICS = tuple(
    f'/model/course_quad/contact/{link}'
    for link in ('base_link', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3')
)
# The farthest rotor cylinder is centered at (0.13, +/-0.22, 0.023) m
# with radius 0.10 m and thickness 0.005 m. Its outer corner lies 0.3565 m
# from the model origin. The 0.47 x 0.47 x 0.11 m base box fits inside too.
X3_RADIUS = 0.36
# Allow the 50 Hz odometry publisher to fill its ten-sample velocity
# filter after the first update, which differentiates from an initial pose.
ODOM_WARMUP_S = 0.25


def seconds(stamp):
    return stamp.sec + stamp.nsec * 1e-9


def xyz(vector):
    return vector.x, vector.y, vector.z


class Gazebo:
    def __init__(self):
        self.node = Node()
        self.lock = Lock()
        self.clock = None
        self.odom = None
        self.contacts = deque()
        self.twist = self.node.advertise(TWIST_TOPIC, Twist)
        if not self.twist.valid():
            raise RuntimeError(f'cannot advertise {TWIST_TOPIC}')
        for msg_type, topic, handler in (
            (Clock, CLOCK_TOPIC, self.on_clock),
            (Odometry, ODOM_TOPIC, self.on_odom),
            *((Contacts, topic, self.on_contacts) for topic in CONTACT_TOPICS),
        ):
            if not self.node.subscribe(msg_type, topic, handler):
                raise RuntimeError(f'cannot subscribe to {topic}')

    def on_clock(self, message):
        with self.lock:
            self.clock = seconds(message.sim)

    def on_odom(self, message):
        with self.lock:
            self.odom = message

    def on_contacts(self, message):
        stamp = seconds(message.header.stamp)
        pairs = [(c.collision1.name, c.collision2.name) for c in message.contact]
        with self.lock:
            self.contacts.extend((stamp, names) for names in pairs)

    def read(self):
        with self.lock:
            contacts = list(self.contacts)
            self.contacts.clear()
            return self.clock, self.odom, contacts

    def command(self, world_velocity, yaw_rate=0.0, orientation=(0, 0, 0, 1)):
        x, y, z, w = orientation
        body_velocity = rotate_body_velocity((-x, -y, -z, w), world_velocity)
        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = body_velocity
        message.angular.z = yaw_rate
        if not self.twist.publish(message):
            raise RuntimeError(f'publish failed on {TWIST_TOPIC}')


class FlightLog:
    def __init__(self, output, summary_path, config, speed_scale):
        self.output = output
        self.summary_path = summary_path
        self.run_id = uuid4().hex
        self.summary_path.unlink(missing_ok=True)
        self.positions = []
        self.obstacles = Counter()
        self.ground_contacts = 0
        self.max_speed = 0.0
        self.write({
            'type': 'metadata',
            'run_id': self.run_id,
            'source': 'Gazebo Sim ground-truth odometry and contacts',
            'world': 'dronehacks',
            'vehicle': 'course_quad',
            'vehicle_model': 'OpenRobotics X3 UAV v4',
            'footprint_radius_m': X3_RADIUS,
            'speed_scale': speed_scale,
            'waypoint_count': len(config['waypoints']),
            'waypoints': config['waypoints'],
            'window': config['window'],
        })

    def write(self, record):
        self.output.write(json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n')

    def log_contacts(self, contacts, start):
        for stamp, names in contacts:
            course_collision = next((n for n in names if 'course_v3' in n), None)
            if course_collision:
                self.obstacles[course_collision] += 1
                if self.obstacles[course_collision] == 1:
                    self.write({
                        'type': 'contact',
                        'sim_time': stamp,
                        'elapsed_sim_s': max(0.0, stamp - start),
                        'collisions': names,
                    })
            elif any('ground_plane' in n for n in names):
                self.ground_contacts += 1

    def summary(self, config, controller, sim_time, start, fault):
        crossing = evaluate_window_crossing(self.positions, config['window'], X3_RADIUS)
        completed = (controller.done and crossing['full_envelope_crossed']
                     and not self.obstacles and not fault)
        record = {
            'type': 'summary',
            'run_id': self.run_id,
            'result': 'completed' if completed else 'failed',
            'sim_time': sim_time,
            'elapsed_sim_s': max(0.0, sim_time - start),
            'route_completed': controller.done,
            'waypoints_reached': controller.waypoint_index,
            'waypoint_count': len(controller.waypoints),
            'window': crossing,
            'course_contact_observations': sum(self.obstacles.values()),
            'course_contact_obstacles': dict(self.obstacles),
            'ground_contact_observations': self.ground_contacts,
            'max_measured_speed_m_s': self.max_speed,
            'footprint_radius_m': X3_RADIUS,
            'fault': fault,
        }
        self.write(record)
        temporary = self.summary_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(record, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.summary_path)
        print(json.dumps(record), flush=True)
        return 0 if completed else 1


def fly(gazebo, controller, config, flight, args):
    start = None
    latest_stamp = None
    clock = 0.0
    result = None
    phase = None
    last_log_time = None
    startup_deadline = time.monotonic() + args.startup_timeout
    while True:
        sim_time, odom, contacts = gazebo.read()
        if start is None:
            if time.monotonic() > startup_deadline:
                raise RuntimeError('waiting for Gazebo clock, odometry or Twist subscriber timed out')
            if (sim_time is None or odom is None or
                    seconds(odom.header.stamp) < ODOM_WARMUP_S or
                    not gazebo.twist.has_connections()):
                time.sleep(0.01)
                continue
            start = sim_time
            position = xyz(odom.pose.position)
            if norm(sub(position, controller.start)) > 0.2:
                raise RuntimeError('vehicle is away from launch. Reset the simulation before starting')
        if result is None:
            flight.log_contacts(contacts, start)
        stamp = seconds(odom.header.stamp)
        if sim_time < clock or (latest_stamp is not None and stamp < latest_stamp):
            fault = 'simulation_clock_reversed'
            break
        clock = sim_time
        if sim_time - stamp > controller.odom_timeout:
            fault = 'stale_odometry'
            break
        if stamp == latest_stamp:
            time.sleep(0.004)
            continue
        latest_stamp = stamp
        position = xyz(odom.pose.position)
        orientation = (*xyz(odom.pose.orientation), odom.pose.orientation.w)
        x, y, z, w = orientation
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        velocity = rotate_body_velocity(orientation, xyz(odom.twist.linear))
        command = controller.update(
            position, velocity, stamp, odom_age=sim_time - stamp, yaw=yaw,
        )
        if result is None:
            flight.positions.append(position)
            flight.max_speed = max(flight.max_speed, norm(velocity))
        gazebo.command(command.velocity, command.yaw_rate, orientation)
        if command.phase != phase:
            phase = command.phase
            print(f'{sim_time - start:6.2f} s  {phase}', flush=True)
        if result is None or sim_time - last_log_time >= 1.0 or command.fault:
            last_log_time = sim_time
            flight.write({
                'type': 'sample',
                'sim_time': sim_time,
                'elapsed_sim_s': max(0.0, sim_time - start),
                'position': position,
                'orientation_xyzw': orientation,
                'velocity': velocity,
                'command_velocity': command.velocity,
                'yaw_rate': command.yaw_rate,
                'phase': command.phase,
                'waypoint_index': controller.waypoint_index,
                'waypoint_count': len(controller.waypoints),
                'target': command.target,
                'done': command.done,
                'fault': command.fault,
            })
        if command.fault:
            fault = command.fault
            break
        if command.done and result is None:
            result = flight.summary(config, controller, sim_time, start, None)
            if args.exit_on_finish:
                return result
        if result is None and sim_time - start > args.max_sim_seconds:
            fault = 'mission_timeout'
            break
    if result is None:
        return flight.summary(config, controller, clock, start, fault)
    print(f'Gazebo handoff hold ended: {fault}', flush=True)
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--course', type=Path, default=ROOT / 'mission/course.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/gazebo/telemetry.jsonl')
    parser.add_argument('--speed-scale', type=float, default=0.65)
    parser.add_argument('--startup-timeout', type=float, default=30.0)
    parser.add_argument('--max-sim-seconds', type=float, default=150.0)
    parser.add_argument('--exit-on-finish', action='store_true',
                        help='exit at the landing handoff instead of holding position')
    args = parser.parse_args()
    config = json.loads(args.course.read_text())
    config['footprint_radius'] = X3_RADIUS
    controller = CourseController(config, speed_scale=args.speed_scale)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    gazebo = Gazebo()

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        with args.output.open('w', buffering=1) as output:
            summary_path = args.output.with_suffix('.summary.json')
            flight = FlightLog(output, summary_path, config, args.speed_scale)
            return fly(gazebo, controller, config, flight, args)
    except KeyboardInterrupt:
        return 130
    finally:
        gazebo.command(ZERO)


if __name__ == '__main__':
    raise SystemExit(main())
