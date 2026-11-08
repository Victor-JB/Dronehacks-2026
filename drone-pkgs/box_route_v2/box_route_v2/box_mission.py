"""Course route mission, v2 (box_route_v2 package; v1 = box_route, untouched).

    climb -> 13 waypoints from the 2026-11-07 odom map (walk_1107) -> land at the last one

Waypoints are copied from test_files/my_mission.py ROUTE: metres relative to where the drone is
when Offboard starts, x forward, y LEFT. Take off 1 ft ahead of the walkthrough's start, facing
the same way as the walkthrough. The route is rotated to the nose heading at Offboard start (it
was -0 deg on the 2026-11-08 flight, so in practice this changes nothing if the drone isn't
turned after the stack starts). It faces along each leg and starts turning toward the next leg
over the last TURN_AHEAD_M of a leg, so the nose turns away from what is ahead; at a corner sharper
than the turn rate allows, the setpoint waits while it finishes turning.

Rules from the 2026-11-07/08 flight data:
  - refuses to start if VIO's height disagrees with PX4's by > HEIGHT_CHECK_M (drifted VIO -> 3 m climb)
  - hovers TAKEOFF_SETTLE_S at the climb point before leg 1 (every take-off slid 0.2-0.4 m, then came back)
  - waypoint heights are judged on PX4's height; every hold is at PX4's height (VIO reads 0.1-0.4 m low)
  - VIO watchdog in flight (no messages / jumped / height or position far from PX4 / waypoint
    not reached 15 s after the setpoint got there) -> stop the route and land
  - SPEED_MPS 0.25 (was 0.4)

Built on test_files/my_mission.py (same contract: autonomy_pipeline.md):
  - before Offboard: publish the current position at 10 Hz (required for the takeover)
  - Offboard ends (pilot took over): stop, reset, never come back until re-armed
  - landing: hover 2 s at the last point, descend by TWIST at 0.3 m/s (the 0.3 m floor
    only applies to poses), keep pushing down after touchdown so PX4 disarms by itself
    (COM_DISARM_LAND, 2 s). No touchdown after 20 s: hover, log "Pilot, take over and land".
Never publishes to /fmu/in/*.
"""
import json
import math
import os

import yaml

try:                                    # the ROS package
    from box_route_v2 import mission_plan as MP
except ImportError:                     # run from the source folder
    import mission_plan as MP

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# ---- ROUTE SETTINGS -------------------------------------------------------------------------------
# DEFAULTS ONLY. The live values come from config/route.yaml (CONFIG_FILE), re-read while disarmed:
# edit that file, not these. These are used only if the file is missing or invalid at start-up.
CONFIG_FILE = '/root/team_ws/src/box_route_v2/config/route.yaml'   # the src copy: edits need no rebuild
SCALE = 1.0                 # every horizontal distance in ROUTE is multiplied by this
HEIGHT_M = 1.1              # flight height, m (~0.12 m under the tunnel overhang at ~1.23 m)
# ---------------------------------------------------------------------------------------------------
# The course route (dronehacks main, mode 5 walk_1107 = mode 7's route, 2026-11-07 evening walk).
# (forward m, left m) from the take-off point. Same numbers as test_files/my_mission.py ROUTE.
ROUTE = [
    (0.875, -0.260),
    (1.145, -4.100),
    (0.885, -4.810),
    (0.555, -5.040),
    (-0.215, -4.820),
    (-1.645, -5.090),
    (-3.035, -4.680),
    (-3.265, -3.520),
    (-3.465, -3.290),
    (-3.545, -1.980),
    (-3.175, -1.530),
    (-3.335, -0.930),
    (-3.355, -0.450),
]
SPEED_MPS = 0.25            # along each leg: the target point slides at this speed
                            # (was 0.4; slower = VIO keeps its features near walls)
WALL_SPEED_MPS = 0.15       # along legs marked wall: true in route.yaml
CLIMB_MPS = 0.25            # the first (straight up) waypoint
TAKEOFF_SETTLE_S = 5.0      # hover this long at the climb point before leg 1: every flight slid
                            # 0.2-0.4 m sideways in the first 2-3 s off the ground, then came back
YAW_RATE_DPS = 45.0         # heading changes at most this fast
FACE_TOL_DEG = 20.0         # the carrot only moves while the heading is this close to the wanted one
STRAIGHT_FACE_TOL_DEG = 5.0 # ...on the leg into a waypoint marked straight: true (the window): lined up
                            # first. 2026-11-08 run 2 was still yawing 166 -> 172 deg as it hit the window
VIO_HOLD_STEP_M = 0.1      # wall phase VIO height hold: max change of the correction per tick (10 Hz: 1 m/s)
ROTATE_WARN_DEG = 3.0       # warn when the nose at Offboard start is this far from the stack-start heading
TURN_AHEAD_M = 0.5          # start turning toward the next leg this far before the end of a leg
                            # (at most half the leg), so the nose turns away from the wall ahead
                            # instead of stopping at the node facing it and turning in place
ARRIVED_M = 0.3             # horizontal distance that counts as reaching a waypoint
ARRIVED_Z_M = 0.15          # height error that counts as reached...
Z_WAIT_S = 3.0              # ...but after this long at the waypoint, height no longer blocks
                            # (flight 2026-11-08: VIO read 0.24 m below PX4's height and the
                            # old 3-D check never passed, so it hovered at waypoint 3 for good)

HOVER_BEFORE_LAND_S = 2.0   # settle at the last waypoint before descending
DESCENT_MPS = 0.3           # descent speed, m/s
LAND_TIMEOUT_S = 20.0       # no touchdown by then: stop pushing down, hover, pilot lands

# VIO watchdog, while flying the route: any of these -> stop the route and land (pilot can take
# over). All three crashes so far (2026-11-07 21:01, 21:56, 2026-11-08 00:45) were VIO failing next
# to a wall: blown up to -41 m, frozen, or jumping 0.56 m in one frame.
VIO_STALE_S = 1.0           # no odometry message for this long (normal: one every 33 ms)
VIO_JUMP_M = 0.4            # odometry moved this far between two frames (normal: ~0.02 m)
VIO_PX4_XY_M = 0.5          # VIO and PX4 horizontal positions this far apart (normal: <= 0.22 m)
VIO_PX4_Z_M = 0.6           # VIO and PX4 heights this far apart (normal: <= 0.42 m). This is what
                            # catches a VIO that is stuck but still publishing (21:56: it kept
                            # changing by millimetres while the drone climbed 1.3 m)
WAYPOINT_TIMEOUT_S = 15.0   # setpoint at a waypoint this long without reaching it -> land (catches a
                            # VIO stuck in level flight, which PX4 would follow, so nothing disagrees)

HEIGHT_CHECK_M = 0.4        # VIO height above its origin vs PX4's own height: refuse to start past this.
                            # A drifted VIO height becomes the route's start height, and with HEIGHT_M on
                            # top the target hits flight_interface's 3.0 m ceiling: the drone climbs to 3 m
                            # (2026-11-08, ~02:40). Normal disagreement seen: 0.24 m (2026-11-08 flight).


# Safety limits and the turn option: route.yaml keys -> (built-in default, min, max). Optional in the
# file: a key left out keeps its default.
LIMITS = {
    'height_check_m': (HEIGHT_CHECK_M, 0.1, 2.0),          # won't start if VIO vs PX4 height differs more
    'vio_quiet_s': (VIO_STALE_S, 0.2, 5.0),                 # in flight: no VIO odometry this long -> land
    'vio_jump_m': (VIO_JUMP_M, 0.1, 2.0),                   # in flight: VIO moves this far in one frame -> land
    'vio_px4_xy_m': (VIO_PX4_XY_M, 0.1, 3.0),               # in flight: VIO vs PX4 sideways -> land
    'vio_px4_z_m': (VIO_PX4_Z_M, 0.1, 3.0),                 # in flight: VIO vs PX4 height -> land
    'waypoint_timeout_s': (WAYPOINT_TIMEOUT_S, 3.0, 120.0), # in flight: waypoint not reached this long -> land
    'turn_ahead_m': (TURN_AHEAD_M, 0.05, 2.0),              # how far before a waypoint the turn starts
}


def default_config():
    cfg = {'height_m': HEIGHT_M, 'scale': SCALE, 'speed_mps': SPEED_MPS,
           'wall_speed_mps': WALL_SPEED_MPS, 'waypoints_to_fly': 0, 'turn_early': True,
           'rotate_to_heading': False, 'sections': MP.parse_sections({}),
           'waypoints': [(float(x), float(y), False, None, False) for x, y in ROUTE]}
    cfg.update({k: v[0] for k, v in LIMITS.items()})
    return cfg


def load_config(path):
    """Read and check route.yaml. Returns the config dict; raises ValueError saying what is wrong."""
    try:
        with open(path) as f:
            raw = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise ValueError(f'cannot read {path}: {e}')
    if not isinstance(raw, dict):
        raise ValueError('the file is not a YAML mapping')

    def num(key, lo, hi):
        v = raw.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi:
            raise ValueError(f'{key} must be a number from {lo} to {hi} (got {v!r})')
        return float(v)

    cfg = {'height_m': num('height_m', 0.3, 2.0), 'scale': num('scale', 0.05, 2.0),
           'speed_mps': num('speed_mps', 0.05, 1.0), 'wall_speed_mps': num('wall_speed_mps', 0.05, 1.0)}
    n = raw.get('waypoints_to_fly', 0)
    if isinstance(n, bool) or not isinstance(n, int) or n < 0:
        raise ValueError(f'waypoints_to_fly must be a whole number >= 0 (got {n!r})')
    cfg['waypoints_to_fly'] = n
    for key, (default, lo, hi) in LIMITS.items():
        cfg[key] = num(key, lo, hi) if key in raw else default
    te = raw.get('turn_early', True)
    if not isinstance(te, bool):
        raise ValueError(f'turn_early must be true or false (got {te!r})')
    cfg['turn_early'] = te
    rh = raw.get('rotate_to_heading', False)
    if not isinstance(rh, bool):
        raise ValueError(f'rotate_to_heading must be true or false (got {rh!r})')
    cfg['rotate_to_heading'] = rh
    wps = raw.get('waypoints')
    if not isinstance(wps, list) or not wps:
        raise ValueError('waypoints must be a non-empty list')
    cfg['waypoints'] = []
    for i, w in enumerate(wps, 1):
        if not isinstance(w, dict):
            raise ValueError(f'waypoint {i} must look like {{fwd: 1.0, left: -0.5, wall: false}}')
        f, l, wall, h = w.get('fwd'), w.get('left'), w.get('wall', False), w.get('height')
        straight = w.get('straight', False)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or abs(v) > 20 for v in (f, l)):
            raise ValueError(f'waypoint {i}: fwd and left must be numbers within +-20 m (got {f!r}, {l!r})')
        if not isinstance(wall, bool):
            raise ValueError(f'waypoint {i}: wall must be true or false (got {wall!r})')
        if not isinstance(straight, bool):
            raise ValueError(f'waypoint {i}: straight must be true or false (got {straight!r})')
        if h is not None and (isinstance(h, bool) or not isinstance(h, (int, float)) or not 0.3 <= h <= 2.0):
            raise ValueError(f'waypoint {i}: height must be a number from 0.3 to 2.0 (got {h!r})')
        cfg['waypoints'].append((float(f), float(l), wall, None if h is None else float(h), straight))
    cfg['sections'] = MP.parse_sections(raw)      # pad1_hover, wall, pad2_hover, reverse (all optional)
    return cfg


def route_body(cfg=None):
    """Waypoints in the start frame: x forward, y LEFT, z up (ENU-style), climb first."""
    cfg = cfg or default_config()
    h, k = cfg['height_m'], cfg['scale']
    # a waypoint's own height (if set) holds only for that waypoint; the climb/descent happens in
    # place at the start of its leg, and the next waypoint goes back to height_m the same way
    return [(0.0, 0.0, h)] + [(k * x, k * y, h if wh is None else wh) for x, y, _, wh, _ in cfg['waypoints']]


def route_heading(cfg, yaw):
    """The heading the route is laid out along: the stack-start heading (0 in the odom frame) unless
    rotate_to_heading. v1 rotated it to the nose at Offboard start; 2026-11-08 run 1 started 4 deg turned,
    the whole route swung 0.3 m sideways at the tunnel exit and it hit the wall there."""
    return (yaw if yaw is not None else 0.0) if cfg.get('rotate_to_heading') else 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BoxMission(Node):

    def __init__(self):
        super().__init__('box_mission')
        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.position = None     # current ENU position
        self.yaw = None          # current heading (ENU, rad, from VIO)
        self.offboard = False
        self.armed = False
        self.landed = None       # PX4's land detector, from /skybeta/state
        self.prev_armed = None   # armed on the previous /skybeta/state (None: none seen yet)
        self.stale = False       # Offboard was already on when the drone armed (or when this node
                                 # started armed): a leftover, not the pilot's switch. Never start.
        # mode: last_waypoint N > 0 flies only waypoints 0..N, then settles and lands at N
        # (box_wp2.launch.py sets 2). 0 = the whole route (box_mission.launch.py).
        self.last_wp = int(self.declare_parameter('last_waypoint', 0).value)
        self.config_file = self.declare_parameter('config_file', CONFIG_FILE).value
        self.cfg = default_config()
        self.body, self.wall, self.straight, self.steps = [], [], [], []
        self.hold_since = None   # when a step with hold_s was reached
        self.wall_base = None    # VIO-PX4 height offset when the wall phase began (PX4 still saw the floor)
        self.z_corr = 0.0        # wall phase: subtracted from the commanded height (VIO height hold)
        self.frozen = None       # (x, y, z, yaw) held after a VIO fault in the wall phase (on_vio_fault: hold)
        self.config_mtime = None
        self.config_error = None
        self.config_pending = False
        self.next_config_check = 0.0
        self.wp = None           # route in the vision frame, set when Offboard starts
        self.start = None        # ENU position when Offboard started
        self.index = 0
        self.carrot = None       # the setpoint actually sent: slides toward the waypoint
        self.at_target_since = None   # when the carrot reached the current waypoint
        self.yaw_sp = None       # commanded heading
        self.phase = 'idle'      # idle -> waypoints -> settle -> land -> done
        self.phase_since = None
        self.origin = None       # flight_interface's envelope origin (ENU, the VIO frame), from /skybeta/state
        self.px4_up = None       # PX4's own height (ENU z of its local position), None if not valid
        self.height_err = None   # VIO height above origin minus PX4 height, m (None: unknown)
        self.height_bad = False  # refused at Offboard start because of height_err: until re-armed
        self.last_height_warn = 0.0
        self.px4_xy = None       # PX4's own horizontal position (ENU), from /skybeta/state
        self.start_px4_up = None # PX4 height when Offboard started (waypoint heights are judged on it)
        self.vio_msg_t = None    # last time an odometry message arrived
        self.vio_fault = None    # watchdog: why VIO is not trusted (set in flight, cleared at begin)
        self.apply_config(self.cfg)                       # defaults, then the file if it is good
        self.check_config(force=True)
        self.create_timer(0.1, self.tick)   # 10 Hz
        # Read-only status for the live view (box_route.live_view). Nothing reads it back: it does
        # not touch the control above.
        self.last_sp = None      # last pose setpoint sent (ENU), None while a twist is in force
        self.status_pub = self.create_publisher(String, '/box_route/status', 10)
        self.create_timer(0.2, self.publish_status)   # 5 Hz

    def apply_config(self, cfg):
        """Use cfg from now on: the route (cut to waypoints_to_fly / last_waypoint) and its speeds."""
        self.cfg = cfg
        steps = MP.build_steps(cfg)                  # course -> [pad 1] -> [wall] -> [pad 2] -> [reverse]
        if 0 < self.last_wp < len(steps) - 1:
            steps = steps[:self.last_wp + 1]
        self.steps = steps
        self.body = [(s['x'], s['y'], s['z']) for s in steps]
        self.wall = [s['slow'] for s in steps]          # wall[i]: the leg INTO step i is slow
        self.straight = [s['straight'] for s in steps]  # straight[i]: the leg INTO step i, lined up
        n = len(self.body) - 1
        self.get_logger().info(f'mission: {len(steps)} steps: {MP.describe(steps)}; lands at the last')
        stop = ''
        walls = [i for i, w in enumerate(self.wall) if w]
        self.get_logger().info(
            f'course route: {cfg["height_m"]} m up, scale {cfg["scale"]}, {n} waypoints{stop}, '
            f'{cfg["speed_mps"]} m/s, {cfg["wall_speed_mps"]} m/s into wall waypoints {walls or "none"}, '
            f'then land. Waypoints (fwd, left, up): {[tuple(round(v, 3) for v in p) for p in self.body]}')
        self.get_logger().info(
            f'limits: start only if VIO/PX4 height within {cfg["height_check_m"]} m; land if VIO quiet '
            f'{cfg["vio_quiet_s"]} s, jumps {cfg["vio_jump_m"]} m, off PX4 by {cfg["vio_px4_xy_m"]} m sideways '
            f'or {cfg["vio_px4_z_m"]} m in height, or a waypoint takes {cfg["waypoint_timeout_s"]} s. '
            f'turn_early {"ON (" + str(cfg["turn_ahead_m"]) + " m ahead)" if cfg["turn_early"] else "OFF (stop, turn, go)"}; '
            f'straight (lined up, no early turn) into waypoints {[i for i, s in enumerate(self.straight) if s] or "none"}; '
            f'route {"ROTATED to the nose at Offboard start" if cfg["rotate_to_heading"] else "fixed to the course (stack-start heading)"}')

    def check_config(self, force=False):
        """Re-read route.yaml when it changed -- only while disarmed and idle, never in flight."""
        try:
            mtime = os.stat(self.config_file).st_mtime
        except OSError:
            if force or self.config_error is None:
                self.config_error = f'{self.config_file} not found: using the built-in defaults'
                self.get_logger().warn(self.config_error)
            return
        if mtime == self.config_mtime and not force:
            return
        if self.armed or self.start is not None:
            if not self.config_pending:
                self.config_pending = True
                self.get_logger().info('route.yaml changed: it is applied after the drone disarms')
            return
        self.config_mtime, self.config_pending = mtime, False
        try:
            cfg = load_config(self.config_file)
        except ValueError as e:
            self.config_error = str(e)
            self.get_logger().error(f'config REJECTED: {e}. Keeping the previous settings.')
            return
        self.config_error = None
        self.get_logger().info(f'config loaded from {self.config_file}')
        self.apply_config(cfg)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pos = (p.x, p.y, p.z)
        if self.position is not None:
            step = math.dist(pos, self.position)
            if step > self.cfg['vio_jump_m'] and self.vio_fault is None:
                self.vio_fault = f'VIO jumped {step:.2f} m in one frame'
        self.vio_msg_t = self.now()
        self.position = pos
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_state(self, msg):
        state = json.loads(msg.data)
        self.offboard = bool(state.get('offboard_active', False))
        self.armed = bool(state.get('armed', False))
        self.landed = state.get('landed')
        origin, local = state.get('origin_enu'), state.get('local_enu')
        if origin:
            self.origin = tuple(origin)
        self.px4_up = float(local[2]) if local else None
        self.px4_xy = (float(local[0]), float(local[1])) if local else None
        if self.position is not None and self.origin is not None and self.px4_up is not None:
            self.height_err = (self.position[2] - self.origin[2]) - self.px4_up
        else:
            self.height_err = None
        # Start only on the real sequence: arm first, THEN Offboard (pilot switches to Position).
        # 2026-11-08: after a kill-switch stop PX4 stayed in OFFBOARD while disarmed; the stack
        # restarted and the old code started the route on the ground.
        if self.armed and not self.prev_armed:          # just armed, or node started armed
            self.stale = self.offboard or self.prev_armed is None
            if self.stale:
                self.get_logger().warn(
                    'armed with Offboard already on (leftover state): NOT starting the mission. '
                    'Disarm, power-cycle the Pixhawk if it persists, re-arm, then switch to Position.')
        elif not self.armed:
            self.stale = False
            self.height_bad = False
            self.vio_fault = None
        self.prev_armed = self.armed
        if (not self.armed and self.height_err is not None and abs(self.height_err) > self.cfg['height_check_m']
                and self.now() - self.last_height_warn > 5.0):
            self.last_height_warn = self.now()
            self.get_logger().warn(
                f'VIO height {self.position[2] - self.origin[2]:+.2f} m vs PX4 {self.px4_up:+.2f} m '
                f'(off by {self.height_err:+.2f} m): the mission will NOT start like this. '
                'Restart the stack on the ground, still: docker rm -f skybeta-stack')

    def height_ok(self):
        """(ok, why): VIO's height above its origin agrees with PX4's own height."""
        if self.origin is None or self.px4_up is None or self.height_err is None:
            return False, ('no PX4 local position or no envelope origin in /skybeta/state, so the VIO '
                           'height cannot be checked')
        if abs(self.height_err) > self.cfg['height_check_m']:
            return False, (f'VIO says {self.position[2] - self.origin[2]:+.2f} m above its origin, PX4 says '
                           f'{self.px4_up:+.2f} m: off by {self.height_err:+.2f} m (limit {self.cfg["height_check_m"]} m). '
                           'VIO has drifted (was the drone carried?) or the stack started before PX4 '
                           'had a position')
        return True, ''

    def hold_here(self):
        """Hold the current spot, at PX4's height (put into the VIO frame) whenever PX4 has one: PX4
        flies on its rangefinder, and VIO reads 0.1-0.4 m low, so a hold at the VIO height is a dip
        (2026-11-07 21:01) or, with a drifted VIO, a climb to the 3 m ceiling."""
        x, y, z = self.position
        if self.origin is not None and self.px4_up is not None:
            z = self.origin[2] + self.px4_up
        self.send(x, y, z)

    def wall_rules(self):
        """The wall section's settings if the current step is in the wall phase, else None."""
        if not self.steps or self.index >= len(self.steps) or self.steps[self.index]['phase'] != 'wall':
            return None
        return self.cfg['sections']['wall']

    def vio_up_px4_equiv(self):
        """VIO's height, put on PX4's scale with the VIO-PX4 offset measured when the wall phase began
        (PX4 still saw the floor then). None if unknown."""
        if self.wall_base is None or self.origin is None or self.start_px4_up is None:
            return None
        return self.position[2] - self.origin[2] - self.wall_base - self.start_px4_up

    def vio_problem(self):
        """The watchdog: why VIO cannot be trusted right now, or None."""
        if self.vio_fault:
            return self.vio_fault
        if self.vio_msg_t is None or self.now() - self.vio_msg_t > self.cfg['vio_quiet_s']:
            return f'no VIO odometry for over {self.cfg["vio_quiet_s"]} s'
        w = self.wall_rules()
        px4_height_trusted = not (w and w['ignore_px4_height'])   # over the wall the rangefinder sees the wall top
        if px4_height_trusted and self.height_err is not None and abs(self.height_err) > self.cfg['vio_px4_z_m']:
            return f'VIO and PX4 heights are {self.height_err:+.2f} m apart (VIO stuck?)'
        if self.px4_xy is not None and self.origin is not None:
            d = math.hypot(self.position[0] - self.origin[0] - self.px4_xy[0],
                           self.position[1] - self.origin[1] - self.px4_xy[1])
            if d > self.cfg['vio_px4_xy_m']:
                return f'VIO and PX4 horizontal positions are {d:.2f} m apart'
        return None

    def leg_heading(self, i):
        """Heading of the leg that ends at waypoint i, or None for a (near) vertical leg."""
        (px, py, _), (tx, ty, _) = self.wp[i - 1], self.wp[i]
        if math.hypot(tx - px, ty - py) <= 0.05:
            return None
        return math.atan2(ty - py, tx - px)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_phase(self, phase):
        self.phase = phase
        self.phase_since = self.now()

    def send(self, x, y, z, yaw=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        if yaw is not None:      # left all-zero (before Offboard): do not command yaw
            msg.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.orientation.w = math.cos(yaw / 2)
        self.pub.publish(msg)
        self.last_sp = (float(x), float(y), float(z))   # for the live view only

    def send_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = float(vx)    # world ENU axes, not body
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.twist_pub.publish(msg)
        self.last_sp = None                              # for the live view only

    def begin(self):
        """Offboard just started: anchor the route at the current position and heading."""
        self.start = self.position
        nose = self.yaw if self.yaw is not None else 0.0
        h0 = route_heading(self.cfg, self.yaw)
        c, s = math.cos(h0), math.sin(h0)
        sx, sy, sz = self.start
        self.wp = [(sx + c * bx - s * by, sy + s * bx + c * by, sz + bz)
                   for bx, by, bz in self.body]
        self.carrot = self.position
        self.yaw_sp = nose                  # start from where the nose points; it turns onto leg 1 from there
        if abs(math.degrees(wrap(nose - h0))) > ROTATE_WARN_DEG:
            self.get_logger().warn(
                f'the nose is {math.degrees(wrap(nose - h0)):+.1f} deg from the route heading: the route stays '
                f'fixed to the course (rotate_to_heading false), the drone turns onto it')
        self.index = 0
        self.at_target_since = None
        self.start_px4_up = self.px4_up
        self.vio_fault = None
        self.hold_since, self.wall_base, self.z_corr = None, None, 0.0
        self.set_phase('waypoints')
        self.get_logger().info(
            f'Offboard active: mission start at {tuple(round(v, 2) for v in self.start)}, '
            f'heading {math.degrees(h0):.0f} deg')

    def publish_status(self):
        """/box_route/status (JSON, ENU metres): what the live view draws. Read-only."""
        route, preview = self.wp, False
        if route is None and self.position is not None:
            # not flying yet: show where the route WOULD go from here (current spot and heading)
            h0 = route_heading(self.cfg, self.yaw)
            c, s = math.cos(h0), math.sin(h0)
            sx, sy, sz = self.position
            route = [(sx + c * bx - s * by, sy + s * bx + c * by, sz + bz) for bx, by, bz in self.body]
            preview = True
        st = {
            'phase': self.phase, 'index': self.index, 'legs': len(self.body) - 1,
            'route': [[round(v, 3) for v in p] for p in route] if route else None,
            'route_preview': preview,
            'start': [round(v, 3) for v in self.start] if self.start else None,
            'setpoint': [round(v, 3) for v in self.last_sp] if self.last_sp and self.start else None,
            'target': [round(v, 3) for v in self.wp[self.index]] if self.wp and self.phase == 'waypoints' else None,
            'stale_offboard': self.stale,
            'height_err': round(self.height_err, 3) if self.height_err is not None else None,
            'height_refused': self.height_bad, 'height_m': self.cfg['height_m'], 'scale': self.cfg['scale'],
            'speed_mps': self.cfg['speed_mps'], 'wall_speed_mps': self.cfg['wall_speed_mps'],
            'body': [[round(v, 3) for v in p[:2]] for p in self.body], 'wall': self.wall,
            'straight': self.straight, 'rotate_to_heading': self.cfg['rotate_to_heading'],
            'step': (self.steps[self.index]['label'] if self.steps and self.index < len(self.steps) else None),
            'step_phases': [s['phase'] for s in self.steps], 'z_corr': round(self.z_corr, 3),
            'sections': {k: v['enabled'] for k, v in self.cfg['sections'].items()},
            'config_error': self.config_error, 'config_pending': self.config_pending,
            'limits': {k: self.cfg[k] for k in LIMITS}, 'turn_early': self.cfg['turn_early'],
        }
        self.status_pub.publish(String(data=json.dumps(st)))

    def tick(self):
        if self.now() >= self.next_config_check:
            self.next_config_check = self.now() + 1.0
            self.check_config()
        if self.position is None:
            return                      # no VIO yet: publish nothing
        if not (self.offboard and self.armed and not self.stale and not self.height_bad):
            if self.start is not None:
                if not self.armed:
                    self.get_logger().info('disarmed: mission over.')
                else:
                    self.get_logger().warn('Offboard ended: the pilot has it. Mission reset.')
                self.start, self.wp, self.index, self.carrot, self.yaw_sp = None, None, 0, None, None
                self.frozen, self.hold_since, self.wall_base, self.z_corr = None, None, None, 0.0
                self.set_phase('idle')
            self.hold_here()            # keeps a fresh setpoint: required for the takeover
            return
        if self.start is None:
            ok, why = self.height_ok()
            if not ok:
                self.height_bad = True
                self.get_logger().error(
                    f'NOT starting the mission: {why}. Holding. Pilot: take over and land, then '
                    'restart the stack on the ground (docker rm -f skybeta-stack) before re-arming.')
                self.hold_here()
                return
            self.begin()

        if self.phase in ('waypoints', 'settle'):
            why = self.vio_problem()
            w = self.wall_rules() if self.phase == 'waypoints' else None
            if why and w and w['on_vio_fault'] == 'hold':
                # over/near the wall a descent could come down ON it: hold the last setpoint, pilot lands
                self.get_logger().error(f'{why} in the WALL phase: HOLDING here (wall.on_vio_fault: hold). '
                                        'PILOT, TAKE OVER AND LAND.')
                self.frozen = (self.carrot[0], self.carrot[1], self.carrot[2] - self.z_corr, self.yaw_sp)
                self.set_phase('freeze')
            elif why:
                self.get_logger().error(f'{why}: route stopped, LANDING. Pilot, take over if needed.')
                self.set_phase('land')

        if self.phase == 'freeze':
            self.send(*self.frozen)
            return

        if self.phase == 'waypoints':
            target = self.wp[self.index]
            facing = True
            heading = self.leg_heading(self.index) if self.index > 0 else None
            if heading is not None:
                # face along the leg; over its last TURN_AHEAD_M, blend toward the next leg's heading,
                # so the turn is spread along the approach and the nose turns away from what is ahead
                want = heading
                nxt = self.leg_heading(self.index + 1) if self.index + 1 < len(self.wp) else None
                straight = self.straight[self.index]   # the window: no turning on this leg at all
                if nxt is not None and self.cfg['turn_early'] and not straight:   # off: stop, turn, go
                    px, py, _ = self.wp[self.index - 1]
                    ahead = min(self.cfg['turn_ahead_m'], 0.5 * math.hypot(target[0] - px, target[1] - py))
                    left = math.hypot(target[0] - self.carrot[0], target[1] - self.carrot[1])
                    if left < ahead:
                        want = wrap(heading + wrap(nxt - heading) * (1.0 - left / ahead))
                turn = math.radians(YAW_RATE_DPS) * 0.1
                self.yaw_sp = wrap(self.yaw_sp + max(-turn, min(turn, wrap(want - self.yaw_sp))))
                # the carrot waits while the heading lags (a sharp corner): then it turns in place
                tol = math.radians(STRAIGHT_FACE_TOL_DEG if straight else FACE_TOL_DEG)
                facing = (abs(wrap(want - self.yaw_sp)) < tol
                          and self.yaw is not None
                          and abs(wrap(want - self.yaw)) < tol)
            step_info = self.steps[self.index]
            w = self.wall_rules()
            if w is not None and self.wall_base is None:
                # entering the wall phase: PX4 still sees the floor, so this is the honest VIO-PX4 offset
                self.wall_base = self.height_err if self.height_err is not None else 0.0
                self.get_logger().info(
                    f'WALL phase: {"ignoring PX4 height in the checks, " if w["ignore_px4_height"] else ""}'
                    f'{"VIO height hold on (guard " + str(w["vio_hold_guard_m"]) + " m), " if w["vio_height_hold"] else ""}'
                    f'VIO fault -> {w["on_vio_fault"]}; VIO-PX4 height offset {self.wall_base:+.2f} m')
            speed = (CLIMB_MPS if self.index == 0 else
                     self.cfg['wall_speed_mps'] if self.wall[self.index] else
                     w['speed_mps'] if w is not None else self.cfg['speed_mps'])
            gap_z = target[2] - self.carrot[2]
            if abs(gap_z) > 1e-6:
                # height change FIRST, in place (a waypoint with its own height, e.g. the window):
                # climb/descend where it is, then fly the leg level
                s = CLIMB_MPS * 0.1
                z = target[2] if abs(gap_z) <= s else self.carrot[2] + math.copysign(s, gap_z)
                self.carrot = (self.carrot[0], self.carrot[1], z)
                if self.carrot[:2] == target[:2] and z == target[2]:
                    self.carrot = target
            else:
                step = speed * 0.1 if facing else 0.0
                gap = math.dist(self.carrot, target)
                if gap <= step:
                    self.carrot = target
                elif step > 0:
                    self.carrot = tuple(c + (t - c) * step / gap for c, t in zip(self.carrot, target))
            # VIO height hold (wall phase): if PX4's rangefinder height is pulled off by the wall top, PX4
            # flies the wrong height; VIO sees it and the command is shifted back (deadband vio_hold_guard_m,
            # up to 1 m/s). Outside the wall phase any correction left unwinds at 0.1 m/s.
            vio_up = self.vio_up_px4_equiv()
            if w is not None and w['vio_height_hold'] and vio_up is not None:
                off = vio_up - (self.carrot[2] - self.start[2])     # > 0: higher than wanted (by VIO)
                g = w['vio_hold_guard_m']
                if off > g:
                    self.z_corr += min(VIO_HOLD_STEP_M, off - g)
                elif off < -g:
                    self.z_corr -= min(VIO_HOLD_STEP_M, -g - off)
                self.z_corr = max(-2.0, min(2.0, self.z_corr))
            elif self.z_corr:
                self.z_corr = math.copysign(max(0.0, abs(self.z_corr) - 0.01), self.z_corr)
            self.send(self.carrot[0], self.carrot[1], self.carrot[2] - self.z_corr, self.yaw_sp)
            if self.carrot != target:
                self.at_target_since = None
            elif self.at_target_since is None:
                self.at_target_since = self.now()
            elif self.now() - self.at_target_since > self.cfg['waypoint_timeout_s']:
                self.get_logger().error(
                    f'waypoint {self.index} not reached {self.cfg["waypoint_timeout_s"]:.0f} s after the setpoint got '
                    'there (VIO stuck?): route stopped, LANDING. Pilot, take over if needed.')
                self.set_phase('land')
                return
            xy = math.hypot(self.position[0] - target[0], self.position[1] - target[1])
            # height judged on PX4's height (what it flies on), VIO's only if PX4 has none
            if w is not None and w['ignore_px4_height'] and vio_up is not None:
                up = vio_up                                  # wall phase: the rangefinder may see the wall top
            elif self.px4_up is not None and self.start_px4_up is not None:
                up = self.px4_up - self.start_px4_up
            else:
                up = self.position[2] - self.start[2]
            dz = up - (target[2] - self.start[2])
            # the height check may only be waived once the drone is really flying: PX4 says airborne
            # and it is at least ~half way up (not sitting on the ground) -- or hovering low over a pad
            airborne = self.landed is False and (up > 0.5 * self.cfg['height_m'] or step_info['hover'])
            if step_info['hover'] and self.landed:
                # PX4 thinks it has landed while hovering over a pad: it would disarm in 2 s. Climb now.
                self.get_logger().warn(f'{step_info["label"]}: PX4 reports LANDED: climbing back at once (no disarm)')
                self.at_target_since, self.hold_since = None, None
                self.index += 1
                return
            waited = (airborne and self.at_target_since is not None
                      and self.now() - self.at_target_since > Z_WAIT_S)
            settled = (self.index > 0 or (self.at_target_since is not None
                                          and self.now() - self.at_target_since > TAKEOFF_SETTLE_S))
            reached = self.carrot == target and xy < ARRIVED_M and (abs(dz) < ARRIVED_Z_M or waited) and settled
            if reached and step_info['hold_s'] > 0:
                if self.hold_since is None:
                    self.hold_since = self.now()
                    self.get_logger().info(f'{step_info["label"]}: holding {step_info["hold_s"]:.1f} s')
                if self.now() - self.hold_since < step_info['hold_s']:
                    self.at_target_since = self.now()        # holding on purpose: not a timeout
                    reached = False
            if reached:
                if abs(dz) >= ARRIVED_Z_M:
                    self.get_logger().warn(f'height off by {dz:+.2f} m: moving on anyway')
                self.at_target_since, self.hold_since = None, None
                if self.index < len(self.wp) - 1:
                    self.index += 1
                    s = self.steps[self.index]
                    self.get_logger().info(f'step {self.index}/{len(self.wp) - 1} [{s["phase"]}] {s["label"]}: '
                                           f'{tuple(round(v, 3) for v in self.body[self.index])} (fwd, left, up)')
                else:
                    self.get_logger().info('last waypoint reached: settling before landing')
                    self.set_phase('settle')

        elif self.phase == 'settle':
            self.send(*self.wp[-1], self.yaw_sp)
            if self.now() - self.phase_since > HOVER_BEFORE_LAND_S:
                self.get_logger().info(f'landing: descending at {DESCENT_MPS} m/s')
                self.set_phase('land')

        elif self.phase == 'land':
            if self.landed or not self.armed:
                self.get_logger().info('touchdown: PX4 disarms by itself in ~2 s')
                self.set_phase('done')
            elif self.now() - self.phase_since > LAND_TIMEOUT_S:
                self.get_logger().warn(
                    f'no touchdown after {LAND_TIMEOUT_S:.0f} s: hovering. Pilot, take over and land.')
                self.set_phase('done')
                self.hold_here()            # hold here, at PX4's height
            else:
                self.send_velocity(0.0, 0.0, -DESCENT_MPS)

        elif self.phase == 'done':
            if self.armed and self.landed:
                self.send_velocity(0.0, 0.0, -DESCENT_MPS)   # stay pressed down until PX4 disarms
            # otherwise publish nothing new: the last pose holds (or the twist expires after
            # 1 s and flight_interface holds position)


def main():
    rclpy.init()
    node = BoxMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                            # Ctrl-C or the stack stopping
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
