"""Offline check of box_route_v2.box_mission without ROS: stubs rclpy and the messages, flies the real
node against a drone that lags its setpoint. python3 test/sim_offline.py [config] [nose_deg]"""
import sys, os, types, math, json
HERE = os.path.dirname(os.path.abspath(__file__)); PKG = os.path.dirname(HERE)
T = [0.0]
class _Msg:
    def __init__(self, **k):
        self.__dict__.update(k)
def _vec(): return _Msg(x=0.0, y=0.0, z=0.0)
class PoseStamped:
    def __init__(self):
        self.header = _Msg(stamp=None, frame_id=''); self.pose = _Msg(position=_vec(), orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))
class TwistStamped:
    def __init__(self):
        self.header = _Msg(stamp=None, frame_id=''); self.twist = _Msg(linear=_vec(), angular=_vec())
class String:
    def __init__(self, data=''): self.data = data
class Odometry: pass
class _Log:
    def __init__(self): self.lines = []
    def _p(self, lvl, s): self.lines.append((T[0], lvl, s)); print(f'{T[0]:7.1f} {lvl} {s[:200]}') if VERBOSE else None
    def info(self, s): self._p('I', s)
    def warn(self, s): self._p('W', s)
    warning = warn
    def error(self, s): self._p('E', s)
class _Clock:
    def now(self): return types.SimpleNamespace(nanoseconds=int(T[0] * 1e9), to_msg=lambda: None)
class Node:
    def __init__(self, name): self._pubs = {}; self._timers = []; self._log = _Log()
    def create_publisher(self, typ, topic, q):
        pub = types.SimpleNamespace(msgs=[], publish=None); pub.publish = pub.msgs.append; self._pubs[topic] = pub; return pub
    def create_subscription(self, *a): pass
    def create_timer(self, period, cb): self._timers.append([period, cb, 0.0])
    def get_logger(self): return self._log
    def get_clock(self): return _Clock()
    def declare_parameter(self, name, default):
        v = PARAMS.get(name, default); return types.SimpleNamespace(value=v)
    def destroy_node(self): pass
mods = {'rclpy': types.ModuleType('rclpy'), 'rclpy.node': types.ModuleType('rclpy.node'),
        'rclpy.executors': types.ModuleType('rclpy.executors'), 'geometry_msgs': types.ModuleType('geometry_msgs'),
        'geometry_msgs.msg': types.ModuleType('geometry_msgs.msg'), 'nav_msgs': types.ModuleType('nav_msgs'),
        'nav_msgs.msg': types.ModuleType('nav_msgs.msg'), 'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': types.ModuleType('std_msgs.msg')}
mods['rclpy.node'].Node = Node; mods['rclpy.executors'].ExternalShutdownException = Exception
mods['geometry_msgs.msg'].PoseStamped = PoseStamped; mods['geometry_msgs.msg'].TwistStamped = TwistStamped
mods['nav_msgs.msg'].Odometry = Odometry; mods['std_msgs.msg'].String = String
sys.modules.update(mods); sys.path.insert(0, PKG)
VERBOSE = '-v' in sys.argv
WALL_FX = next((float(a.split('=')[1]) if '=' in a else 1.0 for a in sys.argv if a.startswith('--wall-fx')), 0.0)
# fraction of the wall-top step PX4's height takes (1 = follows the rangefinder fully; 11-07 data: ~0.14)
VIO_LOW = 0.2 if '--vio-low' in sys.argv else 0.0   # VIO height reads this much low (seen in flight: 0.1-0.4)
args = [a for a in sys.argv[1:] if not a.startswith('-')]
WALL_X = (-1.75, -1.45); WALL_TOP = 1.9   # the wall slab (scan), for --wall-fx and the clearance check
def px4_err(p):
    """how much LOWER than the truth PX4 thinks it is (its height = the rangefinder, over the wall top)"""
    return WALL_TOP * WALL_FX if (WALL_FX and WALL_X[0] - 0.05 < p[0] < WALL_X[1] + 0.05 and -1.2 < p[1] < 0.2) else 0.0
CFG = args[0] if args else os.path.join(PKG, 'config', 'route.yaml')
NOSE = math.radians(float(args[1])) if len(args) > 1 else 0.0
PARAMS = {'config_file': CFG}
from box_route_v2 import box_mission as M

def run(max_t=900.0, dt=0.02):
    n = M.BoxMission(); log = n._log
    pos = [0.0, 0.0, 0.0]; vel = [0.0, 0.0, 0.0]; yaw = NOSE; sp = None; tw = None; armed = False; offb = False; landed = True
    track = []
    def odom():
        q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        m = types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=pos[0], y=pos[1], z=pos[2] - VIO_LOW * min(1.0, pos[2])), orientation=q)))
        n.on_odom(m)
    def state():
        n.on_state(String(json.dumps({'armed': armed, 'offboard_active': offb, 'landed': landed,
                                      'origin_enu': [0, 0, 0], 'local_enu': [pos[0], pos[1], pos[2] - px4_err(pos)]})))
    k = 0
    while T[0] < max_t:
        if T[0] > 2 and not armed: armed = True
        if T[0] > 3 and not offb: offb = True
        if k % 2 == 0: odom()
        if k % 10 == 0: state()
        for tm in n._timers:
            if T[0] >= tm[2]: tm[2] = T[0] + tm[0]; tm[1]()
        p = n._pubs['/skybeta/setpoint/pose'].msgs; w = n._pubs['/skybeta/setpoint/twist'].msgs
        if p: m = p[-1]; sp = (m.pose.position.x, m.pose.position.y, m.pose.position.z, m.pose.orientation); p.clear(); tw = None
        if w: tw = w[-1].twist.linear; w.clear()
        if offb and armed:
            if tw is not None:
                tgt_v = [tw.x, tw.y, tw.z]
            elif sp:
                est = [pos[0], pos[1], pos[2] - px4_err(pos)]       # PX4 flies on its own estimate
                tgt_v = [max(-0.8, min(0.8, 1.2 * (sp[i] - est[i]))) for i in range(3)]
            else:
                tgt_v = [0, 0, 0]
            for i in range(3):
                vel[i] += (tgt_v[i] - vel[i]) * min(1.0, dt / 0.3); pos[i] += vel[i] * dt
            if sp and tw is None and (sp[3].z or sp[3].w != 1.0):
                yw = 2 * math.atan2(sp[3].z, sp[3].w); e = math.atan2(math.sin(yw - yaw), math.cos(yw - yaw))
                yaw += max(-math.radians(60) * dt, min(math.radians(60) * dt, e * dt / 0.3))
            if pos[2] <= 0: pos[2] = 0.0
            landed = pos[2] < 0.03 and T[0] > 10
            if landed and n.phase == 'done': armed = False
        track.append((T[0], *pos, yaw, n.phase, n.index, n.steps[n.index]['phase'] if n.steps and n.index < len(n.steps) else '', n.z_corr))
        T[0] += dt; k += 1
        if not armed and T[0] > 10: break
    return n, track, log.lines

n, track, lines = run()
errs = [l for l in lines if l[1] == 'E']
ph = [t for t in track if t[5] == 'waypoints']
print('phases seen:', sorted({t[5] for t in track}), ' end phase', n.phase, ' end time %.0f s' % T[0])
print('errors:', [e[2][:120] for e in errs] or 'none')
print('warnings:', [l[2][:140] for l in lines if l[1] == 'W'] or 'none')
wp = n.wp or []
body = [tuple(round(v, 3) for v in b) for b in n.body]
# route not rotated: the waypoints the node used vs the config
used = [(round(w[0], 3), round(w[1], 3)) for w in (n.wp or [])]
print('route used (first 5):', used[:5] if used else '(reset after landing)')
# window leg: index 4 = leg 3 -> 4
win = [t for t in track if t[5] == 'waypoints' and t[6] == 4]
if win:
    ys = [t for t in win if abs(t[1] - 0.3) < 0.02]
    print('window leg: yaw range %.1f .. %.1f deg' % (min(math.degrees(t[4]) for t in win if t[1] < 0.8), max(math.degrees(t[4]) for t in win if t[1] < 0.8)),
          '| y at x=0.3: %s' % (sorted({round(t[2], 3) for t in ys})[:3]))
far = max(math.dist(t[1:3], (0, 0)) for t in track)
print('min/max x %.2f %.2f  min y %.2f  max z %.2f' % (min(t[1] for t in track), max(t[1] for t in track), min(t[2] for t in track), max(t[3] for t in track)))
print('last position (%.2f, %.2f, %.2f)' % tuple(track[-1][1:4]))
print('step phases flown:', ' -> '.join(dict.fromkeys(t[7] for t in track if t[7])))
over = [t for t in track if WALL_X[0] - 0.29 < t[1] < WALL_X[1] + 0.29 and -1.2 < t[2] < 0.2]
if over:
    print('over the wall (props within reach of the slab): true height min %.2f max %.2f m (top %.1f), z_corr max %.2f' % (
        min(t[3] for t in over), max(t[3] for t in over), WALL_TOP, max(t[8] for t in over)))
print('true height max over the whole flight %.2f m (flight_interface ceiling 3.0)' % max(t[3] for t in track))
pads = [t for t in track if t[7] in ('pad1', 'pad2') and t[3] < 0.6]
if pads: print('pad hovers: lowest %.2f m' % min(t[3] for t in pads))
for name, lim in (('window 1 (x 0.3)', None),):
    c = [t for t in track if abs(t[1] - 0.3) < 0.01 and t[2] < -4.4]
    print(name, 'crossings y,z:', sorted({(round(t[2], 2), round(t[3], 2)) for t in c}))
c = [t for t in track if -3.9 < t[1] < -3.7 and abs(t[2] + 2.72) < 0.02]
print('window 2 (y -2.72) crossings x,z:', sorted({(round(t[1], 2), round(t[3], 2)) for t in c}))
