"""Minimal stand-ins for rclpy and the message packages used by dh_autonomy.

Lets the real node classes run (without ROS) in a deterministic loop with a
simulated clock, for software-in-the-loop tests on any laptop.
"""
import sys
import types


class _Msg:
    """Generic message: attributes spring into existence as nested messages or zeros."""
    _defaults = {}

    def __init__(self, **kw):
        for k, v in self._defaults.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        val = _Msg()
        object.__setattr__(self, name, val)
        return val


def _msg_class(name, **defaults):
    return type(name, (_Msg,), {"_defaults": defaults})


def _vec3():
    return _msg_class("V3", x=0.0, y=0.0, z=0.0)()


def _quat():
    return _msg_class("Q", x=0.0, y=0.0, z=0.0, w=1.0)()


def _header():
    return _msg_class("Header", frame_id="", stamp=None)()


Pose = _msg_class("Pose", position=_vec3, orientation=_quat)
PoseStamped = _msg_class("PoseStamped", header=_header, pose=Pose)
Twist = _msg_class("Twist", linear=_vec3, angular=_vec3)
TwistStamped = _msg_class("TwistStamped", header=_header, twist=Twist)
Vector3Stamped = _msg_class("Vector3Stamped", header=_header, vector=_vec3)
PoseWithCov = _msg_class("PoseWithCovariance", pose=Pose)
Odometry = _msg_class("Odometry", header=_header, pose=PoseWithCov)
Path = _msg_class("Path", header=_header, poses=list)
Empty = _msg_class("Empty")
Float32 = _msg_class("Float32", data=0.0)
Float32MultiArray = _msg_class("Float32MultiArray", data=list)
String = _msg_class("String", data="")
LaserScan = _msg_class("LaserScan", header=_header, ranges=list)
CameraInfo = _msg_class("CameraInfo", header=_header, k=list, d=list, r=list, p=list, width=0, height=0)
Image = _msg_class("Image", header=_header, data=b"", height=0, width=0, step=0, encoding="mono8")


class ApproximateTimeSynchronizer:
    """message_filters stand-in: the test calls `ApproximateTimeSynchronizer.last.cb(left, right)`."""
    last = None

    def __init__(self, subs, queue, slop):
        ApproximateTimeSynchronizer.last = self
        self.cb = None

    def registerCallback(self, cb):
        self.cb = cb


class TfBuffer:
    """tf2_ros.Buffer stand-in; tests put transforms in TfBuffer.transforms[(target, source)]."""
    transforms = {}

    def lookup_transform(self, target, source, _t):
        if (target, source) not in self.transforms:
            raise RuntimeError("no transform %s <- %s" % (target, source))
        return self.transforms[(target, source)]


class _Trigger:
    Request = _msg_class("TriggerRequest")


class Clock:
    def __init__(self, bus):
        self.bus = bus

    def now(self):
        t = self.bus.t
        return types.SimpleNamespace(nanoseconds=int(t * 1e9), to_msg=lambda: t)


class Bus:
    """Shared pub/sub + simulated time."""
    def __init__(self):
        self.t = 0.0
        self.subs = {}
        self.timers = []
        self.published = {}

    def publish(self, topic, msg):
        self.published.setdefault(topic, []).append((self.t, msg))
        for cb in self.subs.get(topic, []):
            cb(msg)


BUS = Bus()


class _Param:
    def __init__(self, v):
        self.value = v


class _Logger:
    def __init__(self, name):
        self.name = name
        self.lines = []

    def _log(self, lvl, msg, **_):
        self.lines.append((BUS.t, lvl, msg))
        if lvl != "debug":
            print("[%6.2f] %s %s: %s" % (BUS.t, lvl.upper(), self.name, msg))

    def info(self, m, **k): self._log("info", m, **k)
    def warn(self, m, **k):
        if "throttle_duration_sec" in k and self.lines and self.lines[-1][2] == m and BUS.t - self.lines[-1][0] < k["throttle_duration_sec"]:
            return
        self._log("warn", m, **k)
    warning = warn
    def error(self, m, **k): self._log("error", m, **k)
    def debug(self, m, **k): self._log("debug", m, **k)


class Node:
    PARAM_OVERRIDES = {}

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._logger = _Logger(name)

    def declare_parameter(self, name, default):
        self._params[name] = _Param(self.PARAM_OVERRIDES.get(name, default))

    def get_parameter(self, name):
        return self._params[name]

    def create_subscription(self, _type, topic, cb, _qos):
        BUS.subs.setdefault(topic, []).append(cb)

    def create_publisher(self, _type, topic, _qos):
        return types.SimpleNamespace(publish=lambda m, t=topic: BUS.publish(t, m))

    def create_timer(self, period, cb):
        BUS.timers.append([period, 0.0, cb])

    def create_client(self, _t, _name):
        return types.SimpleNamespace(service_is_ready=lambda: False, call_async=lambda r: None)

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return Clock(BUS)

    def destroy_node(self):
        pass


def install():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    q = types.SimpleNamespace(BEST_EFFORT=0, TRANSIENT_LOCAL=0, KEEP_LAST=0)
    mod("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None, spin=lambda n: None)
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", QoSProfile=lambda **k: None, ReliabilityPolicy=q, DurabilityPolicy=q, HistoryPolicy=q)
    mod("geometry_msgs"); mod("geometry_msgs.msg", PoseStamped=PoseStamped, TwistStamped=TwistStamped,
                              Vector3Stamped=Vector3Stamped)
    mod("nav_msgs"); mod("nav_msgs.msg", Odometry=Odometry, Path=Path)
    mod("std_msgs"); mod("std_msgs.msg", Empty=Empty, Float32=Float32, Float32MultiArray=Float32MultiArray, String=String)
    mod("std_srvs"); mod("std_srvs.srv", Trigger=_Trigger)
    mod("sensor_msgs"); mod("sensor_msgs.msg", LaserScan=LaserScan, CameraInfo=CameraInfo, Image=Image)
    mod("message_filters", ApproximateTimeSynchronizer=ApproximateTimeSynchronizer,
        Subscriber=lambda node, typ, topic: None)
    mod("tf2_ros", Buffer=TfBuffer, TransformListener=lambda buf, node: None)
    mod("rclpy.time", Time=lambda *a, **k: 0)
    # px4_msgs (release/1.16 subset used by dh_autonomy)
    VC = _msg_class("VehicleCommand", command=0, param1=0.0, param2=0.0, param7=0.0)
    VC.VEHICLE_CMD_DO_SET_MODE = 176
    VC.VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VC.VEHICLE_CMD_NAV_LAND = 21
    VS = _msg_class("VehicleStatus", nav_state=0, arming_state=1)
    VS.MESSAGE_VERSION = 1
    VS.NAVIGATION_STATE_OFFBOARD = 14
    VS.NAVIGATION_STATE_AUTO_LAND = 18
    VS.ARMING_STATE_ARMED = 2
    VS.ARMING_STATE_STANDBY = 1
    mod("px4_msgs")
    mod("px4_msgs.msg",
        OffboardControlMode=_msg_class("OffboardControlMode", position=False, velocity=False),
        TrajectorySetpoint=_msg_class("TrajectorySetpoint", position=list, velocity=list, acceleration=list,
                                      yaw=0.0, yawspeed=0.0),
        VehicleCommand=VC, VehicleStatus=VS,
        VehicleLandDetected=_msg_class("VehicleLandDetected", landed=True),
        VehicleOdometry=_msg_class("VehicleOdometry"))
