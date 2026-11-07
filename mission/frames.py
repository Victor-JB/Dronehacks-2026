"""Rigid course/odometry alignment. Neither frame is PX4 NED."""
import math
from .core import vector


class CourseFrame:
    """Map CAD coordinates onto startup odometry with measured yaw alignment.

    yaw_offset is the course +X direction in odometry ENU, in radians. The
    observed startup origin corresponds to the course's configured launch point.
    Both frames must have aligned vertical axes (keep rig level at VO startup).
    """
    def __init__(self, course_start, odom_start, yaw_offset=0.0):
        self.course_start = vector(course_start)
        self.odom_start = vector(odom_start)
        if not math.isfinite(yaw_offset):
            raise ValueError('yaw_offset must be finite')
        self.yaw_offset = yaw_offset
        self.c, self.s = math.cos(yaw_offset), math.sin(yaw_offset)

    def velocity_to_odom(self, value):
        x, y, z = vector(value)
        return (self.c*x - self.s*y, self.s*x + self.c*y, z)

    def velocity_to_course(self, value):
        x, y, z = vector(value)
        return (self.c*x + self.s*y, -self.s*x + self.c*y, z)

    def position_to_odom(self, value):
        delta = tuple(a-b for a, b in zip(vector(value), self.course_start))
        return tuple(a+b for a, b in zip(self.odom_start, self.velocity_to_odom(delta)))

    def position_to_course(self, value):
        delta = tuple(a-b for a, b in zip(vector(value), self.odom_start))
        return tuple(a+b for a, b in zip(self.course_start, self.velocity_to_course(delta)))


def rotate_body_velocity(quaternion, velocity):
    """Rotate ROS FLU body velocity into odometry world using xyzw quaternion."""
    x, y, z, w = (float(v) for v in quaternion)
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(n) or n < 1e-6:
        raise ValueError('invalid orientation quaternion')
    x, y, z, w = x/n, y/n, z/n, w/n
    vx, vy, vz = vector(velocity)
    return ((1-2*(y*y+z*z))*vx + 2*(x*y-z*w)*vy + 2*(x*z+y*w)*vz,
            2*(x*y+z*w)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z-x*w)*vz,
            2*(x*z-y*w)*vx + 2*(y*z+x*w)*vy + (1-2*(x*x+y*y))*vz)
