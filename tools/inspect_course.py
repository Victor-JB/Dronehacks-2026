"""Print what the course loader extracts from the official OBJ."""
import time
import _path  # noqa: F401
from _path import COURSE_OBJ
from dh_core.course import Course

t = time.time()
c = Course.from_obj(COURSE_OBJ)
print("load %.2fs" % (time.time() - t))
print("obstacles", c.obstacles)
print("markings", c.markings)
print("boundary", c.boundary, "floor", c.floor)
print("bounds", c.bounds_min, c.bounds_max, "wall_height", c.wall_height)
print("launch", c.launch, "pad", c.pad_center, c.pad_radius)
print("guideline", None if c.guideline is None else c.guideline.round(3).tolist())
for a in c.apertures:
    print("aperture", a.group, a.center.round(3), "normal", a.normal.round(3), "w", round(a.width, 3), "z", round(a.z_min, 3), round(a.z_max, 3))
t = time.time()
g = c.occupancy(0.05, 0.75, 1.05)
print("occupancy %.2fs" % (time.time() - t), g.shape, g.occ.sum())
