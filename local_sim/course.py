"""Real course geometry, extracted from dronehacks-course-v3.obj (see diagrams/route-plan.html
for the same data rendered). Course frame: metres, floor centered at origin, +x = east, +y = north.
NED (n, e) used by refnav.core/px4_msgs maps n=course_y, e=course_x (the drone launches facing
+y, i.e. "forward" = north = +n, confirmed against the real launch placement).
"""
import math

WALL_H = 1.727       # 68 in, every wall/divider/barrier/chicane piece
BOULDER_H = 0.533
BOULDER_R = 0.2285   # 18 in diameter
HALF = 3.048         # net half-extent (20 x 20 ft)

WALL_BOXES = [  # (name, xmin, xmax, ymin, ymax) in course frame
    ("A1 approach wall", -1.026, -0.924, -0.6225, 0.4065),
    ("Barrier 1", -3.0485, -1.4355, 0.406, 0.508),
    ("Barrier 2", -1.4355, 0.1775, 0.406, 0.508),
    ("Barrier 3", 0.1775, 1.5745, 0.406, 0.508),
    ("C1 chicane wall", 0.9255, 1.5725, -1.146, 0.404),
    ("C2a chicane wall (window)", -0.7945, -0.1475, -3.047, -1.497),
    ("C2b chicane wall", -0.2425, 0.2045, -1.5315, -0.5305),
    ("M1 maze divider", -1.981, -1.879, 0.5075, 2.1205),
    ("M2 maze divider", -0.864, -0.762, 1.4355, 3.0485),
    ("M3 maze divider", 0.254, 0.356, 0.5075, 2.1205),
]
BOULDERS = [  # (cx, cy, r)
    (2.012, -2.469, BOULDER_R),
    (2.530, -1.554, BOULDER_R),
    (0.792, -2.347, BOULDER_R),
    (-2.560, -2.499, BOULDER_R),
    (-1.341, -2.377, BOULDER_R),
]
LAUNCH_XY = (-2.515, 0.914)
LANDING_XY = (-2.042, -0.792)


def course_to_ne(x, y):
    return (y, x)  # (n, e)


def ne_to_course(n, e):
    return (e, n)  # (x, y)


def segments():
    """Every obstacle boundary as (n0, e0, n1, e1, height) line segments in NED horizontal
    coordinates, for the ray-cast depth renderer. Boulders approximated as 12-gons."""
    segs = []
    for _, xmin, xmax, ymin, ymax in WALL_BOXES:
        for (xa, ya), (xb, yb) in (
            ((xmin, ymin), (xmin, ymax)), ((xmax, ymin), (xmax, ymax)),
            ((xmin, ymin), (xmax, ymin)), ((xmin, ymax), (xmax, ymax)),
        ):
            n0, e0 = course_to_ne(xa, ya)
            n1, e1 = course_to_ne(xb, yb)
            segs.append((n0, e0, n1, e1, WALL_H))
    for cx, cy, r in BOULDERS:
        ngon = 12
        pts = [course_to_ne(cx + r * math.cos(2 * math.pi * k / ngon), cy + r * math.sin(2 * math.pi * k / ngon))
               for k in range(ngon)]
        for k in range(ngon):
            n0, e0 = pts[k]
            n1, e1 = pts[(k + 1) % ngon]
            segs.append((n0, e0, n1, e1, BOULDER_H))
    return segs


def nearest_obstacle_dist(x, y):
    """Ground-truth clearance (m) from a course-frame point to the nearest wall/boulder surface,
    ignoring height (2D). Independent of whatever the depth sensor reported -- for checking
    whether a planned/flown path actually keeps clear, not what the mission believed."""
    best = math.inf
    for _, xmin, xmax, ymin, ymax in WALL_BOXES:
        dx = max(xmin - x, 0.0, x - xmax)
        dy = max(ymin - y, 0.0, y - ymax)
        best = min(best, math.hypot(dx, dy))
    for cx, cy, r in BOULDERS:
        best = min(best, math.hypot(x - cx, y - cy) - r)
    return best


def outside_net(x, y):
    return abs(x) > HALF or abs(y) > HALF


def true_clearance(x, y, z_bottom=0.0):
    """Ground-truth horizontal clearance (m) from the drone centre to anything it could hit at its
    height: walls (68 in), boulders only if the gear is below their top, and the net walls."""
    best = HALF - max(abs(x), abs(y))
    if z_bottom < WALL_H:
        for _, xmin, xmax, ymin, ymax in WALL_BOXES:
            dx = max(xmin - x, 0.0, x - xmax)
            dy = max(ymin - y, 0.0, y - ymax)
            best = min(best, math.hypot(dx, dy))
    if z_bottom < BOULDER_H:
        for cx, cy, r in BOULDERS:
            best = min(best, math.hypot(x - cx, y - cy) - r)
    return best


def plan_map_route_frame(launch_xy=LAUNCH_XY, with_boulders=False):
    """The course as a refnav `plan_map`: (forward, left) metres from the launch mark, launch facing
    +y (forward = +y, left = -x). Walls only unless with_boulders (boulders are below flight height).
    NOTE: inherits this file's known geometry errors (C1/C2 are modelled as boxes, no tunnel)."""
    lx, ly = launch_xy
    f = lambda y: y - ly          # noqa: E731
    l = lambda x: -(x - lx)       # noqa: E731
    rects = [[f(ymin), l(xmax), f(ymax), l(xmin)] for _, xmin, xmax, ymin, ymax in WALL_BOXES]
    circles = [[f(cy), l(cx), r] for cx, cy, r in BOULDERS] if with_boulders else []
    return {"bounds": [f(-HALF), f(HALF), l(HALF), l(-HALF)], "rects": rects, "circles": circles}
