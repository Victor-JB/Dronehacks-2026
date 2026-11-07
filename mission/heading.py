"""Keep the forward camera looking into the next corridor before a turn."""
import math


def route_yaw(position, target, following, phase, lookahead=1.2):
    """Return a heading from surveyed waypoints and estimated position only.

    A near boundary can hide the floor from a forward camera. Start looking
    into the next corridor while still tracking the current waypoint. Keep
    the window and tunnel aligned until their crossing waypoint is reached.
    """
    dx, dy = target[0] - position[0], target[1] - position[1]
    distance = math.hypot(dx, dy)
    if distance < .03:
        return None
    if phase == 'WINDOW_ALIGN' and following is not None and lookahead > .3:
        # Look through the opening while strafing to its entry point. Facing
        # the nearby boundary net here hides the opening and loses position.
        nx, ny = following[0] - target[0], following[1] - target[1]
        if math.hypot(nx, ny) > .03:
            return math.atan2(ny, nx)
    if following is not None and lookahead > .3 and not (
            phase.startswith('WINDOW') or phase in ('TAKEOFF', 'TUNNEL')):
        nx, ny = following[0] - target[0], following[1] - target[1]
        next_distance = math.hypot(nx, ny)
        if next_distance > .03:
            weight = max(0., min(1., (lookahead-distance)/(lookahead-.3)))
            sx = (1-weight)*dx/distance + weight*nx/next_distance
            sy = (1-weight)*dy/distance + weight*ny/next_distance
            if math.hypot(sx, sy) > .03:
                dx, dy = sx, sy
    return math.atan2(dy, dx)
