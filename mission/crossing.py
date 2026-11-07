"""Conservative spherical-envelope window traversal checks on a sampled path.

Segments between samples are interpreted as straight lines. This is a geometry
report for the simulator, not an official checkpoint judge or a proof about an
unobserved path between telemetry samples.
"""
import math


def _finite(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be finite') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _vector(values, name):
    try:
        result = tuple(_finite(value, name) for value in values)
    except TypeError as exc:
        raise ValueError(f'{name} must have three finite coordinates') from exc
    if len(result) != 3:
        raise ValueError(f'{name} must have three finite coordinates')
    return result


def _dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def _unit(vector, name):
    length = math.sqrt(_dot(vector, vector))
    if length < 1e-12:
        raise ValueError(f'{name} must have nonzero length')
    return tuple(value/length for value in vector)


def evaluate_window_crossing(positions, window, radius):
    """Report full entry-to-exit crossings of a radius-eroded rectangular slot.

    Metadata supplies ``center``, ``normal_toward_entry``, ``horizontal_axis``,
    ``width``, ``height`` and ``thickness`` in metres. Axes must be perpendicular;
    their cross product defines the vertical aperture axis. A valid traversal
    starts strictly beyond ``thickness/2 + radius`` on the entry side and ends
    strictly beyond that distance on the exit side. While the sphere overlaps
    the wall slab, its projected centre must stay at least ``radius`` from each
    aperture edge. This is conservative near the outer faces of the slab.

    Every segment is clipped to the expanded slab. Testing its clipped endpoints
    is sufficient: signed horizontal/vertical coordinates vary linearly and the
    worst absolute coordinate occurs at an endpoint. Thus sparse samples cannot
    skip an intersected rim. The function cannot validate a non-linear or
    physically discontinuous path not present in its input.

    Separate crossings of the infinite plane elsewhere on the course are not
    window passages. The reported minimum belongs to valid forward passages,
    or the best complete/partial forward attempt when none succeeds.
    """
    center = _vector(window['center'], 'center')
    normal = _unit(_vector(window['normal_toward_entry'], 'normal_toward_entry'), 'normal_toward_entry')
    horizontal = _unit(_vector(window['horizontal_axis'], 'horizontal_axis'), 'horizontal_axis')
    if abs(_dot(normal, horizontal)) > 1e-6:
        raise ValueError('window normal and horizontal axis must be perpendicular')
    vertical = (normal[1]*horizontal[2]-normal[2]*horizontal[1],
                normal[2]*horizontal[0]-normal[0]*horizontal[2],
                normal[0]*horizontal[1]-normal[1]*horizontal[0])
    width, height, thickness = (_finite(window[name], name)
                                for name in ('width', 'height', 'thickness'))
    radius = _finite(radius, 'radius')
    if width <= 0 or height <= 0 or thickness < 0 or radius < 0:
        raise ValueError('width/height must be positive; thickness/radius nonnegative')
    limit = thickness/2 + radius
    points = []
    for position in positions:
        p = _vector(position, 'position')
        delta = tuple(a-b for a, b in zip(p, center))
        points.append((_dot(delta, normal), _dot(delta, horizontal), _dot(delta, vertical)))

    def side(point):
        return 1 if point[0] > limit else -1 if point[0] < -limit else 0

    origin = side(points[0]) if points else 0
    margin, overlap = math.inf, False
    forward, reverse, partial = [], [], []
    for a, b in zip(points, points[1:]):
        delta_normal = b[0]-a[0]
        if abs(delta_normal) < 1e-15:
            clipped = (0.0, 1.0) if abs(a[0]) <= limit else None
        else:
            t0, t1 = sorted(((-limit-a[0])/delta_normal, (limit-a[0])/delta_normal))
            lower, upper = max(0.0, t0), min(1.0, t1)
            clipped = (lower, upper) if lower <= upper else None
        if clipped is not None and origin:
            overlap = True
            for t in clipped:
                u, v = (a[i]+t*(b[i]-a[i]) for i in (1, 2))
                margin = min(margin, width/2-radius-abs(u), height/2-radius-abs(v))
        end_side = side(b)
        if end_side:
            if overlap:
                if origin == 1:
                    (forward if end_side == -1 else partial).append(margin)
                elif origin == -1 and end_side == 1:
                    reverse.append(margin)
            origin, margin, overlap = end_side, math.inf, False
    if origin == 1 and overlap:
        partial.append(margin)

    valid = [value for value in forward if value >= 0]
    valid_reverse = [value for value in reverse if value >= 0]
    if valid:
        minimum, direction = min(valid), 'entry_to_exit'
    elif forward or partial:
        minimum = max(forward+partial)
        direction = 'entry_to_exit_blocked' if forward else 'incomplete'
    elif valid_reverse:
        minimum, direction = min(valid_reverse), 'exit_to_entry'
    else:
        minimum, direction = None, 'incomplete'
    return {
        'full_envelope_crossed': bool(valid),
        'min_aperture_clearance_m': minimum,
        'direction': direction,
        'valid_forward_crossings': len(valid),
        'forward_attempts': len(forward),
        'valid_reverse_crossings': len(valid_reverse),
        'expanded_half_thickness_m': limit,
        'path_assumption': 'Linear interpolation between supplied positions; conservative full-radius aperture erosion.',
    }
