"""box_route_v2.mission_plan - the mission as a flat list of steps, built from route.yaml. PURE (no ROS).

    course (start -> pad 1)
      -> [pad1_hover]  hover low over pad 1, climb back, settle          (no disarm: PX4 never lands)
      -> [wall]        over the wall (3-D waypoints) to pad 2
      -> [pad2_hover]  hover low over pad 2, climb back, settle
      -> [reverse]     the wall backwards -> [pad 1 hover again] -> the course backwards -> take-off spot
      -> land at the last step (the only real landing: PX4 disarms there)

Every segment has its own `enabled`. Whatever is enabled last is where it lands. With all of them off
this is v1: fly the course, land at its last waypoint.

A step is a dict: x, y, z (metres from the take-off point: forward, left, up), plus
    slow      fly the leg INTO it at wall_speed_mps
    straight  the leg INTO it: no early turn, move only lined up within 5 deg (windows)
    hold_s    stay this long once reached before going on
    hover     a low hover over a pad (PX4 reporting landed here makes it climb at once)
    phase     'course' | 'pad1' | 'wall' | 'pad2' | 'reverse'   (the wall phase gets the wall rules)
    label     for the log and the live view
"""

PAD_MIN_M = 0.3       # flight_interface clamps pose setpoints below 0.3 m above its origin (envelope floor)
WALL_MAX_M = 2.8      # flight_interface's ceiling is 3.0 m; keep 0.2 m under it

# defaults of every optional section (a key left out of route.yaml keeps its default)
SECTIONS = {
    'pad1_hover': {'enabled': False, 'height_m': 0.35, 'hover_s': 3.0, 'restabilize_s': 3.0},
    'pad2_hover': {'enabled': False, 'height_m': 0.35, 'hover_s': 3.0, 'restabilize_s': 3.0},
    'wall': {'enabled': False, 'speed_mps': 0.25, 'ignore_px4_height': True, 'vio_height_hold': True,
             'vio_hold_guard_m': 0.25, 'on_vio_fault': 'hold', 'waypoints': []},
    'reverse': {'enabled': False, 'wall': True, 'pad1_hover': True},
}
# key -> (type, min, max) for the numbers, or the allowed values for strings
CHECKS = {
    'height_m': (float, PAD_MIN_M, 1.0), 'hover_s': (float, 0.0, 30.0), 'restabilize_s': (float, 0.0, 30.0),
    'speed_mps': (float, 0.05, 1.0), 'vio_hold_guard_m': (float, 0.05, 1.0),
    'on_vio_fault': ('hold', 'land'),
}


def parse_sections(raw):
    """Check the optional sections of route.yaml. Returns {section: settings}; raises ValueError."""
    out = {}
    for name, default in SECTIONS.items():
        got = raw.get(name, {}) or {}
        if not isinstance(got, dict):
            raise ValueError(f'{name} must be a mapping like {{enabled: true, ...}}')
        unknown = set(got) - set(default)
        if unknown:
            raise ValueError(f'{name}: unknown key(s) {sorted(unknown)} (allowed: {sorted(default)})')
        s = dict(default)
        for k, v in got.items():
            d = default[k]
            if isinstance(d, bool):
                if not isinstance(v, bool):
                    raise ValueError(f'{name}.{k} must be true or false (got {v!r})')
            elif k == 'waypoints':
                if not isinstance(v, list):
                    raise ValueError(f'{name}.waypoints must be a list of [fwd, left, up]')
                pts = []
                for i, p in enumerate(v, 1):
                    if (not isinstance(p, (list, tuple)) or len(p) != 3
                            or any(isinstance(c, bool) or not isinstance(c, (int, float)) for c in p)):
                        raise ValueError(f'{name}.waypoints[{i}] must be [fwd, left, up] numbers (got {p!r})')
                    if abs(p[0]) > 20 or abs(p[1]) > 20 or not PAD_MIN_M <= p[2] <= WALL_MAX_M:
                        raise ValueError(f'{name}.waypoints[{i}]: fwd/left within +-20 m, up {PAD_MIN_M}-{WALL_MAX_M} m (got {p!r})')
                    pts.append(tuple(float(c) for c in p))
                v = pts
            elif isinstance(CHECKS.get(k), tuple) and isinstance(CHECKS[k][0], str):
                if v not in CHECKS[k]:
                    raise ValueError(f'{name}.{k} must be one of {CHECKS[k]} (got {v!r})')
            else:
                _, lo, hi = CHECKS[k]
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi:
                    raise ValueError(f'{name}.{k} must be a number from {lo} to {hi} (got {v!r})')
                v = float(v)
            s[k] = v
        out[name] = s
    if out['wall']['enabled'] and len(out['wall']['waypoints']) < 2:
        raise ValueError('wall.enabled needs wall.waypoints (at least 2 [fwd, left, up])')
    if out['wall']['enabled'] and out['reverse']['enabled'] and not out['reverse']['wall']:
        raise ValueError('reverse.wall: false with wall.enabled would fly from pad 2 straight into the course '
                         '(through the wall): set reverse.wall true, or wall.enabled false')
    return out


def _step(x, y, z, phase, label, slow=False, straight=False, hold_s=0.0, hover=False):
    return {'x': x, 'y': y, 'z': z, 'phase': phase, 'label': label, 'slow': slow, 'straight': straight,
            'hold_s': hold_s, 'hover': hover}


def build_steps(cfg):
    """The whole mission as steps. cfg: the checked route.yaml (box_mission.load_config)."""
    h, k = cfg['height_m'], cfg['scale']
    sec = cfg.get('sections') or parse_sections({})
    # the course: step 0 is the climb at the take-off point (box_mission settles there before leg 1)
    course = [_step(0.0, 0.0, h, 'course', 'climb')]
    for i, (x, y, slow, wh, straight) in enumerate(cfg['waypoints'], 1):
        course.append(_step(k * x, k * y, h if wh is None else wh, 'course', f'course {i}', slow, straight))
    n = cfg.get('waypoints_to_fly', 0)
    if 0 < n < len(course) - 1:
        return course[:n + 1]                          # a short test: fly 1..n, land at n (as v1)
    steps = list(course)
    p1, p2, wall, rev = sec['pad1_hover'], sec['pad2_hover'], sec['wall'], sec['reverse']
    pad1 = course[-1]

    def pad_visit(at, settings, name):
        """hover low over `at`, then climb back to its cruise height and settle there"""
        return [_step(at['x'], at['y'], settings['height_m'], name, f'{name}: hover low', hold_s=settings['hover_s'], hover=True),
                _step(at['x'], at['y'], at['z'], name, f'{name}: climb back, settle', hold_s=settings['restabilize_s'])]

    more_after_pad1 = wall['enabled'] or rev['enabled']
    if p1['enabled'] and more_after_pad1:
        steps += pad_visit(pad1, p1, 'pad1')
    end = pad1
    if wall['enabled']:
        for j, (x, y, z) in enumerate(wall['waypoints'], 1):
            steps.append(_step(x, y, z, 'wall', f'wall {j}', slow=False))
        end = steps[-1]
        if p2['enabled'] and rev['enabled']:
            steps += pad_visit(end, p2, 'pad2')
    if rev['enabled']:
        if wall['enabled'] and rev['wall']:
            for j, (x, y, z) in list(enumerate(wall['waypoints'], 1))[::-1][1:]:
                steps.append(_step(x, y, z, 'wall', f'wall back {j}'))
            if rev['pad1_hover'] and p1['enabled']:
                steps += pad_visit(steps[-1], p1, 'pad1')
        # the course backwards: the leg F[i] -> F[i-1] keeps the attributes of the forward leg into F[i]
        # (its height, slow, straight), so a window is crossed the same way in both directions
        for i in range(len(course) - 1, 0, -1):
            f, prev = course[i], course[i - 1]
            steps.append(_step(prev['x'], prev['y'], f['z'], 'reverse',
                               f'back to {prev["label"]}', f['slow'], f['straight']))
    return steps


def describe(steps):
    """One line per segment for the log."""
    out, cur, n0 = [], None, 0
    for i, s in enumerate(steps):
        if s['phase'] != cur:
            if cur is not None:
                out.append(f'{cur} {n0}-{i - 1}')
            cur, n0 = s['phase'], i
    out.append(f'{cur} {n0}-{len(steps) - 1}')
    return ', '.join(out)
