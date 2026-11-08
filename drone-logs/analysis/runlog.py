"""Route used, waypoint progress and log lines of a box_route bag."""
import sys, os, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import mcapr, cdr
for P in sys.argv[1:]:
    print('=====', os.path.basename(P)); t0 = None; body = None; last = None
    for t, s, lt, b in mcapr.read(P, {'/box_route/status', '/rosout'}):
        if t0 is None: t0 = lt
        T = (lt - t0) / 1e9
        if t == '/box_route/status':
            j = json.loads(cdr.R(b).string())
            if j['body'] != body:
                body = j['body']; print(f'{T:6.1f} body', body, 'wall', j['wall'], 'h', j['height_m'], 'v', j['speed_mps'], j['wall_speed_mps'])
            key = (j['phase'], j['index'])
            if key != last:
                last = key; print(f'{T:6.1f} phase {j["phase"]} index {j["index"]} target {j["target"]}')
        else:
            r = cdr.R(b); r.i32(); r.u32(); lvl = r.u8(); name = r.string(); msg = r.string()
            if 'box_mission' in name or 'flight_interface' in name:
                print(f'{T:6.1f} [{name}] {msg[:260]}')
