"""Rebuild the sim's course walls from a hand-carried walk: the OAK's depth + cuVSLAM odometry in a bag.

    python3 sim/scan/walls_from_walk.py --bag drone-logs/20261107-194115_0.mcap --code <checkout of main> \
        [--t-end 186] [--mesh-out sim/models/course_v3/meshes/walk_walls.obj]

The walk starts on the launch mark facing the course's +Y (the stack zeroes VIO there), so odometry
(x forward, y left) maps to the course (CAD) frame as X = LAUNCH_X - y, Y = LAUNCH_Y + x. Depth frames are
integrated into dronehacks_nav.mapping.VoxelMap (5 cm, log-odds: anything that later moves out of the way,
such as a teammate walking ahead, is carved free again); occupied voxels inside the nets, above the floor,
become boxes, as fit_route_to_scan.py does for the point scan. --t-end drops the walk after VIO broke
(20261107-194115: at 186 s, climbing the tall wall). Needs numpy, cv2 and a main checkout (--code) for
refnav.depth and dronehacks_nav.mapping. The bag must be uncompressed MCAP (rosbag2's default).
"""
import argparse
import struct
import sys
from pathlib import Path

import numpy as np

LAUNCH = np.array([-2.5146, 0.9144])            # launch mark, CAD frame; takeoff faces +Y
NET = 3.048
R_BODY_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # base_link <- optical (z fwd, x right, y down)
T_BODY_OPT = np.array([0.1053, 0.0, 0.0304])                        # /tf_static on the drone, 2026-11-07


def mcap_messages(path, topics):
    """(topic, log_time_ns, cdr) from an uncompressed MCAP, streamed."""
    def s(b, o):
        n = struct.unpack_from('<I', b, o)[0]
        return b[o + 4:o + 4 + n].decode(errors='replace'), o + 4 + n
    chans = {}
    with open(path, 'rb') as f:
        assert f.read(8) == b'\x89MCAP0\r\n', 'not an MCAP file'
        while True:
            h = f.read(9)
            if len(h) < 9:
                return
            op, n = h[0], struct.unpack_from('<Q', h, 1)[0]
            if op != 6:
                f.seek(n, 1)
                continue
            body = f.read(n)
            if len(body) < n:
                return                                  # a bag cut off mid-write: keep what is there
            c, o = s(body, 28)
            if c:
                raise SystemExit(f'compressed chunks ({c}) are not supported')
            m = struct.unpack_from('<Q', body, o)[0]
            rec, q = body[o + 8:o + 8 + m], 0
            while q + 9 <= len(rec):
                op2, n2 = rec[q], struct.unpack_from('<Q', rec, q + 1)[0]
                b = rec[q + 9:q + 9 + n2]
                if op2 == 4:
                    cid = struct.unpack_from('<H', b, 0)[0]
                    chans[cid] = s(b, 4)[0]
                elif op2 == 5:
                    cid, _, lt, _ = struct.unpack_from('<HIQQ', b, 0)
                    if chans.get(cid) in topics:
                        yield chans[cid], lt, b[22:]
                q += 9 + n2


class Cdr:
    def __init__(self, b):
        self.b, self.o = b[4:], 0

    def _al(self, n):
        self.o = (self.o + n - 1) // n * n

    def u32(self):
        self._al(4); v = struct.unpack_from('<I', self.b, self.o)[0]; self.o += 4; return v

    def i32(self):
        self._al(4); v = struct.unpack_from('<i', self.b, self.o)[0]; self.o += 4; return v

    def u8(self):
        v = self.b[self.o]; self.o += 1; return v

    def f64(self, n):
        self._al(8); v = struct.unpack_from(f'<{n}d', self.b, self.o); self.o += 8 * n; return v

    def string(self):
        n = self.u32(); v = self.b[self.o:self.o + n - 1].decode(errors='replace'); self.o += n; return v

    def stamp(self):
        sec, ns = self.i32(), self.u32(); self.string(); return sec + ns * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--code', required=True, help='a checkout of main (refnav, dronehacks_nav)')
    ap.add_argument('--t-end', type=float, default=None, help='seconds into the bag to stop at')
    ap.add_argument('--every', type=int, default=3, help='integrate every n-th depth frame')
    ap.add_argument('--mesh-out', default=str(Path(__file__).resolve().parent.parent / 'models' / 'course_v3' / 'meshes' / 'walk_walls.obj'))
    a = ap.parse_args()
    sys.path.insert(0, a.code)
    from refnav import depth as D
    from dronehacks_nav import mapping as M

    odo, t0 = [], None
    for _, lt, b in mcap_messages(a.bag, {'/visual_slam/tracking/odometry'}):
        t0 = t0 or lt
        if a.t_end and (lt - t0) / 1e9 > a.t_end:
            break
        r = Cdr(b); st = r.stamp(); r.string(); p = r.f64(3); q = r.f64(4)
        odo.append((st, *p, *q))
    odo = np.array(odo)
    print(f'1. odometry: {len(odo)} poses, {odo[-1, 0] - odo[0, 0]:.0f} s')

    mp = M.VoxelMap(voxel_m=0.05, size_xy_m=14.0, z_min_m=-0.5, z_max_m=3.0)
    K, n, used = None, 0, 0
    for topic, lt, b in mcap_messages(a.bag, {'/oak/stereo/image_raw', '/oak/stereo/camera_info'}):
        if a.t_end and (lt - t0) / 1e9 > a.t_end:
            break
        r = Cdr(b); st = r.stamp()
        if topic.endswith('camera_info'):
            r.u32(); r.u32(); r.string(); r.f64(r.u32()); k = r.f64(9); K = (k[0], k[4], k[2], k[5])
            continue
        n += 1
        if K is None or n % a.every:
            continue
        h, w, enc = r.u32(), r.u32(), r.string(); r.u8(); step = r.u32(); ln = r.u32()
        j = int(np.argmin(np.abs(odo[:, 0] - st)))
        if abs(odo[j, 0] - st) > 0.05:
            continue
        x, y, z, qx, qy, qz, qw = odo[j, 1:]
        R = D.quat_to_mat(qw, qx, qy, qz)
        d = D.decode_depth(enc, h, w, step, bytes(r.b[r.o:r.o + ln]))
        dirs, ranges = M.camera_rays(d, K, 4)
        mp.integrate(np.array([x, y, z]) + R @ T_BODY_OPT, dirs @ (R @ R_BODY_OPT).T, ranges, 4.0)
        used += 1
    print(f'2. depth: {used} of {n} frames integrated')

    P = mp.occupied()                                     # voxel centres, odometry frame
    C = np.stack([LAUNCH[0] - P[:, 1], LAUNCH[1] + P[:, 0], P[:, 2]], 1)
    keep = (np.abs(C[:, :2]) < NET - 0.07).all(1) & (C[:, 2] > 0.06) & (C[:, 2] < 2.0)
    write_mesh(C[keep], a.mesh_out)


def write_mesh(C, out, V=0.05):
    """Occupied voxels (course frame) merged into boxes: z runs per column, then runs along X."""
    k = np.unique(np.floor(C / V).astype(int), axis=0)
    cols = {}
    for ix, iy, iz in k:
        cols.setdefault((ix, iy), []).append(iz)
    segs = []
    for (ix, iy), zs in cols.items():
        zs = sorted(zs)
        s = e = zs[0]
        for z in zs[1:]:
            if z <= e + 2:
                e = z
            else:
                segs.append((iy, s, e, ix)); s = e = z
        segs.append((iy, s, e, ix))
    segs.sort()
    boxes, cur = [], None
    for iy, s, e, ix in segs:
        if cur and cur[:3] == [iy, s, e] and cur[4] == ix - 1:
            cur[4] = ix
        else:
            if cur:
                boxes.append(cur)
            cur = [iy, s, e, ix, ix]
    if cur:
        boxes.append(cur)
    quads = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
    # Gazebo's physics (dartsim + ODE) drops a mesh without normals and then aborts on the empty shape:
    # one normal per box face, in the same order as quads (-x, +x, -y, +y, -z, +z)
    normals = ((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1))
    with open(out, 'w') as f:
        f.write('# Course walls from a hand-carried walk (sim/scan/walls_from_walk.py): 5 cm voxels merged into boxes\n')
        for nv in normals:
            f.write('vn %d %d %d\n' % nv)
        for b, (iy, s, e, x0, x1) in enumerate(boxes):
            lo = np.array([x0, iy, s]) * V
            hi = np.array([x1 + 1, iy + 1, e + 1]) * V
            for c in range(8):
                f.write('v %.3f %.3f %.3f\n' % tuple(hi[j] if c >> (2 - j) & 1 else lo[j] for j in range(3)))
            for k, q in enumerate(quads):
                i = [8 * b + v + 1 for v in q]
                n = k + 1
                f.write(f'f {i[0]}//{n} {i[1]}//{n} {i[2]}//{n}\nf {i[0]}//{n} {i[2]}//{n} {i[3]}//{n}\n')
    print(f'3. walls: {len(C)} voxels, {len(boxes)} boxes -> {out}')


if __name__ == '__main__':
    main()
