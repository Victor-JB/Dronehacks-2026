"""Point cloud of the course from the 2026-11-07 walk bag (194115): OAK depth (16UC1 mm, 320x200) placed with
VIO odometry, up to 185 s (VIO broke at 186 s, lifted over the wall). Output in the 11-07 odom frame."""
import sys, os, math, numpy as np
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(H, '..'))
import mcapr, cdr
P = os.path.join(H, '..', '20261107-194115_0.mcap')
fx, fy, cx, cy = 198.4, 198.4, 162.5, 101.9
T_BC = np.array([0.1053, 0.0, 0.0304])
def qmat(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
od = []; frames = []; t0 = None
for t, s, lt, m in mcapr.read(P, {'/visual_slam/tracking/odometry', '/oak/stereo/image_raw'}):
    if t0 is None: t0 = lt
    T = (lt - t0) / 1e9
    if T > 185.0: break
    r = cdr.R(m)
    if t[1] == 'v':
        st, _ = r.header(); r.string(); p = r.f64(3); q = r.f64(4); od.append((st, *p, *q))
    elif len(frames) == 0 or T - frames[-1][0] > 0.2:          # 5 Hz of depth is plenty
        st, _ = r.header(); h = r.u32(); w = r.u32(); r.string(); r.u8(); step = r.u32(); n = r.u32()
        img = np.frombuffer(r.b[r.o:r.o + n], '<u2').reshape(h, step // 2)[:, :w]
        frames.append((T, st, img.copy()))
od = np.array(od)
v, u = np.mgrid[2:200:3, 2:320:3]
pts = []
for T, st, img in frames:
    i = np.searchsorted(od[:, 0], st)
    if i == 0 or i >= len(od) or abs(od[i, 0] - st) > 0.05: continue
    p, q = od[i, 1:4], od[i, 4:8]
    Z = img[v, u] / 1000.0; ok = (Z > 0.25) & (Z < 3.5)
    X = (u[ok] - cx) / fx * Z[ok]; Y = (v[ok] - cy) / fy * Z[ok]; Z = Z[ok]
    B = np.c_[Z, -X, -Y] + T_BC                       # optical -> base_link
    W = B @ qmat(*q).T + p
    pts.append(np.c_[W, np.full(len(W), T)])
C = np.vstack(pts).astype(np.float32)
np.save(os.path.join(H, 'depth1107_odom.npy'), C)
print(len(frames), 'depth frames,', len(C), 'points; start yaw %.1f deg' % math.degrees(2 * math.atan2(od[0, 6], od[0, 7])))
np.save(os.path.join(H, 'walk1107_track.npy'), od[:, :4])
