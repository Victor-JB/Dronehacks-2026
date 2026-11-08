"""Render a presentation video from a run_final_course.py recording.

Four panels: chase camera (3-D, ground truth course), the policy's own map (top-down: what it has seen,
frontiers left to explore, planned path), the OAK depth image it receives, and the forward camera view.

  python tools/sim/render_video.py run.pkl out.mp4 [--every 2] [--fps 20]
"""
import argparse
import math
import os
import pickle
import sys

import cv2
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path[:0] = [os.path.join(ROOT, "ros2_ws", "src", "dh_core"), ROOT]
from dh_core.nav.sim_mesh_world import VoxelWorld      # noqa: E402

W, H = 1280, 760
PW, PH = 640, 360
HDR = 40
FONT = cv2.FONT_HERSHEY_SIMPLEX
STATE_TEXT = {"SCAN": "360 deg scan: building the map", "EXPLORE": "exploring the nearest frontier",
              "INSPECT": "checking an opening (window / tunnel)", "TRAVERSE": "passing a verified opening",
              "TO_TARGET": "blue pad seen: approaching", "OVER_TARGET": "over the pad: done",
              "LOOK": "looking around", "HOVER": "hold"}


def look_at(eye, target):
    f = np.asarray(target, float) - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0])
    r /= np.linalg.norm(r)
    dn = np.cross(f, r)
    return np.stack([r, dn, f], axis=1)          # columns: optical x (right), y (down), z (forward)


def project(P, eye, R_wc, K):
    fx, fy, cx, cy = K
    c = (np.atleast_2d(P) - eye) @ R_wc
    ok = c[:, 2] > 0.05
    u = fx * c[:, 0] / np.where(ok, c[:, 2], 1) + cx
    v = fy * c[:, 1] / np.where(ok, c[:, 2], 1) + cy
    return np.stack([u, v], 1), ok


def label(img, text, org=(8, 22), scale=0.55, color=(255, 255, 255)):
    (tw, th), bl = cv2.getTextSize(text, FONT, scale, 1)
    x, y = org
    sub = img[max(y - th - 4, 0):y + bl + 2, max(x - 4, 0):x + tw + 4]
    sub[:] = (sub * 0.35).astype(np.uint8)                 # dark box behind the text
    cv2.putText(img, text, org, FONT, scale, color, 1, cv2.LINE_AA)


class TopDown:
    def __init__(self, world, rec):
        self.s = PH / 6.4                         # px per metre
        self.o = np.array([-3.2, -3.2])
        k = int((0.75 - world.lo[2]) / world.pitch)
        sl = world.occ[:, :, k].T[::-1]           # rows = y descending
        truth = cv2.resize(sl.astype(np.uint8) * 255, (int(world.shape[0] * world.pitch * self.s),) * 2,
                           interpolation=cv2.INTER_NEAREST)
        self.base = np.full((PH, PH, 3), 25, np.uint8)
        x0, y0 = self.px((world.lo[0], world.lo[1] + world.shape[1] * world.pitch))
        h, w = truth.shape
        sub = self.base[y0:y0 + h, x0:x0 + w]
        sub[truth[:sub.shape[0], :sub.shape[1]] > 0] = (90, 90, 90)
        self.rec = rec
        self.map_lo = np.array(rec["map_lo"])
        self.vox = rec["map_voxel"]

    def px(self, p):
        return int(round((p[0] - self.o[0]) * self.s)), int(round(PH - (p[1] - self.o[1]) * self.s))

    def draw(self, i, frames, cls):
        img = self.base.copy()
        if cls is not None:
            ny, nx = cls.shape
            col = np.zeros((ny, nx, 3), np.uint8)
            col[cls == -1] = (40, 40, 40)
            col[cls == 0] = (205, 225, 205)
            col[cls == 1] = (40, 40, 220)
            col = col[::-1]
            sz = int(round(nx * self.vox * self.s))
            col = cv2.resize(col, (sz, sz), interpolation=cv2.INTER_NEAREST)
            x0, y0 = self.px((self.map_lo[0], self.map_lo[1] + ny * self.vox))
            reg = img[max(y0, 0):y0 + sz, max(x0, 0):x0 + sz]
            c2 = col[max(-y0, 0):max(-y0, 0) + reg.shape[0], max(-x0, 0):max(-x0, 0) + reg.shape[1]]
            known = c2.sum(axis=2) != 120
            reg[known] = (0.75 * c2[known] + 0.25 * reg[known]).astype(np.uint8)
            truth_px = self.base[max(y0, 0):y0 + sz, max(x0, 0):x0 + sz]
            wall = truth_px[..., 0] == 90
            reg[wall & ~known] = (90, 90, 90)
        pad = self.rec["pad"]
        a, b = self.px((pad[0] - pad[2] / 2, pad[1] + pad[2] / 2)), self.px((pad[0] + pad[2] / 2, pad[1] - pad[2] / 2))
        cv2.rectangle(img, a, b, (220, 120, 30), 2)
        g = self.rec["geofence"]
        cv2.rectangle(img, self.px((g[0], g[3])), self.px((g[1], g[2])), (0, 200, 255), 1)
        tr = np.array([f["pos"][:2] for f in frames[:i + 1:3]])
        if len(tr) > 1:
            cv2.polylines(img, [np.array([self.px(p) for p in tr], np.int32)], False, (60, 220, 60), 2, cv2.LINE_AA)
        f = frames[i]
        if f.get("path") is not None:
            cv2.polylines(img, [np.array([self.px(p[:2]) for p in f["path"]], np.int32)], False, (255, 230, 0), 1, cv2.LINE_AA)
        if f.get("goal") is not None:
            cv2.drawMarker(img, self.px(f["goal"][:2]), (0, 230, 255), cv2.MARKER_STAR, 14, 2)
        p = f["pos"]
        c = self.px(p)
        tip = self.px((p[0] + 0.35 * math.cos(f["yaw"]), p[1] + 0.35 * math.sin(f["yaw"])))
        cv2.circle(img, c, int(0.24 * self.s), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.arrowedLine(img, c, tip, (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.4)
        # camera field of view
        for s in (-1, 1):
            e = self.px((p[0] + 2.0 * math.cos(f["yaw"] + s * 0.69), p[1] + 2.0 * math.sin(f["yaw"] + s * 0.69)))
            cv2.line(img, c, e, (200, 200, 200), 1, cv2.LINE_AA)
        out = np.zeros((PH, PW, 3), np.uint8)
        out[:, :PH] = img
        lg = [((205, 225, 205), "seen free"), ((40, 40, 220), "seen obstacle"), ((40, 40, 40), "unknown"),
              ((90, 90, 90), "real walls (not given to policy)"), ((60, 220, 60), "flown path"),
              ((255, 230, 0), "planned path"), ((0, 230, 255), "current goal"), ((220, 120, 30), "landing pad"),
              ((0, 200, 255), "geofence")]
        for k, (cc, t) in enumerate(lg):
            y = 60 + 24 * k
            cv2.rectangle(out, (PH + 12, y - 11), (PH + 26, y + 3), cc, -1)
            label(out, t, (PH + 34, y + 2), 0.45)
        return out


def chase(world, f, K):
    p, yaw = f["pos"], f["yaw"]
    back = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    eye0 = p - 1.6 * back + np.array([0, 0, 1.4])
    eye0[:2] = np.clip(eye0[:2], -2.95, 2.95)
    eye = eye0
    for a in np.linspace(1.0, 0.25, 16):                   # pull the camera in until it sees the drone
        e = p + a * (eye0 - p)
        seg = [e + s_ * (p - e) for s_ in np.linspace(0, 0.85, 12)]
        if all(world.distance(q) > 0.06 for q in seg):
            eye = e
            break
    tgt = p + 0.9 * back
    R = look_at(eye, tgt)
    _, rgb = world.render(eye, R, K, PW // 2, PH // 2, max_range=10.0)
    img = cv2.resize(rgb[..., ::-1], (PW, PH), interpolation=cv2.INTER_LINEAR)
    K2 = tuple(v * 2 for v in K)
    if f.get("path") is not None:
        uv, ok = project(f["path"], eye, R, K2)
        pts = uv[ok].astype(np.int32)
        if len(pts) > 1:
            cv2.polylines(img, [pts], False, (255, 230, 0), 2, cv2.LINE_AA)
    # the drone: body + 4 rotors
    c, s = math.cos(yaw), math.sin(yaw)
    arms = [p + 0.17 * np.array([c * dx - s * dy, s * dx + c * dy, 0]) for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
    uv, ok = project(np.vstack([p] + arms), eye, R, K2)
    if ok.all():
        q = uv.astype(int)
        for k in range(1, 5):
            cv2.line(img, tuple(q[0]), tuple(q[k]), (30, 30, 30), 3, cv2.LINE_AA)
            cv2.circle(img, tuple(q[k]), 9, (0, 0, 230) if k <= 2 else (40, 40, 40), 2, cv2.LINE_AA)
        uvn, _ = project(p + 0.4 * np.array([c, s, 0]), eye, R, K2)
        cv2.arrowedLine(img, tuple(q[0]), tuple(uvn[0].astype(int)), (0, 0, 255), 2, cv2.LINE_AA)
    return img


def depth_panel(d):
    dd = np.nan_to_num(d, nan=0.0)
    norm = np.clip(dd / 4.0, 0, 1)
    col = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    col[dd <= 0] = (0, 0, 0)
    img = cv2.resize(col, (PH * 16 // 10, PH), interpolation=cv2.INTER_NEAREST)
    out = np.zeros((PH, PW, 3), np.uint8)
    out[:, :img.shape[1]] = img
    bar = cv2.applyColorMap(np.linspace(255, 0, PH - 80).astype(np.uint8)[:, None].repeat(16, 1), cv2.COLORMAP_TURBO)
    out[50:PH - 30, PW - 40:PW - 24] = bar
    label(out, "0 m", (PW - 70, 60), 0.4)
    label(out, "4 m", (PW - 70, PH - 32), 0.4)
    return out


def camera_panel(rgb, f):
    img = cv2.resize(rgb[..., ::-1], (PH * 16 // 10, PH), interpolation=cv2.INTER_NEAREST)
    blue = (rgb[..., 2] > 150) & (rgb[..., 0] < 80)
    if blue.sum() > 10:
        ys, xs = np.nonzero(blue)
        sx = img.shape[1] / rgb.shape[1]
        cv2.rectangle(img, (int(xs.min() * sx), int(ys.min() * sx)), (int((xs.max() + 1) * sx), int((ys.max() + 1) * sx)),
                      (0, 255, 255), 2)
        label(img, "blue pad detected", (int(xs.min() * sx), max(int(ys.min() * sx) - 6, 14)), 0.5, (0, 255, 255))
    out = np.zeros((PH, PW, 3), np.uint8)
    out[:, :img.shape[1]] = img
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("out")
    ap.add_argument("--every", type=int, default=2)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--title", default="DroneHacks 2026 - CAD-free autonomous navigation (offline sim, FINAL Course v3)")
    a = ap.parse_args()
    rec = pickle.load(open(a.rec, "rb"))
    frames = rec["frames"]
    world = VoxelWorld.load(os.path.join(ROOT, "tools", "sim", "final_course_v3.npz"))
    K = (0.5 * PW / 2 / math.tan(math.radians(40)), 0.5 * PW / 2 / math.tan(math.radians(40)), PW / 4, PH / 4)
    td = TopDown(world, rec)
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))
    cls, speedup = None, a.every * 0.1 * a.fps
    n = len(frames)
    for i in range(0, n, a.every):
        f = frames[i]
        for j in range(i, max(i - a.every * 5, -1), -1):
            if frames[j].get("cls") is not None:
                cls = frames[j]["cls"]
                break
        canvas = np.full((H, W, 3), 15, np.uint8)
        p1 = chase(world, f, K)
        label(p1, "Chase camera (ground truth course)")
        p2 = td.draw(i, frames, cls)
        label(p2, "Policy's own map at flight height (built only from depth)")
        p3 = depth_panel(f["depth"])
        label(p3, "OAK-D stereo depth (what the policy sees)")
        p4 = camera_panel(f["rgb"], f)
        label(p4, "Forward camera, simulated colours (blue-pad detector)")
        canvas[HDR:HDR + PH, :PW] = p1
        canvas[HDR:HDR + PH, PW:] = p2
        canvas[HDR + PH:HDR + 2 * PH, :PW] = p3
        canvas[HDR + PH:HDR + 2 * PH, PW:] = p4
        cv2.line(canvas, (PW, HDR), (PW, H), (80, 80, 80), 1)
        cv2.line(canvas, (0, HDR + PH), (W, HDR + PH), (80, 80, 80), 1)
        t = i * 0.1
        sp = float(np.linalg.norm(f["vel"][:2]))
        label(canvas, a.title, (10, 16), 0.5)
        label(canvas, "t = %5.1f s   state: %-12s %s   speed %.2f m/s   alt %.2f m   (video %gx)" %
              (t, f["state"], STATE_TEXT.get(f["state"], ""), sp, f["pos"][2], speedup), (10, 34), 0.5, (0, 255, 255))
        vw.write(canvas)
        if i % 400 == 0:
            print("frame", i, "/", n, flush=True)
    s = rec["summary"]
    end = canvas.copy()
    ov = end.copy()
    cv2.rectangle(ov, (200, 220), (1080, 560), (0, 0, 0), -1)
    end = cv2.addWeighted(ov, 0.75, end, 0.25, 0)
    lines = ["RESULT: %s" % ("reached the pad, holding over it (descent left to the pilot)" if s["reached"] else "did not reach the pad (%s)" % s["state"]),
             "time %.0f s, path %.1f m, collisions %d" % (s["sim_t"], s["path"], s["collisions"]),
             "closest approach to any surface %.2f m (body radius 0.24 m)" % s["min_clear"],
             "pad error %s m, altitude %.2f-%.2f m (never climbed over walls)" % (s["pad_err"], s["z"][0], s["z"][1]),
             "Offline simulator, not PX4 SITL: idealised dynamics, rendered depth.",
             "Same policy + safety code as the nav_policy ROS node."]
    for k, L in enumerate(lines):
        label(end, L, (230, 270 + 45 * k), 0.7 if k < 4 else 0.55, (255, 255, 255) if k < 4 else (180, 180, 180))
    for _ in range(a.fps * 5):
        vw.write(end)
    vw.release()
    print("wrote", a.out)


if __name__ == "__main__":
    main()
