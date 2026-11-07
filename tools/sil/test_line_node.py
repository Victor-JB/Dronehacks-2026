"""End-to-end test of the real line_node on rendered mono images of a corridor with a floor guideline.

Scene (body frame, x forward, y left, z up): floor at z = -height, two corridor walls at y = +-half_width,
a 2-inch dark guideline on the floor through body (0, lateral) at heading `heading` relative to body x.
The mono camera is pitched down like the OAK-D on Sky Beta. Grey levels are parameters, because
the real floor/line/wall finish is not published. The model colours (floor 0.92, line dark red ~0.3
luminance) give a strong contrast; we also test weak contrast and dark walls.

  python tools/sil/test_line_node.py [--plot out.png]
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_core"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy"))
import cv2  # noqa: E402
import fake_ros  # noqa: E402

fake_ros.install()
from fake_ros import BUS, Node, CameraInfo, Image, make_time  # noqa: E402
from dh_core.camera_geom import optical_to_body  # noqa: E402

W, H = 1024, 640
K = np.array([[560.0, 0, 512.0], [0, 560.0, 320.0], [0, 0, 1]])
PITCH = math.radians(20.0)
CAM_X, CAM_Z = 0.10, -0.02
LINE_W = 0.0508


def render(height, lateral, heading, half_width, floor_g, line_g, wall_g, rng, wall_top=1.727):
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1).reshape(-1, 3)
    db = np.stack(optical_to_body(rays[:, 0], rays[:, 1], rays[:, 2], PITCH, 0.0, 0.0), -1)
    o = np.array([CAM_X, 0.0, CAM_Z])
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(db[:, 2] < -1e-6, (-height - o[2]) / db[:, 2], np.inf)
        t_wl = np.where(db[:, 1] > 1e-6, (half_width - o[1]) / db[:, 1], np.inf)
        t_wr = np.where(db[:, 1] < -1e-6, (-half_width - o[1]) / db[:, 1], np.inf)
    t_wall = np.minimum(t_wl, t_wr)
    z_wall = o[2] + db[:, 2] * t_wall
    t_wall = np.where(z_wall < wall_top - height, t_wall, np.inf)
    img = np.full(len(db), 120.0)                   # far background (above the walls)
    floor = t_floor < t_wall
    wall = (~floor) & np.isfinite(t_wall)
    p = o + db * np.where(floor, t_floor, 0)[:, None]
    nrm = np.array([-math.sin(heading), math.cos(heading)])
    dist = (p[:, 0] - 0.0) * nrm[0] + (p[:, 1] - lateral) * nrm[1]
    on_line = floor & (np.abs(dist) < LINE_W / 2)
    img[floor] = floor_g
    img[on_line] = line_g
    img[wall] = wall_g
    img = img.reshape(H, W)
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    img = img + rng.normal(0, 4.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def run(node_mod, height, lateral, heading, half_width, floor_g, line_g, wall_g, seed=0):
    BUS.t = 1.0
    Node.PARAM_OVERRIDES = {"mount_source": "parameters"}
    BUS.subs.clear(); BUS.published.clear()
    node = node_mod.LineNode()
    ci = CameraInfo(); ci.k = list(K.ravel()); ci.d = [0.0] * 5
    ci.width, ci.height = W, H
    ci.header.frame_id = "optical"
    node._on_info(ci)
    node.hist.add(1.0, (0, 0, height, 0))
    img = render(height, lateral, heading, half_width, floor_g, line_g, wall_g, np.random.default_rng(seed))
    m = Image(); m.height, m.width, m.step, m.encoding, m.data = H, W, W, "mono8", img.tobytes()
    m.header.stamp = make_time(1.0)
    m.header.frame_id = "optical"
    node._on_img(m)
    out = BUS.published.get("/dh/line_obs", [])
    v = out[-1][1].vector if out else None
    return ((v.x, v.y, v.z) if v is not None else None), img


CASES = [  # name, lateral, heading_deg, half_width, floor, line, wall
    ("centred, light walls", 0.00, 0, 0.48, 200, 75, 215),
    ("offset left 0.20", 0.20, 0, 0.48, 200, 75, 215),
    ("offset right 0.25, yaw 8 deg", -0.25, 8, 0.48, 200, 75, 215),
    ("yaw -12 deg", 0.05, -12, 0.48, 200, 75, 215),
    ("weak contrast (line 160 on 200)", 0.10, 0, 0.48, 200, 160, 215),
    ("dark walls", 0.00, 0, 0.48, 200, 75, 60),
    ("dark walls, offset 0.20, yaw 8", 0.20, 8, 0.48, 200, 75, 60),
    ("dark walls, narrow corridor 0.38", 0.05, 0, 0.38, 200, 75, 60),
    ("line 0.12 m from a dark wall", 0.36, 0, 0.48, 200, 75, 60),
    ("weak contrast + dark walls", 0.10, 4, 0.48, 200, 160, 60),
    ("no line, light walls", None, 0, 0.48, 200, 200, 215),
    ("no line, dark walls", None, 0, 0.48, 200, 200, 60),
    ("no line, dark walls, yaw 10", None, 10, 0.48, 200, 200, 60),
    ("no line, dark walls, narrow 0.38", None, 0, 0.38, 200, 200, 60),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", default="")
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "line_node_test.md"))
    ap.add_argument("--height", type=float, default=0.85)
    args = ap.parse_args()
    Node.PARAM_OVERRIDES = {}
    import dh_autonomy.line_node as ln
    rows, imgs = [], []
    for name, lat, hdg, hw, fg, lg, wg in CASES:
        res, img = run(ln, args.height, 0.0 if lat is None else lat, math.radians(hdg), hw, fg, lg, wg)
        imgs.append((name, img))
        if lat is None:
            verdict = "correct (no output)" if res is None or res[2] < 0.5 else "FALSE detection"
            rows.append("| %s | - | - | %s | %s |" % (name, "none" if res is None else "%.2f / %.1f deg / conf %.2f" % (
                res[0], math.degrees(res[1]), res[2]), verdict))
            continue
        if res is None:
            rows.append("| %s | %+.2f | %+d | no output | MISSED |" % (name, lat, hdg))
            continue
        e_lat, e_hdg = res[0] - lat, math.degrees(res[1]) - hdg
        used = res[2] >= 0.5
        good = abs(e_lat) < 0.03 and abs(e_hdg) < 3
        verdict = ("OK" if good else "WRONG") + ("" if used else " (conf < 0.5: ignored by mission)")
        rows.append("| %s | %+.2f | %+d | %+.3f m / %+.1f deg / conf %.2f | %s |" % (
            name, lat, hdg, e_lat, e_hdg, res[2], verdict))
    assert not any(word in row for row in rows for word in ("MISSED", "WRONG", "FALSE")), rows
    txt = "\n".join(["# line_node end-to-end test (rendered mono images)", "",
                     "Real node code on ray-cast 1024x640 mono images: camera pitched 20 deg down, body %.2f m above the "
                     "floor, 2-inch guideline, corridor walls. Grey levels: floor 200, line 75 (model colours: light-grey "
                     "floor, dark-red line), noise sigma 4. The mission only uses observations with conf >= 0.5." % args.height,
                     "", "| case | true lateral (m) | true heading (deg) | error lateral / heading / conf | verdict |",
                     "|---|---|---|---|---|"] + rows) + "\n"
    print(txt)
    open(args.out, "w", encoding="utf-8").write(txt)
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(15, 3.6))
        for ax, i in zip(axs, (2, 6, 8)):
            ax.imshow(imgs[i][1], cmap="gray", vmin=0, vmax=255); ax.set_title(imgs[i][0]); ax.axis("off")
        fig.savefig(args.plot, dpi=100, bbox_inches="tight")


if __name__ == "__main__":
    main()
