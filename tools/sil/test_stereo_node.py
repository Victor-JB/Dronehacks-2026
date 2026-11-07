"""End-to-end test of the real stereo_scan_node on rendered stereo pairs (no ROS, needs OpenCV).

A ray-caster renders a raw (unrectified) mono stereo pair of a textured wall at a known distance in
front of the drone plus a textured floor, with the camera pitched down like the OAK-D on Sky Beta.
The right camera can be slightly mis-rotated against the left, as on a real factory-calibrated pair.
The node is fed the pair through the fake ROS layer and the published /dh/stereo_scan and
/dh/forward_clearance are compared with the true wall distance.

Cases:
  camera_info  driver supplies full stereo R/P (ideal case)
  tf           camera_info has no stereo calibration; extrinsics come from TF (rectify_source auto)
  baseline     no stereo calibration, no TF: pure x-baseline assumption (shows the bias if the pair is rotated)

  python tools/sil/test_stereo_node.py [--plot out.png]
"""
import argparse
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_core"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy"))
import cv2  # noqa: E402
import fake_ros  # noqa: E402

fake_ros.install()
from fake_ros import BUS, Node, CameraInfo, Image, TfBuffer as _TfBuffer, make_time  # noqa: E402
from fake_ros import ApproximateTimeSynchronizer as _Sync  # noqa: E402

from dh_core.camera_geom import optical_to_body, quat_to_rot  # noqa: E402

# ---- scene and cameras --------------------------------------------------------------------------
W, H = 1024, 640
K = np.array([[560.0, 0, 512.0], [0, 560.0, 320.0], [0, 0, 1]])
PITCH = math.radians(20.0)
CAM_X, CAM_Z = 0.10, -0.02
HEIGHT = 0.85              # drone body above the floor
BASE = 0.075


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def texture(seed, n=1024):
    rng = np.random.default_rng(seed)
    t = rng.uniform(0, 255, (n, n)).astype(np.float32)
    t = cv2.GaussianBlur(t, (0, 0), 1.2)
    t = cv2.normalize(t, None, 20, 235, cv2.NORM_MINMAX)
    return t


TEX_WALL, TEX_FLOOR = texture(1), texture(2)
TEXEL = 0.004   # m per texel


def render(origin_l, R_cam_to_left, wall_x):
    """Render one camera. R_cam_to_left rotates this camera's optical rays into the LEFT optical frame;
    origin_l is this camera's centre in the left optical frame."""
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1).reshape(-1, 3)
    d_l = rays @ R_cam_to_left.T
    db = np.stack(optical_to_body(d_l[:, 0], d_l[:, 1], d_l[:, 2], PITCH, 0.0, 0.0), -1)   # directions
    ob = np.array(optical_to_body(*[np.array([c]) for c in origin_l], PITCH, CAM_X, CAM_Z)).ravel()
    with np.errstate(divide="ignore", invalid="ignore"):
        t_wall = np.where(db[:, 0] > 1e-6, (wall_x - ob[0]) / db[:, 0], np.inf)
        t_floor = np.where(db[:, 2] < -1e-6, (-HEIGHT - ob[2]) / db[:, 2], np.inf)
    use_wall = t_wall < t_floor
    t = np.where(use_wall, t_wall, t_floor)
    p = ob + db * t[:, None]
    mx = np.where(use_wall, p[:, 1], p[:, 0]) / TEXEL + 512
    my = np.where(use_wall, p[:, 2], p[:, 1]) / TEXEL + 512
    mx = np.mod(mx, 1023).astype(np.float32).reshape(H, W)
    my = np.mod(my, 1023).astype(np.float32).reshape(H, W)
    wall = cv2.remap(TEX_WALL, mx, my, cv2.INTER_LINEAR)
    floor = cv2.remap(TEX_FLOOR, mx, my, cv2.INTER_LINEAR)
    img = np.where(use_wall.reshape(H, W), wall, floor)
    return np.clip(img, 0, 255).astype(np.uint8)


def make_image(arr, frame):
    msg = Image()
    msg.header.frame_id = frame
    msg.header.stamp = make_time(1.0)
    msg.height, msg.width, msg.step, msg.encoding = arr.shape[0], arr.shape[1], arr.shape[1], "mono8"
    msg.data = arr.tobytes()
    return msg


def make_info(frame, R=None, P=None):
    ci = CameraInfo()
    ci.header.frame_id = frame
    ci.width, ci.height = W, H
    ci.k = list(K.ravel()); ci.d = [0.0] * 5
    ci.r = list((np.eye(3) if R is None else R).ravel())
    ci.p = list((np.c_[K, np.zeros(3)] if P is None else P).ravel())
    return ci


def run_case(case, wall_x, q_rel, with_tf=True, mount_source="parameters"):
    """q_rel: rotation of the right camera relative to the left (as seen in the left optical frame)."""
    BUS.t = 1.0
    BUS.subs.clear(); BUS.published.clear(); _TfBuffer.transforms = {}
    R_rl = quat_to_rot(*q_rel)                       # right-camera axes expressed in the left frame
    c_r = np.array([BASE, 0.0, 0.0])                 # right camera centre in the left frame
    left = render(np.zeros(3), np.eye(3), wall_x)
    right = render(c_r, R_rl, wall_x)
    # p_r = R p_l + T with R = R_rl^T, T = -R_rl^T c_r
    R = R_rl.T; T = -R @ c_r
    Node.PARAM_OVERRIDES = {"rectify_source": "baseline" if case == "baseline" else "auto", "mount_source": mount_source, "tf_wait_s": 0.0, "camera_pitch_deg": 20.0,
                            "camera_x": CAM_X, "camera_z": CAM_Z}
    import importlib
    import dh_autonomy.stereo_scan_node as ssn
    importlib.reload(ssn)
    node = ssn.StereoScanNode()
    if case == "camera_info":
        from dh_core.camera_geom import stereo_rectify
        R1, R2, P1, P2, _ = stereo_rectify(K, np.zeros(5), K, np.zeros(5), (W, H), R, T)
        node._on_info("l", make_info("oak_left_optical", R1, P1))
        node._on_info("r", make_info("oak_right_optical", R2, P2))
    else:
        node._on_info("l", make_info("oak_left_optical"))
        node._on_info("r", make_info("oak_right_optical"))
        if case == "tf" and with_tf:
            w = q_rel[3]; qinv = (-q_rel[0], -q_rel[1], -q_rel[2], w)     # R = R_rl^T
            tr = types.SimpleNamespace(transform=types.SimpleNamespace(
                rotation=types.SimpleNamespace(x=qinv[0], y=qinv[1], z=qinv[2], w=qinv[3]),
                translation=types.SimpleNamespace(x=T[0], y=T[1], z=T[2])))
            _TfBuffer.transforms[("oak_right_optical", "oak_left_optical")] = tr
    if mount_source == "tf":
        q = qmul((0., math.sin(PITCH/2), 0., math.cos(PITCH/2)), (-0.5, 0.5, -0.5, 0.5))
        _TfBuffer.transforms[("base_link", "oak_left_optical")] = types.SimpleNamespace(transform=types.SimpleNamespace(
            rotation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]),
            translation=types.SimpleNamespace(x=CAM_X, y=0., z=CAM_Z)))
    _Sync.last.cb(make_image(left, "oak_left_optical"), make_image(right, "oak_right_optical"))
    scans = BUS.published.get("/dh/stereo_scan", [])
    clear = BUS.published.get("/dh/forward_clearance", [])
    if not scans:
        return None
    sc = scans[-1][1]
    r = np.asarray(sc.ranges, float)
    a = sc.angle_min + np.arange(len(r)) * sc.angle_increment
    ok = np.isfinite(r)
    x = r[ok] * np.cos(a[ok])                       # body-x of each return; a wall at x = wall_x
    return dict(case=case, wall_x=wall_x, n=int(ok.sum()), n_beams=len(r),
                x_med=float(np.median(x)) if len(x) else math.nan,
                x_p10=float(np.percentile(x, 10)) if len(x) else math.nan,
                x_p90=float(np.percentile(x, 90)) if len(x) else math.nan,
                clearance=float(clear[-1][1].range) if clear else math.nan, angles=a, ranges=r,
                imgs=(left, right))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", default="")
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "stereo_node_test.md"))
    args = ap.parse_args()
    a, b = math.radians(1.0) / 2, math.radians(0.5) / 2
    q_rot = qmul((0.0, math.sin(a), 0.0, math.cos(a)), (0.0, 0.0, math.sin(b), math.cos(b)))  # 1 deg yaw, 0.5 roll
    q_id = (0.0, 0.0, 0.0, 1.0)
    rows = []
    for wall in (0.8, 1.5, 2.5):
        rows.append(run_case("camera_info", wall, q_rot))
        rows.append(run_case("tf", wall, q_rot))
        rows.append(run_case("baseline", wall, q_id))
        rows.append(run_case("baseline", wall, q_rot))
    lines = ["# stereo_scan_node end-to-end test (rendered stereo pairs)", "",
             "Real node code with OpenCV SGBM on ray-cast 1024x640 raw mono pairs (textured wall + floor), camera "
             "pitched 20 deg down, 7.5 cm baseline. 'rotated' = right camera yawed 1.0 deg and rolled 0.5 deg "
             "against the left. Body-x of the scan returns should equal the wall distance.", "",
             "| rectification source | pair | wall (m) | beams with returns | median x (m) | error | p10-p90 x (m) | forward clearance (m) |",
             "|---|---|---|---|---|---|---|---|"]
    for i, r in enumerate(rows):
        pair = "aligned" if (i % 4) == 2 else "rotated"
        if r is None:
            lines.append("| %s | %s | - | no scan published | | | | |" % (["camera_info", "tf", "baseline", "baseline"][i % 4], pair))
            continue
        lines.append("| %s | %s | %.2f | %d/%d | %.3f | %+.1f%% | %.3f-%.3f | %.3f |" % (
            r["case"], pair, r["wall_x"], r["n"], r["n_beams"], r["x_med"], 100 * (r["x_med"] / r["wall_x"] - 1),
            r["x_p10"], r["x_p90"], r["clearance"]))
    for i, result in enumerate(rows):
        assert result is not None, "no stereo output for case %d" % i
        if i % 4 != 3:  # deliberately miscalibrated baseline case is a negative control
            assert abs(result["x_med"] / result["wall_x"] - 1) < 0.05, result
    txt = "\n".join(lines) + "\n"
    print(txt)
    open(args.out, "w", encoding="utf-8").write(txt)
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(15, 4))
        left, right = rows[4]["imgs"]
        axs[0].imshow(left, cmap="gray"); axs[0].set_title("rendered left (wall 1.5 m)")
        axs[1].imshow(right, cmap="gray"); axs[1].set_title("rendered right (rotated pair)")
        for r, c in zip(rows[4:8], ["k", "g", "b", "r"]):
            if r is None:
                continue
            ok = np.isfinite(r["ranges"])
            axs[2].plot(np.degrees(r["angles"][ok]), r["ranges"][ok] * np.cos(r["angles"][ok]), c + ".", ms=3,
                        label=r["case"])
        axs[2].axhline(1.5, color="grey", lw=1); axs[2].set_xlabel("beam angle (deg)"); axs[2].set_ylabel("body x (m)")
        axs[2].legend(["camera_info (rotated)", "tf (rotated)", "baseline (aligned)", "baseline (rotated)"])
        fig.savefig(args.plot, dpi=110, bbox_inches="tight")


if __name__ == "__main__":
    main()
