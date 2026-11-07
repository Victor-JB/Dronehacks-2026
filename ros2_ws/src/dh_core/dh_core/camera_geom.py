"""Camera geometry shared by the stereo-scan and floor-line nodes (numpy only, unit-tested).

Conventions
  optical frame : x right, y down, z forward (OpenCV)
  body frame    : x forward, y left, z up (FLU); camera at (cam_x, 0, cam_z), pitched DOWN by `pitch` rad
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def optical_to_body(X, Y, Z, pitch: float, cam_x: float, cam_z: float):
    """Rotate optical-frame points into the body frame (camera pitched down by `pitch`)."""
    xc, yc, zc = Z, -X, -Y                       # optical -> camera FLU
    c, s = math.cos(pitch), math.sin(pitch)
    xb = c * xc + s * zc + cam_x
    zb = -s * xc + c * zc + cam_z
    return xb, yc, zb


def disparity_to_body(disp: np.ndarray, f: float, B: float, cx: float, cy: float,
                      pitch: float, cam_x: float, cam_z: float, min_disp: float = 1.0, R_rect=None):
    """Disparity image -> body-frame points. R_rect (the left rectification rotation R1) maps the
    left optical frame to the rectified frame; points are rotated back by R1^T before the body transform."""
    v, u = np.nonzero(disp > min_disp)
    d = disp[v, u].astype(float)
    Z = f * B / d
    X = (u - cx) * Z / f
    Y = (v - cy) * Z / f
    if R_rect is not None:
        X, Y, Z = np.asarray(R_rect, float).reshape(3, 3).T @ np.vstack([X, Y, Z])
    return optical_to_body(X, Y, Z, pitch, cam_x, cam_z)


def points_to_scan(xb, yb, zb, band_center_z: float, band_half: float, fov: float, n_beams: int,
                   max_range: float, min_pts: int = 5, pct: float = 10.0, return_counts: bool = False):
    """Horizontal-band pseudo-scan: per angular bin, a low percentile of the ranges (robust nearest).

    Beams with fewer than min_pts points within max_range are +inf. With return_counts, also returns the
    number of in-band stereo points per beam at ANY range, so callers can tell "measured, nothing within
    max_range" (many points, all far) from "no stereo data" (no points: textureless, dark, too close)."""
    keep = (np.abs(zb - band_center_z) < band_half) & (xb > 0.05)
    xb, yb = xb[keep], yb[keep]
    amin, amax = -fov / 2, fov / 2
    inc = (amax - amin) / (n_beams - 1)
    counts = np.zeros(n_beams, int)
    if len(xb):
        ia = np.round((np.arctan2(yb, xb) - amin) / inc).astype(int)
        ia = ia[(ia >= 0) & (ia < n_beams)]
        counts = np.bincount(ia, minlength=n_beams)[:n_beams]
    rng = np.hypot(xb, yb)
    ok = rng < max_range
    xb, yb, rng = xb[ok], yb[ok], rng[ok]
    ranges = np.full(n_beams, np.inf)
    if len(xb):
        idx = np.round((np.arctan2(yb, xb) - amin) / inc).astype(int)
        good = (idx >= 0) & (idx < n_beams)
        idx, rng = idx[good], rng[good]
        order = np.argsort(idx)
        idx, rng = idx[order], rng[order]
        bounds = np.searchsorted(idx, np.arange(n_beams + 1))
        for i in range(n_beams):
            seg = rng[bounds[i]:bounds[i + 1]]
            if len(seg) >= min_pts:
                ranges[i] = float(np.percentile(seg, pct))
    if return_counts:
        return ranges, amin, inc, counts
    return ranges, amin, inc


def scan_validity(ranges: np.ndarray, counts: np.ndarray, angles: np.ndarray, half_angle: float,
                  min_pts: int = 5, min_coverage: float = 0.6):
    """REP-117 encoding of a pseudo-scan plus the forward-clearance verdict.

    Returns (ranges_rep117, forward_range):
      ranges_rep117 : per beam finite range; +inf = enough stereo points, none within max_range; NaN = no data.
      forward_range : nearest finite range in |angle| <= half_angle if any (an obstacle is reported even if
                      coverage is poor); otherwise +inf if at least min_coverage of the forward beams have
                      data (measured clear); otherwise NaN (cannot vouch for a clear path).
    """
    r = np.asarray(ranges, float).copy()
    has = np.asarray(counts) >= min_pts
    r[~has] = np.nan
    fwd = np.abs(angles) <= half_angle
    fr = r[fwd]
    finite = fr[np.isfinite(fr)]
    if len(finite):
        return r, float(finite.min())
    cov = float(np.mean(has[fwd])) if fwd.any() else 0.0
    return r, (math.inf if cov >= min_coverage else math.nan)


def rays_to_floor(xn: np.ndarray, yn: np.ndarray, pitch: float, height: float, cam_x: float, cam_z: float):
    """Intersect normalised optical rays (x/z, y/z) with the floor plane z_body = -height.

    `height` is the body origin's height above the floor. Returns body-frame (X, Y) of the hits and a mask.
    """
    xb, yb, zb = optical_to_body(xn, yn, np.ones_like(xn), pitch, 0.0, 0.0)
    down = zb < -1e-3
    t = np.where(down, (height + cam_z) / np.maximum(-zb, 1e-9), np.nan)
    return t * xb + cam_x, t * yb, down


def fit_line(X: np.ndarray, Y: np.ndarray) -> Tuple[float, float, float]:
    """Total-least-squares line fit. Returns (lateral offset at body x=0 [left +], heading [rad], 90% |residual|)."""
    P = np.stack([X, Y], 1)
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    d = vt[0]
    if d[0] < 0:
        d = -d
    resid = np.abs((P - c) @ np.array([-d[1], d[0]]))
    if abs(d[0]) > 0.3:
        lateral = c[1] - d[1] / d[0] * c[0]
    else:  # line nearly perpendicular to the body x axis: use the perpendicular distance
        lateral = float(np.array([-d[1], d[0]]) @ c)
    return float(lateral), float(math.atan2(d[1], d[0])), float(np.percentile(resid, 90))


# ---------------------------------------------------------------------------------------------
# Stereo rectification when camera_info carries no stereo calibration (R = I, P_right[0,3] = 0)

def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def camera_info_has_stereo(R_left, P_right, tol: float = 1e-9) -> bool:
    """True if the driver already filled in stereo rectification (non-zero baseline term in P_right)."""
    P_right = np.asarray(P_right, float).reshape(3, 4)
    return abs(P_right[0, 3]) > tol


def stereo_rectify(K1, D1, K2, D2, size, R, T, alpha: float = 0.0):
    """OpenCV stereoRectify wrapper.

    R, T: pose of the LEFT camera expressed in the RIGHT camera's optical frame, i.e. a point
    p_l in left optical coordinates is p_r = R @ p_l + T (this is what tf2 lookup_transform(
    target=right_optical, source=left_optical) returns). For a horizontal pair T ~ (-B, 0, 0).
    Returns (R1, R2, P1, P2, baseline_m).
    """
    import cv2  # local import: dh_core stays importable without OpenCV
    K1 = np.asarray(K1, float).reshape(3, 3); K2 = np.asarray(K2, float).reshape(3, 3)
    D1 = np.asarray(D1, float).ravel(); D2 = np.asarray(D2, float).ravel()
    R = np.asarray(R, float).reshape(3, 3); T = np.asarray(T, float).reshape(3, 1)
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, (int(size[0]), int(size[1])), R, T,
                                                flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha)
    B = -P2[0, 3] / P2[0, 0] if P2[0, 0] else 0.0
    return R1, R2, P1, P2, float(B)
