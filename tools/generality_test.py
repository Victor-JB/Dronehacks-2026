"""Generality evidence: run the unchanged planner on modified courses.

Variants generated from the official mesh:
  * 12 random perturbations: every wall/boulder/tube moved up to +-15 cm and rotated up to +-5 deg
    (the organizers say the layout "may change slightly"), launch/pad moved up to +-10 cm
  * the whole course mirrored east-west (a genuinely different layout)
  * the window wall replaced by a solid wall (no aperture -> must fall back to Gate B)
Writes docs/generality_results.md and docs/figures/generality.png.
"""
import copy, os
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ, HERE
from dh_core.mesh import MeshGroup, load_obj
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission

rng = np.random.default_rng(2026)
base = load_obj(COURSE_OBJ)


def transform(groups, fn):
    out = {}
    for n, g in groups.items():
        out[n] = MeshGroup(n, fn(n, g.triangles.copy()), g.materials)
    return out


def perturb(n, tri, sig=0.15, rot_deg=5.0, mark_sig=0.10):
    ln = n.lower()
    if "net" in ln or "floor" in ln or "guideline" in ln:
        return tri
    if tri[:, :, 2].max() <= 0.01:                       # floor markings (launch, pad, rings)
        d = rng.uniform(-mark_sig, mark_sig, 2)
        if "pad" in ln or "ring" in ln:
            d = perturb.pad_shift
        tri[:, :, :2] += d
        return tri
    c = tri.reshape(-1, 3)[:, :2].mean(0)
    a = np.radians(rng.uniform(-rot_deg, rot_deg))
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    xy = (tri[:, :, :2] - c) @ R.T + c + rng.uniform(-sig, sig, 2)
    tri[:, :, :2] = xy
    return tri


def mirror(n, tri):
    tri[:, :, 0] *= -1
    return tri[:, ::-1, :]


def solid_window(groups):
    g = copy.deepcopy(groups)
    w = g["C2a chicane wall (window)"]
    v = w.triangles.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    # replace with the 8-vertex box spanned by the same oriented footprint (no hole)
    ref = g["C2b chicane wall"].triangles
    c_old = ref.reshape(-1, 3)[:, :2].mean(0)
    c_new = v[:, :2].mean(0)
    scale = 63.5 / 40.5
    tri = ref.copy()
    tri[:, :, :2] = (tri[:, :, :2] - c_old) * np.array([1.0, 1.0]) * 1.0
    # stretch along the wall axis (70 deg) to 63.5 in
    ax = np.array([np.cos(np.radians(70)), np.sin(np.radians(70))])
    s = tri[:, :, :2] @ ax
    tri[:, :, :2] += np.outer(s.ravel() * (scale - 1), ax).reshape(tri[:, :, :2].shape)
    tri[:, :, :2] += c_new
    g["C2a chicane wall (window)"] = MeshGroup("C2a chicane wall (solid)", tri, w.materials)
    return g


variants = []
for i in range(12):
    perturb.pad_shift = rng.uniform(-0.10, 0.10, 2)
    variants.append(("perturbed #%d" % (i + 1), transform(base, perturb)))
variants.append(("mirrored east-west", transform(base, mirror)))
variants.append(("window bricked up", solid_window(base)))

lines = ["# Generality test - unchanged planner on modified courses\n",
         "| variant | route chosen (auto) | apertures found | length (m) | planned time (s) | min clearance (m) | result |",
         "|---|---|---|---|---|---|---|"]
results = []
for name, groups in [("official v3", base)] + variants:
    c = Course.from_groups(groups)
    try:
        plan = plan_mission(c, RouteParams(route="auto"))
        tr = plan.legs[0].traj
        lines.append("| %s | %s | %d | %.2f | %.1f | %.3f | OK |" % (name, plan.route_used, len(c.apertures), tr.length,
                                                                       tr.duration, plan.info["min_clearance"]))
        results.append((name, c, plan))
    except Exception as e:  # noqa: BLE001
        lines.append("| %s | - | %d | - | - | - | FAILED: %s |" % (name, len(c.apertures), e))
    print(lines[-1], flush=True)
open(os.path.join(HERE, "..", "docs", "generality_results.md"), "w").write("\n".join(lines) + "\n")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
show = [r for r in results if r[0] in ("official v3", "perturbed #1", "perturbed #2", "perturbed #3",
                                       "mirrored east-west", "window bricked up")]
fig, axs = plt.subplots(2, 3, figsize=(13, 9))
for ax, (name, c, plan) in zip(axs.ravel(), show):
    g = plan.grid
    ext = [g.origin[0], g.origin[0] + g.shape[1] * g.res, g.origin[1], g.origin[1] + g.shape[0] * g.res]
    ax.imshow(g.occ, origin="lower", extent=ext, cmap="Greys")
    tr = plan.legs[0].traj
    ax.plot(tr.p[:, 0], tr.p[:, 1], "b-", lw=1.2)
    ax.plot(*c.launch[:2], "r^")
    ax.add_patch(plt.Circle(c.pad_center[:2], c.pad_radius, fill=False, color="r"))
    ax.set_title("%s: %s, %.1f s" % (name, plan.route_used, tr.duration), fontsize=9)
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
fig.suptitle("Same code, different courses: routes are derived from geometry, never typed in")
fig.savefig(os.path.join(HERE, "..", "docs", "figures", "generality.png"), dpi=110, bbox_inches="tight")
print("saved figure")
