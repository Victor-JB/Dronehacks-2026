"""Head-to-head comparison of the three trajectory planners in the software-in-the-loop simulator.

Every planner flies the SAME seeds (paired comparison), both routes, localizer on, under three tests:
  A  nominal      : normal odometry drift (0.01 m/sqrt(m)) and tube blackout error (0.08 m/s)
  B  harsh        : 2.5x drift (0.025) and 2x tube blackout error (0.16 m/s) -> robustness
  C  new layout   : 3 different courses with every wall/boulder/tube moved up to +-15 cm and turned up to
                    +-5 deg, launch/pad moved up to +-10 cm (the organizers say the layout "may change
                    slightly"); planner AND simulator use the changed course -> generality
plus
  S  speed cap    : fillet planner, window route, simulated PX4 MPC_XY_VEL_MAX of 1.0 / 1.5 / 2.0 m/s,
                    planned speed 85 % of the cap (what happens if the organizers allow raising it)

Scoring per run uses the official formula: finished = 35 + 20 * T*/T + 5 (window), with +5 s per wall
touch (R7.8); T* = fastest finish among all runs of that test. "Finished" = landed with the whole
drone inside the pad's outer ring (pad error + 0.27 m drone half-span < 0.762 m) and no crash.
The simulator has no collision physics, so a contact deeper than 10 cm (props well inside a wall) is
counted as a crash; shallower contacts are touches (+5 s). Crashed runs score 0 here (conservative:
real partial checkpoint credit is not modelled).

  python tools/sil/compare_suite.py [--workers 8]
-> docs/planner_suite.md, docs/figures/planner_suite.png, docs/planner_suite_raw.json
"""
import argparse, copy, json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_core"))
from dh_core.mesh import MeshGroup, load_obj  # noqa: E402

COURSE = os.environ.get("DH_COURSE_OBJ", "C:/dhdl/obj/dronehacks-course-v3.obj")
PLANNERS = ["fillet_arc", "fillet", "sfc", "mpl"]
# fillet_arc = the previous fillet (circular arcs, separate along/lateral limits, no window straight run)
PLANNER_ARGS = {"fillet_arc": ["--planner", "fillet", "--corner", "arc", "--a_total", "0", "--window_straight", "0"],
                "fillet": ["--planner", "fillet"], "sfc": ["--planner", "sfc"], "mpl": ["--planner", "mpl"]}
LABEL = {"fillet_arc": "fillet, old (arcs)", "fillet": "fillet, new (clothoids)", "sfc": "sfc", "mpl": "mpl"}
ROUTES = ["window", "gate"]
SEEDS = [101, 102, 103]
PAD_R, HALF_SPAN = 0.762, 0.27
CRASH_DEPTH = 0.10   # m of propeller penetration into a wall counted as a crash


def write_obj(groups, path):
    with open(path, "w", encoding="utf-8") as f:
        n = 0
        for name, g in groups.items():
            f.write("g %s\n" % name)
            for m in g.materials[:1]:
                f.write("usemtl %s\n" % m)
            for tri in g.triangles:
                for v in tri:
                    f.write("v %.6f %.6f %.6f\n" % tuple(v))
                f.write("f %d %d %d\n" % (n + 1, n + 2, n + 3))
                n += 3


def make_variants(out_dir):
    rng = np.random.default_rng(2611)
    base = load_obj(COURSE)
    paths = []
    for k in range(3):
        pad_shift = rng.uniform(-0.10, 0.10, 2)
        out = {}
        for n, g in base.items():
            tri = g.triangles.copy()
            ln = n.lower()
            if not ("net" in ln or "floor" in ln or "guideline" in ln):
                if tri[:, :, 2].max() <= 0.01:
                    tri[:, :, :2] += pad_shift if ("pad" in ln or "ring" in ln) else rng.uniform(-0.10, 0.10, 2)
                else:
                    c = tri.reshape(-1, 3)[:, :2].mean(0)
                    a = np.radians(rng.uniform(-5, 5))
                    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
                    tri[:, :, :2] = (tri[:, :, :2] - c) @ R.T + c + rng.uniform(-0.15, 0.15, 2)
            out[n] = MeshGroup(n, tri, g.materials)
        p = os.path.join(out_dir, "course_variant_%d.obj" % (k + 1))
        write_obj(out, p)
        paths.append(p)
    return paths


def run(job):
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1",
               DH_COURSE_OBJ=job["course"])
    cmd = [sys.executable, "-B", os.path.join(HERE, "run_sil.py"), "--route", job["route"], "--localizer",
           *PLANNER_ARGS[job["planner"]], "--seed", str(job["seed"]), "--drift", str(job["drift"]),
           "--dark_err", str(job["dark_err"]), "--v_dark", str(job["v_max"]), "--v_max", str(job["v_max"]),
           "--v_cap", str(job["cap"]), "--px4_cap", str(job["cap"])]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=900, env=env, cwd="C:/").stdout
        line = [l for l in out.splitlines() if l.startswith("RESULT ")]
        res = json.loads(line[-1][7:]) if line else {"error": out[-400:]}
    except subprocess.TimeoutExpired:
        res = {"error": "timeout"}
    res.update({k: job[k] for k in ("test", "planner", "route", "seed", "cap", "v_max")})
    res["planner"] = job["planner"]
    print("%-4s %-6s %-6s seed %d cap %.1f -> %s" % (job["test"], job["planner"], job["route"], job["seed"], job["cap"],
          {k: res.get(k) for k in ("course_time_s", "touches", "pad_xy_error_m", "route_used", "error")}), flush=True)
    return res


def score(results):
    """Add finished flag, penalised time and official points (T* = fastest finish within the same test)."""
    for r in results:
        t = r.get("course_time_s")
        # The simulator has no collision physics: a drone whose centre gets within 0.14 m of a wall surface
        # (props > 10 cm inside the wall) would have crashed, not just touched -> run ends, no finish.
        r["crashed"] = r.get("min_clearance_m", 0) < -CRASH_DEPTH
        r["finished"] = bool(t is not None and not r["crashed"] and r.get("final_state") in ("LAND", "LANDED", "DONE")
                             and r.get("pad_xy_error_m", 9) + HALF_SPAN < PAD_R)
        r["time_pen_s"] = None if t is None else t + 5.0 * r.get("touches", 0)
    for test in {r["test"] for r in results}:
        fin = [r["time_pen_s"] for r in results if r["test"] == test and r["finished"]]
        tstar = min(fin) if fin else None
        for r in results:
            if r["test"] != test:
                continue
            if r["finished"]:
                win = r.get("route_used") == "window"
                r["points"] = 35 + 20 * tstar / r["time_pen_s"] + (5 if win else 0)
            else:
                r["points"] = 0.0     # conservative: an unfinished run's checkpoint credit is not modelled
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--report-only", action="store_true", help="re-draw from docs/planner_suite_raw.json")
    args = ap.parse_args()
    vdir = os.path.join(HERE, "variants")
    os.makedirs(vdir, exist_ok=True)
    variants = make_variants(vdir)
    jobs = []
    for planner in PLANNERS:
        for route in ROUTES:
            for i, seed in enumerate(SEEDS):
                base = dict(planner=planner, route=route, seed=seed, cap=1.0, v_max=0.85)
                jobs.append(dict(base, test="A", course=COURSE, drift=0.01, dark_err=0.08))
                jobs.append(dict(base, test="B", course=COURSE, drift=0.025, dark_err=0.16))
                jobs.append(dict(base, test="C", course=variants[i], drift=0.01, dark_err=0.08))
    for cap in (1.0, 1.5, 2.0):
        for seed in SEEDS:
            jobs.append(dict(test="S", planner="fillet", route="window", seed=seed, cap=cap, v_max=round(0.85 * cap, 3),
                             course=COURSE, drift=0.01, dark_err=0.08))
    # longest first (MPL plans slowly) for better packing
    jobs.sort(key=lambda j: j["planner"] != "mpl")
    with ThreadPoolExecutor(args.workers) as ex:
        results = list(ex.map(run, jobs))
    results = score([r for r in results if "error" not in r]) + [r for r in results if "error" in r]
    json.dump(results, open(os.path.join(ROOT, "docs", "planner_suite_raw.json"), "w"), indent=1)
    report(results)


def agg(rows):
    ok = [r for r in rows if "error" not in r]
    t = [r["course_time_s"] for r in ok if r.get("course_time_s") is not None]
    return dict(n=len(rows), crashed=sum(1 for r in ok if r.get("crashed")), clean=sum(1 for r in ok if r["finished"] and r["touches"] == 0),
                finished=sum(1 for r in ok if r["finished"]),
                t_mean=np.mean(t) if t else np.nan, t_min=min(t) if t else np.nan, t_max=max(t) if t else np.nan,
                touches=np.mean([r["touches"] for r in ok]) if ok else np.nan,
                clr=min(r["min_clearance_m"] for r in ok) if ok else np.nan,
                pad=np.mean([r["pad_xy_error_m"] for r in ok]) if ok else np.nan,
                est=max((r.get("est_err_max_m") or 0) for r in ok) if ok else np.nan,
                vmax=max(r.get("max_speed_mps", 0) for r in ok) if ok else np.nan,
                plan=np.mean([r.get("plan_compute_s", np.nan) for r in ok]) if ok else np.nan,
                pts=np.mean([r["points"] for r in ok]) if ok else np.nan,
                errors=len(rows) - len(ok))


def report(results):
    names = {"A": "Test A: nominal conditions", "B": "Test B: harsh conditions (2.5x drift, 2x tube blackout error)",
             "C": "Test C: changed layout (3 perturbed courses, map updated)"}
    L = ["# Planner comparison suite (software-in-the-loop, real node code)", "",
         "Same seeds (101, 102, 103) for every planner, localizer on, tube flown at full speed, PX4 cap 1.0 m/s unless noted. "
         "Simulator: PX4-like velocity loop with a 3 m/s^2 horizontal acceleration limit and 0.12 s attitude lag, altitude drift, "
         "ray-cast forward clearance driving the stereo slowdown, and 3-D wall contact at the true height. "
         "All planners share one 1.5 m/s^2 total-acceleration budget (fillet: combined limit; SFC: |a| limit; MPL: |u| <= a_max). "
         "'fillet, old (arcs)' is the previous fillet planner, kept for comparison. "
         "Points use the official finished-run formula with +5 s per touch; T* = fastest finish in that test (all teams = these runs). "
         "'Clean' = finished on the pad with no wall touch; 'crashed' = wall contact deeper than 10 cm (the sim has no collision physics). Plan time is laptop CPU; expect ~2-3x on the Orin Nano.", ""]
    for test in ("A", "B", "C"):
        L += ["## " + names[test], "",
              "| planner | route | clean / finished / crashed / runs | course time mean (min-max) s | touches/run | worst clearance m | pad error m | worst estimate error m | max speed m/s | plan time s | **mean points** |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for p in PLANNERS:
            for route in ROUTES:
                a = agg([r for r in results if r["test"] == test and r["planner"] == p and r["route"] == route])
                L.append("| %s | %s | %d / %d / %d / %d | %.1f (%.1f-%.1f) | %.2f | %+.3f | %.3f | %.3f | %.2f | %.1f | **%.1f** |" % (
                    LABEL[p], route, a["clean"], a["finished"], a["crashed"], a["n"], a["t_mean"], a["t_min"], a["t_max"], a["touches"], a["clr"],
                    a["pad"], a["est"], a["vmax"], a["plan"], a["pts"]))
        L.append("")
    L += ["## Test S: speed cap (fillet planner, window route, planned speed = 85 % of the cap)", "",
          "| PX4 speed cap m/s | planned speed m/s | clean / finished / crashed / runs | course time mean (min-max) s | touches/run | worst clearance m | worst estimate error m | max speed flown m/s | **mean points** |",
          "|---|---|---|---|---|---|---|---|---|"]
    for cap in (1.0, 1.5, 2.0):
        rows = [r for r in results if r["test"] == "S" and r["cap"] == cap]
        a = agg(rows)
        L.append("| %.1f | %.2f | %d / %d / %d / %d | %.1f (%.1f-%.1f) | %.2f | %+.3f | %.3f | %.2f | **%.1f** |" % (
            cap, 0.85 * cap, a["clean"], a["finished"], a["crashed"], a["n"], a["t_mean"], a["t_min"], a["t_max"], a["touches"], a["clr"],
            a["est"], a["vmax"], a["pts"]))
    errs = [r for r in results if "error" in r]
    if errs:
        L += ["", "Runs that crashed the simulator script: %d (%s)" % (len(errs), ", ".join(
            "%s/%s/%s/%s" % (r["test"], r["planner"], r["route"], r["seed"]) for r in errs))]
    txt = "\n".join(L) + "\n"
    open(os.path.join(ROOT, "docs", "planner_suite.md"), "w", encoding="utf-8").write(txt)
    print(txt)
    figure(results)


def figure(results):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"fillet_arc": "#eda100", "fillet": "#2a78d6", "sfc": "#eb6834", "mpl": "#1baf7a"}
    ink, ink2, grid_c, surf = "#0b0b0b", "#52514e", "#e6e5e1", "#fcfcfb"
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.8), facecolor=surf)
    groups = [(t, rt) for t in ("A", "B", "C") for rt in ROUTES]
    labels = ["%s\n%s" % ({"A": "nominal", "B": "harsh", "C": "new layout"}[t], rt) for t, rt in groups]
    x = np.arange(len(groups)); w = 0.2
    for ax, key, title in ((axs[0], "pts", "Mean run points (official formula)"),
                           (axs[1], "t_mean", "Mean course time (s)")):
        for k, p in enumerate(PLANNERS):
            vals = [agg([r for r in results if r["test"] == t and r["route"] == rt and r["planner"] == p])[key] for t, rt in groups]
            bars = ax.bar(x + (k - 1.5) * (w + 0.01), vals, w, color=col[p], label=LABEL[p], edgecolor=surf, linewidth=2)
            for b, v in zip(bars, vals):
                if np.isfinite(v):
                    ax.text(b.get_x() + b.get_width() / 2, v, "%.0f" % v, ha="center", va="bottom", fontsize=7, color=ink2)
        ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8, color=ink2)
        ax.set_title(title, fontsize=10, color=ink, loc="left")
        ax.grid(axis="y", color=grid_c, lw=0.8); ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        ax.spines["left"].set_color(grid_c); ax.spines["bottom"].set_color(grid_c)
        ax.tick_params(colors=ink2, labelsize=8); ax.set_facecolor(surf)
        ax.legend(frameon=False, fontsize=8, labelcolor=ink2)
    ax = axs[2]
    caps = (1.0, 1.5, 2.0)
    for metric, c, lab in (("t_mean", "#2a78d6", "course time (s)"),):
        vals = [agg([r for r in results if r["test"] == "S" and r["cap"] == cap])[metric] for cap in caps]
        ax.plot(caps, vals, "-o", color=c, lw=2, ms=8, label=lab)
        for cx, v in zip(caps, vals):
            a = agg([r for r in results if r["test"] == "S" and r["cap"] == cx])
            ax.annotate("%.1f s\n%d/%d clean, %.1f touches/run" % (v, a["clean"], a["n"], a["touches"]), (cx, v),
                        textcoords="offset points", xytext=(0, 10), ha="center", fontsize=7, color=ink2)
    ax.set_title("Speed cap test: course time (fillet, window)", fontsize=10, color=ink, loc="left")
    ax.set_xlabel("simulated PX4 speed cap (m/s); planned at 85 %", fontsize=8, color=ink2)
    ax.set_xticks(caps); ax.grid(axis="y", color=grid_c, lw=0.8); ax.set_facecolor(surf)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(colors=ink2, labelsize=8)
    ax.margins(y=0.35); ax.set_xlim(0.8, 2.2)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, "docs", "figures", "planner_suite.png"), dpi=110, facecolor=surf)


if __name__ == "__main__":
    if "--report-only" in sys.argv:
        report(json.load(open(os.path.join(ROOT, "docs", "planner_suite_raw.json"))))
    else:
        main()
