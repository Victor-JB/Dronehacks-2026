"""Monte Carlo over local_sim's dynamics model: many seeds x a few estimator-error scenarios.

    cd local_sim && python3 montecarlo.py ../route.json [seeds_per_scenario]

Each seed draws its own launch placement error, VIO drift, turbulence and stereo noise. Reports,
per scenario, how often the run completes vs. collides (ground truth) vs. fails otherwise.
"""
import math
import os
import sys
from multiprocessing import get_context

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim as S  # noqa: E402

SCENARIOS = [
    ("placement only (perfect VIO)", dict(drift_per_sqrt_m=0.0, yaw_drift_per_sqrt_m=0.0)),
    ("VIO drift 0.25% (good)", dict(drift_per_sqrt_m=0.01, yaw_drift_per_sqrt_m=math.radians(0.08))),
    ("VIO drift 0.5% (typical)", dict()),
    ("VIO drift 1% (poor)", dict(drift_per_sqrt_m=0.04, yaw_drift_per_sqrt_m=math.radians(0.3))),
]
JIG = dict(place_xy_sigma=0.005, place_yaw_sigma=math.radians(0.3))
SCENARIOS_JIG = [
    ("jig placement 0.5cm/0.3deg, perfect VIO", dict(JIG, drift_per_sqrt_m=0.0, yaw_drift_per_sqrt_m=0.0)),
    ("jig placement 0.5cm/0.3deg, good VIO 0.25%", dict(JIG, drift_per_sqrt_m=0.01, yaw_drift_per_sqrt_m=math.radians(0.08))),
]
if os.environ.get("MC_SET") == "jig":
    SCENARIOS = SCENARIOS_JIG


VARIANT = os.environ.get("MC_VARIANT", "base")   # base | center | plan | both


def one(job):
    route, name, est, seed = job
    r = S.run(route, seed=seed, est_params=est, variant=VARIANT)
    return name, seed, r["result"], r["legs_done"], r["worst_clearance_m"], r["flight_s"], r["final_est_err_m"]


def main():
    route = sys.argv[1] if len(sys.argv) > 1 else "../route.json"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    jobs = [(route, name, est, 1000 * i + s) for i, (name, est) in enumerate(SCENARIOS) for s in range(n)]
    with get_context("spawn").Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        res = pool.map(one, jobs, chunksize=1)
    print(f"{route}: {n} seeds per scenario, DRONE_RADIUS_M={S.DRONE_RADIUS_M}, variant={VARIANT}")
    for name, _ in SCENARIOS:
        rs = [r for r in res if r[0] == name]
        ok = sum(r[2] == "complete" for r in rs)
        col = sum(r[2] == "COLLISION" for r in rs)
        legs = np.array([r[3] for r in rs])
        clr = np.array([r[4] for r in rs])
        err = np.array([r[6] for r in rs])
        times = [r[5] for r in rs if r[2] == "complete"]
        other = sorted({r[2] for r in rs} - {"complete", "COLLISION"})
        print(f"\n  {name}")
        print(f"    complete {ok}/{n}   collision {col}/{n}   other {n - ok - col}/{n} {other if other else ''}")
        print(f"    legs done: median {np.median(legs):.0f}/26, min {legs.min()}")
        print(f"    worst clearance (cm): median {np.median(clr) * 100:.1f}, p10 {np.percentile(clr, 10) * 100:.1f}")
        print(f"    VIO error at end (cm): median {np.median(err) * 100:.1f}")
        if times:
            print(f"    flight time when complete: median {np.median(times):.0f} s")


if __name__ == "__main__":
    main()
