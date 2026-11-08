"""End-to-end closed loop on the procedural course (unknown at start): explore -> window -> tunnel -> blue square.
Opt-in (about 1-4 minutes): DH_E2E=1 python -m pytest test/test_e2e_course.py
Simplified dynamics and perfect rendering: evidence about the policy's logic, NOT real-flight validation."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))]
from dh_core.nav.sim_run import Sim, SimConfig, course_a  # noqa: E402

pytestmark = pytest.mark.skipif(os.environ.get("DH_E2E") != "1", reason="slow closed-loop test: set DH_E2E=1")
BAND = (0.70, 0.80)


def check(r):
    assert r.reached and r.final_state == "OVER_TARGET", (r.final_state, r.log[-10:])
    assert r.collisions == 0
    assert r.min_clearance >= 0.30                       # >= 6 cm beyond the 0.24 m contact radius
    assert r.pad_error is not None and r.pad_error < 0.3  # inside the 0.6 m square
    assert BAND[0] <= r.min_z and r.max_z <= BAND[1]      # never climbed over a wall
    assert any("opening verified" in L for L in r.log)    # the tunnel was verified before traversal


def test_unknown_course_reaches_the_pad():
    check(Sim(course_a(), SimConfig(t_max=300)).run())


def test_unknown_course_with_a_wall_appearing_on_the_route():
    check(Sim(course_a(), SimConfig(t_max=300, popups=[(20.0, (2.15, 2.35, 1.7, 2.5, 0.0, 1.7))])).run())


def test_final_course_v3_from_the_step_file():
    """The organiser's CAD (FINAL Course v3.step, voxelised by tools/sim/step_to_voxels.py) as hidden truth:
    launch mark -> maze -> tube -> chicanes / C2a window -> 5 ft pad. Regression for the TO_TARGET deadlock
    (pad seen through the window before the space around it was mapped)."""
    from dh_core.nav.online_map import MapParams
    from dh_core.nav.sim_mesh_world import VoxelWorld
    npz = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "tools", "sim", "final_course_v3.npz")
    w = VoxelWorld.load(npz)
    r = Sim(w, SimConfig(t_max=300), start=(-2.515, 0.915, 0.75), mprm=MapParams(lo_xy=(-3.1, -3.1), size_xy=6.2),
            geofence=(-2.8, 2.8, -2.8, 2.8)).run()
    assert r.reached and r.final_state == "OVER_TARGET", (r.final_state, r.log[-10:])
    assert r.collisions == 0 and r.min_clearance >= 0.30
    assert r.pad_error is not None and r.pad_error < w.pad[2] / 2
    assert BAND[0] <= r.min_z and r.max_z <= BAND[1]
