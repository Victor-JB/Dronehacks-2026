"""The generated dh_nav_policy package, checked against MAIN's code (not this branch's): it must be in sync with
its sources, import only main's refnav / dronehacks_nav, run its node on fake ROS, and own no PX4 path.
Needs git (origin/main is extracted with `git archive`). No DDS, no hardware."""
import ast
import filecmp
import glob
import io
import os
import subprocess
import sys
import tarfile
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
PKG = os.path.join(ROOT, "integration", "dh_nav_policy")
sys.path.insert(0, HERE)
import build_nav_policy_pkg as B  # noqa: E402


@pytest.fixture(scope="module")
def main_tree(tmp_path_factory):
    """origin/main's refnav + dronehacks_nav, extracted (the code the package runs against on the drone)."""
    d = tmp_path_factory.mktemp("main")
    data = subprocess.run(["git", "-C", ROOT, "archive", "origin/main", "refnav", "dronehacks_nav"],
                          capture_output=True, check=True).stdout
    tarfile.open(fileobj=io.BytesIO(data)).extractall(d, filter="data")
    return str(d)


def test_committed_package_is_in_sync_with_sources(tmp_path):
    out = tmp_path / "pkg"
    B.build(str(out))
    cmp = filecmp.dircmp(str(out), PKG)
    stale = []

    def walk(c, rel=""):
        stale.extend(os.path.join(rel, f) for f in c.left_only + c.right_only + c.diff_files)
        for name, sub in c.subdirs.items():
            if name != "__pycache__":
                walk(sub, os.path.join(rel, name))
    walk(cmp)
    assert stale == [], "re-run tools/integration/build_nav_policy_pkg.py: stale %s" % stale


def test_package_pure_tests_pass_against_main(main_tree):
    env = dict(os.environ, DRONEHACKS_MAIN=main_tree, PYTHONDONTWRITEBYTECODE="1",
               PYTHONPATH=os.pathsep.join([PKG, main_tree]))
    r = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "test/test_navstack.py"],
                       cwd=PKG, env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]


def test_node_runs_on_fake_ros_with_mains_modules(main_tree):
    """Construct the packaged node (dry run and live) with main's refnav/dronehacks_nav on the path."""
    code = r'''
import sys, types, json
sys.path[:0] = [%r, %r, %r]
import fake_ros; fake_ros.install()
sys.modules.setdefault("isaac_ros_visual_slam_interfaces", types.ModuleType("isaac_ros_visual_slam_interfaces"))
sys.modules["isaac_ros_visual_slam_interfaces.msg"] = types.SimpleNamespace(VisualSlamStatus=type("V", (), {}))
from fake_ros import BUS, Node
import refnav, dronehacks_nav.mapping, dh_nav_policy.node as N
assert refnav.__file__.startswith(%r), refnav.__file__          # main's refnav, not this branch's
Node.PARAM_OVERRIDES = {}
dry = N.NavPolicyNode()
assert {p.topic_name for p in dry.publishers} == {"/dh/dry_run/cmd_vel", "/dh/dry_run/events"}
BUS.subs.clear(); BUS.timers.clear()
Node.PARAM_OVERRIDES = {"dry_run": False, "geofence": [-1.0, 4.0, -2.0, 2.0]}
live = N.NavPolicyNode()
assert {p.topic_name for p in live.publishers} == {"/skybeta/setpoint/twist", "/skybeta/setpoint/pose"}
assert not any(t.startswith("/fmu") for t in BUS.subs)
print("OK")
''' % (PKG, main_tree, os.path.join(ROOT, "tools", "sil"), main_tree)
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                       env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
    assert r.returncode == 0 and "OK" in r.stdout, r.stdout[-2000:] + r.stderr[-2000:]


def test_package_owns_no_px4_path_and_only_organizer_publishers():
    files = glob.glob(os.path.join(PKG, "dh_nav_policy", "**", "*.py"), recursive=True)
    for f in files:
        tree = ast.parse(open(f, encoding="utf-8").read())
        for n in ast.walk(tree):
            if isinstance(n, (ast.Import, ast.ImportFrom)):
                mods = [a.name for a in n.names] + ([n.module] if isinstance(n, ast.ImportFrom) and n.module else [])
                assert not any(m and m.split(".")[0] == "px4_msgs" for m in mods), f
            if isinstance(n, ast.ClassDef):
                assert n.name != "Px4Backend", f
            if isinstance(n, ast.Call) and getattr(n.func, "attr", "") in ("create_publisher", "create_client"):
                assert os.path.basename(f) == "backends.py", "endpoint created outside backends.py: %s" % f
    src = open(os.path.join(PKG, "dh_nav_policy", "backends.py"), encoding="utf-8").read()
    for line in src.splitlines():
        if "create_publisher(" in line and "/fmu" in line:
            raise AssertionError(line)


def test_launch_matches_the_flown_test_files_launch():
    src = open(os.path.join(PKG, "launch", "nav_policy.launch.py"), encoding="utf-8").read()
    tree = ast.parse(src)
    params = [d for d in ast.walk(tree) if isinstance(d, ast.Dict)
              and any(isinstance(k, ast.Constant) and k.value == "auto_offboard_on_arm" for k in d.keys)]
    assert params, "flight_interface parameters not found"
    kv = {k.value: getattr(v, "value", None) for k, v in zip(params[0].keys, params[0].values)}
    assert kv == {"auto_offboard_on_arm": True, "anchor_setpoints": True}          # exactly test_files'
    assert '"skybeta_flight_interface"' in src and 'executable="flight_interface"' in src
    assert src.count('executable="flight_interface"') == 1                          # exactly one flight_interface
    assert 'package="dh_nav_policy", executable="nav_policy"' in src
