#!/usr/bin/env bash
# Future isolated Humble verification only. Does not SSH/install dependencies.
set -euo pipefail
[[ ${ROS_DISTRO:-} == humble ]] || { echo 'Source the approved ROS Humble environment first'; exit 1; }
[[ ${ROS_DOMAIN_ID:-} == 78 && ${ROS_LOCALHOST_ONLY:-} == 1 ]] || { echo 'Require isolated domain78/localhost'; exit 1; }
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${1:?Supply a NEW local evidence directory}"
[[ ! -e "$out" ]] || { echo 'Evidence path already exists; refusing overwrite'; exit 1; }
mkdir -p "$out"
out="$(cd "$out" && pwd)"
for package in dronehacks_nav isaac_ros_visual_slam_interfaces sensor_msgs_py launch_testing_ros; do
  ros2 pkg prefix "$package" >> "$out/dependencies.log"
done
cd "$repo"
colcon --log-base "$out/log" build --base-paths ros2_ws/src --packages-select dh_core dh_autonomy \
  --build-base "$out/build" --install-base "$out/install" > "$out/build.log" 2>&1
source "$out/install/setup.bash"
python3 -m pytest --collect-only -q ros2_ws/src/dh_autonomy/test/test_local_policy_ros.py > "$out/collection.log"
grep -q 'test_native_depth_hold_and_stationary_endpoints' "$out/collection.log"
colcon --log-base "$out/log" test --packages-select dh_autonomy \
  --build-base "$out/build" --install-base "$out/install" --python-testing pytest \
  --return-code-on-test-failure --pytest-args -k local_policy_ros > "$out/test.log" 2>&1
colcon test-result --test-result-base "$out/build" --verbose > "$out/test-result.log"
python3 - "$out" <<'PY'
from pathlib import Path
import sys,xml.etree.ElementTree as ET
out=Path(sys.argv[1]); command=out/'log/latest_test/dh_autonomy/command.log'
if not command.is_file(): raise RuntimeError('Missing actual runner command')
text=command.read_text()
if '-m pytest' not in text or '-m unittest' in text: raise RuntimeError('Wrong actual runner')
cases=[]
for path in (out/'build').rglob('*.xml'):
    try: cases.extend(ET.parse(path).getroot().iter('testcase'))
    except ET.ParseError: pass
required=[c for c in cases if c.get('name')=='test_native_depth_hold_and_stationary_endpoints']
if not required or any(list(c) and any(k.tag in ('skipped','failure','error') for k in c) for c in required):
    raise RuntimeError('Required actual ROS case missing/skipped/failed')
(out/'PASS.txt').write_text('Actual Humble native-depth/startup/graph-isolation case executed and passed. No physical outputs.\n')
PY
