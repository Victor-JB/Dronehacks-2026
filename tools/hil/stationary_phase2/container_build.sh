#!/bin/bash
# PROPOSED PHASE 2 ONLY. No deployment permission is conveyed by this file.
# Run exclusively in the separately approved, inspected network-none container.
set -eo pipefail
test "${1:-}" = --approved-isolated-build
test "$ROS_DOMAIN_ID" = 77
test "$ROS_LOCALHOST_ONLY" = 1
test "$(ls /sys/class/net)" = lo
test "$(id -u)" != 0
test -r /opt/ros/humble/setup.bash
test -r /reviewed/course.obj
cd /audit_ws
mkdir -p evidence home
exec > >(tee evidence/session.log) 2>&1
date -u
echo 'ISOLATED SYNTHETIC BUILD ONLY; NO LIVE SENSORS'
source /opt/ros/humble/setup.bash
test "$ROS_DISTRO" = humble
command -v colcon
python3 - <<'PY'
import importlib
for name in ('numpy', 'cv2', 'rclpy', 'message_filters', 'tf2_ros', 'std_srvs',
             'sensor_msgs', 'nav_msgs', 'geometry_msgs', 'builtin_interfaces',
             'ament_index_python', 'launch', 'launch_ros', 'pytest',
             'launch_testing', 'launch_testing_ros'):
    m = importlib.import_module(name)
    print(name, getattr(m, '__version__', 'import OK'))
PY
colcon list --base-paths /reviewed/ros2_ws/src
test "$(colcon list --base-paths /reviewed/ros2_ws/src --names-only | sort)" = $'dh_autonomy\ndh_core'
colcon build --base-paths /reviewed/ros2_ws/src --packages-select dh_core dh_autonomy \
  --executor sequential --event-handlers console_direct+
source install/setup.bash
python3 - <<'PY'
from dh_autonomy.mission_node import StationaryMissionNode, stationary_main
from dh_autonomy.backends import DryRunBackend
import dh_core.camera_geom, dh_autonomy.stereo_scan_node, dh_autonomy.line_node
if StationaryMissionNode.force_dry_run is not True:
    raise RuntimeError('Stationary hard lock missing')
print('Installed imports OK; no node instantiated by import')
PY
export DH_COURSE_OBJ=/reviewed/course.obj
python3 -m pytest --collect-only -q --color=no \
  /reviewed/ros2_ws/src/dh_autonomy/test/test_ros_graph_dry_run.py \
  /reviewed/ros2_ws/src/dh_autonomy/test/test_ros_perception_dry_run.py > evidence/collection.log 2>&1
cat evidence/collection.log
python3 /reviewed/check_test_evidence.py --stage collection
# Tests execute sequentially: synthetic camera, TF, odometry and /dh/start only.
timeout --signal=INT --kill-after=10s 300s colcon test \
  --base-paths /reviewed/ros2_ws/src --packages-select dh_core dh_autonomy \
  --executor sequential --event-handlers console_direct+ --python-testing pytest \
  --return-code-on-test-failure --pytest-args -v
colcon test-result --verbose | tee evidence/test-results.txt
python3 /reviewed/check_test_evidence.py --stage tests
# Also exercise the dedicated installed stationary executable with hostile parameters.
# No /dh/start is sent here; this is a constructor/graph check in the sandbox only.
mission_pid=
cleanup() {
  if test -n "$mission_pid"; then
    kill -INT -- "-$mission_pid" 2>/dev/null || true
    for count in 1 2 3 4 5; do
      kill -0 "$mission_pid" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-$mission_pid" 2>/dev/null || true
    kill -KILL -- "-$mission_pid" 2>/dev/null || true
    wait "$mission_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
setsid ros2 run dh_autonomy stationary_mission --ros-args \
  -p course_obj:=/reviewed/course.obj -p dry_run:=false -p backend:=px4 \
  -p odom_topic:=/dh_test/odom -p vo_status_topic:=/dh_test/vo \
  > evidence/stationary-constructor.log 2>&1 &
mission_pid=$!
passed=false
for attempt in 1 2 3 4 5 6; do
  kill -0 "$mission_pid"
  set +e
  python3 /reviewed/tools/hil/check_command_isolation.py --wait 8 \
    > "evidence/isolation-${attempt}.log" 2>&1
  rc=$?
  set -e
  cat "evidence/isolation-${attempt}.log"
  if test "$rc" = 0; then passed=true; break; fi
  test "$rc" = 2  # Incomplete discovery can be retried; forbidden/error cannot.
done
test "$passed" = true
cp "evidence/isolation-${attempt}.log" evidence/isolation-final.log
printf '{"executed":true,"returncode":0}\n' > evidence/isolation-result.json
python3 /reviewed/check_test_evidence.py --stage complete
cleanup
mission_pid=
echo 'PHASE 2 SYNTHETIC CHECKS PASSED; STOP BEFORE LIVE SENSOR STARTUP'
