#!/usr/bin/env bash
# Flight recording for test_files. Taken from ~/team_ws/src/dronehacks/scripts/fly.sh, keeping
# only the bag recording (the dronehacks live view, map and foxglove windows need dronehacks_nav).
# Run on the drone's host shell (after ssh), not in the container:
#
#   ~/team_ws/src/testFiles/scripts/fly.sh start           start recording a bag, next to the mission
#   ~/team_ws/src/testFiles/scripts/fly.sh start --depth   the same, plus depth images (~4 MB/s)
#   ~/team_ws/src/testFiles/scripts/fly.sh stop            stop recording (bag closed cleanly). ON THE GROUND.
#   ~/team_ws/src/testFiles/scripts/fly.sh status          what is running, and the mission's last lines
#   ~/team_ws/src/testFiles/scripts/fly.sh attach          watch the recorder (Ctrl-b d leaves)
#   ~/team_ws/src/testFiles/scripts/fly.sh clean           free disk: old bags past the budget, old build logs
#
# The recorder runs in a tmux session "testfiles" inside the skybeta-stack container, next to the
# stack's own "skybeta" session. The mission itself is started by the stack at power-up
# (COMPETITOR_PKG=test_files, COMPETITOR_LAUNCH=my_mission.launch.py in /etc/skybeta/drone.env).
# This script never starts a mission: two flight_interface nodes would fight over Offboard.
set -uo pipefail

C=skybeta-stack
S=testfiles
BAGS_HOST="${HOME}/team_ws/bags"
BAGS=/root/team_ws/bags                 # the same folder, inside the container
DEPTH_TOPIC=/oak/stereo/image_raw
BAG_BUDGET_GB="${BAG_BUDGET_GB:-15}"    # oldest bags deleted until all fit (shared with dronehacks)

# Same topics as dronehacks' fly.sh, minus /refnav/status (dronehacks only). Without depth ~0.1 MB/s.
BAG_TOPICS=(
  /visual_slam/tracking/odometry /visual_slam/status
  /oak/imu/data /oak/left/camera_info /oak/stereo/camera_info
  /skybeta/state /skybeta/setpoint/pose /skybeta/setpoint/twist
  /fmu/in/vehicle_visual_odometry /fmu/in/trajectory_setpoint /fmu/in/offboard_control_mode
  /fmu/in/vehicle_command
  /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry /fmu/out/vehicle_attitude
  /fmu/out/vehicle_status_v1 /fmu/out/vehicle_land_detected /fmu/out/vehicle_control_mode
  /fmu/out/vehicle_command_ack /fmu/out/estimator_status_flags /fmu/out/failsafe_flags
  /fmu/out/manual_control_setpoint /fmu/out/sensor_combined /fmu/out/battery_status
  /fmu/out/timesync_status
  /tf /tf_static /rosout
)

die() { echo "[fly] $*" >&2; exit 1; }
dx() { docker exec "${C}" "$@"; }
running() { dx tmux has-session -t "${S}" 2>/dev/null; }

# the stack's competitor window is running something (our launch file, at power-up), not a bare shell
stack_runs_mission() {
  local busy
  busy="$(dx tmux list-panes -t skybeta:competitor -F '#{pane_current_command}' 2>/dev/null || true)"
  [[ -n "${busy}" && "${busy}" != "bash" ]]
}

# delete the oldest bags until the rest fit in BAG_BUDGET_GB (the newest one is always kept). Bags are
# written by the container's root, so the deleting happens in the container.
prune_bags() {
  dx bash -c "cd ${BAGS} 2>/dev/null || exit 0
    budget=\$(( ${BAG_BUDGET_GB} * 1024 * 1024 ))
    for d in \$(ls -1d 20*/ 2>/dev/null | sort | head -n -1); do
      total=\$(du -sk . | cut -f1)
      (( total > budget )) || break
      echo \"[fly] bag budget ${BAG_BUDGET_GB} GB: deleting the oldest bag, \${d%/} (\$(du -sh \$d | cut -f1))\"
      rm -rf \"\$d\"
    done"
}

start() {
  local depth=""
  [[ "${1:-}" == --depth ]] && depth=1
  docker ps --format '{{.Names}}' | grep -qx "${C}" \
    || die "the stack is not running (sudo systemctl status skybeta-stack). Power on and wait ~1 min."
  running && die "a recording is already running. '$0 status' to look, '$0 stop' to end it."
  stack_runs_mission || die "the stack is not running a mission: skybeta:competitor is idle. Check
      /etc/skybeta/drone.env (COMPETITOR_PKG=test_files, COMPETITOR_LAUNCH=my_mission.launch.py) and
      the build: docker exec -it ${C} tmux attach -t skybeta (window competitor)"
  grep -q '^COMPETITOR_PKG=test_files' /etc/skybeta/drone.env 2>/dev/null \
    || echo "[fly] note: drone.env does not say COMPETITOR_PKG=test_files: the stack is flying another package"

  mkdir -p "${BAGS_HOST}" && touch "${BAGS_HOST}/COLCON_IGNORE"
  prune_bags
  local topics=("${BAG_TOPICS[@]}")
  [[ -n "${depth}" ]] && topics+=("${DEPTH_TOPIC}")
  local bag
  bag="${BAGS}/$(date +%Y%m%d-%H%M%S)"
  dx tmux new-session -d -s "${S}" -n bag -x 200 -y 50
  dx tmux send-keys -t "${S}:bag" "ros2 bag record -s mcap -o ${bag} ${topics[*]}" Enter

  cat <<EOF
[fly] recording: ${BAGS_HOST}/$(basename "${bag}")  ($( [[ -n "${depth}" ]] && echo "with depth, ~4 MB/s" || echo "no depth images: start --depth to add them"))
      watch        $0 attach      (Ctrl-b d to leave)
      mission log  $0 status, or: docker exec -it ${C} tmux attach -t skybeta (window competitor)
      stop         $0 stop        (on the ground, so the bag closes cleanly)
EOF
}

stop() {
  running || { echo "[fly] nothing running"; return 0; }
  echo "[fly] stopping the recording (Ctrl-C, so the bag closes cleanly)"
  dx tmux send-keys -t "${S}:bag" C-c 2>/dev/null || true
  for _ in $(seq 1 20); do
    local busy
    busy="$(dx tmux list-panes -s -t "${S}" -F '#{pane_current_command}' 2>/dev/null | grep -vcx bash || true)"
    [[ "${busy}" == "0" ]] && break
    sleep 0.5
  done
  dx tmux kill-session -t "${S}" 2>/dev/null || true
  echo "[fly] stopped. Bags: ${BAGS_HOST}"
}

status() {
  if stack_runs_mission; then
    echo "--- mission (skybeta:competitor, last lines)"
    dx tmux capture-pane -p -J -t skybeta:competitor | sed '/^[[:space:]]*$/d' | tail -8
  else
    echo "[fly] the stack is NOT running a mission (skybeta:competitor idle): check drone.env and the build"
  fi
  running || { echo "[fly] not recording ('$0 start')"; return 0; }
  dx tmux list-windows -t "${S}" -F '  #{window_name}: #{pane_current_command}'
}

# free disk: bags past the budget, and colcon's build logs except the last 3.
clean() {
  docker ps --format '{{.Names}}' | grep -qx "${C}" || die "the stack is not running: clean needs its container"
  prune_bags
  dx bash -c 'cd /root/team_ws/log 2>/dev/null || exit 0
    old=$(ls -1d build_*/ 2>/dev/null | sort | head -n -3)
    [[ -z "$old" ]] && exit 0
    echo "[fly] deleting $(echo "$old" | wc -l) old colcon build logs ($(du -shc $old | tail -1 | cut -f1))"
    rm -rf $old'
  df -h "${HOME}" | tail -1 | awk '{print "[fly] disk: " $4 " free of " $2}'
}

case "${1:-}" in
  start) start "${2:-}" ;;
  clean) clean ;;
  stop) stop ;;
  status) status ;;
  attach) exec docker exec -it "${C}" tmux attach -t "${S}" ;;
  *) sed -n '2,17p' "$0"; exit 2 ;;
esac
