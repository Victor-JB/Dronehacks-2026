#!/usr/bin/env bash
# Start / stop one flight session on the drone. Run on the drone's host shell (after ssh), not in
# the container:
#
#   ~/team_ws/src/dronehacks/scripts/fly.sh start    bag + foxglove + map + live view, next to the mission
#   ~/team_ws/src/dronehacks/scripts/fly.sh start --depth   the same, and the bag records the depth images too (~4 MB/s)
#   ~/team_ws/src/dronehacks/scripts/fly.sh clean    free disk: old bags past the budget, old colcon build logs
#   ~/team_ws/src/dronehacks/scripts/fly.sh stop     stop them all (bag closed cleanly). ON THE GROUND.
#   ~/team_ws/src/dronehacks/scripts/fly.sh status   what is running, and the mission's last lines
#   ~/team_ws/src/dronehacks/scripts/fly.sh attach   watch it live (Ctrl-b d leaves, Ctrl-b n next window)
#
# Everything runs in a tmux session "dronehacks" inside the skybeta-stack container, next to the
# stack's own "skybeta" session. The stack itself (agent, cuVSLAM, bridge) is started at boot by
# skybeta-stack.service and is never touched here.
#
# The mission itself is started by the stack at power-up (COMPETITOR_PKG=dronehacks_nav in
# /etc/skybeta/drone.env, as the organisers' autonomy_pipeline.md asks): it is already running and
# READY before the pilot arms on Altitude and switches to Position. `start` adds the recording and
# the viewers next to it. It never starts a mission of its own: two flight_interface nodes would
# fight over Offboard.
#
# The mission never arms. The safety pilot arms on Altitude and switches to Position; only that
# makes the drone fly. One go per arming: after the pilot lands and disarms, it is READY again.
set -uo pipefail

C=skybeta-stack
S=dronehacks
BAGS_HOST="${HOME}/team_ws/bags"
BAGS=/root/team_ws/bags                 # the same folder, inside the container
# Bag size (measured 2026-11-07): the depth images were 60% of the bytes and cuVSLAM's vo_path 38%
# (a Path holding the whole trajectory, republished every frame, so it grows the longer the stack
# runs). vo_path is not recorded (odometry has the same poses); depth only with `start --depth`.
# Without depth a bag is ~0.1 MB/s. Before each recording the oldest bags are deleted until all
# of them fit in BAG_BUDGET_GB.
DEPTH_TOPIC=/oak/stereo/image_raw
BAG_BUDGET_GB="${BAG_BUDGET_GB:-15}"
FOXGLOVE_PORT=8765
CAMERA_PORT=8080

# Kept identical to the sim's fly-sim.sh (sim branch) so drone and sim bags compare one to
# one (sim/skybeta/compare.py). Only vehicle_status is versioned (_v1) in px4_msgs release/1.16.
BAG_TOPICS=(
  /visual_slam/tracking/odometry /visual_slam/status
  /oak/imu/data /oak/left/camera_info /oak/stereo/camera_info
  /skybeta/state /skybeta/setpoint/pose /skybeta/setpoint/twist /refnav/status
  /fmu/in/vehicle_visual_odometry /fmu/in/trajectory_setpoint /fmu/in/offboard_control_mode
  /fmu/in/vehicle_command
  /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry /fmu/out/vehicle_attitude
  /fmu/out/vehicle_status_v1 /fmu/out/vehicle_land_detected /fmu/out/vehicle_control_mode
  /fmu/out/vehicle_command_ack /fmu/out/estimator_status_flags /fmu/out/failsafe_flags
  /fmu/out/manual_control_setpoint /fmu/out/sensor_combined /fmu/out/battery_status
  /fmu/out/timesync_status
  /tf /tf_static /rosout
)
FOXGLOVE_TOPICS="['/visual_slam/tracking/odometry','/visual_slam/tracking/vo_path','/map/occupied','/skybeta/state','/skybeta/setpoint/pose','/refnav/status','/fmu/out/vehicle_local_position','/tf','/tf_static','/rosout']"
WINDOWS=(bag foxglove map camera)

die() { echo "[fly] $*" >&2; exit 1; }
dx() { docker exec "${C}" "$@"; }
running() { dx tmux has-session -t "${S}" 2>/dev/null; }

# a tmux window running one command in an interactive shell (it sources the ROS + team overlay)
window() {
  local name="$1" cmd="$2"
  if ! running; then
    dx tmux new-session -d -s "${S}" -n "${name}" -x 200 -y 50
  else
    dx tmux new-window -d -t "${S}" -n "${name}"
  fi
  dx tmux send-keys -t "${S}:${name}" "${cmd}" Enter
}

host_name() { echo "$(hostname).local"; }

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
  running && die "a session is already running. '$0 status' to look, '$0 stop' to end it."
  stack_runs_mission || die "the stack is not running the mission: skybeta:competitor is idle. Check
      /etc/skybeta/drone.env (COMPETITOR_PKG=dronehacks_nav, COMPETITOR_LAUNCH=mission.launch.py) and
      the build: docker exec -it ${C} tmux attach -t skybeta (window competitor)"
  echo "[fly] the stack runs the mission (skybeta:competitor): adding the recording and live view"

  mkdir -p "${BAGS_HOST}" && touch "${BAGS_HOST}/COLCON_IGNORE"
  prune_bags
  local topics=("${BAG_TOPICS[@]}")
  [[ -n "${depth}" ]] && topics+=("${DEPTH_TOPIC}")
  local bag
  bag="${BAGS}/$(date +%Y%m%d-%H%M%S)"
  window bag      "ros2 bag record -s mcap -o ${bag} ${topics[*]}"
  window foxglove "ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=${FOXGLOVE_PORT} topic_whitelist:=\"${FOXGLOVE_TOPICS}\""
  window map      "ros2 run dronehacks_nav map_node"
  window camera   "ros2 run dronehacks_nav camera_web --ros-args -p port:=${CAMERA_PORT}"

  cat <<EOF
[fly] started: ${WINDOWS[*]}
      live view       http://$(host_name):${CAMERA_PORT}/   (position + route, armed/Offboard, mission phase, camera, depth, map)
      foxglove        on the laptop: ssh -N -L ${FOXGLOVE_PORT}:localhost:${FOXGLOVE_PORT} $(whoami)@$(host_name)
                      then connect Foxglove to ws://localhost:${FOXGLOVE_PORT}
      bag             ${BAGS_HOST}/$(basename "${bag}")  ($( [[ -n "${depth}" ]] && echo "with depth, ~4 MB/s" || echo "no depth images: start --depth to add them"))
      watch           $0 attach      (Ctrl-b d to leave; Ctrl-b n for the next window)
      mission log     $0 status, or: docker exec -it ${C} tmux attach -t skybeta (window competitor)
[fly] wait for READY in the mission log, then the pilot arms on Altitude and switches to Position.
EOF
}

stop() {
  running || { echo "[fly] nothing running"; return 0; }
  local wins
  wins="$(dx tmux list-windows -t "${S}" -F '#{window_name}' | tr '\n' ' ')"
  echo "[fly] stopping ${wins}(Ctrl-C to each, so the bag closes cleanly)"
  for w in ${wins}; do dx tmux send-keys -t "${S}:${w}" C-c 2>/dev/null || true; done
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
    echo "[fly] the stack is NOT running the mission (skybeta:competitor idle): check drone.env and the build"
  fi
  running || { echo "[fly] no recording/live-view session ('$0 start')"; return 0; }
  dx tmux list-windows -t "${S}" -F '  #{window_name}: #{pane_current_command}'
}

# free disk on the drone: bags past the budget, and colcon's build logs (one folder per build,
# kept forever) except the last 3. build/ and install/ are overwritten in place by each build, so
# they do not pile up. Docker images left behind by team-image rebuilds are reported, not removed.
clean() {
  docker ps --format '{{.Names}}' | grep -qx "${C}" || die "the stack is not running: clean needs its container"
  prune_bags
  dx bash -c 'cd /root/team_ws/log 2>/dev/null || exit 0
    old=$(ls -1d build_*/ 2>/dev/null | sort | head -n -3)
    [[ -z "$old" ]] && exit 0
    echo "[fly] deleting $(echo "$old" | wc -l) old colcon build logs ($(du -shc $old | tail -1 | cut -f1))"
    rm -rf $old'
  local dangling
  dangling="$(docker images -f dangling=true -q | wc -l)"
  (( dangling > 0 )) && echo "[fly] ${dangling} untagged docker image(s) from old team-image builds: 'docker image prune' frees them (on the ground; check 'docker images' first)"
  df -h "${HOME}" | tail -1 | awk '{print "[fly] disk: " $4 " free of " $2}'
}

case "${1:-}" in
  start) start "${2:-}" ;;
  clean) clean ;;
  stop) stop ;;
  status) status ;;
  attach) exec docker exec -it "${C}" tmux attach -t "${S}" ;;
  *) sed -n '2,23p' "$0"; exit 2 ;;
esac
