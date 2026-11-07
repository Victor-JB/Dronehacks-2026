#!/usr/bin/env bash
# Start / stop one flight session on the drone. Run on the drone's host shell (after ssh), not in
# the container:
#
#   ~/team_ws/src/dronehacks/scripts/fly.sh start    build, then mission + bag + foxglove + map + camera page
#   ~/team_ws/src/dronehacks/scripts/fly.sh stop     stop them all (bag closed cleanly). ON THE GROUND.
#   ~/team_ws/src/dronehacks/scripts/fly.sh status   what is running, and the mission's last lines
#   ~/team_ws/src/dronehacks/scripts/fly.sh attach   watch it live (Ctrl-b d leaves, Ctrl-b n next window)
#
# Everything runs in a tmux session "dronehacks" inside the skybeta-stack container, next to the
# stack's own "skybeta" session. The stack itself (agent, cuVSLAM, bridge) is started at boot by
# skybeta-stack.service and is never touched here.
#
# The mission never arms. After `start` it waits until it is READY; then the safety pilot arms on
# Altitude and switches to Position, and only that makes the drone fly.
set -uo pipefail

C=skybeta-stack
S=dronehacks
BAGS_HOST="${HOME}/team_ws/bags"
BAGS=/root/team_ws/bags                 # the same folder, inside the container
FOXGLOVE_PORT=8765
CAMERA_PORT=8080

# Kept identical to the sim's fly-sim-drone.sh (victor/sim) so drone and sim bags compare one to
# one (sim/skybeta/compare.py). Only vehicle_status is versioned (_v1) in px4_msgs release/1.16.
BAG_TOPICS=(
  /visual_slam/tracking/odometry /visual_slam/status /visual_slam/tracking/vo_path
  /oak/imu/data /oak/left/camera_info /oak/stereo/camera_info /oak/stereo/image_raw
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
WINDOWS=(mission bag foxglove map camera)

die() { echo "[fly] $*" >&2; exit 1; }
dx() { docker exec "${C}" "$@"; }
running() { dx tmux has-session -t "${S}" 2>/dev/null; }

# a tmux window running one command in an interactive shell (it sources the ROS + team overlay)
window() {
  local name="$1" cmd="$2"
  if [[ "${name}" == "${WINDOWS[0]}" ]]; then
    dx tmux new-session -d -s "${S}" -n "${name}" -x 200 -y 50
  else
    dx tmux new-window -d -t "${S}" -n "${name}"
  fi
  dx tmux send-keys -t "${S}:${name}" "${cmd}" Enter
}

host_name() { echo "$(hostname).local"; }

start() {
  docker ps --format '{{.Names}}' | grep -qx "${C}" \
    || die "the stack is not running (sudo systemctl status skybeta-stack). Power on and wait ~1 min."
  running && die "a session is already running. '$0 status' to look, '$0 stop' to end it."
  # the stack's own competitor window must be idle: two flight_interface nodes fight over Offboard
  local busy
  busy="$(dx tmux list-panes -t skybeta:competitor -F '#{pane_current_command}' 2>/dev/null || true)"
  [[ -z "${busy}" || "${busy}" == "bash" ]] \
    || die "skybeta:competitor is running '${busy}' (set COMPETITOR_PKG=manual in /etc/skybeta/drone.env). Stop it first."

  echo "[fly] building dronehacks_nav"
  dx bash -ic "team_build.sh --packages-select dronehacks_nav" >/dev/null 2>&1 \
    || die "build failed: docker exec -it ${C} bash -ic 'team_build.sh --packages-select dronehacks_nav'"

  mkdir -p "${BAGS_HOST}" && touch "${BAGS_HOST}/COLCON_IGNORE"
  local bag
  bag="${BAGS}/$(date +%Y%m%d-%H%M%S)"
  window mission  "ros2 launch dronehacks_nav mission.launch.py"
  window bag      "ros2 bag record -s mcap -o ${bag} ${BAG_TOPICS[*]}"
  window foxglove "ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=${FOXGLOVE_PORT} topic_whitelist:=\"${FOXGLOVE_TOPICS}\""
  window map      "ros2 run dronehacks_nav map_node"
  window camera   "ros2 run dronehacks_nav camera_web --ros-args -p port:=${CAMERA_PORT}"

  cat <<EOF
[fly] started: ${WINDOWS[*]}
      camera + map    http://$(host_name):${CAMERA_PORT}/   (map also in Foxglove: /map/occupied, frame odom)
      foxglove        on the laptop: ssh -N -L ${FOXGLOVE_PORT}:localhost:${FOXGLOVE_PORT} $(whoami)@$(host_name)
                      then connect Foxglove to ws://localhost:${FOXGLOVE_PORT}
      bag             ${BAGS_HOST}/$(basename "${bag}")
      watch           $0 attach      (Ctrl-b d to leave; Ctrl-b n for the next window)
[fly] wait for READY in the mission log, then the pilot arms on Altitude and switches to Position.
EOF
}

stop() {
  running || { echo "[fly] nothing running"; return 0; }
  echo "[fly] stopping ${WINDOWS[*]} (Ctrl-C to each, so the bag closes cleanly)"
  for w in "${WINDOWS[@]}"; do dx tmux send-keys -t "${S}:${w}" C-c 2>/dev/null || true; done
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
  running || { echo "[fly] nothing running"; return 0; }
  dx tmux list-windows -t "${S}" -F '  #{window_name}: #{pane_current_command}'
  echo "--- mission (last lines)"
  dx tmux capture-pane -p -J -t "${S}:mission" | sed '/^[[:space:]]*$/d' | tail -8
}

case "${1:-}" in
  start) start ;;
  stop) stop ;;
  status) status ;;
  attach) exec docker exec -it "${C}" tmux attach -t "${S}" ;;
  *) sed -n '2,15p' "$0"; exit 2 ;;
esac
