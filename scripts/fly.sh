#!/usr/bin/env bash
# Start / stop one flight session on the drone. Run on the drone's host shell (after ssh), not in
# the container:
#
#   ~/team_ws/src/dronehacks/scripts/fly.sh start    bag + foxglove + map + camera page (+ the mission, bench mode only)
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
# the viewers next to it. Bench mode: with COMPETITOR_PKG=manual the stack runs nothing, and `start`
# builds and launches the mission too (gone at the next stack restart).
#
# The mission never arms. The safety pilot arms on Altitude and switches to Position; only that
# makes the drone fly. One go per arming: after the pilot lands and disarms, it is READY again.
set -uo pipefail

C=skybeta-stack
S=dronehacks
BAGS_HOST="${HOME}/team_ws/bags"
BAGS=/root/team_ws/bags                 # the same folder, inside the container
FOXGLOVE_PORT=8765
CAMERA_PORT=8080

# Kept identical to the sim's fly-sim.sh (sim branch) so drone and sim bags compare one to
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
WINDOWS=(bag foxglove map camera)        # plus "mission" first, in bench mode

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

start() {
  docker ps --format '{{.Names}}' | grep -qx "${C}" \
    || die "the stack is not running (sudo systemctl status skybeta-stack). Power on and wait ~1 min."
  running && die "a session is already running. '$0 status' to look, '$0 stop' to end it."
  # Who runs the mission: the stack's competitor window (normal), or this session (bench mode).
  # Never both: two flight_interface nodes fight over Offboard.
  local bench=""
  if stack_runs_mission; then
    echo "[fly] the stack runs the mission (skybeta:competitor): adding the recording and viewers"
  else
    bench=1
    echo "[fly] BENCH MODE: the stack's competitor window is idle (COMPETITOR_PKG=manual?), so this"
    echo "      session runs the mission too. It dies with the stack: for a real run, set"
    echo "      COMPETITOR_PKG=dronehacks_nav in /etc/skybeta/drone.env (README.md, Deploying)."
    echo "[fly] building dronehacks_nav"
    dx bash -ic "team_build.sh --packages-select dronehacks_nav" >/dev/null 2>&1 \
      || die "build failed: docker exec -it ${C} bash -ic 'team_build.sh --packages-select dronehacks_nav'"
  fi

  mkdir -p "${BAGS_HOST}" && touch "${BAGS_HOST}/COLCON_IGNORE"
  local bag
  bag="${BAGS}/$(date +%Y%m%d-%H%M%S)"
  [[ -n "${bench}" ]] && window mission "ros2 launch dronehacks_nav mission.launch.py"
  window bag      "ros2 bag record -s mcap -o ${bag} ${BAG_TOPICS[*]}"
  window foxglove "ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=${FOXGLOVE_PORT} topic_whitelist:=\"${FOXGLOVE_TOPICS}\""
  window map      "ros2 run dronehacks_nav map_node"
  window camera   "ros2 run dronehacks_nav camera_web --ros-args -p port:=${CAMERA_PORT}"

  cat <<EOF
[fly] started: ${bench:+mission }${WINDOWS[*]}
      camera + map    http://$(host_name):${CAMERA_PORT}/   (map also in Foxglove: /map/occupied, frame odom)
      foxglove        on the laptop: ssh -N -L ${FOXGLOVE_PORT}:localhost:${FOXGLOVE_PORT} $(whoami)@$(host_name)
                      then connect Foxglove to ws://localhost:${FOXGLOVE_PORT}
      bag             ${BAGS_HOST}/$(basename "${bag}")
      watch           $0 attach      (Ctrl-b d to leave; Ctrl-b n for the next window)
      mission log     $( [[ -n "${bench}" ]] && echo "$0 attach (window mission)" || echo "$0 status, or: docker exec -it ${C} tmux attach -t skybeta (window competitor)")
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
    echo "[fly] mission: run by the stack (skybeta:competitor)"
    echo "--- mission (last lines)"
    dx tmux capture-pane -p -J -t skybeta:competitor | sed '/^[[:space:]]*$/d' | tail -8
  fi
  running || { echo "[fly] no recording/viewer session"; return 0; }
  dx tmux list-windows -t "${S}" -F '  #{window_name}: #{pane_current_command}'
  if dx tmux list-windows -t "${S}" -F '#{window_name}' | grep -qx mission; then
    echo "--- mission, bench mode (last lines)"
    dx tmux capture-pane -p -J -t "${S}:mission" | sed '/^[[:space:]]*$/d' | tail -8
  fi
}

case "${1:-}" in
  start) start ;;
  stop) stop ;;
  status) status ;;
  attach) exec docker exec -it "${C}" tmux attach -t "${S}" ;;
  *) sed -n '2,21p' "$0"; exit 2 ;;
esac
