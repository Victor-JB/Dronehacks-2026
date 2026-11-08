#!/usr/bin/env bash
# Recording + live view for box_route, next to the mission. Ported from dronehacks main's
# scripts/fly.sh (same windows, same ports); only the viewer changed (box_route live_view, view only).
# Run on the drone's host shell (after ssh), not in the container:
#
#   ~/team_ws/src/box_route_v2/scripts/fly.sh start           bag + foxglove + live view
#   ~/team_ws/src/box_route_v2/scripts/fly.sh start --depth   the same, plus depth + map (bag records depth,
#                                                          ~4 MB/s; costs Jetson time next to cuVSLAM)
#   ~/team_ws/src/box_route_v2/scripts/fly.sh stop            stop them all (bag closed cleanly). ON THE GROUND.
#   ~/team_ws/src/box_route_v2/scripts/fly.sh status          what is running, and the mission's last lines
#   ~/team_ws/src/box_route_v2/scripts/fly.sh attach          watch it (Ctrl-b d leaves, Ctrl-b n next window)
#   ~/team_ws/src/box_route_v2/scripts/fly.sh clean           free disk: old bags past the budget, old build logs
#
# Everything runs in a tmux session "boxroute" inside the skybeta-stack container. The mission
# itself is started by the stack at power-up (COMPETITOR_PKG=box_route_v2 in /etc/skybeta/drone.env).
# This script never starts a mission and nothing it starts can command the drone.
set -uo pipefail

C=skybeta-stack
S=boxroute
BAGS_HOST="${HOME}/team_ws/bags"
# 2026-11-08: both flight bags were cut off by the power going (no metadata.yaml, files ending on a 4 KB
# block): run 2 lost ~30 s, everything from the bridge to window 2. So the bag is split into closed files
# every BAG_SPLIT_S seconds and the disk is flushed every BAG_SYNC_S seconds. 0 turns either off.
BAG_SPLIT_S="${BAG_SPLIT_S:-10}"
BAG_SYNC_S="${BAG_SYNC_S:-2}"
BAGS=/root/team_ws/bags                 # the same folder, inside the container
DEPTH_TOPIC=/oak/stereo/image_raw
BAG_BUDGET_GB="${BAG_BUDGET_GB:-15}"    # oldest bags deleted until all fit (shared with dronehacks)
FOXGLOVE_PORT=8765
CAMERA_PORT=8080

# dronehacks' list, with /box_route/status in place of /refnav/status. Without depth ~0.1 MB/s.
BAG_TOPICS=(
  /visual_slam/tracking/odometry /visual_slam/status
  /oak/imu/data /oak/left/camera_info /oak/stereo/camera_info
  /skybeta/state /skybeta/setpoint/pose /skybeta/setpoint/twist /box_route/status
  /fmu/in/vehicle_visual_odometry /fmu/in/trajectory_setpoint /fmu/in/offboard_control_mode
  /fmu/in/vehicle_command
  /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry /fmu/out/vehicle_attitude
  /fmu/out/vehicle_status_v1 /fmu/out/vehicle_land_detected /fmu/out/vehicle_control_mode
  /fmu/out/vehicle_command_ack /fmu/out/estimator_status_flags /fmu/out/failsafe_flags
  /fmu/out/manual_control_setpoint /fmu/out/sensor_combined /fmu/out/battery_status
  /fmu/out/timesync_status
  /tf /tf_static /rosout
)
FOXGLOVE_TOPICS="['/visual_slam/tracking/odometry','/visual_slam/tracking/vo_path','/map/occupied','/skybeta/state','/skybeta/setpoint/pose','/box_route/status','/fmu/out/vehicle_local_position','/tf','/tf_static','/rosout']"

die() { echo "[fly] $*" >&2; exit 1; }
dx() { docker exec "${C}" "$@"; }
running() { dx tmux has-session -t "${S}" 2>/dev/null; }

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

stack_runs_mission() {
  local busy
  busy="$(dx tmux list-panes -t skybeta:competitor -F '#{pane_current_command}' 2>/dev/null || true)"
  [[ -n "${busy}" && "${busy}" != "bash" ]]
}

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
  stack_runs_mission || die "the stack is not running a mission: skybeta:competitor is idle. Check
      /etc/skybeta/drone.env (COMPETITOR_PKG=box_route_v2, COMPETITOR_LAUNCH=box_mission.launch.py) and
      the build: docker exec -it ${C} tmux attach -t skybeta (window competitor)"
  grep -q '^COMPETITOR_PKG=box_route_v2' /etc/skybeta/drone.env 2>/dev/null \
    || echo "[fly] note: drone.env does not say COMPETITOR_PKG=box_route_v2: the stack is flying another package"

  mkdir -p "${BAGS_HOST}" && touch "${BAGS_HOST}/COLCON_IGNORE"
  prune_bags
  local topics=("${BAG_TOPICS[@]}")
  [[ -n "${depth}" ]] && topics+=("${DEPTH_TOPIC}")
  local bag
  bag="${BAGS}/$(date +%Y%m%d-%H%M%S)"
  local wins=(bag foxglove live)
  local split=""; [[ "${BAG_SPLIT_S}" != "0" ]] && split="--max-bag-duration ${BAG_SPLIT_S}"
  window bag      "ros2 bag record -s mcap ${split} -o ${bag} ${topics[*]}"
  [[ "${BAG_SYNC_S}" != "0" ]] && window bagsync "while sleep ${BAG_SYNC_S}; do sync; done"
  window foxglove "ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=${FOXGLOVE_PORT} topic_whitelist:=\"${FOXGLOVE_TOPICS}\""
  if [[ -n "${depth}" ]]; then
    window map    "ros2 run dronehacks_nav map_node"      # occupancy map from depth (dronehacks package)
    wins+=(map)
  fi
  window live     "ros2 run box_route_v2 live_view --ros-args -p port:=${CAMERA_PORT} -p enable_depth:=$( [[ -n "${depth}" ]] && echo true || echo false )"

  cat <<EOF2
[fly] started: ${wins[*]}
      live view       http://$(host_name):${CAMERA_PORT}/      (view only: no buttons)
      foxglove        on the laptop: ssh -N -L ${FOXGLOVE_PORT}:localhost:${FOXGLOVE_PORT} $(whoami)@$(host_name)
                      then connect Foxglove to ws://localhost:${FOXGLOVE_PORT}
      bag             ${BAGS_HOST}/$(basename "${bag}")  ($( [[ -n "${depth}" ]] && echo "with depth, ~4 MB/s" || echo "no depth: start --depth to add depth + map"))
      watch           $0 attach      (Ctrl-b d to leave; Ctrl-b n for the next window)
      mission log     $0 status, or: docker exec -it ${C} tmux attach -t skybeta (window competitor)
[fly] READY on the page -> pilot arms on Altitude, switches to Position -> the mission flies and lands.
EOF2
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
    echo "[fly] the stack is NOT running a mission (skybeta:competitor idle): check drone.env and the build"
  fi
  running || { echo "[fly] no recording/live-view session ('$0 start')"; return 0; }
  dx tmux list-windows -t "${S}" -F '  #{window_name}: #{pane_current_command}'
}

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
