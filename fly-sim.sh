#!/usr/bin/env bash
# The sim: fly the drone's own code in the Gazebo course, the way the drone runs it. One command.
#
#   ./fly-sim.sh                    fly `main` (what the drone flies)
#   ./fly-sim.sh --code <ref|dir>   another branch/commit, or a folder holding a dronehacks_nav package
#   ./fly-sim.sh --mode <N>         fly modes.json mode N instead of the code's active mode
#   ./fly-sim.sh --headless         no Gazebo window (faster)
#   ./fly-sim.sh stop               stop the sim and its Linux VM (frees the Mac's CPU and memory)
#
# First run builds the images (~45 min, once). Ctrl-C stops a run; the bag is closed first.
# Run files: sim/artifacts/skybeta/<run>/. How it works and why it matches the drone: README.md.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$REPO/sim"
CTX=colima-dronehacks-sim
C=dronehacks-skybeta-sim-1
IMAGE=dronehacks-skybeta:local
COMPOSE=(docker --context "$CTX" compose -f "$SIM/compose.skybeta.yaml")

# The same list as the drone's scripts/fly.sh (main), so sim and drone bags compare one to one,
# plus the sim's ground truth.
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
  /sim/truth/odometry
)

die() { echo "[fly-sim] $*" >&2; exit 1; }
say() { echo "[fly-sim] $*"; }
ros() { docker --context "$CTX" exec -w /workspace "$C" bash -c "source skybeta/env.sh; $*"; }
# SIGINT every container process whose command line contains one of the given words (no procps there)
sigint() {
  local words; words=$(printf "b'%s'," "$@")
  docker --context "$CTX" exec "$C" python3 -c "import os,signal
for p in filter(str.isdigit, os.listdir('/proc')):
    try: c = open(f'/proc/{p}/cmdline','rb').read()
    except OSError: continue
    if any(k in c or k.replace(b' ', bytes(1)) in c for k in ($words)): os.kill(int(p), signal.SIGINT)" >/dev/null 2>&1
}

# ---- arguments
code=main
mode=
export SIM_GUI=1
if [[ "${1:-}" == stop ]]; then
  docker --context "$CTX" info >/dev/null 2>&1 && "${COMPOSE[@]}" down
  colima stop --profile dronehacks-sim
  exit 0
fi
while (( $# )); do
  case "$1" in
    --code) code="${2:?--code needs a branch, commit or folder}"; shift 2 ;;
    --mode) mode="${2:?--mode needs a modes.json mode number}"; shift 2 ;;
    --headless) export SIM_GUI=0; shift ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

# ---- 1. VM: as many CPUs as the Mac can spare
cpus=$(( $(sysctl -n hw.ncpu) - 2 )); (( cpus > 10 )) && cpus=10; (( cpus < 4 )) && cpus=4
mem=$(( $(sysctl -n hw.memsize) / 1073741824 / 2 )); (( mem > 8 )) && mem=8; (( mem < 6 )) && mem=6
row="$(colima list --json 2>/dev/null | grep '"dronehacks-sim"' || true)"
have_cpus="$(sed -nE 's/.*"cpus":([0-9]+).*/\1/p' <<<"$row")"
if [[ "$row" == *'"Running"'* && "${have_cpus:-0}" -lt "$cpus" ]]; then
  say "restarting the VM with $cpus CPUs / ${mem} GB (had ${have_cpus})"
  colima stop --profile dronehacks-sim >/dev/null; row=""
fi
if [[ "$row" != *'"Running"'* ]]; then
  say "starting the VM ($cpus CPUs, ${mem} GB)"
  colima start --profile dronehacks-sim --activate=false --arch aarch64 --vm-type vz \
    --cpu "$cpus" --memory "$mem" --runtime docker >/dev/null || die "colima failed to start"
fi

# ---- 2. images, first run only: Gazebo -> + PX4 v1.16.2 SITL -> + ROS 2, agent, px4_msgs, skybeta
have() { docker --context "$CTX" image inspect "$1" >/dev/null 2>&1; }
if ! have dronehacks-sim:local; then
  say "building the Gazebo image (first run)"
  docker --context "$CTX" build -t dronehacks-sim:local "$SIM" || die "Gazebo image build failed"
fi
if ! have dronehacks-px4:local; then
  say "building PX4 v1.16.2 SITL (first run, ~20-40 min)"
  # this builder reads .dockerignore only: use the PX4 one for this build
  cp "$SIM/.dockerignore" "$SIM/.dockerignore.main"
  cp "$SIM/Dockerfile.px4.dockerignore" "$SIM/.dockerignore"
  docker --context "$CTX" build -f "$SIM/Dockerfile.px4" -t dronehacks-px4:local "$SIM"; built=$?
  mv -f "$SIM/.dockerignore.main" "$SIM/.dockerignore"
  (( built == 0 )) || die "PX4 image build failed"
fi
if ! have "$IMAGE"; then
  say "building the drone-stack image (first run, ~20-30 min)"
  docker --context "$CTX" build -t "$IMAGE" "$SIM/skybeta" || die "drone-stack image build failed"
fi
( cd "$SIM" && python3 skybeta/make_model.py >/dev/null ) || die "airframe model failed (sim/skybeta/make_model.py)"

# ---- 3. the code to fly, copied into the run folder (the container sees sim/ only)
name="$(basename "$code" | tr -c 'A-Za-z0-9._-\n' '_')"
run="${name}-$(date +%Y%m%d-%H%M%S)"
out="$SIM/artifacts/skybeta/$run"
mkdir -p "$out/code"
if [[ -d "$code" ]]; then
  (cd "$code" && tar --exclude=.git --exclude=__pycache__ --exclude=sim -cf - .) | tar -xf - -C "$out/code"
  src="folder $(cd "$code" && pwd)"
  commit=""                               # a folder may hold uncommitted work: it cannot be promoted
else
  git -C "$REPO" rev-parse --verify -q "$code^{commit}" >/dev/null || die "no branch, commit or folder '$code'"
  git -C "$REPO" archive "$code" | tar -xf - -C "$out/code" --exclude=sim
  src="$code @ $(git -C "$REPO" rev-parse --short "$code")"
  commit="$(git -C "$REPO" rev-parse "$code^{commit}")"
fi
# ---- which modes.json mode to fly: the code's active one, or --mode (route.json is rewritten here,
# in the run's copy only, exactly as apply_mode.py does it on the drone)
if [[ -f "$out/code/modes.json" && -f "$out/code/apply_mode.py" ]]; then
  if [[ -n "$mode" ]]; then
    ( cd "$out/code" && python3 - "$mode" <<'PY'
import json, sys
m = json.load(open("modes.json"))
if sys.argv[1] not in m["modes"]:
    names = ", ".join(k + " " + v["name"] for k, v in m["modes"].items())
    sys.exit("[fly-sim] no mode %s in modes.json: %s" % (sys.argv[1], names))
m["active_mode"] = int(sys.argv[1])
json.dump(m, open("modes.json", "w"), indent=2)
PY
    ) || exit 1
  fi
  ( cd "$out/code" && python3 apply_mode.py >/dev/null ) || die "apply_mode.py failed in $out/code"
  mode="$(python3 -c "import json; m = json.load(open('$out/code/modes.json')); print(m['active_mode'], m['modes'][str(m['active_mode'])]['name'])")"
  src="$src, mode $mode"
elif [[ -n "$mode" ]]; then
  die "$src has no modes.json, so --mode does not apply"
fi
[[ -f "$out/code/package.xml" ]] && grep -q '<name>dronehacks_nav</name>' "$out/code/package.xml" \
  || die "$src has no dronehacks_nav package at its root"
params="$out/code/config/drone.sim.yaml"
[[ -f "$out/code/config/drone.yaml" ]] || die "$src has no config/drone.yaml"
# The only change to the team's config: PX4 SITL runs on Gazebo's clock, so the nodes must too.
# Also: the drone's node never exits (a fresh mission per arming); the sim's run ends with the mission.
sed -e 's/use_sim_time:[[:space:]]*false/use_sim_time: true/' -e 's/exit_when_done:[[:space:]]*false/exit_when_done: true/' \
  "$out/code/config/drone.yaml" > "$params"
if ! grep -q 'use_sim_time: true' "$params"; then
  awk '{print} /ros__parameters:/ && !done {print "    use_sim_time: true"; done=1}' "$params" > "$params.tmp" && mv "$params.tmp" "$params"
fi
ln -sfn "$run" "$SIM/artifacts/skybeta/current"
cbase="/workspace/artifacts/skybeta/$run"

# ---- 4. fresh sim (PX4 and the odometry origin do not carry between runs)
say "starting Gazebo + PX4 + the drone stack (GUI ${SIM_GUI})"
"${COMPOSE[@]}" up --no-build -d --force-recreate >/dev/null 2>&1 || die "sim failed to start: ${COMPOSE[*]} logs"
docker --context "$CTX" logs -f "$C" > "$out/stack.log" 2>&1 &
stacklog=$!

mission="" bag="" pages=() tailer=""
cleanup() {
  trap - INT TERM EXIT
  echo; say "stopping"
  sigint session.launch.py camera_web map_node foxglove_bridge
  sleep 3
  sigint 'bag record'
  for _ in $(seq 1 20); do [[ -n "$bag" ]] && kill -0 "$bag" 2>/dev/null || break; sleep 0.5; done
  kill $mission $bag ${pages[@]+"${pages[@]}"} $tailer $stacklog 2>/dev/null
  "${COMPOSE[@]}" stop >/dev/null 2>&1
  say "sim stopped. Run files: $out"
  exit 0
}
trap cleanup INT TERM EXIT

say "waiting for PX4 over uXRCE-DDS, depth and odometry (first start takes a minute or two)"
ready=0
for _ in $(seq 1 60); do
  if ros "timeout 8 ros2 topic echo --once /fmu/out/vehicle_status_v1 >/dev/null 2>&1 &&
          timeout 8 ros2 topic echo --once --field header /oak/stereo/image_raw >/dev/null 2>&1 &&
          timeout 8 ros2 topic echo --once --field header /visual_slam/tracking/odometry >/dev/null 2>&1"; then
    ready=1; break
  fi
  sleep 3
done
(( ready )) || die "the stack did not come up in 5 min: see $out/stack.log"

say "building the team code ($src)"
ros "rm -rf /tmp/team_ws && colcon --log-base /tmp/team_ws/log build --base-paths $cbase/code \
     --build-base /tmp/team_ws/build --install-base /tmp/team_ws/install >/dev/null" \
  > "$out/build.log" 2>&1 || die "colcon build failed: $out/build.log"

# ---- 5. the session, as fly.sh start on the drone: bag, camera page, map, mission (+ the pilot)
ros "ros2 bag record --use-sim-time -s mcap -o $cbase/bag ${BAG_TOPICS[*]}" > "$out/bag.log" 2>&1 &
bag=$!
ros "ros2 run dronehacks_nav camera_web --ros-args -p port:=8080 -p use_sim_time:=true" > "$out/camera.log" 2>&1 &
pages+=($!)
if grep -q "map_node" "$out/code/setup.py"; then
  ros "ros2 run dronehacks_nav map_node --ros-args -p use_sim_time:=true" > "$out/map.log" 2>&1 &
  pages+=($!)
fi
ros "ros2 run foxglove_bridge foxglove_bridge --ros-args -p address:=0.0.0.0 -p port:=8765 -p use_sim_time:=true" > "$out/foxglove.log" 2>&1 &
pages+=($!)
ros "ros2 launch skybeta_sim session.launch.py params_file:=$cbase/code/config/drone.sim.yaml" > "$out/mission.log" 2>&1 &
mission=$!

cat <<EOF2

[fly-sim]       code        $src
[fly-sim]       run files   $out
          gazebo      http://127.0.0.1:8082/vnc.html?autoconnect=1&resize=scale$([[ $SIM_GUI == 0 ]] && echo '   (off: --headless)')
          camera page http://127.0.0.1:8080/        (the team's camera page, as on the drone)
          foxglove    ws://127.0.0.1:8765
          logs        tail -f "$out/mission.log"     # mission + flight interface + pilot
                      tail -f "$out/stack.log"       # gazebo, px4, agent, oak_sim, vio_sim, px4_odom_bridge
                      tail -f "$out/bag.log" "$out/camera.log"
          bag         $out/bag   (mcap; vs a drone bag: python3 sim/skybeta/compare.py DRONE_BAG)
          px4 logs    $SIM/artifacts/px4/log   (.ulg)
[fly-sim]       Ctrl-C stops everything. Mission log follows:

EOF2
for _ in $(seq 1 20); do [[ -s "$out/mission.log" ]] && break; sleep 0.5; done
tail -n +1 -f "$out/mission.log" & tailer=$!
wait "$mission"
# ---- the run's result, for ./promote.sh: which commit, which mode, and how the mission ended
python3 - "$out" "$code" "$commit" "$mode" <<'PY'
import json, re, sys
out, code, commit, mode = sys.argv[1:]
log = open(f"{out}/mission.log", errors="replace").read()
ends = re.findall(r"mission done: (.*)", log)
json.dump({"code": code, "commit": commit, "mode": mode, "result": ends[-1].strip() if ends else "no result (stopped early?)"},
          open(f"{out}/result.json", "w"), indent=2)
PY
say "result: $(python3 -c "import json; print(json.load(open('$out/result.json'))['result'])")  ($out/result.json)"
say "mission ended. Ctrl-C to stop the sim (the bag keeps recording until then)."
while true; do sleep 3600; done
