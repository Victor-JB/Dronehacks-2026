#!/usr/bin/env bash
# One-shot sim flight: start the Gazebo/PX4 sim, fly one controller, stream its logs.
#
#   ./fly-sim.sh controllers/waypoints            fly a controller folder (see controllers/README.md)
#   ./fly-sim.sh controllers/hover --headless     no Gazebo window (faster); the viewer still works
#
# Ctrl-C at any time stops the mission, the viewer and the sim. The Colima VM keeps running
# for a fast next start; sim/stop.sh --vm stops it too.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="${REPO}/sim"
CTX=colima-dronehacks-sim
C=dronehacks-markerfree-sim-1
COMPOSE=(docker --context "$CTX" compose -p dronehacks-markerfree -f "$SIM/compose.px4.yaml" -f "$SIM/compose.markerfree.yaml")

die() { echo "[fly-sim] $*" >&2; exit 1; }
say() { echo "[fly-sim] $*"; }

# ---- arguments
ctrl="${1:-}"; shift || true
[[ -n "$ctrl" ]] || { sed -n '2,8p' "$0"; exit 2; }
ctrl="$(cd "$ctrl" 2>/dev/null && pwd)" || die "no such folder: $1"
[[ "$ctrl" == "$REPO"/* ]] || die "the controller folder must be inside $REPO"
[[ -f "$ctrl/controller.py" ]] || die "$ctrl has no controller.py"
rel="${ctrl#"$REPO"/}"
export SIM_GUI=1
for arg in "$@"; do
  case "$arg" in
    --headless) export SIM_GUI=0 ;;
    *) die "unknown option: $arg" ;;
  esac
done
export SIM_REAL_TIME_FACTOR="${SIM_REAL_TIME_FACTOR:-1.0}"

# ---- 1. VM: as many CPUs as the Mac can spare
cpus=$(( $(sysctl -n hw.ncpu) - 2 )); (( cpus > 10 )) && cpus=10; (( cpus < 4 )) && cpus=4
mem=$(( $(sysctl -n hw.memsize) / 1073741824 / 2 )); (( mem > 8 )) && mem=8; (( mem < 6 )) && mem=6
row="$(colima list --json 2>/dev/null | grep '"dronehacks-sim"' || true)"
have_cpus="$(sed -nE 's/.*"cpus":([0-9]+).*/\1/p' <<<"$row")"
if [[ "$row" == *'"Running"'* && "${have_cpus:-0}" -lt "$cpus" ]]; then
  say "restarting the VM with $cpus CPUs / ${mem} GB (had ${have_cpus})"
  colima stop --profile dronehacks-sim >/dev/null
  row=""
fi
if [[ "$row" != *'"Running"'* ]]; then
  say "starting the VM ($cpus CPUs, ${mem} GB)"
  colima start --profile dronehacks-sim --activate=false --arch aarch64 --vm-type vz \
    --cpu "$cpus" --memory "$mem" --runtime docker >/dev/null || die "colima failed to start"
fi

# ---- 2. fresh sim (PX4 state does not carry between runs)
say "starting Gazebo + PX4 (real-time factor ${SIM_REAL_TIME_FACTOR}, GUI ${SIM_GUI})"
"$SIM/runtime/run-markerfree.sh" >/dev/null || die "sim failed to start: ${COMPOSE[*]} logs"

run="$(basename "$rel")-$(date +%Y%m%d-%H%M%S)"
out="$SIM/artifacts/markerfree/$run"
ln -sfn "$run" "$SIM/artifacts/markerfree/current"

# ---- 3. viewer (camera, depth, telemetry) on the host
viewer=""
if command -v python3 >/dev/null && ! lsof -iTCP:8083 -sTCP:LISTEN >/dev/null 2>&1; then
  (cd "$SIM" && exec python3 -m web.server >/dev/null 2>&1) &
  viewer=$!
fi

# ---- 4. wait for Gazebo to stream depth (the evaluator gives up after 30 s otherwise)
say "waiting for Gazebo's depth camera (first start takes a minute or two)"
ready=0
for _ in $(seq 1 60); do
  if docker --context "$CTX" exec -w /workspace "$C" bash -c \
      'timeout 5 gz topic -e -n 1 -t "$(python3 -c "from vision.rgbd_scene import DEPTH_TOPIC; print(DEPTH_TOPIC)")"' \
      >/dev/null 2>&1; then ready=1; break; fi
  sleep 3
done
(( ready )) || { "${COMPOSE[@]}" stop >/dev/null 2>&1; [[ -n "$viewer" ]] && kill "$viewer"; die "no depth images after 5 min: ${COMPOSE[*]} logs"; }

# ---- 5. the mission, in the container
docker --context "$CTX" exec -w /workspace "$C" python3 -m mission.markerfree_evaluate \
  --output "artifacts/markerfree/$run" -- \
  --task course --controller "/repo/$rel" --course mission/course-window-x500.json \
  --launch-position -2.5146 0.9144 0.24 --launch-yaw-deg 90 --use-px4-tilt &
mission=$!

cleanup() {
  trap - INT TERM EXIT
  echo
  say "stopping"
  if kill -0 "$mission" 2>/dev/null; then
    docker --context "$CTX" exec "$C" python3 -c "import os,signal
for p in filter(str.isdigit, os.listdir('/proc')):
    try:
        if b'mission.markerfree_evaluate' in open(f'/proc/{p}/cmdline','rb').read(): os.kill(int(p), signal.SIGINT)
    except OSError: pass" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$mission" 2>/dev/null || break; sleep 0.5; done
    kill "$mission" 2>/dev/null
  fi
  [[ -n "$tailer" ]] && kill "$tailer" 2>/dev/null
  [[ -n "$viewer" ]] && kill "$viewer" 2>/dev/null
  "${COMPOSE[@]}" stop >/dev/null 2>&1
  say "sim stopped. Run files: $out"
  exit 0
}
tailer=""
trap cleanup INT TERM EXIT

cat <<EOF2

[fly-sim] controller  $rel
[fly-sim] run files   $out
          gazebo      http://127.0.0.1:8082/vnc.html?autoconnect=1&resize=scale$([[ $SIM_GUI == 0 ]] && echo '   (off: --headless)')
          viewer      http://127.0.0.1:8083        (camera, depth, telemetry)
          logs        tail -f "$out/mission.log"
                      tail -f "$out/telemetry.jsonl"
                      tail -f "$out/camera.log" "$out/observer.log"
                      docker --context $CTX logs -f $C      # Gazebo + PX4
                      ls "$SIM/artifacts/px4/log"             # PX4 flight logs (.ulg)
[fly-sim] Ctrl-C stops everything. Mission log follows:

EOF2

for _ in $(seq 1 600); do [[ -f "$out/mission.log" ]] && break; kill -0 "$mission" 2>/dev/null || break; sleep 0.5; done
[[ -f "$out/mission.log" ]] && { tail -n +1 -f "$out/mission.log" & tailer=$!; }

wait "$mission"; code=$?
sleep 1
say "mission finished (exit $code). Result: $out/recording.json"
say "the sim and viewer stay up to look around; Ctrl-C to stop them."
while true; do sleep 3600; done
