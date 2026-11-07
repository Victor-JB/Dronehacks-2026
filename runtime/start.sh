#!/usr/bin/env bash
set -euo pipefail

pids=()
cleanup() {
    trap - EXIT
    if (( ${#pids[@]} )); then
        kill "${pids[@]}" 2>/dev/null || true
        wait "${pids[@]}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 0' INT TERM

start() {
    "$@" &
    pids+=("$!")
}

args=(-r -v 3)
if [[ ${SIM_GUI:-1} == 1 ]]; then
    install -d -m 700 "$XDG_RUNTIME_DIR"
    start Xvfb "$DISPLAY" -screen 0 1200x1000x24 -nolisten tcp
    for ((attempt = 0; attempt < 100; attempt++)); do
        if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
            break
        fi
        sleep 0.1
    done
    if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        echo "Xvfb did not start on $DISPLAY" >&2
        exit 1
    fi
    start x11vnc -display "$DISPLAY" -listen 127.0.0.1 -rfbport 5900 -nopw \
        -forever -shared -noxdamage -quiet
    start websockify --web /usr/share/novnc --heartbeat 30 0.0.0.0:6080 127.0.0.1:5900
else
    args+=(-s)
fi

start gz sim "${args[@]}" "${SIM_WORLD:-worlds/course.sdf}"
services=("${pids[@]}")
if [[ ${SIM_MISSION:-1} == 1 ]]; then
    mkdir -p artifacts/gazebo
    start python3 -m mission.run --output /workspace/artifacts/gazebo/telemetry.jsonl
fi

wait -n "${services[@]}"
