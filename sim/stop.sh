#!/usr/bin/env bash
# Stop every dronehacks-sim container (ground-truth, PX4, marker-free and Sky Beta projects).
# ./stop.sh        stop the containers, keep the VM running (fast restart)
# ./stop.sh --vm   also stop the Colima VM (frees RAM and CPU)
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
ctx=${SIM_DOCKER_CONTEXT:-colima-dronehacks-sim}

if docker --context "$ctx" info >/dev/null 2>&1; then
    docker --context "$ctx" compose -p dronehacks-sim -f compose.yaml down
    docker --context "$ctx" compose -p dronehacks-px4 -f compose.px4.yaml down
    docker --context "$ctx" compose -p dronehacks-markerfree \
        -f compose.px4.yaml -f compose.markerfree.yaml down
    docker --context "$ctx" compose -f compose.skybeta.yaml down
else
    echo "Colima VM is not running; no containers to stop."
fi

if [[ ${1:-} == --vm ]]; then
    colima stop --profile dronehacks-sim
fi
