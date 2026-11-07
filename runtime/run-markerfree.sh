#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p artifacts/markerfree
export GZ_GUI_PORT=${GZ_GUI_PORT:-8082}
if ! docker --context "${SIM_DOCKER_CONTEXT:-colima-dronehacks-sim}" image inspect \
    "${SIM_PX4_IMAGE:-dronehacks-markerfree:base}" >/dev/null 2>&1; then
    docker --context "${SIM_DOCKER_CONTEXT:-colima-dronehacks-sim}" image tag \
        dronehacks-px4:local "${SIM_PX4_IMAGE:-dronehacks-markerfree:base}"
fi
docker --context "${SIM_DOCKER_CONTEXT:-colima-dronehacks-sim}" compose \
    -f compose.px4.yaml -f compose.markerfree.yaml up --no-build -d --force-recreate
echo 'Marker-free forward RGBD Gazebo/PX4 started. This command does not arm or fly.'
