#!/usr/bin/env bash
# Build the PX4 + Gazebo images for the marker-free sim. Does not start the sim.
# Afterwards: ./runtime/run-markerfree.sh to start, ./stop.sh to stop.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
ctx=colima-dronehacks-sim

if ! docker --context "$ctx" info >/dev/null 2>&1; then
    colima start --profile dronehacks-sim --activate=false \
        --arch aarch64 --vm-type vz --cpu 4 --memory 6 --disk 16 --runtime docker
fi

# Base Gazebo image (PX4 image builds on top of it). Cached if already built.
docker --context "$ctx" build -t dronehacks-sim:local .
# PX4 v1.16.2 SITL compile: the long step. The legacy builder ignores
# Dockerfile.px4.dockerignore, so swap it in as .dockerignore for this build.
cp .dockerignore .dockerignore.main
trap 'mv -f .dockerignore.main .dockerignore' EXIT
cp Dockerfile.px4.dockerignore .dockerignore
docker --context "$ctx" build -f Dockerfile.px4 -t dronehacks-px4:local .
mv -f .dockerignore.main .dockerignore
trap - EXIT
docker --context "$ctx" image tag dronehacks-px4:local dronehacks-markerfree:base

echo "Built. Start with ./runtime/run-markerfree.sh (does not arm or fly)."
