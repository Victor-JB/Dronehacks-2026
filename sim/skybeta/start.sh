#!/usr/bin/env bash
# Container entrypoint: Gazebo + PX4 SITL + the drone's always-on stack. The mission is not started
# here; fly-sim.sh starts it per run, as fly.sh does on the drone.
#
#   gazebo     the course with the Sky Beta (skybeta/scene.py)
#   px4        PX4 v1.16.2 SITL, gz_x500 airframe, Sky Beta parameters (skybeta/px4_params.py)
#   agent      MicroXRCEAgent udp4 -p 8888            (drone: run_agent.sh, serial)
#   stack      ros_gz_bridge, oak_sim, vio_sim, px4_odom_bridge   (drone: depthai + cuVSLAM + bridge)
set -eo pipefail
cd /workspace
source skybeta/env.sh

python3 skybeta/scene.py --output /tmp/skybeta-world.sdf

pids=()
cleanup() { trap - EXIT; kill "${pids[@]}" 2>/dev/null || true; wait "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 0' INT TERM
tag() { sed -u "s/^/[$1] /"; }

SIM_MISSION=0 SIM_WORLD=/tmp/skybeta-world.sdf SIM_HEADLESS_RENDERING=1 SIM_SEPARATE_GUI=1 \
    SIM_DESKTOP_SIZE="${SIM_DESKTOP_SIZE:-800x600}" runtime/start.sh > >(tag gazebo) 2>&1 &
pids+=("$!")

(
    workdir=$(mktemp -d /tmp/px4-sitl.XXXXXX)
    mkdir -p /workspace/artifacts/px4/log && ln -s /workspace/artifacts/px4/log "$workdir/log"
    export PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_NAME=course_quad
    export PX4_GZ_WORLD=dronehacks PX4_GZ_STANDALONE=1 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    eval "$(python3 skybeta/px4_params.py)"
    exec /opt/px4/bin/px4 -d -w "$workdir" /opt/px4/etc
) > >(tag px4) 2>&1 &
pids+=("$!")

MicroXRCEAgent udp4 -p 8888 > >(tag agent) 2>&1 &
pids+=("$!")

ros2 launch skybeta_sim stack.launch.py > >(tag stack) 2>&1 &
pids+=("$!")

wait -n "${pids[@]}"
