# The Sky Beta drone stack on top of the PX4 + Gazebo sim image.
#   docker --context colima-dronehacks-sim build -t dronehacks-skybeta:local sim/skybeta
# (fly-sim.sh builds it, and the images under it, the first time.)
#
# The drone runs ROS 2 Humble on Ubuntu 22.04. This image is Ubuntu 24.04 (the sim's Gazebo
# Harmonic), so it gets ROS 2 Jazzy, the release paired with Harmonic. The ROS code is the same
# Python; the versions that talk to PX4 are pinned exactly as on the drone (skybeta 0.3.3):
#   Micro-XRCE-DDS-Agent v2.4.3, px4_msgs release/1.16 (PX4 v1.16.2).
FROM dronehacks-px4:local
SHELL ["/bin/bash", "-c"]
ARG XRCE_AGENT_TAG=v2.4.3
ARG PX4_MSGS_REF=release/1.16

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      curl gnupg ca-certificates git build-essential cmake \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base ros-jazzy-ros-gz-bridge ros-jazzy-tf2-ros \
      ros-jazzy-rosbag2-storage-mcap ros-jazzy-foxglove-bridge \
      python3-colcon-common-extensions python3-numpy python3-opencv \
 && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 -b ${XRCE_AGENT_TAG} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
 && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
 && make -C /tmp/xrce/build -j"$(nproc)" && make -C /tmp/xrce/build install && ldconfig \
 && rm -rf /tmp/xrce

COPY src /opt/skybeta_ws/src
RUN git clone --depth 1 -b ${PX4_MSGS_REF} https://github.com/PX4/px4_msgs.git /opt/skybeta_ws/src/px4_msgs \
 && source /opt/ros/jazzy/setup.bash && cd /opt/skybeta_ws \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf build log

# sim/px4/install.sh removes PX4's uXRCE-DDS client from startup (that sim talks MAVLink only).
# The drone talks to PX4 over uXRCE-DDS, so start it again, after the MAVLink setup, on UDP 8888.
RUN echo 'uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888' >> /opt/px4/etc/init.d-posix/px4-rc.mavlink

ENV ROS_DOMAIN_ID=0 DRONEHACKS_SITL=1
