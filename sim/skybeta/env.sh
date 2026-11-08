# Source in the sim container: ROS 2, the skybeta workspace, and the team workspace once built.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/skybeta_ws/install/setup.bash
[[ -f /tmp/team_ws/install/setup.bash ]] && source /tmp/team_ws/install/setup.bash
# ROS Jazzy's gz_tools_vendor puts its own `gz` first on PATH and its own folder in GZ_CONFIG_PATH;
# neither knows `gz sim` (Gazebo Harmonic here comes from the OSRF packages: /usr/bin/gz, whose
# command files are in /usr/share/gz). Put the system ones first.
export PATH="/usr/bin:$PATH"
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}"
set -u 2>/dev/null || true
