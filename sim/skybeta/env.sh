# Source in the sim container: ROS 2, the skybeta workspace, and the team workspace once built.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/skybeta_ws/install/setup.bash
[[ -f /tmp/team_ws/install/setup.bash ]] && source /tmp/team_ws/install/setup.bash
# ROS Jazzy's gz_tools_vendor puts its own `gz` first on PATH, and that one has no `gz sim` (Gazebo
# Harmonic here comes from the OSRF packages, whose `gz` is /usr/bin/gz): use the system one.
export PATH="/usr/bin:$PATH"
set -u 2>/dev/null || true
