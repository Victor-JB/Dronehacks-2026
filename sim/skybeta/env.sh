# Source in the sim container: ROS 2, the skybeta workspace, and the team workspace once built.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/skybeta_ws/install/setup.bash
[[ -f /tmp/team_ws/install/setup.bash ]] && source /tmp/team_ws/install/setup.bash
set -u 2>/dev/null || true
