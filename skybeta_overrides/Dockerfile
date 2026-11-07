# Team image for the skybeta stack: the pinned skybeta base plus our camera config.
# On the drone ~/team_ws/Dockerfile is a symlink to this file; build with
#   ~/skybeta/host/run.sh build
# and restart the stack (docker rm -f skybeta-stack, on the ground, drone still).
# The build context is ~/team_ws; its .dockerignore re-includes only this folder.
# Rebuild after every skybeta update: the stack falls back to the plain base until then.
ARG BASE
FROM ${BASE}

COPY src/dronehacks/skybeta_overrides/oakd_vslam.yaml \
     /root/skybeta_ws/src/oakd_vslam_bringup/config/oakd_vslam.yaml
