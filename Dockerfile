FROM ubuntu:24.04

RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main\n' \
       "$(dpkg --print-architecture)" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       gz-tools2 gz-sim8-cli gz-transport13-cli libgz-sim8-plugins \
       libgz-physics7-dartsim libgz-rendering8-ogre2 \
       python3-gz-sim8 \
       libgl1-mesa-dri libglx-mesa0 libegl-mesa0 \
       qml-module-qt-labs-platform qml-module-qt-labs-settings \
       qml-module-qtgraphicaleffects qml-module-qtquick-controls \
       qml-module-qtquick-controls2 qml-module-qtquick-dialogs \
       qml-module-qtquick-layouts qml-module-qtquick-window2 qml-module-qtquick2 \
       fonts-dejavu-core novnc websockify x11-utils x11vnc xvfb \
    && rm -rf /var/lib/apt/lists/*

# OSRF ships the plugin loader links and Ogre shader assets in the dev packages.
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       libgz-physics7-dartsim-dev libgz-rendering8-ogre2-dev libqt5svg5 \
    && rm -rf /var/lib/apt/lists/* \
    && sed -i -e 's/<width>1000<\/width>/<width>1280<\/width>/' \
       -e 's/<height>845<\/height>/<height>800<\/height>/' \
       -e 's/name="quick_start" show_again="true"/name="quick_start" show_again="false"/' \
       /usr/share/gz/gz-sim8/gui/gui.config

ENV DISPLAY=:99 \
    GZ_PARTITION=dronehacks_sim \
    GZ_SIM_RESOURCE_PATH=/workspace/models:/workspace/assets \
    LIBGL_ALWAYS_SOFTWARE=1 \
    LP_NUM_THREADS=2 \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/xdg-runtime \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /workspace

COPY runtime/start.sh /usr/local/bin/dronehacks-start
CMD ["/usr/local/bin/dronehacks-start"]
