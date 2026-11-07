# dronehacks-sim

A local Gazebo Harmonic simulation of the DroneHacks course and its 15-waypoint window route. The project is public at [machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim); the simulator and browser viewer run on your machine, not on that GitHub page.

## Run on macOS

Install Docker CLI with Compose and [Colima](https://github.com/abiosoft/colima), then clone this repository. In one terminal, start the simulation:

```sh
./runtime/run-mac.sh
```

This creates or starts the dedicated ARM64 Colima profile `dronehacks-sim` (4 CPUs, 6 GiB memory, 16 GiB disk), builds the image, and starts Gazebo. In another terminal, start the local route status page:

```sh
./web/run.sh
```

Open the status page at [http://localhost:8081](http://localhost:8081). Open the Gazebo desktop at [http://localhost:8080/vnc.html?autoconnect=1&resize=scale](http://localhost:8080/vnc.html?autoconnect=1&resize=scale). These addresses are local to the Mac running the simulation.

To reset and replay the mission, run `./runtime/replay.sh`; it recreates only the simulation container. To stop the containers, run `docker --context colima-dronehacks-sim compose down`. To stop the VM as well, run `colima stop --profile dronehacks-sim`.

For the optional faster window profile, replay with:

```sh
SIM_COURSE=mission/course-window-fast.json SIM_SPEED_SCALE=1 ./runtime/replay.sh
```

## What is simulated

The container uses Ubuntu 24.04 (Noble) with Gazebo Harmonic 8.15, DART 7.9 physics, Ogre2 rendering, and Mesa software rendering. It loads the Open Robotics X3 v4 quadrotor with Gazebo multicopter velocity-control and four-motor plugins. The existing course controller follows the imported course using Gazebo ground-truth odometry and simulator time. The browser page reports simulator telemetry; it is not a browser-rendered simulation.

In GUI runs, the default profile completed all 15 waypoints to the landing handoff in 61.780 simulated seconds, with zero obstacle contacts and 0.0905 m of clearance at the padded window check. The optional tuned window profile completed the same route in 47.736 simulated seconds, with zero obstacle contacts and 0.0906 m clearance. These are simulator ground-truth results, not evidence of hardware reliability. Setting speed scale to 1 on the default route failed the conservative window-clearance check; that attempt is not a passing result. See [docs/results.json](docs/results.json) for run details, [docs/browser.md](docs/browser.md) for viewer details, and [mission/PROVENANCE.md](mission/PROVENANCE.md) for course and controller provenance.

This uses simulator ground truth, not PX4 SITL. It does not validate camera or visual-inertial odometry, and the simulated vehicle dynamics are not a claim about real aircraft performance. Isaac Sim, Pegasus, PX4, and ROS 2 are not required. Linux Docker Compose may work with Docker installed, but this setup has only been tested on Apple Silicon macOS with Colima.
