# dronehacks-sim

A local Gazebo Harmonic simulation of the DroneHacks course. It includes an experimental PX4 SITL path that uses camera-localized mapped fiducials for navigation, plus a separate Gazebo ground-truth controller for comparison. The project is public at [machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim); the simulation and viewer run on your machine.

## PX4 camera demo (recommended path)

Install Docker CLI with Compose and [Colima](https://github.com/abiosoft/colima). From this directory, build and start the dedicated PX4 simulation once:

```sh
./runtime/run-px4.sh
```

In another terminal, start the local viewer:

```sh
./web/run.sh
```

Open the viewer at [http://localhost:8081](http://localhost:8081). To reset the PX4 simulation and run the full course in the background:

```sh
./runtime/replay-px4.sh course
```

The replay helper also accepts `hover`. Follow progress with `tail -f artifacts/px4/mission.log`. Each replay recreates the simulation container, so use the helper for the next run instead of starting another mission with `docker exec` against an active run.

To check camera-loss handling in a headless hover:

```sh
SIM_GUI=0 ./runtime/replay-px4.sh hover --blackout-after 18
```

See [docs/px4-camera.md](docs/px4-camera.md) for the navigation architecture, verified results, and limitations. See [docs/vision.md](docs/vision.md) for camera calibration and marker-map details.

## PX4 demo status and limits

Verified runs include a hover-height acquisition in 30.040 simulated seconds and a GUI run completing all 15 waypoints in 94.696 simulated seconds at 640×480 camera resolution. The camera-loss check stopped forwarding pose measurements at 18 simulated seconds while the vehicle was climbing at about 0.5 m; the route aborted 0.568 simulated seconds later, and PX4 reported landing and disarming. These runs confirm external-vision position, height, and yaw fusion with GNSS, barometer, range, and optical-flow fusion inactive. Waypoint completion uses PX4 estimated position; collisions and swept clearance were not scored. The native 480×360 camera run also completed all 15 waypoints in 96.376 simulated seconds. See [docs/px4-results.json](docs/px4-results.json) for the recorded runs.

This is a camera-loop baseline, not a complete sim-to-real validation. It adds a downward-facing monocular camera and surveyed AprilTag markers to the Gazebo course. The physical aircraft uses a forward-facing OAK-D-S2 stereo camera with cuVSLAM visual-inertial odometry on Jetson. The stock x500 is not the physical airframe. These runs do not validate the real camera stack, real flight, collision safety, or course clearance.

The camera runs at 30 Hz of simulation time. A 650-response MAVLink time-sync warmup at 50 Hz completes before arming. Software rendering may advance simulation slower than wall time, so use simulation timestamps when comparing camera and vehicle data. The Python course-control core is shared, but this SITL path uses MAVLink and PX4 position feedback; the physical cuVSLAM path uses a ROS 2 bridge.

## Gazebo ground-truth comparison mode

The older non-PX4 demo uses the Open Robotics X3 v4 quadrotor and follows the course from Gazebo ground-truth odometry. To run it, start the simulation with `./runtime/run-mac.sh`, then launch the same viewer with `./web/run.sh`. It also uses [http://localhost:8081](http://localhost:8081), with the Gazebo desktop at [http://localhost:8080/vnc.html?autoconnect=1&resize=scale](http://localhost:8080/vnc.html?autoconnect=1&resize=scale).

That separate baseline completed all 15 waypoints in 61.780 simulated seconds with zero obstacle contacts and 0.0905 m clearance at the padded window check. The optional tuned window profile completed in 47.736 simulated seconds with zero contacts and 0.0906 m clearance. These ground-truth results are not directly comparable to PX4 camera runs: vehicle models, navigation sources, and scoring differ. Setting speed scale to 1 on the default ground-truth route failed the conservative window-clearance check. See [docs/results.json](docs/results.json) for details. Isaac Sim and ROS 2 are not required for this comparison demo. Linux Docker Compose may work, but the setup has been tested on Apple Silicon macOS with Colima.
