# Sky Beta drone stack in the course sim

`./fly-sim-drone.sh` at the repository root. It runs the drone's software as the drone runs it:

| On the drone | In the sim |
|---|---|
| PX4 v1.16.2 on the Pixhawk 6C, fleet parameters | PX4 v1.16.2 SITL, the same parameters (`px4_params.py`) |
| Sky Beta airframe | `models/skybeta` (`make_model.py`, `AIRFRAME.md`) |
| MicroXRCEAgent v2.4.3, serial | MicroXRCEAgent v2.4.3, UDP 8888 |
| px4_msgs release/1.16 | the same |
| `skybeta_flight_interface`, `px4_odom_bridge` (skybeta 0.3.3) | the same code, `src/` |
| OAK-D S2 + depthai: `/oak/stereo/image_raw` | Gazebo camera at the OAK's position: `oak_sim` |
| cuVSLAM: `/visual_slam/tracking/odometry`, `/visual_slam/status` | Gazebo's true pose, nothing added: `vio_sim` |
| Safety pilot: arms on Altitude, Position, lands | `pilot`: the same steps over MAVLink |
| The team's `dronehacks_nav` (jetson branch), fly.sh: mission, bag, camera page, map | the same code and launch file, `use_sim_time: true` |
| ROS 2 Humble, Ubuntu 22.04 | ROS 2 Jazzy, Ubuntu 24.04 (Gazebo Harmonic's pairing) |

Every node runs on Gazebo's clock (`use_sim_time`), as PX4 SITL does, with `UXRCE_DDS_SYNCT = 0`.

| File | What |
|---|---|
| `Dockerfile` | the image: PX4 sim image + ROS 2 + agent + px4_msgs + `src/` |
| `start.sh` | container entrypoint: Gazebo, PX4, agent, `stack.launch.py` |
| `scene.py` | the course world with the Sky Beta on the launch mark |
| `src/skybeta_sim` | `oak_sim`, `vio_sim`, `pilot`, `stack.launch.py`, `session.launch.py` |
| `src/isaac_ros_visual_slam_interfaces` | only cuVSLAM's `VisualSlamStatus` message, so the team code imports it |
| `params/` | skybeta's parameter files and drone 16's dump |
| `compare.py` | sim bag vs drone bag |
