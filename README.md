# dronehacks-sim

A separate Gazebo simulation project for the DroneHacks course and the window-slot route.

The implementation is being built and tested on an Apple Silicon Mac. Gazebo runs locally; this project does not connect to or command the physical drone. The course assets come from the supplied `dronehacks-course-v3-obj.zip` and the route/controller developed in `AgastyaK87/Drone008_Dronehacks`.

## Current status

Gazebo runtime setup and course integration are in progress. Launch commands and measured results will be added after an actual Gazebo run succeeds. This is not yet a PX4 or hardware-validation result.

## Scope

- Import the supplied course without filling the window or tunnel openings.
- Run a feedback-controlled quadrotor using Gazebo physics.
- Provide a local browser URL to inspect the running simulation.
- Preserve model provenance and report simulator limitations.

The Isaac Sim / Pegasus stack is not required for this Gazebo project.
