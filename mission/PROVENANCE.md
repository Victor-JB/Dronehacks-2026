# Route and controller

`course.json`, `core.py`, `frames.py` and `crossing.py` were copied from
[Drone008_Dronehacks at 31d58c7](https://github.com/AgastyaK87/Drone008_Dronehacks/tree/31d58c767b6e72cbe58ea3e5690e9ac9ec2b59e8).
Their original locations were `sim/course_v3/course.json`,
`src/course_mission/course_mission/{core,frames}.py` and
`tools/course_demo/crossing.py`.

The Gazebo adapter uses the X3 collision envelope for its bounds guard and
window report. It consumes Gazebo ground-truth odometry and simulator time,
rotates velocity requests into the measured full body orientation, and
publishes Twist commands for the Gazebo motor controller. The final route
state is a position hold at the landing handoff, 0.70 m above the course floor.
