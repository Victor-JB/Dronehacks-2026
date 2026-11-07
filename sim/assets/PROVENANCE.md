# Course source

`course-v3.zip` is the supplied preliminary DroneHacks 2026 course archive from
`sim/course_v3/course-v3.zip` in
[Drone008_Dronehacks at 31d58c7](https://github.com/AgastyaK87/Drone008_Dronehacks/tree/31d58c767b6e72cbe58ea3e5690e9ac9ec2b59e8/sim/course_v3).
The archive SHA-256 is
`52498970486fdb72ff6460d9b6297fdc128eabc089c7a25a7df6cf445990167c`.

Run `python3 tools/build_world.py` to regenerate `models/course_v3/model.sdf`,
its mesh files, and the per-group bounds manifest. The importer splits the
original triangles and vertex normals by named group, keeps metres and Z up,
and applies display colors. Walls, tunnel, cradles, boulders and the transparent
net retain their triangle geometry as static collisions. The flat Gazebo plane
supports the vehicle at z = 0. The course floor and markings are visual meshes.

The supplied layout is preliminary. The geometry, navigation route, and
simulated window check do not establish official course dimensions or scoring.
