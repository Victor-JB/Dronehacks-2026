# Marker-free PX4 status

The active experiment uses a forward 384×256 rendered RGB-D camera and downward ToF on the x500. Depth registration against the supplied course survey provides position and attitude to PX4 EKF2. An optional capture-aligned PX4 roll/pitch prior helps weak depth geometry. It supplies no yaw or position measurement.

Marker-free hover and landing have been demonstrated. The latest completed course attempt reached window alignment and safely aborted after losing visual observability. Full-route and learned-policy success remain unverified. See the [current runbook and evidence](markerfree-rgbd.md) and [local viewer](http://127.0.0.1:8083/).

Artificial markers are disabled in the active scene. Older downward-camera previews and marker-assisted runs are historical diagnostics, described in [px4-camera.md](px4-camera.md). They do not establish navigation on the physical course.
