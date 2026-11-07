# Marker-free depth navigation experiment

This experiment trains a small PyTorch policy to output bounded velocity
setpoints from a forward synthetic depth image, estimated velocity, estimated
attitude, and a relative waypoint. It follows the existing Window Fast plan
from hover over the launch pad through the landing approach. It does not model
takeoff or touchdown. There are no tags, landmark IDs, obstacle coordinates, or
absolute positions in the policy input.

Run it from the repository root with a Python that has NumPy and PyTorch:

```sh
python -u -m learning.train --output artifacts/learning/my-run --max-seconds 18000
```

Use a fresh output directory. Try `--max-seconds 55 --eval-interval 12` for a
short smoke run. The trainer defaults to two CPU threads; use `--device mps`
on a supported Mac for faster training updates. The five-hour wall-time limit includes
evaluation. Send SIGTERM or Ctrl-C to stop at the next safe point. The trainer
refreshes `status.json` and retains an atomic `latest.pt`, a validation-selected
`best.pt`, and bounded `metrics.json`. The training replay stays in memory.

The course comes from `mission/course-window-fast.json` and the checked-in CAD
bounds and wall vertices. Straight walls use boxes, the rotated window wall
has its measured opening, and a polygonal shell approximates the hollow oval
tube. Boulders are conservative boxes and net faces are opaque proxies in the
depth view. Randomized examples shift obstacles, move or resize the aperture,
and perturb estimator readings, velocity response and wind. Later curriculum
stages increase depth and estimator noise and include bursts of missing pixels. The 24×16
forward pinhole camera has an assumed 90° horizontal field of view and produces
analytic ray ranges with noise and dropout. It does not process RGB images or
compute stereo depth. The estimator is a synthetic biased/noisy state source
with local goal generation from the known route; it does not implement VIO.

Training uses a privileged short-horizon geometry teacher to label velocity
actions, including actions from student-driven rollouts (DAgger-style
supervised imitation). There is no Dreamer/world-model or RL implementation.
The student receives no geometry query or true pose. Its output is velocity
in heading-aligned FLU: forward, left, and up in m/s. PX4 transport would need
to rotate this into the requested NED frame, connect real state/depth inputs,
and provide the separate yaw, takeoff, landing and safety logic.

Validation and final held-out seeds are disjoint from training and each
other. Full-course rollouts measure route completion, continuous
sphere-vs-box proxy contact, full aperture passage and simulated traversal
time. Metrics compare the trained depth policy, the same policy with a blank
image, a direct waypoint controller, and the privileged teacher. The blank
image is an input ablation, so it is out of the training distribution.
Per-scenario results and paired velocity changes for fixed state with visible
versus blank depth are recorded. These are analytic simulation diagnostics,
not Gazebo/PX4 integration results or hardware clearance measurements.
