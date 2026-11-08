# Offline course simulation (FINAL Course v3)

The policy is the same Explorer + CommandSafety code that the `nav_policy` ROS node runs. It never sees the
CAD. It gets rendered depth/RGB at 10 Hz and noisy odometry, and builds its own map. The course is ground
truth only, used for rendering and for collision scoring.

```bash
# once, laptop only (needs: pip install cascadio trimesh)
python tools/sim/step_to_voxels.py "FINAL Course v3.step" tools/sim/final_course_v3.npz
# fly it (add --noise for 2% depth noise, 10% dropout and odometry drift)
python tools/sim/run_final_course.py run.pkl --verbose
# 4-panel presentation video: chase cam, the policy's map, depth, camera
python tools/sim/render_video.py run.pkl run.mp4
# regression (about 3 min)
DH_E2E=1 python -m pytest ros2_ws/src/dh_core/test/test_e2e_course.py
```

**Limits.** This is not PX4 SITL and not Gazebo. The dynamics are an idealised first-order velocity loop,
and depth is rendered exactly, with optional noise. It tells you about the policy's logic, not real flight.
Floor decals (pad rings, guideline) are not obstacles. The pad is drawn as a blue 5 ft square.
