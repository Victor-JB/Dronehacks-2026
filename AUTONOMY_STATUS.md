# Autonomy status — Codex takeover

Branch: `saketh/autonomy`. Local development is complete, checkpointed in `893dbfd` plus the final documentation commit. Ready only for the stationary verification boundary. No push or hardware access has occurred.

## Git and team ownership

Claude's `02f6209`, `b40ea66`, `46925ee` are preserved. Merge `4e2866b` joins the team's latest user-pulled main `e9532fa` without overwriting either history. The verified original checkout at `C:/Users/saket/OneDrive/Documents/GitHub/Dronehacks-2026` remains clean on main. This working checkout is under this chat's `work/dronehacks_autonomy`. `origin` is the team URL; `handoff` is the original bundle. No upstream team feature branch is assumed; fetch/compare authenticated remote refs before any later publishing.

Only `shared-context/saketh-desu.md` was edited. The other four files and shared-context README remain intact. User instructions to stay on the autonomy branch and avoid publishing override the shared folder's direct-main push convention. Claude's Gazebo worktree was never inspected or modified.

## Runtime interfaces

| Input/output | Contract |
|---|---|
| OAK Image + CameraInfo | raw left/right, sensor-data QoS; pair <=5 ms; calibrated rectification and timestamped optical-to-body TF required by default |
| VO odometry | nav_msgs/Odometry, `odom` -> `base_link`, nonzero monotonic capture stamp, finite pose/unit quaternion/covariance; sensor-data QoS |
| VO status | isaac_ros_visual_slam_interfaces/VisualSlamStatus; fresh state 1 required by default; unknown/lost/stale inhibits motion |
| Stereo scan / clearance | LaserScan + Range in base_link at image capture time; +inf measured clear, NaN unknown, -inf/short finite stop |
| Map / line fixes | absolute course PoseWithCovarianceStamped and body Vector3Stamped evaluated against historical odometry; duplicate/stale/frame/innovation gates |
| Commands | default dry-run TwistStamped to `/dh/dry_run/cmd_vel`; no flight-control publishers/clients |
| Sky Beta, non-dry mode only | ENU odom TwistStamped; only zero readiness before fresh authority; nonzero motion/progression after Offboard; stale/lost/frozen state latches abort. Never arms/lands or publishes FMU |
| Raw PX4 | simulation-only; not authorized for hardware |

## Completed changes

- Unknown stereo always stops; removed creep and tube-geometry exceptions. Missing bracketing capture pose/view changes also hold. Disabling the stereo node no longer disables the guard automatically.
- Historical pose/height and capture-time TF throughout camera-derived processing. Replay cannot refresh health. Default measured TF mount and no automatic nominal-baseline fallback.
- Freshness/uncertainty/XYZ/yaw discontinuity gates; pause + confirmed map recovery or abort. Bounded, observable map matching and a final mission innovation gate.
- Source-verified zero-twist startup handshake, strict boolean authority parsing, frozen-interface protection and permanent takeover abort. Dry-run isolation stays default.
- Illuminated nominal tube and height-aware offline camera renderer. Fixed the false full-height obstacle extrusion that blocked the real OpenCV loop. A* + clothoid planning and tube-axis anchors remain unchanged.
- Added real ROS launch/DDS/TF/image tests, fake-bus regressions, stationary guide and persistent AGENTS instructions preserving team rules.

## Evidence and remaining boundary

Detailed commands, results, prior failures, source revisions and limitations: `docs/LOCAL_VALIDATION.md`.

- Unit tested: 17 core tests, 30 preserved team tests; 88 organizer pure Python checks pass with two ROS/PX4-dependent checks skipped.
- Fake-bus node integration: 12 existing + 12 safety/OpenCV tests pass.
- Synthetic camera processing: 12 stereo scenarios, 14 line scenarios pass assertions.
- Python ray-cast SIL: 4/4 route/seed runs reach pad hover with zero contacts; fake-PX4 return reaches DONE. These are not actual PX4 SITL or Gazebo.
- Rendered images through actual OpenCV: 4/4 window/gate seeds 0/1 reach pad hover, zero contacts, 2.2-3.1 cm pad error and <=11.0 cm maximum pose error. Source/seed details are in the validation record.
- Organizer-compatible library pair on local Windows: numpy 1.26.4/OpenCV 4.10 also passes 47 unit, 24 fake-bus and all stereo/line scenarios. Not ARM/ROS qualification.
- Actual ROS 2 integration: NOT TESTED. Both launch tests are written/syntax-checked and require isolated domain 77.
- Jetson hardware, real cameras, motion/flight: NOT TESTED. No SSH, physical commands, flight battery or propellers involved.

Next operation boundary: stop before SSH/hardware. The later procedure is `docs/JETSON_VERIFICATION.md`: user/organizer approval, Nano sensor-only environment, build + isolated ROS launch tests, real camera/TF/time checks, stationary dry-run and command-isolation checks. No flight or arming is part of it.

Known limitations: event image and numpy<2/ARM untested; measured mount/radius/resting height and Nano CPU throughput needed; limited stereo height coverage; small synthetic clearance margins; conservative timing can slow or hold; Sky Beta cannot land/disarm; substantial localization resets may require restart. Older planner-suite scores remain historical.
