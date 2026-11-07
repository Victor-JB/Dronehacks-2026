# Marker-free flight trials

This inventory contains 20 unique run IDs across 21 flight telemetry archive paths; `current` is an identical alias of `hover-04`. No overall reliability percentage is inferred across changing code, configuration, and diagnostic protocols.

## Full-route attempts

| Archive | Controller / checkpoint | Outcome / fault | Controller waypoints | Independent truth | Landing |
|---|---|---|---:|---:|---|
| course-01 | waypoint course controller / `—` | odometry_jump | 1/15 | not scored | not confirmed in archived telemetry/summary |
| course-02 | waypoint course controller / `—` | rgbd_pose_unavailable:tracking_lost:survey:unobservable_depth_geometry:0.0035633;depth:unobservable_depth_geometry:3.4445e-11;rgb:insufficient_correspondences | 1/15 | 1/15 | confirmed touchdown and disarm |
| course-03 | waypoint course controller / `—` | rgbd_pose_unavailable:tracking_lost:survey:unobservable_depth_geometry:0.0039908;depth:unobservable_depth_geometry:5.5805e-11;rgb:insufficient_correspondences | 1/15 | 1/15 | confirmed touchdown and disarm |
| course-04 | waypoint course controller / `—` | rgbd_pose_unavailable:tracking_lost:survey:unobservable_depth_geometry:0.0026728;depth:unobservable_depth_geometry:0.00073811;rgb:insufficient_texture | 12/15 | 10/15 | confirmed touchdown and disarm |
| course-05 | waypoint course controller / `—` | px4_power_health:battery_status_timestamp_not_advancing | 1/15 | 1/15 | confirmed touchdown and disarm |
| course-06 | waypoint course controller / `—` | px4_power_health:battery_status_listener_timeout | 9/15 | 9/15 | confirmed touchdown and disarm |
| course-07 | waypoint course controller / `—` | rgbd_pose_unavailable:tracking_lost:survey:unobservable_depth_geometry:0.0028614;depth:unobservable_depth_geometry:0.0016497;rgb:insufficient_texture | 1/15 | 1/15 | confirmed touchdown and disarm |
| course-08 | waypoint course controller / `—` | completed | 15/15 | 15/15 ordered | confirmed touchdown and disarm |
| course-09 | waypoint course controller / `—` | completed | 15/15 | 15/15 ordered | confirmed touchdown and disarm |
| course-10 | waypoint course controller / `—` | completed | 15/15 | 15/15 ordered | confirmed touchdown and disarm |
| policy-01 | learned policy / `872ba44c` | invalid_policy_exchange_record:response.json | 0/15 | not scored | no armed flight; touchdown not applicable |
| policy-02 | learned policy / `872ba44c` | course_boundary_breach | 6/15 | 6/15 | confirmed touchdown and disarm |
| policy-03 | learned policy / `3602a898` | completed | 15/15 | 15/15 ordered | confirmed touchdown and disarm |
| policy-04 | learned policy / `3602a898` | px4_power_health:battery_status_stale_sim_timestamp | 0/15 | 0/15 | confirmed touchdown and disarm |
| policy-05 | learned policy / `a1919ecd` | completed | 15/15 | 15/15 ordered | confirmed touchdown and disarm |

## Independent score details

Only a score file whose run ID matches its telemetry archive is joined. Abort-landing clearance and contact coverage are reported separately from route completion.

- **course-02:** truth 1/15 ordered=False.
- **course-03:** truth 1/15 ordered=False.
- **course-04:** truth 10/15 ordered=False.
- **course-05:** truth 1/15 ordered=False.
- **course-06:** truth 9/15 ordered=False.
- **course-07:** truth 1/15 ordered=False.
- **course-08:** truth 15/15 ordered=True.
- **course-09:** truth 15/15 ordered=True.
- **course-10:** truth 15/15 ordered=True.
- **policy-02:** truth 6/15 ordered=False.
- **policy-03:** truth 15/15 ordered=True.
- **policy-04:** truth 0/15 ordered=False; abort landing 49.192→59.46 s, CAD clear through disarm=True, contact coverage=partial_topic_coverage.
- **policy-05:** truth 15/15 ordered=True.

## Diagnostic hovers

| Archive | Outcome | Fault | Landing | Independent score |
|---|---|---|---|---|
| hover-01 | completed | none | confirmed touchdown and disarm | not scored |
| hover-02 | completed | none | confirmed touchdown and disarm | matched |
| hover-03 | completed | none | confirmed touchdown and disarm | matched |
| hover-04 | failed | rgbd_pose_unavailable:tracking_lost:survey:insufficient_depth_overlap;depth:insufficient_depth_overlap;rgb:insufficient_inliers | confirmed touchdown and disarm | matched |

## Blackout diagnostic

| Archive | Outcome | Injection/fault | Controller waypoints | Independent truth | Landing |
|---|---|---|---:|---:|---|
| blackout-01 | failed | rgbd_pose_unavailable:stale_capture | 1/15 | 1/15 | confirmed touchdown and disarm |

## Interpretation and provenance

The JSON preserves each run ID, checkpoint and braking-guard metadata, event and summary simulation times, matching independent-score checks, and SHA-256 fingerprints of source records. Missing scores remain unknown; the inventory does not infer truth waypoints from controller progress.

All results are simulator evidence. Contact-topic coverage may be partial; absence of unexpected observed contacts is not a zero-physical-collision claim.
