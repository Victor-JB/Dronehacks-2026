# Questions to ask the organizers (copy into Slack / ask at the briefing)

Ordered by how much the answer changes the code. Each item notes which parameter or backend it decides.

1. **Start, landing and return authority.** The rules want one start command, autonomous takeoff, landing and disarm on the pad, and (for the return bonus) an autonomous re-arm and takeoff. The competitor interface in the fleet image never arms or changes mode, and it clamps altitude at ≥ 0.3 m. What is the intended mechanism?
   - May our node send PX4 `VehicleCommand` (arm, offboard, `NAV_LAND`, disarm) itself, with the flight_interface off?
   - Or will the interface gain start/land/arm services?
   - *Decides `backend: skybeta | px4` and `enable_return`.*
2. **PX4 parameters.** May we change non-safety parameters, such as `MPC_XY_VEL_MAX` (1.0 m/s on the bench dump), `MIS_TAKEOFF_ALT` (2.5 m, above the walls), the land-detector timing, and EKF2 fusion settings?
   - Which parameters count as the protected "safety settings"?
   - *Decides `v_cap`/`v_max`.*
3. **Camera pipeline.** May we enable on-device StereoDepth, change the stereo resolution, or turn on cuVSLAM IMU fusion? Or must the vision pipeline stay exactly as shipped?
   - *Decides whether `stereo_scan` runs on the Jetson (current design) or uses OAK depth.*
4. **Rule 6.4 and the SLAM map.** cuVSLAM runs in SLAM mode with loop closure. Should we restart it (fresh map) before every scored run? Is a localizer that uses only the published course model fine?
5. **Auxiliary sensors.** Is a small 360° 2D LiDAR such as the YDLIDAR T-mini Plus (45 g, Class 1, ~$79–95) acceptable, connected to the Jetson UART or USB?
   - What weighing tolerance applies near the 50 g limit?
6. **Course facts.**
   - Final course version before scored runs.
   - How dark the tube really is.
   - Wall and floor surface finish (texture for stereo).
   - Whether checkpoint lines are marked on the floor.
   - Which ring counts for CP5: the 60-in pad ring, or the 79-in "approach ring"?
7. **Launch placement.** Do we place the drone on the launch mark ourselves, and may we choose its heading? (We assume facing north, up the first corridor.)
8. **Pre-event code (R9.1).** Is it acceptable to arrive with prepared code, or must the stack be written during the event?
