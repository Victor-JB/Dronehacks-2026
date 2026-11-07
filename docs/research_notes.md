# Research notes (agent reports, 2026-11-05)

Collected by research agents from public pages; every claim is labelled VERIFIED (URL) / UNVERIFIED / INFERENCE in the original text below. Google Scholar blocked automated access, so the professor's two Scholar links could not be resolved; the GitHub/VGGT links were researched directly.

---

# Professor-recommended resources: fit for DroneHacks 2026 (as of 2026-11-05)

Labels: **VERIFIED** (I read the claim at the URL given), **UNVERIFIED** (seen only in a secondary source or snippet, or recalled from memory), **INFERENCE** (my own reasoning). I only read pages. No code was downloaded or run.

Course facts used below (from the brief): openings 36.5–40 in (0.93–1.02 m), window 36 in (0.91 m), tube 58x72 in (1.47x1.83 m), MPC_XY_VEL_MAX = 1.0 m/s. A 7-inch quad spans about 0.30 m motor to motor and about 0.45–0.5 m including props (INFERENCE). That means every gap is about twice the vehicle's width or more, so attitude-aware "fly through a gap tilted" planning is not needed.

---

## 1. Motion Primitive Library (MPL) + mpl_ros + DecompUtil/DecompROS + jps3d

**Links:** https://github.com/sikang/motion_primitive_library · https://github.com/sikang/mpl_ros · https://github.com/sikang/DecompUtil · https://github.com/sikang/DecompROS · https://github.com/KumarRobotics/jps3d. Papers: IROS 2017 arXiv:1709.05401 (https://ar5iv.labs.arxiv.org/html/1709.05401), RA-L 2018 arXiv:1710.02748 (https://ar5iv.labs.arxiv.org/html/1710.02748), RA-L 2017 "Safe Flight Corridors" (RA-L vol. 2, no. 3, pp. 1688–1695).

### Repo facts
| Repo | License | Deps / build | ROS | Last commit | Stars |
|---|---|---|---|---|---|
| motion_primitive_library v1.2 | Apache-2.0 | Eigen, yaml-cpp, OpenCV (plotting), plain CMake or catkin | none needed (core C++) | **Apr 19 2019** | 266 |
| mpl_ros | Apache-2.0 | ROS "Indigo+", catkin_simple, SDL1.2, submodules MPL + DecompROS | **ROS 1 only** | Apr 17 2025 (a Melodic callback fix); before that Apr 2019 | 758 |
| DecompUtil | BSD-3 | header-only C++, CMake | optional catkin | Sep 10 2018 | 289 |
| DecompROS | BSD-3 | ROS Indigo+, catkin_simple, Qt4+ | ROS 1 | not shown | 243 |
| jps3d | BSD-3 | Eigen3, yaml-cpp, Boost | optional catkin | not shown | 484 |

All rows VERIFIED from each repo's README and commits page. The mpl_ros README says the package is "still under maintenance" and its API may change (VERIFIED).

**ROS 2 ports:** I found none for mpl_ros, DecompROS or MPL (search, VERIFIED negative result only for what I searched). KumarRobotics/kr_autonomous_flight, which uses this planning family, has an *experimental* `ros2_dev` branch that targets **ROS 2 Jazzy / Ubuntu 24.04**, not Humble, and is "not extensively tested". Its licence is the Penn Software License (non-commercial) (VERIFIED, https://github.com/KumarRobotics/kr_autonomous_flight). A kr_mp_design note says some MP/GCOPTER dependencies need you to email the maintainer for access (UNVERIFIED, search snippet).

### Algorithm (precise)
VERIFIED from arXiv:1709.05401 unless marked otherwise.
- **Model:** the state is x = [p, v, a, …] ∈ R^{3n}. The control u is the n-th derivative of position, so n=1 means velocity control, 2 acceleration, 3 jerk, 4 snap. The dynamics are ẋ = Ax + Bu, with A a block shift matrix and B = [0;…;0;I₃].
- **Cost (LQMT):** J = ∫₀ᵀ ‖u‖² dt + ρT, where ρ ≥ 0 trades control effort against time.
- **Primitives:** hold u_m constant for a fixed duration τ, which gives the closed form x(τ) = e^{Aτ}x₀ + G(τ)u_m. Each primitive is a polynomial of degree n in t.
- **Control discretization:** use μ samples per axis with step d_u = u_max/μ. U_M = {−u_max, …, 0, …, u_max}³ gives M = (2μ+1)³ primitives in 3D, or (2μ+1)² in 2D. The experiments used 9 in 2D and 27 in 3D, i.e. μ=1. The paper proves that starting from a lattice state yields a finite lattice, which is what makes a graph search possible.
- **Search:** A*. Edge cost = (‖u_m‖² + ρ)·τ. A successor is accepted only if (a) the per-axis extrema of v, a (and j) on [0,τ] satisfy ‖v‖∞ ≤ v_max, ‖a‖∞ ≤ a_max, and (b) positions sampled at t_i = iτ/I are collision-free in the voxel map, with (τ/I)·v_max ≤ grid resolution.
- **Heuristics:**
  - h₁ = ρ‖p_f − p₀‖∞ / v_max (a minimum-time bound).
  - h₂ = LQMT closed form. For n=2: C(T) = 12‖d‖²/T³ − 12(v₀+v_f)·d/T² + 4(‖v₀‖²+v₀·v_f+‖v_f‖²)/T + ρT, with d = p_f − p₀ and T* the positive root of dC/dT = 0.
  - Clearing denominators gives the quartic ρT⁴ − 4(‖v₀‖²+v₀·v_f+‖v_f‖²)T² + 24(v₀+v_f)·d·T − 36‖d‖² = 0 (INFERENCE: my own algebra).
  - On one test, Dijkstra took 0.16 s and expanded 2707 nodes, A*+h₁ took 0.064 s / 1282 nodes, and A*+h₂ took 0.016 s / 376 nodes.
- **Refinement:** an optional unconstrained QP minimizes Σ∫‖p^{(n)}‖² using the segment times and waypoints that the search returned (Mellinger–Kumar style). The paper notes the result may be unsafe or infeasible.
- **Reported runtimes (Intel NUC i7, Table I, seconds):**
  - 2D: acceleration-controlled 0.016 avg / 0.086 max; jerk-controlled 0.147 avg / 2.13 max.
  - 3D: acceleration-controlled 0.094 avg / 0.515 max; jerk-controlled 2.98 avg / 9.50 max.
  - Snap control was not evaluated.
  - Flights: replanning at 3 Hz in simulation at 3 m/s; real flights at 2 m/s on an AscTec Pelican with a Hokuyo laser.
- **SE(3) paper (RA-L 2018, VERIFIED via ar5iv):**
  - Robot model: an ellipsoid of radius r and height h; attitude comes from the thrust direction a + g·z_w with yaw fixed; collisions are checked against a point cloud using a KD-tree crop.
  - Hierarchical search: a cheap acceleration-controlled trajectory is used as an (inadmissible) heuristic for the jerk-controlled search.
  - Parameters: τ = 0.2 s, v_max = 7 m/s, a_max = 10 m/s², jerk u_max = 50 m/s³, ρ = 10000 (the table text in the source is garbled).
  - Runtimes: in one example, velocity / acceleration / jerk control took 2 ms / 60 ms / 1646 ms. In the office map, jerk directly took 89 s versus 2.03 s refined.
  - Gaps of 0.35–0.75 m were flown at roll up to 90° in simulation and 40° in real flight, on a Hummingbird with **VICON** and a prebuilt map.
- **Safe Flight Corridors (RA-L 2017):**
  - Pipeline: a JPS path → DecompUtil inflates ellipsoids along each segment into convex polytopes (the corridor) → a QP piecewise polynomial is solved inside those polytopes.
  - Replanning takes 50–300 ms on large cluttered maps (UNVERIFIED: abstract-level summary seen only via an aggregator).

### Example parameters in the repos
The MPL README's example uses dt = 1.0, v_max = 1.0, a_max = 1.0, u_max = 0.5, w (ρ) = 10, tol_dis = 0.5, ds = 0.01, dv/da/dj = 0.1, `heur_ignore_dynamics` = 1 (VERIFIED).

### Fit for this course and a simplified Python version (INFERENCE)
- **The planning problem is small.** The map is known and static, and v_max is effectively 1 m/s because PX4 MPC caps it. The sophistication of MPL (online replanning at 4–7 m/s, SE(3) gaps) buys little here. What *is* useful is the shape of the output: a time-parameterized, dynamically feasible path through the maze that you can sample into `/skybeta/setpoint/pose`. You can plan it **offline** on a laptop from the published 3D model, then check it in sim.
- **Do not port mpl_ros.** It is ROS 1, depends on catkin, SDL1.2 and Qt4, and was last substantively updated in 2019. Porting it to Humble inside the Isaac ROS container is a multi-day job.
- **A ~200-line Python reimplementation:**
  1. Voxelize the course mesh at 0.05–0.10 m (e.g. trimesh/open3d offline). Inflate obstacles by the vehicle radius (~0.25 m) plus a 0.10–0.15 m margin for VO drift. Plan in 2D at fixed altitude per segment, or in 3D for the window and tube.
  2. Use acceleration-controlled primitives (n=2): p(t) = p₀ + v₀t + ½ut², v(t) = v₀ + ut. Because v is linear in t, checking the endpoints is enough for the velocity bound.
  3. Parameters: τ = 0.25–0.5 s, u_max (accel) = 1.0–2.0 m/s², μ = 1–2 (9–25 primitives in 2D), v_max = 0.9 m/s (below MPC_XY_VEL_MAX, so PX4 never saturates and lags behind the plan), ρ = 10–1000 (higher means more time-optimal), collision sample spacing ≤ grid resolution.
  4. Search: A* with the n=2 LQMT heuristic above, solving the quartic with `numpy.roots` and taking the positive real root. Discretize states by rounding (p, v) to the lattice and hashing them in a dict. The goal is reached when ‖p − p_f‖ < tol and ‖v − v_f‖ < tol.
  5. Execution: sample the trajectory at 20–50 Hz and publish pose setpoints. Also send velocity feedforward if the organizers' twist interface accepts it. Because ρ is time-weighted, the optimal plan rides the v_max limit, which matches "speed is scored".
- **A cheaper alternative that may be equally good:** jps3d-style grid A*/JPS → a handful of waypoints → straight-line pose setpoints, with PX4's MPC (jerk-limited when MPC_POS_MODE uses smoothing) doing the smoothing (INFERENCE). Prefer MPL-style primitives only if corner-cutting or overshoot in the serpentine becomes a problem.

**Hardware:** CPU only, trivial at this map size. **Verdict: reimplement simplified, offline.** Read the IROS 2017 paper for the equations. Do not integrate the ROS 1 code. Do not use the SE(3)/ellipsoid planner: the gaps are much wider than the drone, and its demos relied on VICON.

---

## 2. ORB-SLAM3

**Links:** https://github.com/UZ-SLAMLab/ORB_SLAM3 · paper arXiv:2007.11898 (Campos, Elvira, Gómez, Montiel, Tardós, T-RO 2021).

- **Summary / I/O:** feature-based (ORB) visual and visual-inertial SLAM for mono, stereo or RGB-D cameras, with or without IMU. It takes images and IMU and outputs camera pose, a sparse map, and loop closures (VERIFIED, repo README).
- **License:** **GPLv3**. A closed-source commercial licence is available from the authors (VERIFIED).
- **Dependencies:** C++11, Pangolin, OpenCV ≥3.0 (tested 3.2 and 4.4), Eigen ≥3.1, bundled DBoW2/g2o. Tested on Ubuntu 16.04/18.04 with ROS **Melodic** (ROS 1) examples (VERIFIED).
- **Maintenance:** v1.0 is dated 22 Dec 2021, the last changelog entry. The repo has 542 open issues and 32 open PRs. It is effectively unmaintained upstream (VERIFIED for the counts and date; "unmaintained" is INFERENCE).
- **Accuracy (VERIFIED, ar5iv 2007.11898):**
  - EuRoC stereo-inertial: average RMS ATE **0.035 m** (MH05 worst at 0.082 m; V102/V202 best at 0.014 m).
  - TUM-VI room: 0.009 m.
  - Runtime on an i7-7700 desktop: tracking **33.05 ± 9.29 ms/frame**, local mapping 197 ms. The paper says it runs in real time at "30–40 frames" per second.
- **Atlas / tracking loss (VERIFIED, paper):**
  - A frame counts as visually lost when fewer than 15 map points are tracked.
  - Short-term recovery: IMU dead-reckoning plus a widened search window for **5 s**. After that it starts a *new* map.
  - Tracking loss within 15 s of IMU initialization discards the new map.
  - Relocalization uses DBoW2 + MLPnP against all maps, and maps are merged when place recognition matches.
- **In the dark tube (INFERENCE):** ORB detection fails in low light exactly as cuVSLAM's does, and 5 s of IMU coasting from a consumer BNO086 will drift. ORB-SLAM3 gives no advantage there. You would need illumination (onboard LEDs, if the rules allow) or to fly the tube on the IMU and VL53L1X alone.
- **Map save/load:** v1.0 "Added load/save map functionalities" (VERIFIED, Changelog.md). The settings keys `System.SaveAtlasToFile` / `System.LoadAtlasFromFile` are documented by the suchetanrs wrapper (VERIFIED, below). **Rule compliance:** never set LoadAtlasFromFile, delete any saved atlas between runs, and audit the YAML. Treat cuVSLAM's map save/load in Isaac ROS the same way (UNVERIFIED that the organizers' build exposes it).
- **ROS 2 Humble wrappers:**
  - **suchetanrs/ORB-SLAM3-ROS2-Docker:** Humble on Ubuntu 22.04, supports `stereo_imu`, has a CUDA build flag, and documents atlas save/load. It does not mention Jetson or aarch64, and no license is stated (VERIFIED).
  - **guyuehome/ORB_SLAM3_ROS2:** Humble, stereo-inertial node, no Jetson mention (UNVERIFIED, search snippet).
  - **Mechazo11/ros2_orb_slam3:** Humble, *monocular only*, tested only on x86_64, and it warns it may not work on Jetson Nano-class hardware. GPL-3 (VERIFIED).
  - **Multi-Agent-Robotics fork:** AGX Orin but ROS 2 Foxy (UNVERIFIED snippet).
  - None is proven on Orin Nano + Humble + stereo-inertial.
- **Orin-class runtime:** I found no published stereo-inertial FPS for stock ORB-SLAM3 on an Orin Nano. "Jetson-ORB-SLAM3" (arXiv:2608.17874, Aug 2026) puts ORB extraction on the GPU and reports **32 FPS mean, mono-inertial**, on an Orin Nano. No code link was given (VERIFIED, https://arxiv.org/abs/2608.17874). Stock CPU ORB-SLAM3 at 1024x640 stereo on the Orin Nano's 6× A78AE will likely run below 30 FPS and compete with ROS/PX4 bridge processes for the CPU (INFERENCE).
- **vs cuVSLAM:**
  - cuVSLAM paper Table 1: stereo tracking **1.8 ms** and stereo-inertial **3.8 ms** per frame on AGX Orin (768x480).
  - Table 2 on EuRoC: cuVSLAM SLAM stereo APE 0.054 vs ORB-SLAM3 0.068. Its ORB-SLAM3 stereo-inertial row (APE 0.066, 72° avgRE) conflicts with ORB-SLAM3's own 0.035 m and is probably a configuration artifact.
  - On the Orin Nano 25 W, mono-depth was stable at 30 FPS only at 424x240.
  - All VERIFIED, https://arxiv.org/html/2506.04359.
  - cuVSLAM is already integrated into EKF2 by the organizers, runs on the GPU, and leaves the CPU free.
- **Licence implications:** GPLv3 imposes nothing on internal or hackathon use. If you *distribute* a binary or Docker image containing it (e.g. hand the image to organizers or publish it), you must offer the corresponding source under GPLv3, and your linked code becomes GPL-covered. This is not legal advice (INFERENCE).

**Verdict: read only.** Do not replace or parallel cuVSLAM two days out: the CPU cost, ROS 1-era code, unproven Humble/aarch64 stereo-inertial builds, no advantage in darkness, and map-reuse risk all argue against it. A more useful lever (INFERENCE) is to ask the organizers whether cuVSLAM's stereo-inertial mode (BNO086 at 200 Hz) can be enabled with a validated camera-IMU calibration, to help through the tube. Note, though, that cuVSLAM's own EuRoC table shows stereo-inertial odometry slightly *worse* than stereo (0.19 vs 0.13 APE). Test before trusting it.

---

## 3. VGGT (Visual Geometry Grounded Transformer)

**Links:** https://vgg-t.github.io/ · https://github.com/facebookresearch/vggt · arXiv:2503.11651 (Wang, Chen, Karaev, Vedaldi, Rupprecht, Novotny; CVPR 2025 Best Paper).

- **Outputs (VERIFIED):** from 1 to hundreds of images, a single feed-forward pass predicts camera extrinsics and intrinsics, per-frame depth maps, point maps, and features for 3D point tracks (CoTracker2-style head). Unprojecting the depth through the predicted cameras is more accurate than the point-map head (ETH3D 0.677 vs 0.709).
- **Size:** about **1.2 B parameters**, inputs resized to a 518 px max side (VERIFIED).
- **Runtime/memory (H100, flash-attention v3, 336x518, Table 9, VERIFIED):**

| frames | 1 | 2 | 4 | 8 | 10 | 20 | 50 | 100 |
|---|---|---|---|---|---|---|---|---|
| time (s) | 0.04 | 0.05 | 0.07 | 0.11 | 0.14 | 0.31 | 1.04 | 3.12 |
| memory (GB) | 1.88 | 2.07 | 2.45 | 3.23 | 3.63 | 5.58 | 11.41 | 21.15 |

  A May 2026 repo update says a memory fix allows about 2–3x more frames for the same memory (VERIFIED).
- **Dependencies:** PyTorch, torchvision, huggingface_hub. bf16 on Ampere and newer, so Orin's sm_87 qualifies (VERIFIED for the bf16 rule; that Orin qualifies is INFERENCE).
- **License:** the custom "VGGT License v1" (Meta). It prohibits military uses, and derivatives must stay under the same terms. The **VGGT-1B** checkpoint is non-commercial, a separate **VGGT-1B-Commercial** exists, and checkpoint access requires an application form (VERIFIED).
- **Maintenance:** active. Training code was released Jul 2025, and VGGT-Omega was announced May 2026. VGGT-500M and VGGT-200M are listed as *not yet released* (VERIFIED).
- **Orin Nano 8 GB:**
  - No official Jetson support exists.
  - The weights alone are about 2.4 GB in fp16 and about 4.8 GB in fp32, in memory shared with Ubuntu, ROS, cuVSLAM and the camera pipeline. The H100 is roughly 30x the Orin's memory bandwidth (3.35 TB/s vs 102 GB/s) and far ahead on compute. Flash-attention v3 is Hopper-only.
  - Expected rate: about **1–3 s per 2-frame pass at best** in fp16 PyTorch, i.e. ≤1 Hz, and it would starve cuVSLAM (INFERENCE).
  - An NVIDIA forum report shows a VGGT TensorRT engine giving numerically wrong "flat plane" output on Orin NX/Nano with JetPack 6.2 / TRT 10.3 (VERIFIED as a user report, https://forums.developer.nvidia.com/t/.../365536).
  - VersaQ-3D (arXiv:2601.20317) states that VGGT's requirements "drastically exceed" Orin NX capability and reports W4A4 speedups of about 5.3–6x over Orin NX, but its absolute numbers appear only in figures (VERIFIED).
- **Lighter derivatives:**
  - FastVGGT (arXiv:2509.02560): training-free token merging, about 4x at 1000 frames. It helps long sequences, not 2-frame latency.
  - LiteVGGT (arXiv:2512.04939): claims about 10x.
  - QuantVGGT: 4-bit, 3.7x less memory, 2.5x faster.
  - StreamVGGT (arXiv:2507.11539): causal and streaming, still about 1 B params.
  - Depth Anything 3 Small: **0.08 B** params, model card says Apache-2.0, though another listing says CC BY-NC. It is the only one plausibly real-time on an Orin Nano.
  - All VERIFIED via search snippets or abstracts; none of their Jetson runtimes were found.
- **Realistic uses here (INFERENCE):**
  1. **Offline log analysis on a laptop GPU:** run it on recorded OAK-D frames after a practice run to get an independent camera trajectory and dense geometry. Use that to spot cuVSLAM drift or scale error near the tube and window.
  2. **Checking the course model:** reconstruct the as-built course from a phone video and compare it to the published 3D model (gap positions, tube offset) before tuning setpoints. This uses a capture of the *course*, not a flight map, but confirm with the organizers.
  3. **Onboard depth:** not with VGGT. Get stereo depth from the OAK-D's own on-device stereo engine by enabling the depth stream (UNVERIFIED that the organizers' driver config allows it), or from Isaac ROS ESS/stereo disparity on the Orin.
  4. **Relocalization against a pre-captured course reconstruction:** likely violates the no-reused-maps rule, and it is too slow anyway.

**Verdict: read only, plus optional offline use on a laptop.** Do not deploy it on the drone.

---

## 4. Other directly useful works (short)

- **Mellinger & Kumar, minimum-snap trajectories (ICRA 2011):** the refinement step in MPL. A min-snap or min-jerk polynomial through hand-picked gap-center waypoints is the simplest way to get a smooth, fast pose and twist stream through a known course (UNVERIFIED citation details; the method is standard). **Reimplement simplified.**
- **Falanga, Mueggler, Faessler, Scaramuzza, "Aggressive quadrotor flight through narrow gaps with onboard sensing and computing using active vision":** arXiv:1612.00291. Note the venue is **ICRA 2017, not IROS** (VERIFIED, https://arxiv.org/abs/1612.00291).
  - Results: 80% success at gap tilts up to 45°, 3 m/s, gaps 1.5x the vehicle size, single camera + IMU.
  - Key lesson: plan the approach so the camera **keeps the gap in view**, because relative pose uncertainty grows roughly quadratically with distance.
  - Applies directly here (INFERENCE): yaw toward the window and tube entrance on approach, slow down for the last ~1 m, and keep features in the forward camera's FOV. **Read only.**
- **Loianno, Brunner, McGrath, Kumar, RA-L 2017** (vol. 2, no. 2, pp. 404–411): a ~250 g quad with one downward VGA camera + IMU, VIO at up to 4.5 m/s and >1.5 g, flights through windows. The obstacle positions relative to the start were *given in advance* (VERIFIED via IEEE Spectrum and search). Its lesson matches this event: with a known map, good VIO and a smooth trajectory, no online mapping is needed. **Read only.**
- **KumarRobotics/kr_autonomous_flight:** a complete GPS-denied stack (VIO, mapping, JPS + MPL planning). ROS 1, with an experimental ROS 2 Jazzy branch, under the non-commercial Penn license (VERIFIED). Too heavy to adopt now. **Read only** for architecture.
- **Likely recommenders (INFERENCE):** the Vijay Kumar (UPenn GRASP) / Nikolay Atanasov (UCSD) lineage for MPL; Tardós/Montiel (Zaragoza) for ORB-SLAM3; Vedaldi/Rupprecht (Oxford VGG) and Novotny (Meta) for VGGT.

---

## Bottom line
- **Highest-value action:** an offline Python planner (A* over acceleration primitives or JPS waypoints → min-jerk) built on the published course model, with v_max at 0.9 m/s and obstacle inflation of about 0.35–0.4 m, streamed as pose setpoints. Add active-vision approach headings from Falanga et al.
- **Keep cuVSLAM.** ORB-SLAM3 and VGGT are read-only for this event (VGGT is fine offline on a laptop). Never load saved maps or atlases.

---

# Code and documentation sources: ROS 2 Humble + PX4 v1.16 + Orin Nano Super + OAK-D S2

Researched 2026-11-05. I read web pages only and did not run any code. Labels used:
- **VERIFIED (URL):** I read the claim on that page today.
- **UNVERIFIED:** from memory or from a secondary source.
- **INFERENCE:** my own reasoning.

## 0. What the Isaac ROS 3.2 base image already contains (affects items 1, 5 and 6)

`isaac_ros_common` release-3.2 `docker/Dockerfile.base` installs the following. VERIFIED (https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/release-3.2/docker/Dockerfile.base):
- `python3-scipy` (apt) and `scipy>=1.7.0` (pip)
- `numpy>=1.24.4,<2`
- `python3-opencv=4.5.4+dfsg-9ubuntu4` (the Ubuntu apt build)
- `trimesh`, `scikit-image` and `cupy-cuda12x` (pip)
- `libnvvpi3` and `vpi3-dev` 3.2.4 (arm64 .deb)

The VPI install block contains no Python bindings package.

Consequences:
- **scipy and trimesh:** both are available, so no pure-numpy fallback is needed inside this image.
- **OpenCV:** the apt build almost certainly has no CUDA, so the `cv2.cuda` StereoSGM is not available. (INFERENCE)
- **VPI from Python:** you need `sudo apt install python3.10-vpi3`. The VPI 3.2 install docs name this package for Ubuntu 22.04 and say numpy must be installed separately. VERIFIED (https://docs.nvidia.com/vpi/3.2/installation.html, via search summary). Inside the container this needs the L4T apt repository. (INFERENCE)

## 1. Stereo depth from the raw OAK pair

### A. OpenCV StereoSGBM / StereoBM (CPU)
- **Licence:** Apache-2.0 for OpenCV 4.5 and later. (UNVERIFIED; docs.opencv.org returned HTTP 403 to my fetch)
- **API:**
  - `cv2.stereoRectify(K1,D1,K2,D2,size,R,T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)` returns `R1,R2,P1,P2,Q,...`
  - `cv2.initUndistortRectifyMap(K, D, R, P, size, cv2.CV_16SC2)` and `cv2.remap(img, m1, m2, cv2.INTER_LINEAR)`
  - `cv2.StereoSGBM_create(minDisparity, numDisparities, blockSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, mode)`
  - `cv2.StereoBM_create(numDisparities, blockSize)`
  - `cv2.reprojectImageTo3D(disp, Q)`
  (UNVERIFIED, standard OpenCV API)
- **Using ROS CameraInfo directly:** with ROS stereo conventions, `CameraInfo.r` is the rectification rotation and `CameraInfo.p` is the rectified projection. So `initUndistortRectifyMap(K, D, R, P[:3,:3], ...)` per camera rectifies without calling `stereoRectify`. Baseline = `-P_right[0,3]/P_right[0,0]`. Depth Z = fx·B/d. (UNVERIFIED convention, standard ROS image_geometry)
- Check whether depthai-ros fills R and P with real rectification or identity. If they are identity, compute R and T from the device calibration and call `stereoRectify`. (INFERENCE)
- **Parameters at 512x320 (half of 1024x640):** (INFERENCE / common practice)
  - Convert bgr8 to gray first, or use `/oak/left/image_mono`.
  - Downscale with `cv2.resize(INTER_AREA)`.
  - Scale K and P rows 0 and 1 by 0.5.
  - `numDisparities=64` (must be a multiple of 16), `blockSize=5`, `P1=8*1*5**2`, `P2=32*1*5**2`, `uniquenessRatio=10`, `speckleWindowSize=100`, `speckleRange=2`, `mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY`.
  - Output is int16 fixed-point (×16), so divide by 16.0.
  - With f≈0.5×(full-resolution fx) and B=7.5 cm, d=64 corresponds to roughly 0.5 m minimum range. Check fx in your camera_info.
- **Runtime on ARM:** I found no Orin Nano benchmark. A TX2 paper reports about 27 fps for "OpenCV" stereo at VGA but does not name the matcher. (VERIFIED weakly via search summary: koreascience.kr CFKO201815540966637)
  - SGBM_3WAY at 512x320 with 64 disparities on 6 A78AE cores should give about 10–20 Hz; StereoBM is roughly 3–5× faster. (INFERENCE; benchmark on the device)
- **Pitfalls:**
  - DepthAI calibration uses an 8-coefficient rational model. cv2 accepts 8 D coefficients. (INFERENCE)
  - Left and right frames must be time-synchronised: use `message_filters.ApproximateTimeSynchronizer`.
  - The dark tube and plain white floor are low-texture, so expect holes there.
- **Alternative with no code:** `stereo_image_proc` (ros-humble-stereo-image-proc) DisparityNode uses OpenCV BM/SGBM on the CPU. (UNVERIFIED)

### B. NVIDIA VPI stereo disparity
- **Docs:** https://docs.nvidia.com/vpi/3.2/algo_stereo_disparity.html. JetPack 6 ships VPI 3.2.
- **Python call:** VERIFIED from the page:
  ```python
  with vpi.Backend.CUDA:
      out = vpi.stereodisp(left, right, window=5, maxdisp=64).convert(vpi.Format.U8, scale=255.0/(32*64))
  ```
  - Inputs must be rectified.
  - Disparity output is S16. The scale above implies 5 fractional bits, so divide by 32 for pixels.
  - Wrap numpy arrays with `vpi.asimage(arr)` and read results back with `.cpu()` or `.lock_cpu()`. (UNVERIFIED names)
- **Backend constraints:** VERIFIED (https://docs.nvidia.com/vpi/3.2/group__VPI__StereoDisparityEstimator.html)
  - **CUDA:** maxDisparity ≤ 256. Window is ignored (a fixed 9x7 census is used). downscale must be 1. Output S16, confidence U16. Supports `uniqueness`, `minDisparity`, `p1` and `p2` (P2 < 256).
  - **OFA:** maxDisparity must be 128 or 256. downscale 1/2/4/8. Output S16_BL. No confidence output.
  - **OFA+PVA+VIC:** window 1/3/5/7.
  - In the Python API the keyword names correspond to `confthreshold`, `p1`, `p2`, `uniqueness` and `mindisp`. (UNVERIFIED; only `window` and `maxdisp` are shown)
- **Orin Nano hardware:**
  - No PVA and no DLA. Connect Tech and Holybro module tables show "-" for the Vision Accelerator. (VERIFIED via search summary, https://connecttech.com/orin-module-comparison/)
  - OFA is disputed. Release notes for VPI 2.0 said OFA was AGX-Orin-only. One forum user reports the VPI stereo sample running on OFA on an Orin Nano, but no NVIDIA staff confirmed it. VERIFIED (https://forums.developer.nvidia.com/t/does-jetson-orin-nano-have-an-optical-flow-accelerator/309323)
  - **Use the CUDA backend.** OFA+PVA+VIC is impossible on Orin Nano because there is no PVA. (INFERENCE)
- **ROS wrapper:** `isaac_ros_stereo_image_proc` DisparityNode with `backend` CUDA (default) or JETSON. JETSON requires 128 or 256 max disparity and NV12 input. Defaults: `max_disparity` 256, `window_size` 7, `p1` 8, `p2` 120. It subscribes to `left/image_rect`, `left/camera_info`, `right/image_rect` and `right/camera_info`, and publishes `disparity` (stereo_msgs/DisparityImage). VERIFIED (https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_pipeline/isaac_ros_stereo_image_proc/index.html)
  - Use backend CUDA on Orin Nano. (INFERENCE)

### C. Isaac ROS ESS (DNN stereo)
- **Published numbers** on the current (release-5.0) performance page, Orin Nano Super 8GB column. VERIFIED (https://nvidia-isaac-ros.github.io/performance/index.html)
  - Full ESS at 576p: 34.8 fps, 30 ms
  - Light ESS at 288p: 82.2 fps, 13 ms
- I could not extract the release-3.2 table. Its numbers may differ.
- ESS is a C++/NITROS graph: RectifyNode then ESS, with TensorRT engine generation on first run. The model is downloaded from NGC under NVIDIA model terms. (UNVERIFIED licence)
- It adds GPU and memory load next to cuVSLAM, which already uses the same images. (INFERENCE)

### Recommendation (INFERENCE)
- For Python at 10 Hz or more, the most practical choice is the VPI CUDA `stereodisp` on 512x320 rectified mono frames at maxdisp 64. Rectify once with cv2 maps, or with VPI `remap`. Expect it to take only a few ms.
- OpenCV SGBM_3WAY at 512x320 is the zero-dependency fallback. Measure it before relying on it.
- Use Light ESS only if you accept a C++ launch graph.
- For window and tube approach you can skip dense depth: a sparse disparity check inside an ROI is enough.

## 2. Line following (dark-red 2-inch tape)
- **Pipeline:** (INFERENCE / standard)
  1. Threshold in HSV or Lab. Red wraps around hue 0, so use two `cv2.inRange` bands, or threshold Lab a* above T.
  2. Clean the mask with `cv2.morphologyEx` OPEN/CLOSE.
  3. Take the largest contours, or the mask pixels in the bottom ROI.
  4. Fit `cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)`, which returns (vx, vy, x0, y0).
  5. Project to the ground and compute cross-track error and heading error, which feed the velocity and yaw-rate setpoint.
- **Note:** the OAK sensors are mono (bgr8 is grey replicated), so colour thresholding is impossible. Dark red reads as dark grey. Use an adaptive threshold or `THRESH_OTSU` on darkness plus a width check (2 in ≈ 5 cm) after IPM. (INFERENCE, but follows from OV9282 being mono)
- **Ground-plane projection (IPM):** a pixel ray r = R_wc·K⁻¹[u,v,1]ᵀ intersects the floor z=0 at p = c − (c_z/r_z)·r, where c is the camera position (height from cuVSLAM or the rangefinder) and R_wc comes from the cuVSLAM attitude plus the camera extrinsic pitch.
  - Equivalent homography: H = K·[r1 r2 t] for the plane, and `cv2.warpPerspective` gives a bird's-eye view. VERIFIED concept (https://www.ce.unipr.it/people/medici/geometry_en/node154.html)
  - Use the full attitude, not a fixed pitch: the quad tilts 5–15° when it moves. (INFERENCE)
  - A forward camera sees the floor only beyond a minimum distance set by its pitch and vertical FOV. A 2-inch line far away is only a few pixels wide.
- **ROS library:** `ros-sports/ipm` provides ipm_library, ipm_interfaces, ipm_service and ipm_image_node. It maps image points onto a plane using CameraInfo and TF. Apache-2.0, Humble and later. VERIFIED (https://github.com/ros-sports/ipm, docs https://ipm-docs.readthedocs.io). Python class names were not verified.
- **Example repos (rclpy, cv_bridge, ground robots):**
  - Tom Howard's ROS 2 course, Part 6 line follower: https://tom-howard.github.io/ros2/course/part6/line_follower
  - isa1asn/line_following_robot_ros2 (Humble, Gazebo, small)
  - Drone-specific: BWSI UAV line following notes (https://bwsi-uav.github.io/website/line_following.html), not ROS 2
  - All found via search. VERIFIED existence only; licences not checked.

## 3. Landing pad (black concentric rings with a cross on white)
- **API (OpenCV, UNVERIFIED signatures):**
  - `cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT_ALT, dp=1.5, minDist, param1=300, param2=0.9, minRadius, maxRadius)`. The ALT variant exists in 4.5 or later.
  - `cv2.findContours(..., cv2.RETR_TREE, ...)` then `cv2.fitEllipse(cnt)` for contours with at least 5 points.
  - Group ellipses whose centres lie within a few px of each other. The ratios of the major axes should match 9:20:30 in, which rejects false positives.
  - `cv2.HoughLinesP` finds the cross for yaw.
- **Why ellipses:** a forward or oblique view turns the rings into ellipses, so fitEllipse plus the RETR_TREE hierarchy is more robust than HoughCircles. Keipour et al., "Real-Time Ellipse Detection for Robotics Applications" (RA-L 2021, arXiv 2102.12670), is a UAV landing use case; code is said to be at https://theairlab.org/landing-on-vehicle (licence not checked). VERIFIED paper exists via search.
- **Ground position:**
  - Intersect the ray through the ellipse centre with z=0, as in item 2. (INFERENCE)
  - Better: the image of the circle centre is not the ellipse centre under perspective. Use `cv2.solvePnP` on 4 or more points sampled on the known 30-inch ring with matching ellipse points, or on the cross endpoints. Alternatively, range = f·R/semi-major axis as a check. (INFERENCE)
  - When directly overhead at under 1 m, the outer ring leaves the forward FOV, so switch to the inner 9-inch ring or the cross.
- **Academic ROS/Python examples:** UMD ENAE788M bullseye reports: https://prgaero.github.io/Reports/p3b/holumerik.pdf and https://prgaero.github.io/Reports/p3b/rehmnicholas.pdf (Hough on a binary mask plus fitEllipse). VERIFIED via search.

## 4. Time parameterisation

### TOPP-RA (https://github.com/hungpham2511/toppra)
- **Licence:** MIT. `pip install toppra`. VERIFIED (repo)
- The README warns that "Python support for the library will be dropped" in favour of the C++ library with bindings. VERIFIED (repo)
- **PyPI:** latest is 0.6.11. The files list has only x86-64 wheels for cp39–3.14: no aarch64 wheel and no sdist listed. VERIFIED (https://pypi.org/project/toppra/)
  - So pip on Jetson will either fail or fall back to an older sdist and need Cython plus a compiler. (INFERENCE)
- **API:** VERIFIED (https://hungpham2511.github.io/toppra/auto_examples/plot_kinematics.html)
  ```python
  import toppra as ta, toppra.constraint as constraint, toppra.algorithm as algo
  path = ta.SplineInterpolator(ss, way_pts)            # way_pts: N x dof (x,y,z[,yaw])
  pc_vel = constraint.JointVelocityConstraint(vlims)   # per-axis |v|
  pc_acc = constraint.JointAccelerationConstraint(alims)
  inst = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
  traj = inst.compute_trajectory()   # traj.duration; traj(t), traj(t,1), traj(t,2)
  ```
- `JointAccelerationConstraintVarying` and `ParameterizeTrajectory` do not appear in that example. I could not verify either name. (UNVERIFIED)
- Per-axis limits form a box, not a Euclidean norm limit. Use conservative values, e.g. a/√3. (INFERENCE)

### Simpler forward-backward profile (pure numpy, recommended for this team; INFERENCE, standard method)
1. Resample the path at equal arc length ds.
2. Compute curvature κᵢ, then v_capᵢ = min(v_max, √(a_lat/κᵢ)).
3. Set v₀ and v_N to their boundary values.
4. Forward pass: vᵢ₊₁ = min(vᵢ₊₁, √(vᵢ² + 2·a_max·ds)).
5. Backward pass: vᵢ = min(vᵢ, √(vᵢ₊₁² + 2·a_max·ds)).
6. Time: tᵢ₊₁ = tᵢ + 2ds/(vᵢ + vᵢ₊₁).

About 30 lines, no dependencies, and it is the same result as TOPP for a point mass with a friction-circle limit.

### Ruckig (https://github.com/pantor/ruckig)
- **Licence:** MIT. `pip install ruckig`. VERIFIED (repo)
- **PyPI:** 0.19.4 (2026-07-15). No aarch64 wheels, only an sdist, so it builds on Jetson with cmake and a C++ compiler. VERIFIED file list (https://pypi.org/project/ruckig/); build needs are INFERENCE.
- **API:** VERIFIED (https://raw.githubusercontent.com/pantor/ruckig/main/examples/01_position.py)
  ```python
  from ruckig import InputParameter, OutputParameter, Result, Ruckig
  otg = Ruckig(3, 0.02); inp = InputParameter(3); out = OutputParameter(3)
  inp.current_position/velocity/acceleration = ...; inp.target_position/velocity/acceleration = ...
  inp.max_velocity, inp.max_acceleration, inp.max_jerk = [...], [...], [...]
  while otg.update(inp, out) == Result.Working:
      send(out.new_position, out.new_velocity, out.new_acceleration); out.pass_to_input(inp)
  ```
  - `out.trajectory.duration` is available.
  - Offline: `otg.calculate(inp, traj)` with `Trajectory(dof)` and `traj.at_time(t)`. VERIFIED names (repo README)
- Intermediate waypoints are a Pro/cloud feature in the Community version. (UNVERIFIED)
- Good for state-to-state segments such as hover to window entry. Feed `new_velocity` and `new_acceleration` as feed-forward in TrajectorySetpoint. (INFERENCE)

## 5. A*/Theta* and distance transform
- **scipy:** `scipy.ndimage.distance_transform_edt` is available in the Isaac ROS 3.2 base image (§0). Plain `ros:humble` images do not include scipy by default; `apt install python3-scipy` adds it. (UNVERIFIED)
- **No-scipy option:** `cv2.distanceTransform(free_u8, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)` gives an exact Euclidean distance transform, which is fast and already in cv2. (UNVERIFIED flag semantics, standard OpenCV)
- **Library:** `pathfinding` (https://github.com/brean/python-pathfinding). MIT, `pip install pathfinding`. Pure Python. Algorithms: A*, Dijkstra, Best-First, bi-directional A*, BFS, IDA*. No Theta*. VERIFIED (repo)
  - Slow on large grids. A custom `heapq` A* over a numpy grid at 5–10 cm resolution is simplest. (INFERENCE)
- **Theta*** (INFERENCE): same as A*, but when line-of-sight(parent(s), s′) holds, relax via parent(s). The line-of-sight check is a Bresenham walk over the inflated occupancy grid. Alternatively, run A* and then a string-pulling shortcut pass. About 60 lines total.
- Inflate obstacles with distance-transform values above vehicle radius plus margin. A 7-inch quad is about 0.25 m tip to tip. Add a cost term w/(d+ε) to keep away from walls.

## 6. OBJ loading
- **trimesh:** already pip-installed in the Isaac ROS 3.2 base (§0, VERIFIED). trimesh is pure Python with numpy as the only hard dependency, so `pip install trimesh` works on aarch64. (UNVERIFIED but typical)
  - API: `trimesh.load(path, force='mesh')`, `.vertices`, `.faces`, `mesh.section(plane_origin, plane_normal)` to slice at flight height, and `.voxelized(pitch)`. Section and voxelise are UNVERIFIED names.
- **Pure numpy parser** (INFERENCE):
  - Read lines starting with `v ` into float arrays.
  - For lines starting with `f `, split on spaces, take `int(tok.split('/')[0])`.
  - Handle negative indices (relative) and 1-based indices, and fan-triangulate polygons.
  - Then rasterise to a 2D occupancy grid: mark triangles that straddle z_flight, or project all vertices between z_min and z_max with `cv2.fillPoly` per face.
- **Pitfalls:** OBJ units (inches vs metres) and the Y-up vs Z-up export convention.

## 7. Gazebo Harmonic world with a mesh, PX4 v1.16 SITL
- **Gazebo:** Harmonic is the PX4 target. VERIFIED (https://docs.px4.io/main/en/sim_gazebo_gz/)
  - Humble officially pairs with Fortress. Harmonic plus Humble is "possible, use with caution" via `apt-get install ros-humble-ros-gzharmonic` from packages.osrfoundation.org, which conflicts with ros-humble-ros-gz*. VERIFIED (https://gazebosim.org/docs/harmonic/ros_installation/)
- **Mesh formats:** gz-common 5 (Harmonic) MeshManager loads STL, COLLADA, OBJ, glTF/GLB and FBX. GLB and FBX use assimp; `GZ_MESH_FORCE_ASSIMP` forces assimp for all formats. VERIFIED (https://gazebosim.org/api/common/5/classgz_1_1common_1_1MeshManager.html)
- **World file:**
  - Copy `worlds/default.sdf` from PX4-gazebo-models. It has world name `default`, `max_step_size` 0.004, WGS84 spherical_coordinates, ENU, and a ground plane, with no `<plugin>` elements. VERIFIED (https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/worlds/default.sdf)
  - Plugins therefore come from PX4's server.config. (INFERENCE: keep it that way)
  - Rename the world to `course` and add:
  ```xml
  <model name="course"><static>true</static><pose>0 0 0 0 0 0</pose>
    <link name="link">
      <visual name="v"><geometry><mesh><uri>model://course/meshes/course.glb</uri><scale>1 1 1</scale></mesh></geometry></visual>
      <collision name="c"><geometry><mesh><uri>model://course/meshes/course.obj</uri></mesh></geometry></collision>
    </link></model>
  ```
  (INFERENCE / standard SDF 1.9)
  - Put the file at `Tools/simulation/gz/worlds/course.sdf`, or use an absolute `file:///` URI or a `GZ_SIM_RESOURCE_PATH` entry.
  - Use a simplified OBJ for collision.
  - Add `<scale>0.0254 0.0254 0.0254</scale>` if the mesh is in inches.
- **Commands:** VERIFIED (PX4 docs above and https://docs.px4.io/v1.16/en/ros2/user_guide)
  ```
  cd PX4-Autopilot && make px4_sitl gz_x500_depth            # SYS_AUTOSTART 4002, default world
  PX4_GZ_WORLD=course make px4_sitl gz_x500_depth            # custom world
  PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0" ...                      # spawn pose: x,y,z,r,p,y
  MicroXRCEAgent udp4 -p 8888                                 # Agent v2.4.3 from source, or snap
  ```
  - For the `make px4_sitl gz_x500_depth_course` target style, the docs also say to add the world name to `src/modules/simulation/gz_bridge/CMakeLists.txt`. VERIFIED
- **Camera in x500_depth:**
  - It includes `OakD-Lite` at pose 0.12 0.03 0.242. VERIFIED (https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/models/x500_depth/model.sdf)
  - Sensors: RGB camera `IMX214` (1920x1080, 30 Hz, no `<topic>`, so it uses the default scoped gz topic) and `StereoOV7251` depth_camera with `<topic>depth_camera</topic>` (640x480 R_FLOAT32). VERIFIED (https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/models/OakD-Lite/model.sdf)
  - It does **not** simulate a mono stereo pair. To test the real pipeline, make a model with two `camera` sensors (L8 format, 1024x640, 30 Hz, 7.5 cm apart, topics `oak/left/image_raw` and `oak/right/image_raw`). (INFERENCE)
- **Bridge:**
  ```
  ros2 run ros_gz_bridge parameter_bridge /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image \
     /world/course/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
     /world/course/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
     --ros-args -r <gz_topic>:=/oak/left/image_raw
  ```
  - The basic syntax `topic@ROS_type@gz_type` is VERIFIED (ros_gz humble README). `[` means gz→ROS only (UNVERIFIED for this branch).
  - The scoped IMX214 topic path is UNVERIFIED: confirm with `gz topic -l`. Forum threads report the depth topic hard-coded to `/depth_camera`. VERIFIED (https://discuss.px4.io/t/gz-sim-multiple-x500-depth-cameras-bridged-to-ros2/32779)

## 8. ROS 2 + PX4 offboard state machines
- **Official Python example:** `px4_ros_com/src/examples/offboard_py/offboard_control.py`. VERIFIED (https://raw.githubusercontent.com/PX4/px4_ros_com/main/src/examples/offboard_py/offboard_control.py)
  - QoS: BEST_EFFORT, TRANSIENT_LOCAL, KEEP_LAST, depth 1.
  - Publishes `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` and `/fmu/in/vehicle_command`.
  - Subscribes `/fmu/out/vehicle_local_position` and `/fmu/out/vehicle_status`.
  - 10 Hz timer: after 10 setpoints it sends DO_SET_MODE (param1=1, param2=6), then ARM_DISARM (param1=1). It climbs to z=-5 NED, then sends `VEHICLE_CMD_NAV_LAND` and exits.
  - Commands set target_system=1, target_component=1, `from_external=True`.
  - The file has no licence header; the repo is BSD-3-Clause. (UNVERIFIED)
  - C++ version: `src/examples/offboard/offboard_control.cpp`. Its docs say PX4 leaves offboard if the OffboardControlMode stream drops below about 2 Hz. VERIFIED (https://docs.px4.io/main/en/ros2/offboard_control)
- **ARK-Electronics/ROS2_PX4_Offboard_Example** (Humble, PX4 release/1.15, Python velocity_control.py with keyboard teleop): Space arms, then it takes off and switches to offboard; landing disarms; re-arm with Space. It does not use VehicleLandDetected. It is deprecated in favour of a C++ Custom Modes example (ros2_px4_teleop_example). Licence not shown. VERIFIED (https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example)
- **Jaeyoung-Lim/px4-offboard:** BSD-3-Clause, Python offboard position control plus an RViz visualiser. No landing logic. VERIFIED (https://github.com/Jaeyoung-Lim/px4-offboard)
- **Marnonel6/ROS2_offboard_drone_control:** Humble, Python and C++ path following. Exists per search; not inspected.
- **v1.16 message facts:** VERIFIED (raw msg files on release/1.16)
  - In `msg/versioned/VehicleStatus.msg`, `MESSAGE_VERSION = 1`, `NAVIGATION_STATE_OFFBOARD = 14`, `ARMING_STATE_ARMED = 2`.
  - VehicleCommand, VehicleLandDetected, VehicleLocalPosition and TrajectorySetpoint are version 0.
  - VehicleLandDetected has `landed`, `maybe_landed`, `ground_contact` and `freefall`.
  - Command IDs: ARM_DISARM=400, DO_SET_MODE=176, NAV_LAND=21, NAV_TAKEOFF=22.
  - The DDS client appends `_v<N>` for non-zero versions. VERIFIED (https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html)
  - **So on v1.16 the status topic should be `/fmu/out/vehicle_status_v1`.** (INFERENCE; confirm with `ros2 topic list`)
  - Use the px4_msgs branch matching 1.16; otherwise run the message translation node. Subscribers need sensor-data-compatible QoS. VERIFIED (v1.16 user guide)
- **Suggested state machine** (INFERENCE):
  1. INIT: wait for local position and status.
  2. PREFLIGHT: stream setpoints at 20–50 Hz holding the current position for 1 s or more.
  3. OFFBOARD + ARM: send commands, retry until nav_state == 14 and arming_state == 2.
  4. TAKEOFF: ramp the z setpoint.
  5. MISSION: waypoints, time-parameterised trajectory, line or window logic.
  6. LAND: either a descent setpoint over the pad in offboard, or NAV_LAND.
  7. Wait for `landed`, then DISARM (or let `COM_DISARM_LAND` auto-disarm; UNVERIFIED).
  8. RE-ARM: restart setpoint streaming before switching to offboard and arming again.
- **Notes:**
  - Keep the OffboardControlMode heartbeat in every state.
  - Timestamps are in µs: `self.get_clock().now().nanoseconds // 1000`.
  - Convert cuVSLAM ENU to NED: x_n = y_e, y_n = x_e, z_n = -z_e, ψ_n = π/2 - ψ_e.
  - For PX4 to use cuVSLAM, publish it to `/fmu/in/vehicle_visual_odometry` (VehicleOdometry, listed in v1.16 dds_topics.yaml, VERIFIED) and set the EKF2_EV_* parameters.
  - For a v1.15/1.16-native alternative, see Auterion px4-ros2-interface-lib (C++ custom modes). (UNVERIFIED details)
