# Forward RGBD noise diagnostic

This opt-in SITL trial adds zero-mean Gaussian noise with 0.010 m standard deviation to the forward RGBD depth render. It keeps the downward range sensor, camera geometry, route, airframe, PX4 settings, and frozen checkpoint fixed. `SIM_RGBD_DEPTH_NOISE=off` is the default. Use `control` and `gaussian-10mm` for the paired diagnostic. Both use Gazebo server seed `4417`.

The wrapper inserts `<camera><noise>` only in the generated `/tmp/dronehacks-px4.sdf` before Gazebo starts. The navigation audit hashes that exact world and the unchanged recursively included models. The mode, seed, configured renderer, topic, and before/after world hashes are recorded in `/tmp/dronehacks-rgbd-depth-noise.json` and printed at startup. The seed makes the setup reproducible, while actual image bytes depend on the renderer and execution order.

Gazebo Harmonic reads the camera Gaussian noise setting and attaches it to the RGBD depth camera's Ogre2 render path. This tests sensitivity to idealized depth noise. Stereo disparity confidence, missing returns, and occlusion-edge behavior require separate measurements. [SDFormat camera schema](https://sdformat.org/spec/1.10/sensor/#sensor_camera_noise) · [Harmonic RGBD sensor source](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/RgbdCameraSensor.cc#L315-L334) · [Ogre2 depth noise source](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8/ogre2/src/Ogre2DepthCamera.cc#L1282-L1306) · [Luxonis StereoDepth](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth)

## Unarmed static verification

Use the dedicated marker-free simulator and a new evidence directory. The following Bash commands recreate that simulator twice. Run them only after any active run has landed and disarmed. Keep the launch fixture stationary and unarmed for both captures.

```bash
evidence=artifacts/markerfree/rgbd-noise-static-01
mkdir "$evidence"
docker_context=${SIM_DOCKER_CONTEXT:-colima-dronehacks-sim}
sim_container=dronehacks-markerfree-sim-1

capture_noise_case() {
    local noise_case=$1
    SIM_RGBD_DEPTH_NOISE="$noise_case" ./runtime/run-markerfree.sh || return
    docker --context "$docker_context" exec "$sim_container" \
        python3 -m vision.capture_rgbd --frames 24 --timeout 45 \
        --output "/workspace/$evidence/$noise_case.npz" || return
    docker --context "$docker_context" exec "$sim_container" \
        cat /tmp/dronehacks-rgbd-depth-noise.json > "$evidence/$noise_case.config.json" || return
    docker --context "$docker_context" exec "$sim_container" \
        python3 -m mission.navigation_audit > "$evidence/$noise_case.audit.json"
}

capture_noise_case control && capture_noise_case gaussian-10mm
```

The two raw float32 depth captures are taken from `/dronehacks/rgbd/depth_image`, which supplies navigation. Use a Python environment with NumPy to verify them.

```bash
python3 -m tools.check_gazebo_depth_noise verify "$evidence"
```

The default image patch starts at pixel `(176, 64)` and spans `32 × 32` pixels. This patch looked at a planar wall in the earlier launch captures. If the frame differs, use `--patch X Y W H` with a finite planar patch common to both captures and record its coordinates. The check requires an audit match for each configured world, an unchanged include graph, a stable planar control, consistent mean depths, and 6–16 mm of added temporal depth variation. It writes `verification.json` and exits nonzero on failure. Preserve both NPZ files, manifests, audits, and the report with the trial. The ordinary flight PNG archive quantizes depth to millimeters.

Proceed to a noisy flight only after the static check passes. Preserve the same `gaussian-10mm` config and compare with a matched `control` flight using the same checkpoint and route. Score waypoint order, envelope clearance, the window crossing, and landing with the independent observer. Keep simulator state separate from navigation.
