# Browser viewer

The local viewer displays the **actual Gazebo Harmonic desktop** through noVNC and reads ground-truth mission telemetry from `artifacts/gazebo/telemetry.jsonl`. It does not render a browser simulation. Each poll reads a bounded head and tail of the JSONL; the course writer also maintains `artifacts/gazebo/telemetry.summary.json` so the final mission result remains available during a long hold after landing.

Start the Gazebo runtime in one terminal:

```sh
./runtime/run-mac.sh
```

When its container is up, start the viewer in another terminal:

```sh
./web/run.sh
```

Open [http://127.0.0.1:8081](http://127.0.0.1:8081). The embedded Gazebo GUI is served by the runtime at `http://127.0.0.1:8080/vnc.html?autoconnect=1&resize=scale`; use **Open Gazebo full window** to view it directly. The viewer does not start Gazebo or a mission.

`GET /api/state` returns the newest valid `sample`, plus metadata, summary, and recent events. It reports `waiting` if there is no usable sample and `stale` when the telemetry file has not changed for three seconds. When a summary exists, its elapsed mission time and result stay latched while the vehicle holds position. A `done` state is shown only when a sample explicitly contains `done: true`. Labels distinguish simulator ground truth from PX4 or physical-drone data.

The runtime script starts the dedicated ARM64 Colima profile and publishes the Gazebo GUI at port 8080. The browser viewer is a separate process on port 8081.
