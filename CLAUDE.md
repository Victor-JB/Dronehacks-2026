# DroneHacks — `jetson` branch (real drone)

This branch runs on the physical skybeta drone. Read `README.md` first, and on
the drone `~/skybeta/docs/competitor_guide.md` and `~/skybeta/CLAUDE.md`.
The simulator workspace and its instructions live on `main`.

- Talk only to skybeta's contract: subscribe `/visual_slam/tracking/odometry`,
  `/skybeta/state`, `/visual_slam/status`, and read-only `/fmu/out/*`; publish
  `/skybeta/setpoint/pose` or `/skybeta/setpoint/twist`. **Never publish
  `/fmu/in/*`**, never arm, never request a mode, never add `px4_msgs` to the
  workspace.
- Keep `refnav/core.py` and `refnav/depth.py` identical to `main`; put
  drone-specific behaviour in `dronehacks_nav/`. Never restore the sim-only files
  this branch deleted (`refnav/node.py`, `onboard/`, `main.py`).
- Never run flight code merely to inspect it. Ask the team to approve a mission
  before the pilot arms. First runs of new code: props off. A safety pilot on RC
  is always required.
- Do not edit `~/skybeta` on the drone (organiser-owned, overwritten on update).
  Team code lives in `~/team_ws/src/dronehacks`.
- Restarting the stack re-zeroes the VIO origin: only on the ground, drone still.
- Never put secrets, passwords, keys or cookies in code, prompts or Git. Never SSH
  into another team's drone.
