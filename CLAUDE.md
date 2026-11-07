# DroneHacks — `main` branch (real drone)

This branch runs on the physical skybeta drone (`jetson` is its old name, kept
equal to it). Read `README.md` first, and on the drone
`~/skybeta/docs/competitor_guide.md` and `~/skybeta/CLAUDE.md`. The simulator
lives on `victor/sim` and flies this branch's code (`SYSTEM.md` §10).

- Talk only to skybeta's contract: subscribe `/visual_slam/tracking/odometry`,
  `/skybeta/state`, `/visual_slam/status`, and read-only `/fmu/out/*`; publish
  `/skybeta/setpoint/pose` or `/skybeta/setpoint/twist`. **Never publish
  `/fmu/in/*`**, never arm, never request a mode, never add `px4_msgs` to the
  workspace.
- Mission logic goes in `refnav/` (the sim flies it too); drone-specific
  behaviour in `dronehacks_nav/`. Never restore the sim-only files
  this branch deleted (`refnav/node.py`, `onboard/`, `main.py`).
- Never run flight code merely to inspect it. Ask the team to approve a mission
  before the pilot arms. First runs of new code: props off. A safety pilot on RC
  is always required.
- Do not edit `~/skybeta` on the drone (organiser-owned, overwritten on update).
  Team code lives in `~/team_ws/src/dronehacks`.
- Restarting the stack re-zeroes the VIO origin: only on the ground, drone still.
- Never put secrets, passwords, keys or cookies in code, prompts or Git. Never SSH
  into another team's drone.
