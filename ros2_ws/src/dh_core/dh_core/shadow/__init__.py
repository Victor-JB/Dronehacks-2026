"""Passive (shadow-mode) sensor telemetry and obstacle-safety evaluation. PURE: no ROS imports.

Nothing in this package publishes, commands, or calls a service. The only ROS-facing code is
dh_autonomy/shadow_node.py, which subscribes and logs; `guard.py` enforces that it never gains a
flight-control endpoint. See docs/STAGE4_SHADOW.md.
"""
