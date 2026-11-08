"""CAD-independent exploration + navigation prototype (Stage 5 sprint). PURE Python, offline.

Policy code (online_map, openings, landing, safety3d, explorer) sees only sensor-shaped data: depth and
RGB images, intrinsics, extrinsics and odometry. sim_world.py is the hidden ground truth and is used
only by sim_run.py to render sensors and score collisions; explorer never imports it (a test checks).
Nothing here publishes; the explorer returns an Intent that the simulator executes.
"""
