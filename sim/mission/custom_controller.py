"""Run a team controller directory inside the marker-free PX4 runner.

The directory holds controller.py defining ``Controller(course)`` with ``step(obs) -> dict``.
The runner keeps takeoff, the hover check, landing and every safety check; the team
controller decides the motion in between. See controllers/README.md at the repository root.
"""
import importlib.util
from pathlib import Path
import sys
import traceback

from .core import Command, ZERO, limited, vector

REPO = Path('/repo')   # repository root, mounted read-only by compose.markerfree.yaml


class CustomRoute:
    def __init__(self, directory, course):
        directory = Path(directory).resolve()
        source = directory/'controller.py'
        if not source.is_file():
            raise ValueError(f'{source} does not exist')
        for path in (REPO, directory):      # so a controller can import refnav or its own modules
            if path.is_dir() and str(path) not in sys.path:
                sys.path.insert(0, str(path))
        spec = importlib.util.spec_from_file_location('team_controller', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.controller = module.Controller(course)
        self.waypoints = course['waypoints']
        self.waypoint_index = 0
        self.max_speed = float(course['limits'].get('max_speed', 1.0))
        self.max_yaw_rate = float(course['limits'].get('max_yaw_rate', .7))
        self.max_accel = float(course['limits'].get('max_accel', .8))
        self.last_position = None
        self.yaw = None

    def update(self, position, velocity, stamp, frame=None, yaw=None):
        self.last_position = position
        frame = frame or {}
        obs = {'t': stamp, 'position': tuple(position), 'velocity': tuple(velocity), 'yaw': yaw,
               'depth': frame.get('depth_z_m'),
               'depth_t': frame['sim_usec']/1e6 if 'sim_usec' in frame else None,
               'tof_range_m': frame.get('tof_range_m')}
        try:
            out = self.controller.step(obs) or {}
            velocity_cmd = limited(vector(out.get('velocity', ZERO)), self.max_speed)
        except Exception as error:
            traceback.print_exc()
            sys.stdout.flush()
            return Command(ZERO, 0., 'FAULT', False, f'controller_error:{type(error).__name__}', vector(position))
        self.yaw = out.get('yaw')
        return Command(velocity_cmd, 0., str(out.get('phase', 'CUSTOM')), bool(out.get('done', False)),
                       out.get('fault'), vector(out.get('target', position)))
