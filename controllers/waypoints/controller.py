"""The sim's own waypoint baseline: fly the surveyed window route from the course file."""
from mission.core import CourseController
from mission.heading import route_yaw

HEADING_LOOKAHEAD_M = 1.8


class Controller:
    def __init__(self, course):
        self.route = CourseController(course)

    def step(self, obs):
        c = self.route.update(obs["position"], obs["velocity"], obs["t"])
        i = self.route.waypoint_index + 1
        following = self.route.waypoints[i]["position"] if i < len(self.route.waypoints) else None
        yaw = route_yaw(obs["position"], c.target, following, c.phase, HEADING_LOOKAHEAD_M)
        return {"velocity": c.velocity, "yaw": yaw, "done": c.done, "phase": c.phase,
                "fault": c.fault, "target": c.target}
