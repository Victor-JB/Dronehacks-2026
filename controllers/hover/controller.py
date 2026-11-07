"""Do nothing after the runner's takeoff and hover check: land straight away."""


class Controller:
    def __init__(self, course):
        pass

    def step(self, obs):
        return {"velocity": (0.0, 0.0, 0.0), "done": True, "phase": "HOVER_DONE"}
