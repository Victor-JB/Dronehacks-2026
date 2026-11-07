import math
import unittest

from mission.heading import route_yaw


class HeadingTests(unittest.TestCase):
    def test_first_boundary_turns_to_visible_corridor_before_net(self):
        goal, following = (-2.5146, 2.53, .95), (-1.32, 2.53, .95)
        early = route_yaw((-2.5146, 1., .95), goal, following, 'FIRST_CORRIDOR')
        late = route_yaw((-2.5146, 2.15, .95), goal, following, 'FIRST_CORRIDOR')
        self.assertAlmostEqual(early, math.pi/2)
        self.assertLess(abs(late), math.radians(10))

    def test_window_does_not_look_away_before_crossing(self):
        self.assertAlmostEqual(route_yaw((0, 0, 0), (0, .4, 0), (1, .4, 0),
                                        'WINDOW_CROSS'), math.pi/2)

    def test_window_alignment_looks_through_opening_while_strafing(self):
        heading = route_yaw((0, 0, 0), (0, -.5, 0), (-1, -.5, 0), 'WINDOW_ALIGN')
        self.assertAlmostEqual(heading, math.pi)

    def test_final_waypoint_and_disabled_lookahead(self):
        self.assertAlmostEqual(route_yaw((0, 0, 0), (.4, 0, 0), None, 'MAZE'), 0.)
        self.assertAlmostEqual(route_yaw((0, 0, 0), (.4, 0, 0), (.4, 1, 0), 'MAZE', 0), 0.)
