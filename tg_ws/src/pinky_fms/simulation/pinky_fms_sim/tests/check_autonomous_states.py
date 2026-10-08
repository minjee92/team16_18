"""Controlled-input checks of unchanged robot methods, NOT a Gazebo mission test.

Run in a sourced ROS workspace with pinky_autonomous installed. Pose, distance
and detector measurements below are fixtures, not real sensor observations.
"""
import math
from types import MethodType, SimpleNamespace
import unittest

from pinky_autonomous.autonomous_drive_node import AutonomousDriveNode, LaneTracker


def fixture():
    n = SimpleNamespace(
        state='driving', drive_speed=0.08, caution_factor=0.5,
        crosswalk_min_area=0.05, cw_check_time=1.5, cw_stop_dist=0.30,
        cw_rearm_time=2.0, cw_block_timeout=6.0, cw_armed=True,
        cw_check_active=False, cw_clear_since=None, cw_block_t0=None,
        last_cw_seen=0, junction_frames=0, junction_latched=False,
        junction_min_width=0.6, collision_dist=0.05, stop_dist=0.10,
        obstacle_dist=0.35, return_enabled=True, start_pose=(0, 0, 0),
        return_phase=None, return_t0=0, advance_dist=0.25,
        advance_clear=0.20, junction_pause=1.0, map_sub=None,
        pose_frame='odom', map_frame='map', turn_angular=0.8,
        lane_bias=None, arrive_dist=0.30, bias_dist=0.8, bias_time=8.0,
        distance=1.0, pose=(1.0, 0.0, 0.0),
        tracker=LaneTracker(1280, 720, heading_gain=0, far_fallback=False),
    )
    logger = SimpleNamespace(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    n.get_logger = lambda: logger
    n._sonar_distance = lambda now: n.distance
    n._current_pose = lambda: n.pose
    n._ensure_map_sub = lambda: None
    n._wrap = AutonomousDriveNode._wrap
    for name in ('_update_crosswalk', '_decide_state', '_compute_command',
                 '_return_step', '_advance_speed', '_plan_exit', '_begin_turn',
                 '_turn_command', '_start_following', '_check_arrival'):
        setattr(n, name, MethodType(getattr(AutonomousDriveNode, name), n))
    return n


class StateChecks(unittest.TestCase):
    def test_crosswalk_clear_slows_then_resumes_and_rearms(self):
        n = fixture()
        n._decide_state(10, 0.06, 0)
        self.assertEqual(n.state, 'caution')
        self.assertAlmostEqual(n._compute_command(0, True)[0], 0.04)
        n._decide_state(10.1, 0.06, 0)
        n._decide_state(11.7, 0.06, 0)
        self.assertEqual(n.state, 'driving')
        self.assertFalse(n.cw_armed)
        n._decide_state(14, 0, 0)
        self.assertTrue(n.cw_armed)

    def test_crosswalk_obstacle_stops_then_clears(self):
        n = fixture()
        n.distance = 0.25
        n._decide_state(10, 0.06, 0)
        n._decide_state(10.1, 0.06, 0)
        self.assertEqual(n.state, 'crosswalk_blocked')
        self.assertEqual(n._compute_command(0, True), (0, 0, True))
        n.distance = 1.0
        n._decide_state(11, 0.06, 0)
        n._decide_state(12.6, 0.06, 0)
        self.assertEqual(n.state, 'driving')

    def test_persistent_crosswalk_obstacle_timeout_is_current_behavior(self):
        n = fixture()
        n.distance = 0.25
        for t in (10, 10.1, 16.2):
            n._decide_state(t, 0.06, 0)
        # Existing policy releases CW stop after six seconds, then normal caution.
        self.assertFalse(n.cw_check_active)
        self.assertEqual(n.state, 'caution')
        self.assertGreater(n._compute_command(0, True)[0], 0)

    def test_junction_advance_odom_fallback_turn_and_arrival(self):
        n = fixture()
        for t in (10, 10.1, 10.2, 10.3, 10.4):
            n._decide_state(t, 0, 0.8)
        self.assertTrue(n.junction_latched)
        self.assertEqual(n.state, 'junction')
        n._return_step(11, False)
        self.assertEqual(n.return_phase, 'advance')
        n.pose = (1.26, 0, 0)
        self.assertEqual(n._advance_speed(12), 0)
        n._return_step(13.1, False)
        n._return_step(13.2, False)
        self.assertEqual(n.exit_choice, 'back')
        self.assertEqual(n.return_phase, 'turning')
        self.assertGreater(abs(n._turn_command(14)), 0)
        n.pose = (1.26, 0, math.pi)
        self.assertEqual(n._turn_command(15), 0)
        self.assertEqual(n.return_phase, 'following')
        n._check_arrival(19)
        self.assertEqual(n.return_phase, 'following')
        n.pose = (0.2, 0, math.pi)
        n._check_arrival(20)
        n._decide_state(20, 0, 0)
        self.assertEqual(n.state, 'home')
        self.assertEqual(n._compute_command(0, True), (0, 0, True))

    def test_return_without_start_pose_fails_stopped(self):
        n = fixture()
        n.start_pose = None
        n.junction_latched = True
        n._return_step(10, False)
        n._decide_state(10, 0, 0)
        self.assertEqual(n.state, 'return_failed')
        self.assertEqual(n._compute_command(0, True), (0, 0, True))


if __name__ == '__main__':
    unittest.main(verbosity=2)
