"""Explicit simulation inputs for unchanged autonomous mission methods.

Odometry supplies pose. Forward LiDAR clearance supplies the distance input,
not a claim that a physical ultrasonic sensor is being tested. No SLAM map is
provided: the existing odom-only U-turn fallback is exercised.
"""
from types import MethodType

import numpy as np
from pinky_autonomous.autonomous_drive_node import AutonomousDriveNode, LaneTracker


class SimulationMission:
    def __init__(self, model, logger, width, height, heading_gain, far_fallback):
        self.model, self.logger = model, logger
        logger.info('Simulation mission: pose=odom, distance=forward LiDAR; '
                    'original log labels may say sonar. No SLAM map or physical sonar is tested.')
        self.cam_w, self.cam_h = width, height
        self.yolo_imgsz, self.model_conf = 320, 0.35
        self.class_conf = dict(left_lane=0.5, right_lane=0.5, crosswalk=0.5, cross_lane=0.35)
        self.lane_classes = ['left_lane', 'right_lane']
        self.crosswalk_classes, self.junction_classes = ['crosswalk'], ['cross_lane']
        self.lane_flip_fix, self.swap_lanes = False, False
        self.prev_lane = {'left': None, 'right': None}
        self.state, self.drive_speed, self.caution_factor = 'driving', 0.08, 0.5
        self.crosswalk_min_area, self.cw_check_time, self.cw_stop_dist = 0.05, 1.5, 0.30
        self.cw_rearm_time, self.cw_block_timeout = 2.0, 6.0
        self.cw_armed, self.cw_check_active = True, False
        self.cw_clear_since = self.cw_block_t0 = None
        self.last_cw_seen = 0.0
        self.junction_frames, self.junction_latched, self.junction_min_width = 0, False, 0.6
        self.collision_dist, self.stop_dist, self.obstacle_dist = 0.05, 0.10, 0.35
        self.return_enabled, self.start_pose = True, None
        self.return_phase, self.return_t0 = None, 0.0
        self.advance_dist, self.advance_clear, self.junction_pause = 0.25, 0.20, 1.0
        self.map_sub, self.pose_frame, self.map_frame = None, 'odom', 'map'
        self.turn_angular, self.arrive_dist = 0.8, 0.30
        self.lane_bias, self.bias_dist, self.bias_time, self.bias_search_w = None, 0.8, 8.0, 0.4
        self.ramp_t0, self.ramp_n, self.ramp_time, self.ramp_frames = None, 0, 2.0, 10
        self.tracker = LaneTracker(width, height, heading_gain, far_fallback)
        self._wrap = AutonomousDriveNode._wrap
        for name in ('_detect', '_update_crosswalk', '_decide_state', '_compute_command',
                     '_return_step', '_advance_speed', '_plan_exit', '_begin_turn',
                     '_turn_command', '_start_following', '_check_arrival',
                     '_update_bias', '_ramp_progress'):
            setattr(self, name, MethodType(getattr(AutonomousDriveNode, name), self))

    def get_logger(self):
        return self.logger

    def _capture_bgr(self):
        return self.frame

    def _sonar_distance(self, now):
        return self.clearance  # Explicit LiDAR input; sonar is not emulated.

    def _current_pose(self):
        return self.pose

    def _ensure_map_sub(self):
        # There is deliberately no map subscription in this odom-only test.
        pass

    def step(self, frame, now, pose, clearance):
        self.frame, self.pose, self.clearance = frame, pose, clearance
        if self.start_pose is None:
            self.start_pose = pose
        if self.return_phase == 'following':
            self._update_bias(now)
            self._check_arrival(now)
        if self.junction_latched and self.return_phase != 'following':
            self._decide_state(now, 0.0, 0.0)
            blocked = self.state in ('collision', 'obstacle_stop')
            self._return_step(now, blocked)
            speed, angular = 0.0, 0.0
            if self.return_phase == 'turning' and not blocked:
                angular = self._turn_command(now)
            elif self.return_phase == 'advance' and not blocked:
                speed = self._advance_speed(now)
            return None, 0.0, False, 'MISSION', speed, angular, False, 0.0, 0.0
        result, left, right, cw, junction = self._detect()
        self._decide_state(now, cw, junction)
        _, steer, valid, mode = self.tracker.update(left, right)
        speed, angular, stopped = self._compute_command(steer, valid)
        ramp = self._ramp_progress(now)
        if ramp < 1.0 and not stopped:
            speed = min(speed, self.drive_speed * (0.5 + 0.5 * ramp))
            angular *= ramp
        return result, steer, valid, mode, speed, float(np.clip(angular, -1.5, 1.5)), True, cw, junction
