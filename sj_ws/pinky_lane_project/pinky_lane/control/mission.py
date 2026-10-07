"""State-machine mixin for the ROS adapter.

Host owns the lock, shared mission data and perception helpers, and provides
_snapshot(), _map_pose(), _goal_distance(), _publish_zero(), get_logger().
ROS callbacks enqueue requests; state transitions run in the perception loop.
"""
import math, time
from typing import Optional, Tuple
from pinky_lane.config import MISSION, MOTION, LANE
from pinky_lane.core.types import DriveState, TurnChoice, LaneEstimate
from pinky_lane.core.math_utils import wrap_angle, clamp

class MissionControlMixin:

    def _choose_turn(self) -> Optional[TurnChoice]:
        goal, _, _, _, _, _ = self._snapshot()
        if goal is None:
            return TurnChoice.STRAIGHT

        pose = self._map_pose()
        if pose is None:
            self.get_logger().warning("map pose unavailable; holding")
            return None

        rx, ry, robot_yaw = pose
        target_bearing = math.atan2(goal[1] - ry, goal[0] - rx)
        delta = wrap_angle(target_bearing - robot_yaw)
        threshold = math.radians(MISSION.turn_threshold_deg)

        if delta > threshold:
            if MISSION.allow_left_turn:
                return TurnChoice.LEFT
            self.get_logger().info("goal is left, but left turn is disabled")
            return TurnChoice.STRAIGHT

        if delta < -threshold:
            return TurnChoice.RIGHT

        return TurnChoice.STRAIGHT

    def _set_state(self, new_state: DriveState) -> None:
        if new_state == self._state:
            return
        self.get_logger().info(f"state: {self._state.name} -> {new_state.name}")
        self._state = new_state
        self._state_since = time.monotonic()

    def _odom_fresh(self, now: float, stamp: float) -> bool:
        return stamp > 0.0 and 0.0 <= now - stamp <= MOTION.odom_stale_s

    @staticmethod
    def _distance_from(
        start: Optional[Tuple[float, float, float]],
        current: Optional[Tuple[float, float, float]],
    ) -> float:
        if start is None or current is None:
            return 0.0
        return math.hypot(current[0] - start[0], current[1] - start[1])

    def _control(
        self,
        lane: LaneEstimate,
        crosswalk_event: bool,
        now: float,
    ) -> Tuple[float, float]:
        goal, _, enabled, enabled_since, odom_pose, odom_stamp = self._snapshot()
        now = time.monotonic()
        self._note = ""

        if not enabled:
            self._set_state(DriveState.IDLE)
            return 0.0, 0.0

        if enabled_since is not None and now - enabled_since < MOTION.start_delay_s:
            self._note = "ARMING"
            return 0.0, 0.0

        if self._state == DriveState.FAULT:
            self._note = self._fault_reason
            return 0.0, 0.0
        if not self._odom_fresh(now, odom_stamp):
            self._fault("ODOM STALE: disable then enable")
            return 0.0, 0.0
        if goal is not None and self._map_pose() is None:
            self._fault("MAP TF STALE: disable then enable")
            return 0.0, 0.0
        distance = self._goal_distance()
        if distance is not None and distance <= MISSION.goal_radius_m:
            self._set_state(DriveState.ARRIVED)

        if self._state == DriveState.ARRIVED:
            self._note = "GOAL REACHED"
            return 0.0, 0.0

        if self._state == DriveState.ENTER_INTERSECTION:
            if not self._odom_fresh(now, odom_stamp):
                self._note = "ENTER: ODOM STALE"
                return 0.0, 0.0

            traveled = self._distance_from(self._enter_start_pose, odom_pose)
            self._note = f"ENTER {traveled:.2f}/{MISSION.enter_distance_m:.2f}m"

            if traveled >= MISSION.enter_distance_m:
                self._begin_turn(odom_pose)
                return 0.0, 0.0

            if now - self._state_since > MISSION.enter_timeout_s:
                self._fault("ENTRY TIMEOUT: disable then enable")
                return 0.0, 0.0

            return MOTION.min_speed, 0.0

        if self._state == DriveState.TURNING:
            return self._run_turn(now, odom_pose, odom_stamp)

        if crosswalk_event:
            choice = self._choose_turn()
            if choice is None:
                self._fault("MAP TF LOST: disable then enable")
                return 0.0, 0.0

            if choice == TurnChoice.STRAIGHT:
                self._note = "INTERSECTION: STRAIGHT"
            else:
                if not self._odom_fresh(now, odom_stamp):
                    self._note = "TURN REQUEST: ODOM STALE"
                    return 0.0, 0.0

                target = math.radians(MISSION.turn_angle_deg)
                if choice == TurnChoice.RIGHT:
                    target *= -1.0

                self._turn_target = target
                self._enter_start_pose = odom_pose
                self._set_state(DriveState.ENTER_INTERSECTION)
                self._note = f"INTERSECTION: {choice.name}"
                return MOTION.min_speed, 0.0

        self._set_state(DriveState.FOLLOW)

        # 차선을 완전히 잃었을 때는 마지막 steering을 잠시만 사용한다.
        if not lane.valid:
            self._note = f"LANE LOST {lane.lost_seconds:.1f}s"

            if not lane.can_coast or lane.lost_seconds >= LANE.lost_stop_s:
                return 0.0, 0.0

            angular = self._lane_to_angular(lane.steering)
            if lane.lost_seconds >= LANE.lost_slow_s:
                return MOTION.min_speed, angular

            return MOTION.base_speed * 0.65, angular

        # 신뢰도가 낮은 프레임은 자동으로 속도를 조금 더 줄인다.
        curve = min(abs(lane.steering), 1.0)
        curve_factor = 1.0 - MOTION.corner_slowdown * curve
        confidence_factor = 0.65 + 0.35 * lane.confidence
        speed = MOTION.base_speed * curve_factor * confidence_factor
        speed = max(speed, MOTION.min_speed)

        return speed, self._lane_to_angular(lane.steering)

    def _lane_to_angular(self, steering: float) -> float:
        image_steer = steering * self._vision_steer_sign
        angular = -MOTION.lane_angular_gain * image_steer
        return clamp(angular, -MOTION.max_angular, MOTION.max_angular)

    def _begin_turn(self, odom_pose: Optional[Tuple[float, float, float]]) -> None:
        if odom_pose is None:
            self._set_state(DriveState.FOLLOW)
            return

        self._turn_start_yaw = odom_pose[2]
        self._lane.reset_control()
        self._set_state(DriveState.TURNING)

    def _run_turn(
        self,
        now: float,
        odom_pose: Optional[Tuple[float, float, float]],
        odom_stamp: float,
    ) -> Tuple[float, float]:
        if (
            self._turn_start_yaw is None
            or odom_pose is None
            or not self._odom_fresh(now, odom_stamp)
        ):
            self._fault("TURN ODOM LOST: disable then enable")
            return 0.0, 0.0

        turned = wrap_angle(odom_pose[2] - self._turn_start_yaw)
        remain = wrap_angle(self._turn_target - turned)
        remain_deg = math.degrees(remain)

        self._note = (
            f"TURN {math.degrees(turned):+.0f}/"
            f"{math.degrees(self._turn_target):+.0f} deg"
        )

        if abs(remain_deg) <= MISSION.turn_tolerance_deg:
            self._lane.reset_control()
            self._set_state(DriveState.FOLLOW)
            return MOTION.min_speed, 0.0

        if now - self._state_since > MISSION.turn_timeout_s:
            self._fault("TURN TIMEOUT: disable then enable")
            return 0.0, 0.0

        # 목표각에 가까워질수록 회전 속도를 낮춰 overshoot를 줄인다.
        requested = MOTION.turn_kp * abs(remain)
        turn_rate = clamp(requested, MOTION.turn_rate_min, MOTION.turn_rate_max)
        angular = math.copysign(turn_rate, remain)
        angular = clamp(angular, -MOTION.max_angular, MOTION.max_angular)

        return MOTION.turn_forward_speed, angular

    def _fault(self, reason: str) -> None:
        self._fault_reason = reason
        self._note = reason
        self._set_state(DriveState.FAULT)
        self._publish_zero()

    def _consume_requests(self) -> int:
        with self._data_lock:
            epoch = self._gate.epoch
            arming_epoch = self._arming_epoch
            enabled = self._enabled
            pending_goal = self._pending_goal
            self._pending_goal = False
        if epoch != self._seen_epoch:
            self._seen_epoch = epoch
            rearmed = arming_epoch != self._seen_arming_epoch
            self._seen_arming_epoch = arming_epoch
            # A new goal during a maneuver is held; rearm explicitly.
            maneuver = self._state in (DriveState.TURNING, DriveState.ENTER_INTERSECTION)
            self._lane.reset_control()
            self._intersection.reset()
            self._enter_start_pose = None
            self._turn_start_yaw = None
            if pending_goal and enabled and maneuver:
                self._fault("GOAL CHANGED DURING TURN: disable then enable")
            elif self._state != DriveState.FAULT or not enabled or rearmed:
                self._set_state(DriveState.FOLLOW if enabled else DriveState.IDLE)
        return epoch
