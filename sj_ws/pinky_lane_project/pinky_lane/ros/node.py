from __future__ import annotations
import math, threading, time
from typing import Optional, Tuple
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_ros import Buffer, TransformListener
from pinky_lane import config
from pinky_lane.config import VISION, MISSION, MOTION
from pinky_lane.core.types import DriveState
from pinky_lane.core.math_utils import quaternion_to_yaw, clamp
from pinky_lane.control.command_gate import CommandGate
from pinky_lane.control.mission import MissionControlMixin
from pinky_lane.perception.lane_estimator import LaneEstimator
from pinky_lane.perception.intersection_detector import IntersectionDetector
from pinky_lane.perception.segmentation import LaneSegmenter, segmentation_masks, largest_mask, largest_area_ratio
from pinky_lane.hardware.camera import PinkyCamera
from pinky_lane.ui.display import PinkyDisplay
from pinky_lane.ui.overlay import DebugOverlay

class LaneMissionController(MissionControlMixin, Node):

    def __init__(self) -> None:
        super().__init__("lane_mission_controller")

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(PoseStamped, "/mission/goal_pose", self._on_goal, 10)
        self.create_subscription(Bool, "/mission/enable", self._on_enable, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._data_lock = threading.Lock()
        self._gate = CommandGate(MOTION.vision_stale_s)
        self._seen_epoch = 0
        self._arming_epoch = 0
        self._seen_arming_epoch = 0
        self._pending_goal = False
        self._closed = False
        self._camera = None
        self._display = None
        self._output_group = MutuallyExclusiveCallbackGroup()
        self._output_timer = self.create_timer(
            1.0 / MOTION.control_hz, self._output_tick,
            callback_group=self._output_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._goal_xy: Optional[Tuple[float, float]] = None
        self._goal_yaw: Optional[float] = None
        self._enabled = False
        self._enabled_since: Optional[float] = None
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._odom_stamp = 0.0

        self._state = DriveState.IDLE
        self._state_since = time.monotonic()
        self._note = ""

        self._enter_start_pose: Optional[Tuple[float, float, float]] = None
        self._turn_start_yaw: Optional[float] = None
        self._turn_target = 0.0

        self._vision_steer_sign = -1.0 if VISION.mirror_input else 1.0

        self._lane = LaneEstimator(VISION.frame_width, VISION.frame_height)
        self._intersection = IntersectionDetector()
        self._frames = 0
        self._fps_started = time.monotonic()
        if MISSION.enter_timeout_s <= MISSION.enter_distance_m / MOTION.min_speed:
            raise ValueError("entry timeout must exceed distance / entry speed")

    def start_hardware(self) -> None:
        self.get_logger().info("loading segmentation model...")
        self._segmenter = LaneSegmenter()
        self.get_logger().info("starting camera...")
        self._camera = PinkyCamera()
        self._camera.start()
        self._display = PinkyDisplay(VISION.use_lcd, VISION.lcd_every_n_frames)
        self._overlay = DebugOverlay()
        self._frames = 0
        self._fps_started = time.monotonic()

    def _on_goal(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != MISSION.map_frame:
            self.get_logger().error(f"goal frame must be {MISSION.map_frame!r}; rejected {msg.header.frame_id!r}")
            return
        xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        if not all(math.isfinite(v) for v in xy):
            self.get_logger().error("nonfinite goal rejected")
            return
        with self._data_lock:
            self._goal_xy = xy
            self._goal_yaw = None  # final yaw alignment is not part of this controller
            self._pending_goal = True
            self._gate.invalidate()
            self._publish_twist_unlocked(0.0, 0.0)
        self.get_logger().info(f"new map goal: {xy}")

    def _on_enable(self, msg: Bool) -> None:
        now = time.monotonic()
        with self._data_lock:
            requested = bool(msg.data) and not self._closed
            changed = requested != self._enabled
            self._enabled = requested
            if changed and requested:
                self._arming_epoch += 1
            if changed:
                self._enabled_since = now if requested else None
            self._gate.set_enabled(requested)
            if not requested:
                self._publish_twist_unlocked(0.0, 0.0)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pose = (
            float(p.x),
            float(p.y),
            quaternion_to_yaw(msg.pose.pose.orientation),
        )
        if not all(math.isfinite(v) for v in pose):
            return
        with self._data_lock:
            self._odom_pose = pose
            self._odom_stamp = time.monotonic()

    def _snapshot(self):
        with self._data_lock:
            return (
                self._goal_xy,
                self._goal_yaw,
                self._enabled,
                self._enabled_since,
                self._odom_pose,
                self._odom_stamp,
            )

    def _map_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(MISSION.map_frame, MISSION.robot_frame, rclpy.time.Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= MOTION.map_stale_s:
                return None
            t = tf.transform.translation
            result = (float(t.x), float(t.y), quaternion_to_yaw(tf.transform.rotation))
            return result if all(math.isfinite(v) for v in result) else None
        except Exception:
            return None

    def _goal_distance(self) -> Optional[float]:
        goal, _, _, _, _, _ = self._snapshot()
        if goal is None:
            return None

        pose = self._map_pose()
        if pose is None:
            return None

        return math.hypot(goal[0] - pose[0], goal[1] - pose[1])

    def _send_velocity(self, linear: float, angular: float, epoch: int, captured_at: float) -> None:
        with self._data_lock:
            self._gate.submit(epoch, captured_at, linear, angular)

    def _publish_twist_unlocked(self, linear: float, angular: float) -> None:
        if not config.DRIVE_OUTPUT:
            return
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _output_tick(self) -> None:
        now = time.monotonic()
        with self._data_lock:
            now = time.monotonic()
            linear, angular = self._gate.current(now)
            if self._closed or not self._odom_fresh(now, self._odom_stamp):
                linear, angular = 0.0, 0.0
            self._publish_twist_unlocked(linear, angular)

    def _publish_zero(self, repeats: int = 3) -> None:
        for _ in range(repeats):
            with self._data_lock:
                self._gate.capture_time = float('-inf')
                self._gate.velocity = (0.0, 0.0)
                self._publish_twist_unlocked(0.0, 0.0)

    def step(self) -> None:
        epoch = self._consume_requests()
        captured_at = time.monotonic()
        try:
            frame = self._camera.capture()
            height, width = frame.shape[:2]
            if (width,height) != (VISION.frame_width,VISION.frame_height):
                raise ValueError("Unexpected camera dimensions")
            result = self._segmenter.predict(frame)
            now = time.monotonic()  # timestamps after inference, not before it
            masks = segmentation_masks(result, width, height)
            lane = self._lane.update(largest_mask(masks.get("left_lane")),
                                     largest_mask(masks.get("right_lane")),now)
            crosswalk_ratio = largest_area_ratio(masks.get("crosswalk"),width*height)
            _,_,enabled,enabled_since,_,_ = self._snapshot()
            armed = enabled and enabled_since is not None and now-enabled_since >= MOTION.start_delay_s
            eligible = armed and self._state == DriveState.FOLLOW and now-captured_at <= MOTION.vision_stale_s
            event = self._intersection.update(crosswalk_ratio,now) if eligible else False
            if now-captured_at > MOTION.vision_stale_s:
                self._note = "VISION STALE: OUTPUT HELD"
                linear,angular = 0.0,0.0
            else:
                linear,angular = self._control(lane,event,now)
            linear = clamp(float(linear),0.0,MOTION.base_speed)
            angular = clamp(float(angular),-MOTION.max_angular,MOTION.max_angular)
            self._send_velocity(linear,angular,epoch,captured_at)
            if self._display.available and (self._frames+1) % max(1,VISION.lcd_every_n_frames) == 0:
                visual = self._overlay.render(result.plot(), lane, crosswalk_ratio, linear, angular,
                                              self._state, self._note, config.DRIVE_OUTPUT)
                self._display.frame(visual,force=True)
            self._frames += 1
            if self._frames % 20 == 0:
                fps = self._frames/max(time.monotonic()-self._fps_started,1e-6)
                with self._data_lock:
                    sent = self._gate.current(time.monotonic())
                self.get_logger().info(f"{fps:.1f}fps | {self._state.name} | desired {linear:.2f}/{angular:+.2f} | gated {sent} | {self._note}")
        except Exception:
            self._publish_zero()
            raise

    def shutdown(self) -> None:
        with self._data_lock:
            self._closed = True
            self._enabled = False
            self._gate.set_enabled(False)
            self._publish_twist_unlocked(0.0,0.0)
        self._output_timer.cancel()
        self._publish_zero(repeats=5)
        if self._camera is not None:
            try:
                self._camera.stop()
            except Exception:
                pass
            try:
                self._camera.close()
            except Exception:
                pass
        if self._display is not None:
            self._display.text("STOP",(0,0,255))
