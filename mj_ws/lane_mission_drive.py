#!/usr/bin/env python3
"""
Pinky 차선 추종 미션 주행

차선을 따라 달리되, 교차로에서는 목표 지점 방향을 보고 진행 방향을 정한다.
Nav2는 주행에 관여하지 않는다. 맵과 AMCL은 위치 파악에만 쓴다.

  구독:
    /mission/goal_pose  (PoseStamped)  목표 지점
    /mission/enable     (Bool)         주행 시작/정지
    /odom               (Odometry)     회전량·이동거리 측정
    TF map→base_footprint              전역 위치 (AMCL)

  발행:
    /cmd_vel            (Twist)

실행 순서:
    1) ros2 launch pinky_bringup bringup_robot.launch.xml
    2) ros2 launch pinky_navigation localization_launch.xml map:=<맵 yaml>
       (Nav2 전체를 띄우면 cmd_vel이 충돌하니 localization만 실행)
    3) python3 mission_gui.py <맵 yaml>
    4) python3 lane_mission_drive.py

처음에는 PUBLISH_CMD = False로 두고 동작만 확인하세요.
"""

import math
import os
import select
import sys
import termios
import time
import tty

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


# ============================== 설정 ==============================

MODEL_PATH = 'best_ncnn_model'
INFER_SIZE = 320
CONF = 0.5
FRAME_SIZE = (640, 480)
CONTROL_HZ = 10.0

PUBLISH_CMD = True               # False면 계산만 하고 바퀴는 멈춤
START_DELAY = 3.0

# --- 카메라 전처리 (학습 데이터와 동일하게) ---
ROTATE_180 = True
MIRROR_INPUT = False
SWAP_RB = False

# --- 속도 ---
BASE_SPEED = 0.10
MIN_SPEED = 0.05
MAX_ANGULAR = 1.5
CORNER_SLOWDOWN = 0.5

# --- 조향 ---
STEER_GAIN = 1.0
STEER_D_GAIN = 0.25
STEER_SMOOTH = 0.4
STEER_TO_ANGULAR = 2.0

# --- 차선 읽기 ---
SAMPLE_ROWS = (0.90, 0.80, 0.70, 0.60)
ROW_WEIGHTS = (0.40, 0.30, 0.20, 0.10)
ROAD_HALF_INIT = 0.28
ROAD_HALF_SMOOTH = 0.1
LOST_SLOW_AFTER = 5
LOST_STOP_AFTER = 20

# --- 교차로 판단 ---
CROSSWALK_MIN_AREA = 0.02         # 화면 면적 대비. 이 이상이면 교차로 진입으로 봄
INTERSECTION_COOLDOWN = 6.0       # 한 번 처리한 뒤 이 시간 동안 재반응 안 함
TURN_THRESHOLD_DEG = 50.0         # 목표 방위가 이보다 벌어지면 회전
ALLOW_LEFT_TURN = False           # 시계방향 일방통행이면 False
ENTER_DISTANCE = 0.18             # 교차로 중앙까지 진입할 거리 m
TURN_ANGLE_DEG = 90.0             # 한 번에 도는 각도
TURN_ANGULAR = 1.2                # 회전 각속도 rad/s
TURN_FORWARD = 0.02               # 회전 중 전진 속도 (0이면 제자리)

# --- 도착 판정 ---
GOAL_RADIUS = 0.25                # 목표에 이만큼 가까워지면 정지

USE_LCD = True
LCD_UPDATE_INTERVAL = 2

MAP_FRAME = 'map'
ROBOT_FRAME = 'base_footprint'

STEER_SIGN = -1.0 if MIRROR_INPUT else 1.0


# ============================== 유틸 ==============================

def normalize_angle(angle):
    """-pi ~ pi 범위로."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ========================== 키 입력 감시 ==========================

class KeyWatcher:

    ENTER_KEYS = ('\r', '\n')
    ESC_KEY = '\x1b'

    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else None
        self.old_settings = None

    def __enter__(self):
        if self.enabled:
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.enabled and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return False

    def _readable(self, timeout=0.0):
        return bool(select.select([sys.stdin], [], [], timeout)[0])

    def stop_requested(self):
        if not self.enabled:
            return False
        stop = False
        while self._readable():
            try:
                char = os.read(self.fd, 1).decode('utf-8', errors='ignore')
            except OSError:
                break
            if not char:
                break
            if char in self.ENTER_KEYS:
                stop = True
            elif char == self.ESC_KEY:
                if self._readable(timeout=0.02):
                    while self._readable():
                        os.read(self.fd, 1)
                    continue
                stop = True
        return stop


# =========================== LCD 미리보기 ===========================

class LcdPreview:

    def __init__(self, enabled=True, update_interval=1):
        self.lcd = None
        self.image_module = None
        self.update_interval = max(1, update_interval)
        self.frame_index = 0

        if not enabled:
            return
        try:
            from pinky_lcd import LCD
            from PIL import Image
            self.image_module = Image
            self.lcd = LCD()
        except Exception as error:
            print(f'[참고] LCD를 사용할 수 없습니다: {error}')
            self.lcd = None

    def _send(self, rgb):
        try:
            self.lcd.img_show(self.image_module.fromarray(rgb))
        except Exception as error:
            print(f'[참고] LCD 출력 중단: {error}')
            self.lcd = None

    def show(self, bgr, force=False):
        if self.lcd is None:
            return
        self.frame_index += 1
        if not force and self.frame_index % self.update_interval != 0:
            return
        self._send(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

    def message(self, text, color=(255, 255, 255)):
        if self.lcd is None:
            return
        canvas = np.zeros((FRAME_SIZE[1], FRAME_SIZE[0], 3), np.uint8)
        cv2.putText(canvas, text, (26, FRAME_SIZE[1] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, 3, cv2.LINE_AA)
        self._send(canvas)


# ========================== 마스크 처리 ==========================

def build_masks(result, width, height):
    out = {}
    if result.masks is None or result.boxes is None:
        return out

    names = result.names
    data = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)

    for mask, cls_id in zip(data, classes):
        resized = cv2.resize(mask.astype(np.uint8), (width, height),
                             interpolation=cv2.INTER_NEAREST)
        out.setdefault(names[cls_id], []).append(resized)
    return out


def pick_main(masks):
    return max(masks, key=lambda m: int(m.sum())) if masks else None


def x_at_row(mask, y):
    if mask is None or not (0 <= y < mask.shape[0]):
        return None
    xs = np.flatnonzero(mask[y])
    return float(xs.mean()) if xs.size else None


# ========================== 차선 추적 ==========================

class LaneTracker:

    def __init__(self, width, height):
        self.width = width
        self.rows = [int(height * r) for r in SAMPLE_ROWS]
        self.half_width = {y: width * ROAD_HALF_INIT for y in self.rows}
        self.steer = 0.0
        self.prev_error = 0.0
        self.lost_frames = 0

    def update(self, left_mask, right_mask):
        centers, weights = [], []

        for y, w in zip(self.rows, ROW_WEIGHTS):
            lx = x_at_row(left_mask, y)
            rx = x_at_row(right_mask, y)

            if lx is not None and rx is not None:
                cx = (lx + rx) / 2.0
                measured = abs(rx - lx) / 2.0
                self.half_width[y] += ROAD_HALF_SMOOTH * (measured - self.half_width[y])
                source = 'both'
            elif lx is not None:
                cx = lx + self.half_width[y]
                source = 'left'
            elif rx is not None:
                cx = rx - self.half_width[y]
                source = 'right'
            else:
                continue

            centers.append((int(cx), y, source))
            weights.append(w)

        if not centers:
            self.lost_frames += 1
            if self.lost_frames > LOST_STOP_AFTER:
                self.steer = 0.0
            return centers, self.steer, False

        self.lost_frames = 0
        total = sum(weights)
        target_x = sum(c[0] * w for c, w in zip(centers, weights)) / total
        error = (target_x - self.width / 2.0) / (self.width / 2.0)

        derivative = error - self.prev_error
        self.prev_error = error

        raw = STEER_GAIN * error + STEER_D_GAIN * derivative
        self.steer += STEER_SMOOTH * (raw - self.steer)
        self.steer = float(np.clip(self.steer, -1.0, 1.0))

        return centers, self.steer, True

    def reset(self):
        self.steer = 0.0
        self.prev_error = 0.0
        self.lost_frames = 0


# ============================ 시각화 ============================

def draw_hud(frame, centers, steer, valid, state, note, enabled):
    height, width = frame.shape[:2]
    mid = width // 2

    cv2.line(frame, (mid, 0), (mid, height), (110, 110, 110), 1)

    for x, y, source in centers:
        color = (0, 255, 0) if source == 'both' else (0, 200, 255)
        cv2.circle(frame, (x, y), 6, color, -1)

    if len(centers) >= 2:
        pts = np.array([[x, y] for x, y, _ in centers], np.int32)
        cv2.polylines(frame, [pts], False, (0, 255, 0), 3)

    bar_y = height - 30
    cv2.rectangle(frame, (mid - 160, bar_y - 12), (mid + 160, bar_y + 12),
                  (50, 50, 50), -1)
    tip = int(mid + np.clip(steer, -1, 1) * 160)
    cv2.rectangle(frame, (mid, bar_y - 12), (tip, bar_y + 12),
                  (0, 165, 255), -1)
    cv2.line(frame, (mid, bar_y - 16), (mid, bar_y + 16), (255, 255, 255), 2)

    color = (255, 255, 255) if valid else (0, 0, 255)
    cv2.putText(frame, state, (14, 38), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, color, 2, cv2.LINE_AA)

    if note:
        cv2.putText(frame, note, (14, 76), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 0, 255), 2, cv2.LINE_AA)

    if not enabled:
        cv2.putText(frame, 'DISABLED', (width - 230, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2, cv2.LINE_AA)
    elif not PUBLISH_CMD:
        cv2.putText(frame, 'DRY RUN', (width - 200, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2, cv2.LINE_AA)

    return frame


# =========================== 주행 노드 ===========================

class LaneMissionDrive(Node):

    IDLE = 'IDLE'
    FOLLOW = 'FOLLOW'
    ENTER = 'ENTER'
    TURN = 'TURN'
    ARRIVED = 'ARRIVED'

    def __init__(self):
        super().__init__('lane_mission_drive')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(PoseStamped, '/mission/goal_pose',
                                 self.on_goal, 10)
        self.create_subscription(Bool, '/mission/enable',
                                 self.on_enable, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal = None              # (x, y)
        self.enabled = False

        self.odom_pose = None         # (x, y, yaw)
        self.state = self.IDLE
        self.state_since = time.monotonic()
        self.turn_start_yaw = None
        self.turn_target = 0.0
        self.enter_start = None
        self.last_intersection = 0.0
        self.note = ''

        self.get_logger().info('모델 로딩 중...')
        from ultralytics import YOLO
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f'클래스: {self.model.names}')

        self.get_logger().info('카메라 시작...')
        from picamera2 import Picamera2
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_video_configuration(
            main={"size": FRAME_SIZE, "format": "RGB888"}))
        self.camera.start()
        time.sleep(1.0)

        self.tracker = LaneTracker(*FRAME_SIZE)
        self.preview = LcdPreview(USE_LCD, LCD_UPDATE_INTERVAL)

        self.frames = 0
        self.start_time = time.monotonic()

    # ---------- 콜백 ----------

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'목표 수신: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        if self.state == self.ARRIVED:
            self.set_state(self.FOLLOW)

    def on_enable(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            self.set_state(self.IDLE)
        elif self.state == self.IDLE:
            self.tracker.reset()
            self.set_state(self.FOLLOW)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    # ---------- 위치 ----------

    def map_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quaternion(tf.transform.rotation)

    def decide_direction(self):
        """
        목표 방위와 현재 방향을 비교해 진행 방향을 정한다.
        반환: 'straight' | 'left' | 'right'
        """
        if self.goal is None:
            return 'straight'

        pose = self.map_pose()
        if pose is None:
            self.get_logger().warn('map 위치를 모릅니다 (AMCL 확인). 직진합니다.')
            return 'straight'

        rx, ry, ryaw = pose
        bearing = math.atan2(self.goal[1] - ry, self.goal[0] - rx)
        diff = normalize_angle(bearing - ryaw)
        threshold = math.radians(TURN_THRESHOLD_DEG)

        if diff > threshold:
            if not ALLOW_LEFT_TURN:
                self.get_logger().info('좌회전 금지 구간 → 직진')
                return 'straight'
            return 'left'

        if diff < -threshold:
            return 'right'

        return 'straight'

    def distance_to_goal(self):
        if self.goal is None:
            return None
        pose = self.map_pose()
        if pose is None:
            return None
        return math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])

    # ---------- 상태 ----------

    def set_state(self, state):
        if state == self.state:
            return
        self.get_logger().info(f'상태: {self.state} → {state}')
        self.state = state
        self.state_since = time.monotonic()

    # ---------- 카메라 ----------

    def grab(self):
        frame = self.camera.capture_array()
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if SWAP_RB:
            frame = frame[:, :, ::-1].copy()
        if MIRROR_INPUT:
            frame = cv2.flip(frame, 1)
        return frame

    # ---------- 한 주기 ----------

    def step(self):
        frame = self.grab()
        height, width = frame.shape[:2]

        result = self.model.predict(source=frame, conf=CONF,
                                    imgsz=INFER_SIZE, verbose=False)[0]
        masks = build_masks(result, width, height)

        left_mask = pick_main(masks.get('left_lane'))
        right_mask = pick_main(masks.get('right_lane'))

        centers, steer_raw, valid = self.tracker.update(left_mask, right_mask)
        steer = steer_raw * STEER_SIGN

        crosswalk_area = 0.0
        if masks.get('crosswalk'):
            crosswalk_area = max(int(m.sum()) for m in masks['crosswalk'])
            crosswalk_area /= float(width * height)

        speed, angular = self.control(steer, valid, crosswalk_area)

        angular = float(np.clip(angular, -MAX_ANGULAR, MAX_ANGULAR))
        self.send(speed, angular)

        view = draw_hud(result.plot(), centers, steer, valid,
                        self.state, self.note, self.enabled)
        self.preview.show(view)

        self.frames += 1
        if self.frames % 20 == 0:
            fps = self.frames / (time.monotonic() - self.start_time)
            dist = self.distance_to_goal()
            dist_text = f'{dist:.2f}m' if dist is not None else '-'
            self.get_logger().info(
                f'{fps:4.1f}fps | {self.state:7s} | steer {steer:+.2f} | '
                f'v {speed:.2f} w {angular:+.2f} | 목표까지 {dist_text}')

    # ---------- 상태별 제어 ----------

    def control(self, steer, valid, crosswalk_area):
        now = time.monotonic()
        self.note = ''

        if not self.enabled:
            self.set_state(self.IDLE)
            return 0.0, 0.0

        # --- 도착 판정 ---
        dist = self.distance_to_goal()
        if dist is not None and dist < GOAL_RADIUS:
            self.set_state(self.ARRIVED)

        if self.state == self.ARRIVED:
            self.note = 'GOAL REACHED'
            return 0.0, 0.0

        # --- 교차로 진입 준비 ---
        if self.state == self.ENTER:
            traveled = self.traveled_since(self.enter_start)
            self.note = f'ENTER {traveled:.2f}/{ENTER_DISTANCE:.2f}m'
            if traveled >= ENTER_DISTANCE:
                self.begin_turn()
            return MIN_SPEED, 0.0

        # --- 회전 중 ---
        if self.state == self.TURN:
            return self.run_turn()

        # --- 교차로 감지 ---
        if (crosswalk_area >= CROSSWALK_MIN_AREA
                and now - self.last_intersection > INTERSECTION_COOLDOWN):

            self.last_intersection = now
            direction = self.decide_direction()

            if direction == 'straight':
                self.note = 'INTERSECTION → 직진'
            else:
                self.turn_target = math.radians(TURN_ANGLE_DEG)
                if direction == 'right':
                    self.turn_target = -self.turn_target
                self.enter_start = self.odom_pose
                self.set_state(self.ENTER)
                self.note = f'INTERSECTION → {direction.upper()}'
                return MIN_SPEED, 0.0

        # --- 일반 차선 추종 ---
        self.set_state(self.FOLLOW)

        if not valid:
            lost = self.tracker.lost_frames
            self.note = f'LANE LOST {lost}'
            if lost > LOST_STOP_AFTER:
                return 0.0, 0.0
            if lost > LOST_SLOW_AFTER:
                return MIN_SPEED, -STEER_TO_ANGULAR * steer
            return BASE_SPEED * 0.7, -STEER_TO_ANGULAR * steer

        magnitude = min(abs(steer), 1.0)
        speed = max(BASE_SPEED * (1.0 - CORNER_SLOWDOWN * magnitude), MIN_SPEED)
        return speed, -STEER_TO_ANGULAR * steer

    def traveled_since(self, start):
        if start is None or self.odom_pose is None:
            return 0.0
        return math.hypot(self.odom_pose[0] - start[0],
                          self.odom_pose[1] - start[1])

    def begin_turn(self):
        self.turn_start_yaw = self.odom_pose[2] if self.odom_pose else None
        self.tracker.reset()
        self.set_state(self.TURN)

    def run_turn(self):
        """odom yaw를 보며 목표 각도만큼 돈다."""
        if self.turn_start_yaw is None or self.odom_pose is None:
            self.set_state(self.FOLLOW)
            return MIN_SPEED, 0.0

        turned = normalize_angle(self.odom_pose[2] - self.turn_start_yaw)
        remain = self.turn_target - turned
        self.note = (f'TURN {math.degrees(turned):+.0f}'
                     f'/{math.degrees(self.turn_target):+.0f}°')

        if abs(remain) < math.radians(8.0):
            self.tracker.reset()
            self.set_state(self.FOLLOW)
            return MIN_SPEED, 0.0

        # 시간 초과 보호
        if time.monotonic() - self.state_since > 8.0:
            self.get_logger().warn('회전 시간 초과 → 차선 추종으로 복귀')
            self.tracker.reset()
            self.set_state(self.FOLLOW)
            return MIN_SPEED, 0.0

        angular = math.copysign(TURN_ANGULAR, remain)
        return TURN_FORWARD, angular

    # ---------- 출력 ----------

    def send(self, linear_x, angular_z):
        if not PUBLISH_CMD:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def stop(self, times=5):
        msg = Twist()
        for _ in range(times):
            self.publisher.publish(msg)
            time.sleep(0.02)

    def shutdown(self):
        self.stop()
        try:
            self.camera.stop()
            self.camera.close()
        except Exception:
            pass
        self.preview.message('STOP', (0, 0, 255))


# ============================= 메인 =============================

def main(args=None):
    rclpy.init(args=args)

    node = None
    period = 1.0 / CONTROL_HZ

    try:
        node = LaneMissionDrive()

        print()
        print('=' * 56)
        print(f'모드        : {"주행" if PUBLISH_CMD else "DRY RUN (바퀴 정지)"}')
        print(f'기본 속도   : {BASE_SPEED:.2f} m/s')
        print(f'좌회전      : {"허용" if ALLOW_LEFT_TURN else "금지 (일방통행)"}')
        print('GUI에서 [주행 시작]을 눌러야 움직입니다.')
        print('종료        : Enter 또는 ESC')
        print('=' * 56)
        print()

        with KeyWatcher() as keys:
            next_deadline = time.monotonic()

            while rclpy.ok():
                if keys.stop_requested():
                    print('\n정지 요청')
                    break

                rclpy.spin_once(node, timeout_sec=0.0)
                node.step()

                next_deadline += period
                sleep_time = next_deadline - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_deadline = time.monotonic()

    except KeyboardInterrupt:
        print('\n사용자 중단(Ctrl+C)')

    except Exception as error:
        print(f'오류: {error}')
        import traceback
        traceback.print_exc()

    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('종료 완료')


if __name__ == '__main__':
    main()
