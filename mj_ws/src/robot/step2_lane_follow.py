#!/usr/bin/env python3
"""
2단계: 차선 가운데로 주행 (로봇)

카메라 → ncnn 차선 세그멘테이션 → 공통 후처리(common/lane_postprocess.py) → cmd_vel

  - 차선을 못 보면 시간 기준 단계적 대응 (0.5초 감속 / 2.0초 정지)
  - 전방 초음파 비상 정지: 차선 검출과 무관하게 전방이 가까우면 즉시 정지 (fail-safe, 수동 재개)
  - 출력은 cmd_vel(m/s, rad/s)만. 바퀴 rpm·부호·다이나믹셀 단위 변환은 pinky_bringup 이 한다

  구독: /us_sensor/range (sensor_msgs/Range)   ← pinky_sensor_adc 가 publish
  발행: cmd_vel (Twist)

실행 순서 (PLAN.md "2단계 실행 절차"):
    1) ros2 launch pinky_bringup bringup_robot.launch.xml
    2) ros2 run pinky_sensor_adc main_node         ← bringup 에 없음. 필수
    3) ros2 topic hz /us_sensor/range               ← 약 20 Hz 확인
    4) python3 src/robot/step2_lane_follow.py --dry-run   (처음엔 바퀴 없이 확인)
       python3 src/robot/step2_lane_follow.py

키 (실행한 터미널에서):
    Space       출발 / 비상 정지 후 재개 (초음파가 정상이고 장애물이 없을 때만 받아들임)
    Enter, ESC  정지 후 종료

PC 테스트: --video 로 카메라 대신 저장 영상을 넣는다 (ROS 환경 필요).
"""

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range

SRC_DIR = Path(__file__).resolve().parent.parent     # mj_ws/src/
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))                 # src/common 을 import 하기 위해

from common.lane_postprocess import LaneTracker, group_masks, pick_main  # noqa: E402
from drive_control import FrontStop, LaneFollowController                # noqa: E402


# ============================== 설정 ==============================

BASE_DIR = SRC_DIR.parent   # mj_ws/

MODEL_PATH = str(BASE_DIR / 'models' / 'lane_model_ncnn' / 'best_ncnn_model')   # 폴더명에 _ncnn_model 필수
INFER_SIZE = 320
CONF = 0.5
FRAME_SIZE = (640, 480)
CONTROL_HZ = 10.0
LOG_EVERY = 10                   # 이 스텝마다 상태 한 줄 (10Hz 면 1초)

# --- 카메라 전처리 (학습 데이터와 동일하게, lane_mission_drive.py 와 같은 값) ---
ROTATE_180 = True
MIRROR_INPUT = False
SWAP_RB = False

# --- 속도 (cmd_vel) ---
BASE_SPEED = 0.10                # m/s, 권장 0.10~0.15
MIN_SPEED = 0.05
MAX_ANGULAR = 1.5                # rad/s
CORNER_SLOWDOWN = 0.5

# --- 조향 ---
STEER_GAIN = 1.0
STEER_D_GAIN = 0.25
STEER_SMOOTH = 0.4
STEER_TO_ANGULAR = 2.0
STEER_CLIP = 1.0

# --- 차선 읽기 ---
SAMPLE_ROWS = (0.90, 0.80, 0.70, 0.60)
ROW_WEIGHTS = (0.40, 0.30, 0.20, 0.10)
ROAD_HALF_INIT = 0.28
ROAD_HALF_SMOOTH = 0.1

# --- 차선 상실 (초). lane_mission_drive.py 의 5/20 스텝 @ 10Hz 와 같은 값. 첫 주행에서 관찰 후 조정 ---
LOST_SLOW_SEC = 0.5
LOST_STOP_SEC = 2.0

# --- 전방 초음파 비상 정지 ---
SONAR_TOPIC = '/us_sensor/range'
FRONT_STOP_DISTANCE = 0.15       # m. 이 거리 이하면 정지 (센서 측정 상한 약 0.97 m 보다 충분히 작게)
SONAR_TIMEOUT = 0.3              # s. 측정이 이보다 오래 안 오면 정지 (센서 20Hz = 0.05초 주기의 6배)
SONAR_MIN_RANGE = 0.02           # m. 드라이버의 min_range. 이보다 작으면 무효값(I2C 실패 −0.03 등) → 정지

STEER_SIGN = -1.0 if MIRROR_INPUT else 1.0


# ============================ 입력 ============================

class KeyInput:
    """터미널 키 입력 (비차단). 터미널이 아니면 키를 받지 않는다."""

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

    def read(self):
        """눌린 키 목록: 'start' (Space), 'quit' (Enter / ESC)."""
        keys = []
        while self.enabled and self._readable():
            try:
                char = os.read(self.fd, 1).decode('utf-8', errors='ignore')
            except OSError:
                break
            if not char:
                break
            if char == ' ':
                keys.append('start')
            elif char in ('\r', '\n'):
                keys.append('quit')
            elif char == '\x1b':
                if self._readable(timeout=0.02):     # 방향키 등 ESC 시퀀스는 무시
                    while self._readable():
                        os.read(self.fd, 1)
                    continue
                keys.append('quit')
        return keys


class PiCameraSource:
    """로봇 카메라 (Picamera2)."""

    def __init__(self, size):
        from picamera2 import Picamera2
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_video_configuration(
            main={'size': size, 'format': 'RGB888'}))
        self.camera.start()
        time.sleep(1.0)

    def read(self):
        frame = self.camera.capture_array()
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if SWAP_RB:
            frame = frame[:, :, ::-1].copy()
        if MIRROR_INPUT:
            frame = cv2.flip(frame, 1)
        return frame

    def close(self):
        try:
            self.camera.stop()
            self.camera.close()
        except Exception:
            pass


class VideoSource:
    """PC 테스트용: 저장 영상 (이미 모델 입력 방향이므로 전처리 없음)."""

    def __init__(self, path, size):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f'영상을 열 수 없습니다: {path}')
        self.size = size

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return cv2.resize(frame, self.size)

    def close(self):
        self.cap.release()


def build_masks(result, width, height):
    """ultralytics Results → 클래스 이름별 원본 크기 마스크 (후처리는 common 모듈)."""
    if result.masks is None or result.boxes is None:
        return {}
    names = result.names
    data = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    confs = result.boxes.conf.cpu().numpy()
    return group_masks(data, [names[c] for c in classes], confs, width, height)


# =========================== ROS 노드 ===========================

class Step2LaneFollow(Node):
    """
    cmd_vel 발행과 초음파 비상 정지를 맡는다.
    초음파 콜백은 별도 스레드(executor)에서 돌므로, 추론 루프를 기다리지 않고 바로 정지 명령을 낸다.
    콜백과 메인 루프의 발행은 같은 lock 으로 묶어, 정지 직후 주행 명령이 끼어들지 않게 한다.
    """

    def __init__(self, publish_cmd):
        super().__init__('step2_lane_follow')
        self.publish_cmd = publish_cmd
        self.lock = threading.Lock()
        self.front = FrontStop(FRONT_STOP_DISTANCE, SONAR_TIMEOUT, SONAR_MIN_RANGE)
        self.started = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Range, SONAR_TOPIC, self.on_range, 10)

    # ---------- 초음파 (executor 스레드) ----------

    def on_range(self, msg):
        now = time.monotonic()
        with self.lock:
            was_tripped = self.front.tripped
            self.front.on_range(msg.range, now)
            if self.front.check(now) and not was_tripped:
                self._publish(0.0, 0.0)              # 즉시 정지
                if self.started:
                    self.get_logger().warn(
                        f'비상 정지: {self.front.trip_reason} (range {msg.range:.3f} m) — Space 로 재개')

    # ---------- 메인 루프에서 호출 ----------

    def try_start(self, now):
        """Space: 출발 또는 비상 정지 후 재개. 초음파가 정상이고 장애물이 없을 때만."""
        with self.lock:
            ok, reason = self.front.clear(now)
            if ok:
                self.started = True
        return ok, reason

    def drive(self, v, w, now):
        """주행 명령을 낸다. 비상 정지 중이거나 출발 전이면 0. 반환: (실제 v, 실제 w, 상태)."""
        with self.lock:
            was_tripped = self.front.tripped
            tripped = self.front.check(now)          # 측정 끊김(timeout)은 콜백이 안 오므로 여기서 잡는다
            if tripped and not was_tripped and self.started:
                self.get_logger().warn(f'비상 정지: {self.front.trip_reason} — Space 로 재개')

            if not self.started:
                v, w, state = 0.0, 0.0, 'WAIT'
            elif tripped:
                v, w, state = 0.0, 0.0, f'ESTOP {self.front.trip_reason}'
            else:
                state = 'DRIVE'
            self._publish(v, w)
        return v, w, state

    def sonar_status(self, now):
        with self.lock:
            if self.front.last_time is None:
                return 'sonar -'
            return f'sonar {self.front.last_range:.2f}m ({now - self.front.last_time:.2f}s 전)'

    def stop(self, times=5):
        for _ in range(times):
            with self.lock:
                self._publish(0.0, 0.0)
            time.sleep(0.02)

    def _publish(self, v, w):
        if not self.publish_cmd:
            return
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


# ============================= 메인 =============================

def parse_args():
    parser = argparse.ArgumentParser(description='2단계: 차선 가운데로 주행 (cmd_vel 발행)')
    parser.add_argument('--model', default=MODEL_PATH,
                        help='ncnn 모델 폴더 (이름에 _ncnn_model 필수, 기본: %(default)s)')
    parser.add_argument('--dry-run', action='store_true',
                        help='계산만 하고 cmd_vel 을 발행하지 않는다 (바퀴 정지)')
    parser.add_argument('--video', default=None,
                        help='PC 테스트용: 카메라 대신 저장 영상 사용')
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = Step2LaneFollow(publish_cmd=not args.dry_run)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    source = None
    try:
        from ultralytics import YOLO
        node.get_logger().info(f'모델 로딩: {args.model}')
        model = YOLO(args.model, task='segment')

        source = VideoSource(args.video, FRAME_SIZE) if args.video else PiCameraSource(FRAME_SIZE)
        width, height = FRAME_SIZE

        tracker = LaneTracker(
            width, height,
            sample_rows=SAMPLE_ROWS, row_weights=ROW_WEIGHTS,
            road_half_init=ROAD_HALF_INIT, road_half_smooth=ROAD_HALF_SMOOTH,
            steer_gain=STEER_GAIN, steer_smooth=STEER_SMOOTH,
            lost_stop_sec=LOST_STOP_SEC, steer_d_gain=STEER_D_GAIN, steer_clip=STEER_CLIP,
        )
        controller = LaneFollowController(
            BASE_SPEED, MIN_SPEED, MAX_ANGULAR, CORNER_SLOWDOWN,
            STEER_TO_ANGULAR, LOST_SLOW_SEC, LOST_STOP_SEC,
        )

        print()
        print('=' * 60)
        print(f'모드     : {"DRY RUN (cmd_vel 발행 안 함)" if args.dry_run else "주행"}'
              f'{"  / 입력: " + args.video if args.video else ""}')
        print(f'속도     : {BASE_SPEED:.2f} m/s, 차선 상실 {LOST_SLOW_SEC}s 감속 / {LOST_STOP_SEC}s 정지')
        print(f'비상 정지: 전방 {FRONT_STOP_DISTANCE:.2f} m 이하, 초음파 {SONAR_TIMEOUT}s 끊김, 무효값')
        print('Space     : 출발 / 재개      Enter, ESC : 종료')
        print('=' * 60)
        print()

        period = 1.0 / CONTROL_HZ
        steps = 0
        loop_start = time.monotonic()

        with KeyInput() as keys:
            if not keys.enabled:
                print('경고: 터미널이 아니라 키를 받을 수 없습니다. 출발하지 않습니다.')
            next_deadline = time.monotonic()

            while rclpy.ok():
                quit_requested = False
                for key in keys.read():
                    if key == 'quit':
                        quit_requested = True
                    elif key == 'start':
                        ok, reason = node.try_start(time.monotonic())
                        print('출발' if ok else f'출발 불가: {reason}')
                if quit_requested:
                    print('\n종료 요청')
                    break

                frame = source.read()
                if frame is None:
                    print('\n영상 끝')
                    break
                now = time.monotonic()

                result = model.predict(source=frame, conf=CONF, imgsz=INFER_SIZE, verbose=False)[0]
                masks = build_masks(result, width, height)

                centers, error, steer_raw, valid = tracker.update(
                    pick_main(masks.get('left_lane')), pick_main(masks.get('right_lane')), now=now)
                steer = steer_raw * STEER_SIGN

                v, w, note = controller.command(steer, valid, tracker.lost_time)
                v, w, state = node.drive(v, w, now)

                steps += 1
                if steps % LOG_EVERY == 0:
                    fps = steps / (time.monotonic() - loop_start)
                    lane = 'lane OK' if valid else note
                    print(f'{fps:4.1f}fps | {state:20s} | {lane:24s} | steer {steer:+.2f} | '
                          f'v {v:.2f} w {w:+.2f} | {node.sonar_status(time.monotonic())}')

                next_deadline += period
                sleep_time = next_deadline - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_deadline = time.monotonic()

    except KeyboardInterrupt:
        print('\n사용자 중단 (Ctrl+C)')

    finally:
        node.stop()
        if source is not None:
            source.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('종료 완료')


if __name__ == '__main__':
    main()
