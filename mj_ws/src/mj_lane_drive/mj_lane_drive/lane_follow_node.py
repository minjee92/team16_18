#!/usr/bin/env python3
"""
2단계: 차선 가운데로 주행 (로봇)

카메라 → ncnn 차선 세그멘테이션 → 공통 후처리(mj_lane_core) → cmd_vel

  - 차선을 못 보면 시간 기준 단계적 대응 (0.5초 감속 / 2.0초 정지)
  - 전방 초음파 비상 정지: 차선 검출과 무관하게 전방이 가까우면 즉시 정지 (fail-safe, 수동 재개)
  - 출력은 cmd_vel(m/s, rad/s)만. 바퀴 rpm·부호·다이나믹셀 단위 변환은 pinky_bringup 이 한다

  구독: /us_sensor/range (sensor_msgs/Range)   ← pinky_sensor_adc 가 publish (--sonar-topic 으로 바꿈)
        camera/image_raw (sensor_msgs/Image)    ← --camera-topic 일 때만 (시뮬)
  발행: cmd_vel (Twist)                         ← --namespace amr_01 이면 /amr_01/cmd_vel
        lane_debug/image (+ /compressed)       ← 검출 표시 영상. 구독자가 있을 때만 그림 (--no-debug-image 로 끔)

시간: 기본은 벽시계(time.monotonic, 실물). --use-sim-time 이면 /clock 의 시뮬 시간으로 판단·제어 주기를 잰다.
  차선 상실 시간은 제어 주기(0.1초) 칸 수로 세어 판단에 넘긴다 (lane_mission_drive.py 의 "놓친 프레임 수" 와 같은 규칙).

실행 순서 (PLAN.md "2단계 실행 절차"):
    1) ros2 launch pinky_bringup bringup_robot.launch.xml
    2) ros2 run pinky_sensor_adc main_node         ← bringup 에 없음. 필수
    3) ros2 topic hz /us_sensor/range               ← 약 20 Hz 확인
    4) python3 src/robot/step2_lane_follow.py --dry-run   (처음엔 바퀴 없이 확인)
       python3 src/robot/step2_lane_follow.py [--record]

키 (실행한 터미널에서):
    Space       출발 / 비상 정지 후 재개 (초음파가 정상이고 장애물이 없을 때만 받아들임)
    Enter, ESC  정지 후 종료

주행 기록 (outputs/, 실행 시각을 붙인 같은 이름):
    result_step2_lane_follow_<YYYYmmdd_HHMMSS>.csv   프레임별 로그 (항상)
        좌/우 차선 검출·conf, 샘플 행별 lx/rx/중앙점/도로 반폭, target_x, steer, v/w, 초음파
    result_step2_lane_follow_<YYYYmmdd_HHMMSS>.mp4   마스크 오버레이 영상 (--record 일 때만)
        화면의 step 번호로 CSV 줄과 맞춘다. 그리기·인코딩은 별도 스레드라 제어 루프를 막지 않는다

PC 테스트: --video 로 카메라 대신 저장 영상을 넣는다 (ROS 환경 필요).
시뮬: --camera-topic --namespace amr_01 --sonar-topic us_sensor/range --use-sim-time
      (mj_lane_sim 의 run_mj_sim.sh drive 가 이렇게 띄운다)
"""

import argparse
import os
import select
import signal
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Range

from mj_lane_core.drive_control import FrontStop, LaneFollowController
from mj_lane_core.lane_postprocess import LaneTracker, group_masks, pick_main
from mj_lane_drive.drive_record import (REPORT_CLASSES, DriveLog, Recorder, _fmt, class_confs,
                                        output_paths)
from mj_lane_drive.debug_view import DebugImagePublisher
from mj_lane_drive.sources import NO_FRAME, PiCameraSource, RosImageSource, VideoSource


# ============================== 설정 ==============================

# mj_ws/ (이 파일: mj_ws/src/mj_lane_drive/mj_lane_drive/). colcon 으로 복사 설치했을 때는 MJ_WS 로 알려 준다
BASE_DIR = Path(os.environ.get('MJ_WS', Path(__file__).resolve().parents[3]))

MODEL_PATH = str(BASE_DIR / 'models' / 'lane_model_ncnn' / 'best_ncnn_model')   # 폴더명에 _ncnn_model 필수
INFER_SIZE = 320
CONF = 0.5
FRAME_SIZE = (640, 480)
CONTROL_HZ = 10.0
LOG_EVERY = 10                   # 이 스텝마다 터미널에 상태 한 줄 (10Hz 면 1초)

# --- 주행 기록 ---
OUTPUT_DIR = BASE_DIR / 'outputs'

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
SONAR_MIN_RANGE = 0.02           # m. 드라이버의 min_range. 이보다 작거나 NaN/inf 면 무효값 (I2C 실패 −0.03 등)
SONAR_INVALID_COUNT = 3          # 무효값이 이만큼 연속이면 정지 (20 Hz 면 0.15초). 그보다 짧은 튐은 무시하고 센다

# --- ROS 카메라 토픽 입력 (--camera-topic) ---
CAMERA_TOPIC = 'camera/image_raw'  # 노드 기준 상대 이름. --namespace amr_01 이면 /amr_01/camera/image_raw
CAMERA_TIMEOUT = 0.5             # s (벽시계). 새 영상이 이만큼 안 오면 정지 명령을 내고 계속 기다린다
CLOCK_WAIT = 10.0                # s (벽시계). --use-sim-time 인데 /clock 이 이만큼 안 오면 종료

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


def verify_input_size(model, expected):
    """
    워밍업 추론을 한 번 하고, 모델이 실제로 쓰는 입력 크기가 expected 와 같은지 확인한다.
    반환: (ok, 메시지)

    export 된 고정 크기 모델(ncnn)은 첫 predict 에서만 export 크기로 바뀌고, 다음 predict 부터는
    요청한 크기가 다시 들어가 두 번째 프레임부터 검출이 0 이 된다. 로봇이 "차선 없음"으로 조용히
    멈추는 대신, 시작할 때 이 불일치를 잡아 종료한다.
    """
    dummy = np.zeros((FRAME_SIZE[1], FRAME_SIZE[0], 3), np.uint8)
    model.predict(source=dummy, conf=CONF, imgsz=expected, verbose=False)
    used = model.predictor.args.imgsz
    used = list(used) if isinstance(used, (list, tuple)) else [used, used]
    if used == [expected, expected]:
        return True, f'입력 크기 {expected} 확인'
    return False, (f'모델 입력 크기 불일치: 이 모델은 {used[1]}x{used[0]} 로 고정되어 있는데 INFER_SIZE = {expected} 입니다.\n'
                   f'  이대로 주행하면 두 번째 프레임부터 차선이 하나도 검출되지 않습니다.\n'
                   f'  INFER_SIZE 를 {used[0]} 로 바꾸거나, 모델을 {expected} 로 다시 export 하세요.')


class ControlTicks:
    """차선 상실 판단에 넘기는 시각을 제어 주기(0.1초) 칸 수로 만든다.

    상실 시간을 실제 시계로 그대로 재면 10 Hz 루프의 5·20번째 프레임이 경계값 0.5·2.0초에 1 ms 안쪽으로 걸려,
    실행마다 감속·정지 시작 프레임이 한 칸씩 갈렸다. 칸 수로 세면 놓친 프레임 k 개 = k × 0.1초 가 되어
    lane_mission_drive.py 의 "놓친 프레임 수 > 5 / > 20" 과 같은 규칙으로 고정된다.

    프레임마다 "앞 프레임과의 간격 ÷ 주기" 를 반올림해 더한다 (최소 1칸).
    처음(A4)에는 시작 시각부터의 경과를 반올림했는데, 루프가 한 번 0.14초쯤 밀리면 그 뒤 프레임들이 칸의
    정확히 중간에 찍혀 반올림이 위아래로 갈렸다 (상실 시간이 0초·0.2초씩 튐). 앞 프레임 기준이면 보통 간격
    (0.05–0.15초)은 늘 1칸이고, 정말 밀린 만큼만(0.15초 넘게) 칸을 더 센다."""

    def __init__(self, period):
        self.period, self.prev, self.ticks = period, None, 0

    def time(self, now):
        if self.prev is not None:
            self.ticks += max(1, round((now - self.prev) / self.period))
        self.prev = now
        return self.ticks * self.period


def build_masks(result, width, height):
    """ultralytics Results → 클래스 이름별 원본 크기 마스크 (후처리는 mj_lane_core)."""
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
    시각은 모두 now_sec() 하나로 잰다 (벽시계 또는 시뮬 시간).
    """

    def __init__(self, publish_cmd, namespace='', sonar_topic=SONAR_TOPIC, use_sim_time=False):
        overrides = [Parameter('use_sim_time', Parameter.Type.BOOL, True)] if use_sim_time else []
        super().__init__('step2_lane_follow', namespace=namespace, parameter_overrides=overrides)
        self.publish_cmd = publish_cmd
        self.use_sim_time = use_sim_time
        self.lock = threading.Lock()
        self.front = FrontStop(FRONT_STOP_DISTANCE, SONAR_TIMEOUT, SONAR_MIN_RANGE, SONAR_INVALID_COUNT)
        self.started = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sonar_sub = self.create_subscription(Range, sonar_topic, self.on_range, 10)

    def now_sec(self):
        """판단에 쓰는 시각 (초). 기본은 벽시계, --use-sim-time 이면 /clock (오기 전에는 0)."""
        if self.use_sim_time:
            return self.get_clock().now().nanoseconds * 1e-9
        return time.monotonic()

    def sleep_until(self, deadline, should_stop):
        """now_sec() 기준 deadline 까지 기다린다. 시뮬 시간은 멈출 수도 있어 짧게 나눠 기다리며 종료 요청을 본다."""
        if not self.use_sim_time:
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            return
        while not should_stop() and self.now_sec() < deadline:
            time.sleep(0.005)

    # ---------- 초음파 (executor 스레드) ----------

    def on_range(self, msg):
        now = self.now_sec()
        with self.lock:
            was_tripped = self.front.tripped
            self.front.on_range(msg.range, now)
            if self.front.check(now) and not was_tripped:
                self._publish(0.0, 0.0)              # 즉시 정지
                if self.started:
                    self.get_logger().warn(
                        f'비상 정지: {self.front.trip_reason} (range {msg.range:.3f} m, '
                        f'무효 연속 {self.front.invalid_streak}) — Space 로 재개')

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

    def sonar_snapshot(self, now):
        """초음파 상태 한 벌 (로그·영상용). 측정이 없으면 range/age 는 None.
          range: 마지막 측정(무효값 포함), valid_range: 판정에 쓰는 마지막 유효 거리,
          age: 측정 후 지난 초, streak: 현재 무효 연속 수, ignored: 지금까지 무시한 무효 측정 수,
          reason: 비상 정지 중이면 그 이유"""
        with self.lock:
            front = self.front
            return {
                'range': front.last_range,
                'valid_range': front.valid_range,
                'age': None if front.last_time is None else now - front.last_time,
                'streak': front.invalid_streak,
                'ignored': front.invalid_summary()[0],
                'reason': front.trip_reason if front.tripped else '',
            }

    def sonar_invalid_summary(self):
        with self.lock:
            return self.front.invalid_summary()

    def sonar_status(self, now):
        sonar = self.sonar_snapshot(now)
        if sonar['range'] is None:
            return 'sonar -'
        return f'sonar {sonar["range"]:.2f}m ({sonar["age"]:.2f}s 전)'

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

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='2단계: 차선 가운데로 주행 (cmd_vel 발행)')
    parser.add_argument('--model', default=MODEL_PATH,
                        help='ncnn 모델 폴더 (이름에 _ncnn_model 필수, 기본: %(default)s)')
    parser.add_argument('--dry-run', action='store_true',
                        help='계산만 하고 cmd_vel 을 발행하지 않는다 (바퀴 정지)')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--video', default=None,
                        help='PC 테스트용: 카메라 대신 저장 영상 사용')
    source.add_argument('--camera-topic', nargs='?', const=CAMERA_TOPIC, default=None, metavar='TOPIC',
                        help=f'카메라 대신 ROS 영상 토픽 사용 (값 없이 쓰면 {CAMERA_TOPIC}, 노드 네임스페이스 기준)')
    parser.add_argument('--namespace', default='',
                        help='노드 네임스페이스 (예: amr_01 → /amr_01/cmd_vel). 기본: 없음')
    parser.add_argument('--sonar-topic', default=SONAR_TOPIC,
                        help='초음파 Range 토픽. / 로 시작하지 않으면 네임스페이스 기준 (기본: %(default)s)')
    parser.add_argument('--use-sim-time', action='store_true',
                        help='판단·제어 주기를 /clock 시뮬 시간으로 잰다 (시뮬 전용. 기본은 벽시계)')
    parser.add_argument('--record', action='store_true',
                        help='마스크 오버레이 주행 영상도 저장 (프레임별 CSV 로그는 항상 저장)')
    parser.add_argument('--no-debug-image', action='store_true',
                        help='검출 표시 영상 토픽(lane_debug/image)을 만들지 않는다 (기본: 구독자가 있을 때만 그림)')
    parser.add_argument('--out-dir', default=str(OUTPUT_DIR),
                        help='CSV/영상 저장 폴더 (기본: %(default)s)')
    return parser.parse_args(argv)


def wait_for_clock(node, stop):
    """--use-sim-time: /clock 이 와서 시각이 0 보다 커질 때까지 기다린다 (벽시계 CLOCK_WAIT 초)."""
    deadline = time.monotonic() + CLOCK_WAIT
    while not stop.requested and time.monotonic() < deadline:
        if node.now_sec() > 0:
            return True
        time.sleep(0.05)
    return False


class StopRequest:
    """
    종료 요청 (키·영상 끝·신호를 한 경로로). rclpy 의 신호 처리 대신 직접 처리한다.

    rclpy 기본 처리는 SIGINT/SIGTERM 에서 context 를 바로 내려서, 그 뒤 정지 명령 publish 가
    "publisher's context is invalid" 로 실패하고 녹화 마무리(moov)를 건너뛰었다. SIGHUP(SSH 끊김)은
    아예 처리하지 않아 즉시 죽었다. 여기서는 신호가 오면 플래그만 세우고, 메인 루프가 빠져나온 뒤
    정해진 순서로 정리한다. 정리 중에 다시 오는 신호는 무시해 정리가 끊기지 않게 한다.
    """

    SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)

    def __init__(self):
        self.requested = False
        self.reason = ''

    def install(self):
        for sig in self.SIGNALS:
            signal.signal(sig, self._on_signal)

    def ignore_all(self):
        """정리가 끝난 뒤: 신호를 완전히 무시한다. 파이썬 종료 과정은 직접 단 핸들러를 기본값으로 되돌리지만
        SIG_IGN 은 그대로 두므로, 종료 직전에 온 신호가 프로세스를 죽여 종료 코드를 바꾸지 않는다."""
        for sig in self.SIGNALS:
            signal.signal(sig, signal.SIG_IGN)

    def request(self, reason):
        if not self.requested:
            self.requested = True
            self.reason = reason

    def _on_signal(self, signum, frame):
        name = signal.Signals(signum).name
        if self.requested:
            print(f'\n{name}: 정리 중 — 정지 명령과 영상 저장을 마칠 때까지 기다립니다', flush=True)
            return
        self.request(f'신호 {name}')


def spin_quietly(executor):
    """ROS executor 스레드. context 가 밖에서 내려가도 traceback 없이 끝낸다."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def run_cleanup(steps):
    """정리 단계를 순서대로 실행한다. 한 단계가 실패해도 다음 단계는 실행한다."""
    for name, action in steps:
        if action is None:
            continue
        try:
            action()
        except Exception as error:
            print(f'정리 실패 ({name}): {type(error).__name__}: {error}', flush=True)


def main():
    args = parse_args(remove_ros_args(sys.argv)[1:])         # --ros-args ... 는 rclpy 몫

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # 신호는 StopRequest 가 처리
    stop = StopRequest()
    stop.install()

    node = Step2LaneFollow(publish_cmd=not args.dry_run, namespace=args.namespace,
                           sonar_topic=args.sonar_topic, use_sim_time=args.use_sim_time)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=spin_quietly, args=(executor,), daemon=True)
    spin_thread.start()

    source = None
    log = None
    recorder = None
    debug = None
    exit_code = 0
    try:
        from ultralytics import YOLO
        node.get_logger().info(f'모델 로딩: {args.model}')
        model = YOLO(args.model, task='segment')
        ok, message = verify_input_size(model, INFER_SIZE)
        if not ok:
            print(f'\n오류: {message}\n')
            stop.request('입력 크기 불일치')
            exit_code = 2
            return exit_code                 # finally 에서 정리 후 종료
        node.get_logger().info(message)

        if args.video:
            source = VideoSource(args.video, FRAME_SIZE)
            input_text = args.video
        elif args.camera_topic:
            source = RosImageSource(node, args.camera_topic, FRAME_SIZE, CAMERA_TIMEOUT,
                                    warn=node.get_logger().warn)
            input_text = f'토픽 {source.topic}'
        else:
            source = PiCameraSource(FRAME_SIZE, rotate_180=ROTATE_180, mirror=MIRROR_INPUT, swap_rb=SWAP_RB)
            input_text = 'Picamera2'
        width, height = FRAME_SIZE

        if args.use_sim_time:
            if not wait_for_clock(node, stop):
                if not stop.requested:
                    print(f'\n오류: --use-sim-time 인데 {CLOCK_WAIT:.0f}초 동안 /clock 이 오지 않았습니다 '
                          f'(시뮬이 떠 있는지, 같은 ROS_DOMAIN_ID 인지 확인)\n')
                    stop.request('/clock 없음')
                    exit_code = 2
                return exit_code

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

        csv_path, video_path = output_paths(args.out_dir)
        log = DriveLog(csv_path, SAMPLE_ROWS)
        if not args.no_debug_image:
            camera_frame = f'{args.namespace.strip("/")}/front_camera_link' if args.namespace else 'front_camera_link'
            debug = DebugImagePublisher(node, frame_id=camera_frame)
        if args.record:
            recorder = Recorder(video_path, FRAME_SIZE, CONTROL_HZ,
                                on_rendered=debug.publish_rendered if debug is not None else None)

        print()
        print('=' * 60)
        print(f'모드     : {"DRY RUN (cmd_vel 발행 안 함)" if args.dry_run else "주행"}  / 입력: {input_text}')
        print(f'토픽     : 발행 {node.cmd_pub.topic_name}, 초음파 {node.sonar_sub.topic_name}')
        print(f'시간     : {"시뮬 시간 (/clock)" if args.use_sim_time else "벽시계"}, 제어 {CONTROL_HZ:.0f} Hz')
        print(f'속도     : {BASE_SPEED:.2f} m/s, 차선 상실 {LOST_SLOW_SEC}s 감속 / {LOST_STOP_SEC}s 정지')
        print(f'비상 정지: 전방 {FRONT_STOP_DISTANCE:.2f} m 이하, 초음파 {SONAR_TIMEOUT}s 끊김, '
              f'무효값 {SONAR_INVALID_COUNT}회 연속')
        print(f'주행 로그: {csv_path}')
        print(f'주행 영상: {video_path if recorder else "(--record 없음)"}')
        print(f'표시 영상: {debug.topic + " (구독자가 있을 때만)" if debug else "(끔)"}')
        print('Space     : 출발 / 재개      Enter, ESC : 종료')
        print('=' * 60)
        print()

        period = 1.0 / CONTROL_HZ
        steps = 0
        wall_start = time.monotonic()                # fps 표시용 (항상 벽시계)
        loop_start = node.now_sec()
        ticks = ControlTicks(period)
        camera_stalled = False

        with KeyInput() as keys:
            if not keys.enabled:
                print('경고: 터미널이 아니라 키를 받을 수 없습니다. 출발하지 않습니다.')
            next_deadline = node.now_sec()

            while not stop.requested:
                for key in keys.read():
                    if key == 'quit':
                        stop.request('키 Enter/ESC')
                    elif key == 'start':
                        ok, reason = node.try_start(node.now_sec())
                        print('출발' if ok else f'출발 불가: {reason}')
                if stop.requested:
                    break

                frame = source.read()
                if frame is None:
                    stop.request('영상 끝')
                    break
                if frame is NO_FRAME:                # 토픽 영상이 끊김: 멈추고 다시 기다린다 (기록 줄은 쓰지 않음)
                    node.drive(0.0, 0.0, node.now_sec())
                    if not camera_stalled:
                        node.get_logger().warn(f'카메라 영상이 {CAMERA_TIMEOUT}초 넘게 오지 않음 — 정지하고 기다립니다')
                        camera_stalled = True
                    next_deadline = node.now_sec()
                    continue
                if camera_stalled:
                    node.get_logger().info('카메라 영상 다시 받음')
                    camera_stalled = False
                now = node.now_sec()

                result = model.predict(source=frame, conf=CONF, imgsz=INFER_SIZE, verbose=False)[0]
                masks = build_masks(result, width, height)

                centers, error, steer_raw, valid = tracker.update(
                    pick_main(masks.get('left_lane')), pick_main(masks.get('right_lane')),
                    now=ticks.time(now))
                steer = steer_raw * STEER_SIGN

                v, w, note = controller.command(steer, valid, tracker.lost_time)
                v, w, state = node.drive(v, w, now)

                step, t = steps, now - loop_start
                confs = class_confs(masks)
                sonar = node.sonar_snapshot(node.now_sec())   # 프레임 시각 이후 도착한 측정도 있어 지금 시각 기준
                recorded = False
                show = debug is not None and recorder is None and debug.wanted()   # 녹화 중이면 녹화 그림을 같이 씀
                if recorder is not None or show:
                    events = [text for name, text in (('crosswalk', 'CROSSWALK'), ('cross_lane', 'CROSS LANE'))
                              if confs[name] is not None]
                    det = '  '.join(f'{short} {_fmt(confs[name], 2) or "-"}' for name, short in REPORT_CLASSES)
                    sonar_text = ('-' if sonar['range'] is None else
                                  f'{sonar["range"]:.2f}m ({sonar["age"]:.2f}s)  '
                                  f'inv {sonar["streak"]}/{SONAR_INVALID_COUNT} ign {sonar["ignored"]}')
                    item = {
                        'frame': frame, 'masks': masks, 'centers': centers, 'error': error, 'steer': steer,
                        'valid': valid, 'events': events, 'rows_obs': tracker.rows_obs,
                        'tracker': SimpleNamespace(rows=tracker.rows, target_x=tracker.target_x),
                        'lines': [f'step {step}  t {t:.1f}s  {state}',
                                  f'steer {steer:+.3f}  v {v:.2f}  w {w:+.2f}  lost {tracker.lost_time:.2f}s',
                                  det,
                                  f'sonar {sonar_text}  {sonar["reason"]}'],
                    }
                    if recorder is not None:
                        recorded = recorder.submit(item)
                    else:
                        debug.submit(item)
                log.write(step, t, state, valid, tracker, confs, error, steer, v, w, sonar, recorded)

                steps += 1
                if steps % LOG_EVERY == 0:
                    fps = steps / (time.monotonic() - wall_start)
                    lane = 'lane OK' if valid else note
                    print(f'{fps:4.1f}fps | {state:20s} | {lane:24s} | steer {steer:+.2f} | '
                          f'v {v:.2f} w {w:+.2f} | {node.sonar_status(node.now_sec())}')

                next_deadline += period
                if next_deadline > node.now_sec():
                    node.sleep_until(next_deadline, lambda: stop.requested)
                else:
                    next_deadline = node.now_sec()

    except Exception:
        import traceback
        traceback.print_exc()
        stop.request('예외')
        exit_code = 1

    finally:
        stop.request('정리')                      # 이후 오는 신호는 무시 (정리가 끊기지 않게)
        print(f'\n종료: {stop.reason}', flush=True)

        def close_recorder():
            recorder.close()
            print(f'영상 저장 완료: {recorder.path} ({recorder.written}프레임)', flush=True)
            if recorder.dropped or recorder.failed:
                print(f'  녹화 버림 {recorder.dropped}, 오류 {recorder.failed}', flush=True)

        def close_log():
            log.close()
            print(f'주행 로그: {log.path} ({log.rows}줄)', flush=True)

        def report_sonar():
            ignored, spikes, stops = node.sonar_invalid_summary()
            print(f'초음파 무효값: 무시 {ignored}회 (단발 튐 {spikes}번), 무효로 정지 {stops}번', flush=True)

        def stop_spin():
            executor.shutdown()
            spin_thread.join(timeout=5)

        run_cleanup([
            ('정지 명령', node.stop),                                  # context 가 살아 있을 때 먼저
            ('카메라', source.close if source is not None else None),
            ('녹화', close_recorder if recorder is not None else None),
            ('표시 영상', debug.close if debug is not None else None),   # 녹화가 끝난 뒤 (녹화 스레드가 발행을 부름)
            ('주행 로그', close_log if log is not None else None),
            ('초음파 요약', report_sonar),
            ('ROS spin 스레드', stop_spin),
            ('ROS 노드', node.destroy_node),
            ('rclpy', lambda: rclpy.ok() and rclpy.shutdown()),
            ('신호 무시', stop.ignore_all),        # rclpy.shutdown() 이 핸들러를 되돌려 놓으므로, 끝난 뒤엔 무시
        ])
        print('종료 완료', flush=True)
    return exit_code

if __name__ == '__main__':
    sys.exit(main())
