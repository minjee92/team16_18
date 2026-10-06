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
"""

import argparse
import csv
import os
import queue
import select
import signal
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Range

SRC_DIR = Path(__file__).resolve().parent.parent     # mj_ws/src/
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))                 # src/common 을 import 하기 위해

from common.lane_draw import draw_debug, draw_tracking                     # noqa: E402
from common.lane_postprocess import LaneTracker, group_masks, pick_main  # noqa: E402
from drive_control import FrontStop, LaneFollowController                # noqa: E402


# ============================== 설정 ==============================

BASE_DIR = SRC_DIR.parent   # mj_ws/

MODEL_PATH = str(BASE_DIR / 'models' / 'lane_model_ncnn' / 'best_ncnn_model')   # 폴더명에 _ncnn_model 필수
INFER_SIZE = 320
CONF = 0.5
FRAME_SIZE = (640, 480)
CONTROL_HZ = 10.0
LOG_EVERY = 10                   # 이 스텝마다 터미널에 상태 한 줄 (10Hz 면 1초)

# --- 주행 기록 ---
OUTPUT_DIR = BASE_DIR / 'outputs'
LOG_FLUSH_EVERY = 10             # CSV 를 이 줄마다 디스크에 씀 (중간에 죽어도 남게)
RECORD_QUEUE = 20                # 녹화 대기 프레임 상한. 넘치면 그 프레임은 버린다 (제어 루프 보호)

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


def build_masks(result, width, height):
    """ultralytics Results → 클래스 이름별 원본 크기 마스크 (후처리는 common 모듈)."""
    if result.masks is None or result.boxes is None:
        return {}
    names = result.names
    data = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    confs = result.boxes.conf.cpu().numpy()
    return group_masks(data, [names[c] for c in classes], confs, width, height)


# =========================== 주행 기록 ===========================

REPORT_CLASSES = (('left_lane', 'L'), ('right_lane', 'R'), ('crosswalk', 'CW'), ('cross_lane', 'CL'))


def output_paths(out_dir):
    """실행 시각을 붙인 CSV/영상 경로 (같은 이름, 확장자만 다름). 같은 초에 다시 실행하면 _1, _2 …"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f'result_step2_lane_follow_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    name, n = stem, 1
    while (out_dir / f'{name}.csv').exists() or (out_dir / f'{name}.mp4').exists():
        name, n = f'{stem}_{n}', n + 1
    return out_dir / f'{name}.csv', out_dir / f'{name}.mp4'


def class_confs(masks):
    """클래스별 최고 conf (검출이 없으면 None)."""
    return {name: max((d['conf'] for d in masks[name]), default=None) if masks.get(name) else None
            for name, _ in REPORT_CLASSES}


def _fmt(value, digits):
    return '' if value is None else f'{value:.{digits}f}'


class DriveLog:
    """프레임(제어 스텝)별 주행 로그 CSV. 값이 없으면 빈칸."""

    def __init__(self, path, sample_rows):
        self.path = path
        self.tags = [f'{round(r * 100)}' for r in sample_rows]          # '90', '80', '70', '60'
        columns = ['step', 't', 'state', 'valid', 'lost_time',
                   'left_det', 'left_conf', 'right_det', 'right_conf', 'crosswalk_det', 'cross_lane_det']
        for tag in self.tags:
            columns += [f'lx_{tag}', f'rx_{tag}', f'src_{tag}', f'cx_{tag}', f'hw_{tag}']
        columns += ['target_x', 'error', 'steer', 'v', 'w',
                    'sonar_range', 'sonar_age', 'estop_reason', 'recorded']
        self.file = open(path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        self.writer.writeheader()
        self.rows = 0

    def write(self, step, t, state, valid, tracker, confs, error, steer, v, w, sonar, recorded):
        row = {
            'step': step, 't': _fmt(t, 3), 'state': state, 'valid': int(valid),
            'lost_time': _fmt(tracker.lost_time, 3),
            'left_det': int(confs['left_lane'] is not None), 'left_conf': _fmt(confs['left_lane'], 3),
            'right_det': int(confs['right_lane'] is not None), 'right_conf': _fmt(confs['right_lane'], 3),
            'crosswalk_det': int(confs['crosswalk'] is not None),
            'cross_lane_det': int(confs['cross_lane'] is not None),
            'target_x': _fmt(tracker.target_x, 2), 'error': _fmt(error, 4), 'steer': _fmt(steer, 4),
            'v': _fmt(v, 3), 'w': _fmt(w, 3),
            'sonar_range': _fmt(sonar[0], 3), 'sonar_age': _fmt(sonar[1], 3), 'estop_reason': sonar[2],
            'recorded': int(recorded),
        }
        for tag, obs in zip(self.tags, tracker.rows_obs):
            row[f'lx_{tag}'] = _fmt(obs['lx'], 1)
            row[f'rx_{tag}'] = _fmt(obs['rx'], 1)
            row[f'src_{tag}'] = obs['source'] or ''
            row[f'cx_{tag}'] = '' if obs['cx'] is None else obs['cx']
            row[f'hw_{tag}'] = _fmt(obs['half_width'], 1)
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % LOG_FLUSH_EVERY == 0:
            self.file.flush()

    def close(self):
        self.file.close()


def render_frame(item):
    """녹화 한 프레임: 마스크 오버레이(ultralytics) + step1 과 같은 추적 그림 + 행별 관측·정보 줄."""
    view = item['result'].plot()
    view = draw_tracking(view, item['centers'], item['error'], item['steer'], item['valid'],
                         item['events'], item['tracker'])
    return draw_debug(view, item['rows_obs'], item['lines'], top=58 + 28 * len(item['events']))   # 상태·이벤트 글자 아래


class Recorder:
    """
    주행 기록 영상. 그리기와 인코딩은 별도 스레드에서 한다.
    제어 루프는 submit() 으로 넘기기만 하고, 큐가 차 있으면 그 프레임을 버린다 (기다리지 않음).
    """

    def __init__(self, path, size, fps):
        self.path = path
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        if not self.writer.isOpened():
            raise RuntimeError(f'녹화 파일을 열 수 없습니다: {path}')
        self.queue = queue.Queue(maxsize=RECORD_QUEUE)
        self.written = 0
        self.dropped = 0
        self.failed = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, item):
        """반환: 녹화 대기열에 들어갔는지 (False 면 버려짐)."""
        try:
            self.queue.put_nowait(item)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        while True:
            item = self.queue.get()
            if item is None:
                break
            try:
                self.writer.write(render_frame(item))
                self.written += 1
            except Exception as error:           # 녹화 실패가 주행을 멈추지 않게
                self.failed += 1
                if self.failed == 1:
                    print(f'녹화 오류 (이후 같은 오류는 세기만 함): {error}')

    def close(self):
        self.queue.put(None)                     # 남은 프레임을 다 쓴 뒤 끝남
        self.thread.join(timeout=60)
        self.writer.release()


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

    def sonar_snapshot(self, now):
        """(마지막 거리 m, 측정 후 지난 초, 비상 정지 이유) — 측정이 없으면 None."""
        with self.lock:
            if self.front.last_time is None:
                return None, None, self.front.trip_reason if self.front.tripped else ''
            return (self.front.last_range, now - self.front.last_time,
                    self.front.trip_reason if self.front.tripped else '')

    def sonar_status(self, now):
        distance, age, _ = self.sonar_snapshot(now)
        if distance is None:
            return 'sonar -'
        return f'sonar {distance:.2f}m ({age:.2f}s 전)'

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
    parser.add_argument('--record', action='store_true',
                        help='마스크 오버레이 주행 영상도 저장 (프레임별 CSV 로그는 항상 저장)')
    parser.add_argument('--out-dir', default=str(OUTPUT_DIR),
                        help='CSV/영상 저장 폴더 (기본: %(default)s)')
    return parser.parse_args()


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
    args = parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # 신호는 StopRequest 가 처리
    stop = StopRequest()
    stop.install()

    node = Step2LaneFollow(publish_cmd=not args.dry_run)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=spin_quietly, args=(executor,), daemon=True)
    spin_thread.start()

    source = None
    log = None
    recorder = None
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

        csv_path, video_path = output_paths(args.out_dir)
        log = DriveLog(csv_path, SAMPLE_ROWS)
        if args.record:
            recorder = Recorder(video_path, FRAME_SIZE, CONTROL_HZ)

        print()
        print('=' * 60)
        print(f'모드     : {"DRY RUN (cmd_vel 발행 안 함)" if args.dry_run else "주행"}'
              f'{"  / 입력: " + args.video if args.video else ""}')
        print(f'속도     : {BASE_SPEED:.2f} m/s, 차선 상실 {LOST_SLOW_SEC}s 감속 / {LOST_STOP_SEC}s 정지')
        print(f'비상 정지: 전방 {FRONT_STOP_DISTANCE:.2f} m 이하, 초음파 {SONAR_TIMEOUT}s 끊김, 무효값')
        print(f'주행 로그: {csv_path}')
        print(f'주행 영상: {video_path if recorder else "(--record 없음)"}')
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

            while not stop.requested:
                for key in keys.read():
                    if key == 'quit':
                        stop.request('키 Enter/ESC')
                    elif key == 'start':
                        ok, reason = node.try_start(time.monotonic())
                        print('출발' if ok else f'출발 불가: {reason}')
                if stop.requested:
                    break

                frame = source.read()
                if frame is None:
                    stop.request('영상 끝')
                    break
                now = time.monotonic()

                result = model.predict(source=frame, conf=CONF, imgsz=INFER_SIZE, verbose=False)[0]
                masks = build_masks(result, width, height)

                centers, error, steer_raw, valid = tracker.update(
                    pick_main(masks.get('left_lane')), pick_main(masks.get('right_lane')), now=now)
                steer = steer_raw * STEER_SIGN

                v, w, note = controller.command(steer, valid, tracker.lost_time)
                v, w, state = node.drive(v, w, now)

                step, t = steps, now - loop_start
                confs = class_confs(masks)
                sonar = node.sonar_snapshot(time.monotonic())   # 프레임 시각 이후 도착한 측정도 있어 지금 시각 기준
                recorded = False
                if recorder is not None:
                    events = [text for name, text in (('crosswalk', 'CROSSWALK'), ('cross_lane', 'CROSS LANE'))
                              if confs[name] is not None]
                    det = '  '.join(f'{short} {_fmt(confs[name], 2) or "-"}' for name, short in REPORT_CLASSES)
                    sonar_text = '-' if sonar[0] is None else f'{sonar[0]:.2f}m ({sonar[1]:.2f}s)'
                    recorded = recorder.submit({
                        'result': result, 'centers': centers, 'error': error, 'steer': steer,
                        'valid': valid, 'events': events, 'rows_obs': tracker.rows_obs,
                        'tracker': SimpleNamespace(rows=tracker.rows, target_x=tracker.target_x),
                        'lines': [f'step {step}  t {t:.1f}s  {state}',
                                  f'steer {steer:+.3f}  v {v:.2f}  w {w:+.2f}  lost {tracker.lost_time:.2f}s',
                                  det,
                                  f'sonar {sonar_text}  {sonar[2]}'],
                    })
                log.write(step, t, state, valid, tracker, confs, error, steer, v, w, sonar, recorded)

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

        def stop_spin():
            executor.shutdown()
            spin_thread.join(timeout=5)

        run_cleanup([
            ('정지 명령', node.stop),                                  # context 가 살아 있을 때 먼저
            ('카메라', source.close if source is not None else None),
            ('녹화', close_recorder if recorder is not None else None),
            ('주행 로그', close_log if log is not None else None),
            ('ROS spin 스레드', stop_spin),
            ('ROS 노드', node.destroy_node),
            ('rclpy', lambda: rclpy.ok() and rclpy.shutdown()),
            ('신호 무시', stop.ignore_all),        # rclpy.shutdown() 이 핸들러를 되돌려 놓으므로, 끝난 뒤엔 무시
        ])
        print('종료 완료', flush=True)
    return exit_code

if __name__ == '__main__':
    sys.exit(main())
