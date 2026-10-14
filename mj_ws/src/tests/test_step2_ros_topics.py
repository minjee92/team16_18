"""
[느림: ncnn 모델 + 영상 + ROS 환경, 약 1분] 주행 노드를 ROS 토픽 입력으로 띄우는 시험 (A4)

가제보 없이 가짜 발행자로 시뮬 상황을 흉내 낸다.
  A. 카메라 토픽 + 네임스페이스 (벽시계)
     저장 영상 프레임을 /mjtest/camera/image_raw 에 rgb8 800x600 으로 보내고 (규격과 다른 크기),
     가짜 초음파는 /mjtest/us_sensor/range (노드에는 상대 이름 us_sensor/range 로 줌).
     → /mjtest/cmd_vel 로만 나가는지, 영상이 끊기면 멈추고 다시 오면 가는지, 크기 경고가 한 번인지.
     검출 표시 영상 /mjtest/lane_debug/image 를 구독해 받는지 (구독자가 있을 때만 그림).
  B. 시뮬 시간 (--use-sim-time)
     가짜 /clock 을 실제의 절반 속도(RTF 0.5)로 보낸다. 차선이 없는 검은 영상이라 계속 "상실".
     → CSV 의 시각·상실 시간이 시뮬 초 기준(줄마다 0.1초)인지, 제어 루프가 실제 시간으로는 5 Hz 로 도는지.
  C. --use-sim-time 인데 /clock 이 없음 → 오류 안내 후 종료 코드 2.

안전: ROS_DOMAIN_ID=87, LOCALHOST (다른 시험과 같음).
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import csv           # noqa: E402
import pty           # noqa: E402
import statistics    # noqa: E402
import subprocess    # noqa: E402
import sys           # noqa: E402
import tempfile      # noqa: E402
import threading     # noqa: E402
import time          # noqa: E402
from pathlib import Path  # noqa: E402

import cv2           # noqa: E402
import numpy as np   # noqa: E402

import testlib       # noqa: E402

T = testlib.Checker('step2_ros_topics')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')
testlib.require(testlib.VIDEO, '입력 영상')

import rclpy                                        # noqa: E402
from builtin_interfaces.msg import Time             # noqa: E402
from geometry_msgs.msg import Twist                 # noqa: E402
from rclpy.executors import MultiThreadedExecutor   # noqa: E402
from rclpy.node import Node                         # noqa: E402
from rclpy.qos import qos_profile_sensor_data       # noqa: E402
from rosgraph_msgs.msg import Clock                 # noqa: E402
from sensor_msgs.msg import Image, Range            # noqa: E402

import mj_lane_drive.lane_follow_node as S2       # noqa: E402

NS = 'mjtest'
STALL_STOP_MAX = S2.CAMERA_TIMEOUT + 0.35         # 영상 끊김 → 0 명령까지 (기다림 + 제어 한 번 + 여유)


class Harness(Node):
    def __init__(self):
        super().__init__('step2_topics_harness')
        self.image_on = False
        self.black = False
        self.clock_on = False
        self.sim_ns = 1_000_000_000                    # 시뮬 시각 1초부터
        self.cmds, self.root_cmds, self.debug = [], [], []
        cap = cv2.VideoCapture(str(testlib.VIDEO))
        self.frames = []
        for _ in range(200):
            ok, frame = cap.read()
            if not ok:
                break
            self.frames.append(cv2.cvtColor(cv2.resize(frame, (800, 600)), cv2.COLOR_BGR2RGB).tobytes())
        self.index = 0
        self.img_pub = self.create_publisher(Image, f'/{NS}/camera/image_raw', qos_profile_sensor_data)
        self.range_pub = self.create_publisher(Range, f'/{NS}/us_sensor/range', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.create_subscription(Twist, f'/{NS}/cmd_vel',
                                 lambda m: self.cmds.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.root_cmds.append(m), 50)
        self.create_subscription(Image, f'/{NS}/lane_debug/image',
                                 lambda m: self.debug.append((m.width, m.height, m.encoding)), 5)
        self.create_timer(0.05, self.tick_image)       # 20 Hz
        self.create_timer(0.05, self.tick_sonar)       # 20 Hz (실제 드라이버)
        self.create_timer(0.01, self.tick_clock)       # 실제 0.01초마다 시뮬 0.005초 → RTF 0.5

    def tick_image(self):
        if not self.image_on:
            return
        msg = Image()
        msg.width, msg.height, msg.encoding, msg.step = 800, 600, 'rgb8', 800 * 3
        msg.data = bytes(800 * 600 * 3) if self.black else self.frames[self.index % len(self.frames)]
        self.index += 1
        self.img_pub.publish(msg)

    def tick_sonar(self):
        msg = Range()
        msg.min_range, msg.max_range, msg.range = 0.02, 3.0, 0.5
        self.range_pub.publish(msg)

    def tick_clock(self):
        if self.clock_on:
            self.sim_ns += 5_000_000
            self.clock_pub.publish(Clock(clock=Time(sec=self.sim_ns // 1_000_000_000,
                                                    nanosec=self.sim_ns % 1_000_000_000)))


rclpy.init()
harness = Harness()
executor = MultiThreadedExecutor(num_threads=3)
executor.add_node(harness)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()


def start_node(extra, label):
    out_dir = tempfile.mkdtemp(prefix=f'step2_topics_{label}_')
    log_path = Path(out_dir) / 'node.log'
    log = open(log_path, 'w')
    master, slave = pty.openpty()
    proc = subprocess.Popen(
        [sys.executable, '-u', str(testlib.SRC_DIR / 'robot' / 'step2_lane_follow.py'), '--out-dir', out_dir] + extra,
        stdin=slave, stdout=log, stderr=subprocess.STDOUT, cwd=str(testlib.MJ_DIR))
    os.close(slave)
    return proc, master, log, log_path, Path(out_dir)


def wait_for(log_path, text, proc, limit=90):
    deadline = time.monotonic() + limit
    while text not in log_path.read_text() and proc.poll() is None and time.monotonic() < deadline:
        time.sleep(0.2)
    return text in log_path.read_text()


def finish(proc, master, log):
    os.write(master, b'\r')
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    os.close(master)
    log.close()


def read_csv(out_dir):
    files = list(out_dir.glob('*.csv'))
    return list(csv.DictReader(open(files[0]))) if files else []


# ---------------- A. 카메라 토픽 + 네임스페이스 (벽시계) ----------------
harness.image_on = True
proc, master, log, log_path, out_dir = start_node(
    ['--camera-topic', '--namespace', NS, '--sonar-topic', 'us_sensor/range'], 'A')
loaded = wait_for(log_path, '주행 로그:', proc)
T.check('[A] 노드가 뜨고 모델 로드', loaded, str(log_path))
time.sleep(1.5)
t_start = time.monotonic()
os.write(master, b' ')
time.sleep(3.0)
harness.image_on = False
t_stall = time.monotonic()
time.sleep(1.5)
harness.image_on = True
t_resume = time.monotonic()
time.sleep(2.0)
finish(proc, master, log)
text = log_path.read_text()
cmds = list(harness.cmds)


def first_after(t, pred, limit=5.0):
    return next((tc - t for tc, v, w in cmds if t <= tc <= t + limit and pred(v, w)), None)


moving = lambda v, w: v > 0.0          # noqa: E731
zero = lambda v, w: v == 0.0 and w == 0.0   # noqa: E731
T.check('[A] 정상 종료 (exit 0)', proc.returncode == 0, f'exit {proc.returncode}')
T.check(f'[A] 토픽 이름: 입력 /{NS}/camera/image_raw, 발행 /{NS}/cmd_vel, 초음파 /{NS}/us_sensor/range',
        f'토픽 /{NS}/camera/image_raw' in text and f'발행 /{NS}/cmd_vel, 초음파 /{NS}/us_sensor/range' in text)
T.check('[A] 네임스페이스 없는 /cmd_vel 로는 안 나감', len(harness.root_cmds) == 0, f'{len(harness.root_cmds)}개')
latency = first_after(t_start, moving, 1.0)
T.check('[A] Space → 1초 안에 주행 명령', latency is not None, f'{latency}')
latency = first_after(t_stall, zero)
T.check(f'[A] 영상 끊김 → {STALL_STOP_MAX:.2f}초 안에 정지', latency is not None and latency <= STALL_STOP_MAX,
        f'{latency:.3f}s' if latency is not None else '정지 안 함')
if latency is not None:
    T.check('[A] 끊긴 동안 주행 명령 0개',
            not [1 for tc, v, w in cmds if t_stall + latency <= tc < t_resume and not zero(v, w)])
T.check('[A] 영상이 다시 오면 주행 재개 (1초 안)', first_after(t_resume, moving, 1.0) is not None)
T.check('[A] 끊김·재개 안내', '카메라 영상이' in text and '카메라 영상 다시 받음' in text)
T.check('[A] 크기 경고(800x600 → 640x480) 한 번', text.count('카메라 토픽 크기 800x600') == 1)
rows = read_csv(out_dir)
drive = [r for r in rows if r['state'] == 'DRIVE']
T.check('[A] 검출 표시 영상 받음 (640x480 bgr8, 10장 이상)', len(harness.debug) >= 10
        and set(harness.debug) == {(640, 480, 'bgr8')}, f'{len(harness.debug)}장 {set(harness.debug)}')
T.check('[A] 표시 영상 토픽 안내', f'표시 영상: /{NS}/lane_debug/image' in text)
T.check('[A] CSV 에 DRIVE 줄, 차선 검출 있음', len(drive) >= 30 and sum(int(r['valid']) for r in drive) > len(drive) // 2,
        f'DRIVE {len(drive)}줄, valid {sum(int(r["valid"]) for r in drive)}')

# ---------------- B. 시뮬 시간 (RTF 0.5) ----------------
harness.black = True
harness.clock_on = True
time.sleep(0.5)
proc, master, log, log_path, out_dir = start_node(
    ['--camera-topic', '--namespace', NS, '--sonar-topic', 'us_sensor/range', '--use-sim-time', '--dry-run'], 'B')
loaded = wait_for(log_path, '주행 로그:', proc)
T.check('[B] 노드가 뜨고 시간 원천이 시뮬 시간', loaded and '시간     : 시뮬 시간 (/clock)' in log_path.read_text())
t0 = time.monotonic()
time.sleep(4.0)
wall = time.monotonic() - t0
finish(proc, master, log)
T.check('[B] 정상 종료 (exit 0)', proc.returncode == 0, f'exit {proc.returncode}')
rows = read_csv(out_dir)
ts = [float(r['t']) for r in rows]
dts = [b - a for a, b in zip(ts, ts[1:])]
T.check('[B] CSV 시각이 시뮬 초: 줄 간격 중앙값 0.1초', dts and abs(statistics.median(dts) - 0.1) < 0.011,
        f'중앙값 {statistics.median(dts):.3f}' if dts else '줄 없음')
lost = [float(r['lost_time']) for r in rows[2:]]     # 처음 두 줄은 첫 추론이 느려 같은 0.1초 칸에 들 수 있어 뺀다
steps = [round(b - a, 6) for a, b in zip(lost, lost[1:])]
ones = sum(1 for x in steps if x == 0.1)
T.check('[B] 상실 시간이 시뮬 초로 0.1초 칸씩 늘어남 (셋째 줄부터. 모두 0.1 의 배수, 90% 이상 정확히 0.1 —\n'
        '    루프가 시뮬 시간으로 0.15초 넘게 밀리면 그만큼 칸을 더 세는 것은 설계대로)',
        steps and all(x >= 0.1 and abs(x * 10 - round(x * 10)) < 1e-6 for x in steps) and ones >= 0.9 * len(steps),
        f'0.1 칸 {ones}/{len(steps)}, 값 {sorted(set(steps))[:5]}')
rate = len(rows) / wall
T.check('[B] 제어 루프가 실제 시간으로는 약 5 Hz (RTF 0.5)', 3.5 <= rate <= 6.5, f'{rate:.1f} 줄/초 (실제 {wall:.1f}s)')

# ---------------- C. /clock 없음 ----------------
harness.clock_on = False
time.sleep(0.5)
proc, master, log, log_path, out_dir = start_node(['--camera-topic', '--namespace', NS, '--use-sim-time', '--dry-run'], 'C')
try:
    proc.wait(timeout=90)
except subprocess.TimeoutExpired:
    proc.kill()
    proc.wait()
os.close(master)
log.close()
text = log_path.read_text()
T.check('[C] /clock 없음 → 종료 코드 2, 안내', proc.returncode == 2 and '/clock 이 오지 않았습니다' in text,
        f'exit {proc.returncode}')

executor.shutdown()
spin_thread.join(timeout=5)        # spin 이 끝난 뒤에 노드를 지운다 (아니면 가끔 C++ 쪽이 abort)
harness.destroy_node()
rclpy.shutdown()
T.finish()
