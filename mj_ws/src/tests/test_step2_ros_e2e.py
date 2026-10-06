"""
[느림: ncnn 모델 + 영상 + ROS 환경, 약 1분] step2_lane_follow.py ROS 연동 테스트

실제 노드를 --video 로 띄우고, 가짜 초음파(/us_sensor/range, 20 Hz)를 발행하면서
cmd_vel 을 기록하고 키(Space / Enter)는 pty 로 보낸다.

안전: 이 스크립트는 ROS_DOMAIN_ID=87, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 로 고정해
같은 네트워크의 실제 로봇에 cmd_vel 이 가지 않게 한다 (자식 노드도 이 환경을 물려받음).
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import pty           # noqa: E402
import subprocess    # noqa: E402
import sys           # noqa: E402
import tempfile      # noqa: E402
import threading     # noqa: E402
import time          # noqa: E402

import testlib       # noqa: E402

T = testlib.Checker('step2_ros_e2e')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')
testlib.require(testlib.VIDEO, '입력 영상')

import rclpy                                       # noqa: E402
from geometry_msgs.msg import Twist                # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node                        # noqa: E402
from sensor_msgs.msg import Range                  # noqa: E402

STOP_LATENCY_MAX = 0.10       # 장애물·무효값 → 0 명령까지 (센서 주기 0.05초 + 여유)
TIMEOUT_LATENCY_MAX = 0.55    # 끊김 → 0 명령까지 (SONAR_TIMEOUT 0.3초 + 센서 주기 + 제어 루프 0.1초 + 여유)


class Harness(Node):
    def __init__(self):
        super().__init__('step2_e2e_harness')
        self.range = 0.5
        self.sonar_on = False
        self.cmds = []                                   # (수신 시각, v, w)
        self.pub = self.create_publisher(Range, '/us_sensor/range', 10)
        self.create_subscription(Twist, 'cmd_vel',
                                 lambda m: self.cmds.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
        self.create_timer(0.05, self.tick)               # 실제 드라이버와 같은 20 Hz

    def tick(self):
        if self.sonar_on:
            msg = Range()
            msg.radiation_type = Range.ULTRASOUND
            msg.min_range, msg.max_range, msg.range = 0.02, 3.0, float(self.range)
            self.pub.publish(msg)


rclpy.init()
harness = Harness()
executor = SingleThreadedExecutor()
executor.add_node(harness)
spin = threading.Thread(target=executor.spin, daemon=True)
spin.start()

log_path = tempfile.NamedTemporaryFile(prefix='step2_e2e_', suffix='.log', delete=False).name
log = open(log_path, 'w')
master, slave = pty.openpty()
proc = subprocess.Popen(
    [sys.executable, '-u', str(testlib.SRC_DIR / 'robot' / 'step2_lane_follow.py'), '--video', str(testlib.VIDEO),
     '--out-dir', tempfile.mkdtemp(prefix='step2_e2e_out_')],   # 주행 로그 CSV 가 outputs/ 에 쌓이지 않게
    stdin=slave, stdout=log, stderr=subprocess.STDOUT, cwd=str(testlib.MJ_DIR))
os.close(slave)

events = {}


def key(char, label):
    events[label] = time.monotonic()
    os.write(master, char.encode())
    time.sleep(0.4)


def mark(label):
    events[label] = time.monotonic()


def node_log():
    log.flush()
    return open(log_path).read()


# 모델 로드(첫 추론)가 끝날 때까지 기다림
deadline = time.monotonic() + 90
while 'for NCNN inference' not in node_log() and proc.poll() is None and time.monotonic() < deadline:
    time.sleep(0.5)
time.sleep(1.5)

key(' ', 'space_no_sonar')                     # 초음파 없음 → 거부
harness.sonar_on = True
time.sleep(1.0)
key(' ', 'start')                              # 출발
time.sleep(3.0)
harness.range = 0.12
mark('obstacle')                               # 장애물 → 즉시 0
time.sleep(1.0)
harness.range = 0.50
time.sleep(1.5)                                # 치워져도 latch
harness.range = 0.12
time.sleep(0.3)
key(' ', 'space_obstacle')                     # 장애물 중 → 거부
harness.range = 0.50
time.sleep(0.5)
key(' ', 'resume1')
time.sleep(2.0)
harness.range = -0.03
mark('invalid')                                # 무효값 → 즉시 0
time.sleep(1.0)
harness.range = 0.50
time.sleep(0.3)
key(' ', 'resume2')
time.sleep(2.0)
harness.sonar_on = False
mark('sonar_off')                              # 끊김 → timeout 뒤 0
time.sleep(1.5)
key(' ', 'space_timeout')                      # 끊긴 상태 → 거부
harness.sonar_on = True
time.sleep(0.5)
key(' ', 'resume3')
time.sleep(1.5)
key('\r', 'quit')
try:
    proc.wait(timeout=20)
except subprocess.TimeoutExpired:
    proc.kill()
time.sleep(0.5)

cmds = list(harness.cmds)
text = node_log()


def first_after(t, pred, limit=5.0):
    for tc, v, w in cmds:
        if t <= tc <= t + limit and pred(v, w):
            return tc - t
    return None


def within(latency, limit):
    return latency is not None and latency <= limit


def is_zero(v, w):
    return v == 0.0 and w == 0.0


def is_moving(v, w):
    return v > 0.0


def nonzero_between(t0, t1):
    return sum(1 for tc, v, w in cmds if t0 <= tc < t1 and not is_zero(v, w))


T.check('노드 정상 종료 (exit 0)', proc.returncode == 0, f'exit {proc.returncode}, log {log_path}')
T.check('cmd_vel 수신', len(cmds) > 50, f'{len(cmds)}개')
T.check('출발 전에는 주행 명령 없음', nonzero_between(0, events['start']) == 0)
T.check('초음파 없이 Space → 출발 거부', '출발 불가: NO SONAR' in text)
T.check('Space → 출발 (0.5초 안에 주행 명령)', within(first_after(events['start'], is_moving), 0.5))

for name, nxt in (('obstacle', 'resume1'), ('invalid', 'resume2')):
    latency = first_after(events[name], is_zero)
    T.check(f'{name} → {STOP_LATENCY_MAX}초 안에 정지', latency is not None and latency <= STOP_LATENCY_MAX,
            f'{latency:.3f}s' if latency is not None else '정지 안 함')
    if latency is not None:
        T.check(f'{name} 정지 후 재개 전까지 주행 명령 0개',
                nonzero_between(events[name] + latency, events[nxt]) == 0)
    T.check(f'{nxt} → 다시 주행', within(first_after(events[nxt], is_moving), 0.5))

T.check('장애물 중 Space → 출발 거부', '출발 불가: OBSTACLE' in text)
latency = first_after(events['sonar_off'], is_zero)
T.check(f'초음파 끊김 → {TIMEOUT_LATENCY_MAX}초 안에 정지', latency is not None and latency <= TIMEOUT_LATENCY_MAX,
        f'{latency:.3f}s' if latency is not None else '정지 안 함')
if latency is not None:
    T.check('끊김 정지 후 재개 전까지 주행 명령 0개', nonzero_between(events['sonar_off'] + latency, events['resume3']) == 0)
T.check('끊긴 상태에서 Space → 출발 거부', '출발 불가: SONAR TIMEOUT' in text)
T.check('resume3 → 다시 주행', within(first_after(events['resume3'], is_moving), 0.5))
after_quit = [c for c in cmds if c[0] >= events['quit']]
T.check('종료 후 마지막 명령은 0', bool(after_quit) and is_zero(*after_quit[-1][1:]))

executor.shutdown()
spin.join(timeout=2)
harness.destroy_node()
rclpy.shutdown()
T.finish()
