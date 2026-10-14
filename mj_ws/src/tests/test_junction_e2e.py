"""
[느림: ncnn 모델 + ROS 환경, 약 1분] 갈림길 화살표 키를 실제 주행 노드로 (D2)

가제보 없이 가짜 발행자로:
  - 카메라: 검은 영상 (차선 없음 → 계속 상실), 초음파: 0.5 m (장애물 시험 때만 0.10 m)
  - odom: 노드가 낸 /mjj/cmd_vel 을 적분 (아주 단순한 로봇)
  0. 옵션 없이: 화살표를 눌러도 아무 일 없음, CSV 열이 지금 그대로 (갈림길 열 없음)
  1. --junction-keys: 출발 → 상실 정지 → 대기. 대기 전 화살표는 무시
     → 키 → 들어가기(0.05 m/s, 약 20 cm) → 우회전(−ω, 약 70°) → 다시 잡기 → 차선이 없어 제한 시간 → 정지·대기
     → ← 키 → 꺾는 중 장애물 → 비상 정지로 취소 → Space 재개 뒤에도 대기 (다시 키를 기다림)
  junctions.csv 의 사건 순서, 대기 사진, 주행 CSV 의 갈림길 열을 본다.

안전: ROS_DOMAIN_ID=87, LOCALHOST.
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import csv           # noqa: E402
import math          # noqa: E402
import pty           # noqa: E402
import subprocess    # noqa: E402
import sys           # noqa: E402
import tempfile      # noqa: E402
import threading     # noqa: E402
import time          # noqa: E402
from pathlib import Path  # noqa: E402

import testlib       # noqa: E402

T = testlib.Checker('junction_e2e')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')

import rclpy                                        # noqa: E402
from geometry_msgs.msg import Twist                 # noqa: E402
from nav_msgs.msg import Odometry                   # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor   # noqa: E402
from rclpy.node import Node                         # noqa: E402
from rclpy.qos import qos_profile_sensor_data       # noqa: E402
from sensor_msgs.msg import Image, Range            # noqa: E402

import mj_lane_drive.lane_follow_node as S2       # noqa: E402

NS = 'mjj'
RIGHT, LEFT, UP = b'\x1b[C', b'\x1b[D', b'\x1b[A'


class Harness(Node):
    def __init__(self):
        super().__init__('junction_e2e_harness')
        self.range = 0.5
        self.v = self.w = 0.0
        self.x = self.y = self.yaw = 0.0
        self.cmds = []
        self.last = time.monotonic()
        self.img_pub = self.create_publisher(Image, f'/{NS}/camera/image_raw', qos_profile_sensor_data)
        self.range_pub = self.create_publisher(Range, f'/{NS}/us_sensor/range', 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{NS}/odom', 10)
        group = ReentrantCallbackGroup()                 # 영상이 초음파를 늦추지 않게 따로 돈다
        self.black = bytes(640 * 480 * 3)
        self.create_subscription(Twist, f'/{NS}/cmd_vel', self.on_cmd, 50, callback_group=group)
        self.create_timer(0.05, self.tick, callback_group=group)          # odom·초음파 20 Hz
        self.create_timer(0.1, self.tick_image, callback_group=group)     # 영상 10 Hz

    def tick_image(self):
        img = Image()
        img.width, img.height, img.encoding, img.step = 640, 480, 'rgb8', 640 * 3
        img.data = self.black
        self.img_pub.publish(img)

    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z
        self.cmds.append((time.monotonic(), m.linear.x, m.angular.z))

    def tick(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        odom = Odometry()
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.odom_pub.publish(odom)
        r = Range()
        r.min_range, r.max_range, r.range = 0.02, 3.0, self.range
        self.range_pub.publish(r)


rclpy.init()
harness = Harness()
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(harness)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()


def start_node(extra):
    out_dir = Path(tempfile.mkdtemp(prefix='junction_e2e_'))
    log_path = out_dir / 'node.log'
    log = open(log_path, 'w')
    master, slave = pty.openpty()
    proc = subprocess.Popen([sys.executable, '-u', str(testlib.SRC_DIR / 'robot' / 'step2_lane_follow.py'),
                             '--camera-topic', '--namespace', NS, '--sonar-topic', 'us_sensor/range',
                             '--out-dir', str(out_dir)] + extra,
                            stdin=slave, stdout=log, stderr=subprocess.STDOUT, cwd=str(testlib.MJ_DIR))
    os.close(slave)
    deadline = time.monotonic() + 90
    while '주행 로그:' not in log_path.read_text() and proc.poll() is None and time.monotonic() < deadline:
        time.sleep(0.2)
    time.sleep(1.5)
    return proc, master, log, log_path, out_dir


def finish(proc, master, log):
    os.write(master, b'\r')
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    os.close(master)
    log.close()


def wait_text(log_path, text, limit):
    deadline = time.monotonic() + limit
    while text not in log_path.read_text() and time.monotonic() < deadline:
        time.sleep(0.05)
    return text in log_path.read_text()


# ---------------- 0. 옵션 없음 ----------------
proc, master, log, log_path, out_dir = start_node([])
os.write(master, b' ')
time.sleep(0.5)
for k in (RIGHT, LEFT, UP):
    os.write(master, k)
    time.sleep(0.2)
finish(proc, master, log)
text = log_path.read_text()
header = next(csv.reader(open(next(out_dir.glob('result_*[0-9].csv')))))
T.check('[0] 옵션 없음: 정상 종료, 화살표에 반응 없음, 갈림길 안내 없음',
        proc.returncode == 0 and '갈림길' not in text, f'exit {proc.returncode}')
T.check('[0] 옵션 없음: CSV 열이 지금 그대로 (갈림길 열 없음, 마지막 열 recorded)',
        'junction_state' not in header and header[-1] == 'recorded', f'{len(header)}열')
T.check('[0] 옵션 없음: 갈림길 CSV 없음', not list(out_dir.glob('*_junctions.csv')))

# ---------------- 1. --junction-keys ----------------
harness.cmds.clear()
proc, master, log, log_path, out_dir = start_node(['--junction-keys'])
T.check('[1] 갈림길 안내 줄 (odom 토픽)', f'odom /{NS}/odom' in log_path.read_text())
os.write(master, RIGHT)                                       # 대기 전 → 무시
time.sleep(0.3)
os.write(master, b' ')                                        # 출발 → 검은 영상이라 상실 → 2.1초 뒤 정지 → 대기
T.check('[1] 상실 정지 → 갈림길 대기', wait_text(log_path, '갈림길 대기 (LANE LOST STOP)', 6))
T.check('[1] 대기 전 화살표는 무시하고 알림', '갈림길 대기 중이 아니라 무시' in log_path.read_text())
time.sleep(0.5)
t_key = time.monotonic()
x0, y0, yaw0 = harness.x, harness.y, harness.yaw
os.write(master, RIGHT)
T.check('[1] → 키 받음', wait_text(log_path, '갈림길 키 right: right 시작', 2))
T.check('[1] 제한 시간(8초) 넘으면 정지·다시 대기', wait_text(log_path, '갈림길 right: timeout', 11))
moves = [(t, v, w) for t, v, w in harness.cmds if t >= t_key]
enter = [c for c in moves if c[1] > 0 and c[2] == 0.0]
turn = [c for c in moves if c[1] == 0.0 and c[2] < 0]
T.check('[1] 들어가기 0.05 m/s → 제자리 우회전 −0.6 rad/s 순서', enter and turn and enter[0][0] < turn[0][0]
        and abs(enter[0][1] - S2.JUNCTION_SPEED) < 1e-9 and abs(turn[0][2] + S2.JUNCTION_TURN_RATE) < 1e-9,
        f'들어가기 {len(enter)}, 돌기 {len(turn)}')
T.check('[1] 돈 각도 약 70°', 65 <= -math.degrees(harness.yaw - yaw0) <= 90, f'{-math.degrees(harness.yaw - yaw0):.0f}°')
time.sleep(0.5)
T.check('[1] 시간 초과 뒤 0 명령', harness.cmds[-1][1:] == (0.0, 0.0), f'{harness.cmds[-1]}')
os.write(master, LEFT)
T.check('[1] ← 키 받음', wait_text(log_path, '갈림길 키 left: left 시작', 2))
time.sleep(0.8)                                               # 들어가는 중
harness.range = 0.10
T.check('[1] 꺾는 중 장애물 → 비상 정지로 꺾기 취소', wait_text(log_path, '갈림길 left: aborted: ESTOP OBSTACLE', 2))
t_estop = time.monotonic()
time.sleep(0.5)
os.write(master, RIGHT)                                       # 비상 정지 중 키 → 거부
T.check('[1] 비상 정지 중 화살표는 거부 (먼저 Space)', wait_text(log_path, '비상 정지 중(OBSTACLE) — 먼저 Space 로 재개', 2))
harness.range = 0.5
time.sleep(0.3)
scenario_log = log_path.read_text()                           # 여기까지가 판정에 쓰는 구간
os.write(master, b' ')                                        # 재개해도 다시 키를 기다림
time.sleep(1.5)
T.check('[1] Space 재개 뒤에도 대기 (주행 명령 0)', all(v == 0.0 and w == 0.0 for t, v, w in harness.cmds if t > t_estop + 0.3),
        f'{[c for c in harness.cmds if c[0] > t_estop + 0.3 and (c[1] or c[2])][:3]}')
finish(proc, master, log)
T.check('[1] 정상 종료', proc.returncode == 0, f'exit {proc.returncode}, log {log_path}')

jfile = next(out_dir.glob('*_junctions.csv'), None)
rows = list(csv.DictReader(open(jfile))) if jfile else []
seq = [(r['kind'], r['direction'] or r['wait_reason'] or r['result']) for r in rows]
expected = [('wait', 'LANE LOST STOP'), ('key', 'right'), ('result', 'right'), ('wait', 'TIMEOUT'),
            ('key', 'left'), ('result', 'left'), ('wait', 'ESTOP OBSTACLE')]
T.check('[1] 갈림길 CSV 사건 순서 (대기→키→시간 초과→대기→키→비상 정지 취소→대기)', seq == expected, f'{seq}')
T.check('[1] 결과 열: timeout / aborted', [r['result'] for r in rows if r['kind'] == 'result'] ==
        ['timeout', 'aborted: ESTOP OBSTACLE'])
T.check('[1] 시나리오 동안 초음파가 끊기지 않음 (시험 장치 확인. 마지막 재개 뒤는 판정에 안 써서 제외)',
        'SONAR TIMEOUT' not in scenario_log)
T.check('[1] 키 줄에 odom 위치·기다린 시간·검출 상태', rows and rows[1]['odom_x'] != '' and float(rows[1]['wait_sec']) > 0
        and rows[1]['left_lane_det'] == '0' and 'cross_lane_conf' in rows[1])
images = rows[0]['images'].split() if rows else []
T.check('[1] 첫 대기의 사진 2장 (원본·검출 표시)', len(images) == 2 and all((out_dir / p).exists() for p in images), f'{images}')
main = list(csv.DictReader(open(next(out_dir.glob('result_*[0-9].csv')))))
states = {r['junction_state'] for r in main}
T.check('[1] 주행 CSV 에 갈림길 열 (FOLLOW·WAIT·ENTER·TURN·REACQUIRE 가 나옴)',
        {'FOLLOW', 'WAIT', 'ENTER', 'TURN', 'REACQUIRE'} <= states, f'{states}')

# ---------------- 2. 선택 기능 켬: --speed-scaled-stop (B) --spin-near-wall (D), ↓ 수동 정지 ----------------
DOWN = b'\x1b[B'


def wait_cmd(pred, since, limit):
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        if any(pred(v, w) for t, v, w in list(harness.cmds) if t >= since):
            return True
        time.sleep(0.05)
    return False


harness.range = 0.5
harness.cmds.clear()
proc, master, log, log_path, out_dir = start_node(['--junction-keys', '--speed-scaled-stop', '--spin-near-wall'])
T.check('[2] 선택 기능 안내 줄', '속도별 정지 거리' in log_path.read_text() and f'갈림길 회전은 {S2.SPIN_MIN_DISTANCE} m 까지 허용' in log_path.read_text())
os.write(master, b' ')
T.check('[2] 상실 정지 → 대기', wait_text(log_path, '갈림길 대기 (LANE LOST STOP)', 6))
os.write(master, RIGHT)
t_key = time.monotonic()
T.check('[2] 회전 단계 시작', wait_cmd(lambda v, w: v == 0.0 and w < 0, t_key, 8))
harness.range = 0.12                                          # D: 회전 중 앞 0.12 m → 회전은 계속
t_near = time.monotonic()
time.sleep(0.6)
T.check('[2] D: 회전 중 0.12 m 장애물이면 비상 정지 없이 회전 계속',
        'OBSTACLE' not in log_path.read_text() and wait_cmd(lambda v, w: v == 0.0 and w < 0, t_near + 0.3, 0.1))
harness.range = 0.04                                          # SPIN_MIN_DISTANCE(0.07 m)보다 가까움 → 회전도 막음
T.check(f'[2] D: 0.04 m (< {S2.SPIN_MIN_DISTANCE} m) 면 회전도 막고 비상 정지 → 꺾기 취소', wait_text(log_path, 'aborted: ESTOP OBSTACLE', 2))
t_stop = time.monotonic()
time.sleep(0.5)
T.check('[2] D: 정지 뒤 명령 0', all(v == 0.0 and w == 0.0 for t, v, w in harness.cmds if t > t_stop + 0.2))
harness.range = 0.5
time.sleep(0.3)
os.write(master, b' ')                                        # 재개 → 대기
time.sleep(0.5)
os.write(master, UP)                                          # 직진 = 다시 잡기 0.05 m/s
t_up = time.monotonic()
T.check('[2] ↑ 직진: 0.05 m/s 로 전진', wait_cmd(lambda v, w: abs(v - S2.JUNCTION_SPEED) < 1e-9, t_up, 2))
harness.range = 0.12                                          # B: 0.05 m/s 면 정지 거리 0.105 m → 0.12 m 는 통과
t_b = time.monotonic()
time.sleep(0.8)
T.check('[2] B: 0.05 m/s 일 때 0.12 m 는 멈추지 않음 (기본이면 0.15 m 라 멈춤)',
        'OBSTACLE' not in log_path.read_text()[log_path.read_text().find('갈림길 키 up'):]
        and wait_cmd(lambda v, w: v > 0, t_b + 0.4, 0.1))
harness.range = 0.09                                          # 진짜 장애물이 바닥값(0.10 m)보다 가까움 → 정지
T.check('[2] B: 0.09 m 장애물이면 느려도 비상 정지', wait_text(log_path, '비상 정지: OBSTACLE (range 0.090', 2))
t_real = time.monotonic()
time.sleep(0.5)
os.write(master, b' ')
T.check('[2] 장애물이 남아 있으면 재개 거부 (수동 재개 유지)', wait_text(log_path, '출발 불가: OBSTACLE', 2)
        and all(v == 0.0 for t, v, w in harness.cmds if t > t_real + 0.2))
harness.range = 0.5
time.sleep(0.3)
os.write(master, b' ')
time.sleep(0.5)
os.write(master, UP)
time.sleep(1.0)
os.write(master, DOWN)                                        # ↓ 수동 정지
T.check('[2] ↓ 수동 정지 → 대기 (MANUAL), 꺾기 취소', wait_text(log_path, '갈림길 대기 (MANUAL)', 2)
        and '갈림길 straight: aborted: MANUAL' in log_path.read_text())
t_pause = time.monotonic()
time.sleep(0.6)
T.check('[2] ↓ 뒤 명령 0', all(v == 0.0 and w == 0.0 for t, v, w in harness.cmds if t > t_pause + 0.3))
finish(proc, master, log)
T.check('[2] 정상 종료', proc.returncode == 0, f'exit {proc.returncode}, log {log_path}')
rows = list(csv.DictReader(open(next(out_dir.glob('*_junctions.csv')))))
manual = [r for r in rows if r['kind'] == 'wait' and r['wait_reason'] == 'MANUAL']
T.check('[2] 갈림길 CSV 에 수동 대기와 사진', manual and len(manual[0]['images'].split()) == 2)

executor.shutdown()
spin_thread.join(timeout=5)
harness.destroy_node()
rclpy.shutdown()
T.finish()
