#!/usr/bin/env python3
"""
시뮬 상자 시험: "진짜 장애물 앞에서는 여전히 멈추고, 장애물이 있는 동안 재개를 거부하고, 치우면 재개한다".
world·robot 이 떠 있어야 한다 (run_mj_sim.sh world / robot, 로봇은 기본 출발점 1.80 0.10 서쪽).
주행 노드는 시뮬 설정 그대로 (run_mj_sim.sh drive: --speed-scaled-stop --spin-near-wall 켬) 를 --record 로 띄운다.

  1. 로봇 앞 (기본 x 1.40, y 0.10) 에 상자를 놓는다. 시뮬 초음파는 라이다 높이(약 12.5 cm)라 상자 높이 20 cm
  2. Space 출발 → 상자 앞에서 비상 정지 (OBSTACLE) — 그때 초음파 거리를 기록
  3. 2초 동안 주행 명령 0, Space → "출발 불가: OBSTACLE"
  4. 상자를 치움 → Space → 다시 전진
사용: bash run_mj_sim.sh obstacle-test [x y]   (시스템 python3, 결과 PASS/FAIL 과 종료 코드)
"""
import os
import pty
import re
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

RUN, NS = sys.argv[1], sys.argv[2]
BOX_X = float(sys.argv[3]) if len(sys.argv) > 3 else 1.40
BOX_Y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.10
BOX = 'mj_test_box'
SDF = ("<sdf version='1.9'><model name='%s'><static>true</static><link name='l'>"
       "<collision name='c'><geometry><box><size>0.06 0.10 0.20</size></box></geometry></collision>"
       "<visual name='v'><geometry><box><size>0.06 0.10 0.20</size></box></geometry>"
       "<material><diffuse>0.9 0.2 0.2 1</diffuse></material></visual></link></model></sdf>") % BOX
HERE = os.path.dirname(os.path.realpath(__file__))
results = []


def check(label, ok, detail=''):
    results.append(ok)
    print(f'{"PASS" if ok else "FAIL"}  {label}{f"  ({detail})" if detail else ""}', flush=True)


def gz_env():
    env = dict(os.environ, GZ_PARTITION='mj_sim', ROS_DOMAIN_ID='95', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    env.pop('ROS_LOCALHOST_ONLY', None)
    return env


class Watch(Node):
    def __init__(self):
        super().__init__('mj_obstacle_test')
        self.cmds = []
        self.create_subscription(Twist, f'/{NS}/cmd_vel', lambda m: self.cmds.append((time.monotonic(), m.linear.x, m.angular.z)), 50)


subprocess.run(['bash', os.path.join(HERE, 'gz_remove_model.sh'), 'fms_course', BOX], env=gz_env(), capture_output=True)
spawn = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'fms_course', '-name', BOX, '-string', SDF,
                        '-x', str(BOX_X), '-y', str(BOX_Y), '-z', '0.10'], env=gz_env(), capture_output=True, text=True, timeout=60)
check('상자 놓기', 'successful' in (spawn.stdout + spawn.stderr).lower(), f'({BOX_X}, {BOX_Y})')

os.environ.update(ROS_DOMAIN_ID='95', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
os.environ.pop('ROS_LOCALHOST_ONLY', None)
rclpy.init()
watch = Watch()
executor = SingleThreadedExecutor()
executor.add_node(watch)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

out_dir = tempfile.mkdtemp(prefix='mj_obstacle_test_')
log_path = os.path.join(out_dir, 'drive.log')
master, slave = pty.openpty()
with open(log_path, 'w') as f:
    proc = subprocess.Popen(['bash', RUN, 'drive', '--record', '--out-dir', out_dir], stdin=slave, stdout=f,
                            stderr=subprocess.STDOUT, env=dict(gz_env(), PYTHONUNBUFFERED='1'), start_new_session=True)
os.close(slave)


def text():
    return open(log_path).read()


def wait_text(pattern, limit):
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        m = re.search(pattern, text())
        if m:
            return m
        time.sleep(0.1)
    return None


try:
    check('주행 노드 준비 (시뮬 설정: 속도별 정지 거리 켬)', wait_text('주행 로그:', 120) is not None and '속도별 정지 거리' in text())
    time.sleep(2.0)
    os.write(master, b' ')
    check('출발', wait_text('\n출발', 5) is not None)
    stop = wait_text(r'비상 정지: OBSTACLE \(range ([0-9.]+) m', 40)
    check('상자 앞에서 비상 정지 (OBSTACLE)', stop is not None, f'정지 순간 초음파 {stop.group(1)} m' if stop else '정지 안 함')
    t_stop = time.monotonic()
    time.sleep(2.0)
    moving = [c for c in watch.cmds if c[0] > t_stop + 0.2 and (c[1] != 0.0 or c[2] != 0.0)]
    check('정지 뒤 2초 동안 주행 명령 0', not moving, f'{moving[:2]}')
    os.write(master, b' ')
    check('상자가 있는 동안 Space → 재개 거부', wait_text('출발 불가: OBSTACLE', 3) is not None)
    subprocess.run(['bash', os.path.join(HERE, 'gz_remove_model.sh'), 'fms_course', BOX], env=gz_env(), capture_output=True)
    time.sleep(1.5)
    t_go = time.monotonic()
    os.write(master, b' ')
    time.sleep(2.0)
    check('상자를 치우면 Space 로 재개 → 다시 전진', any(c[1] > 0 for c in watch.cmds if c[0] > t_go))
finally:
    os.write(master, b'\r')
    try:
        proc.wait(timeout=40)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, 9)
    os.close(master)
    subprocess.run(['bash', os.path.join(HERE, 'gz_remove_model.sh'), 'fms_course', BOX], env=gz_env(), capture_output=True)
    executor.shutdown()                  # 정리 순서: spin 을 멈추고 스레드가 끝난 뒤에 노드·rclpy 를 내린다
    spin_thread.join(timeout=5)          #   (순서가 바뀌면 끝에 'terminate called … Aborted' 가 날 수 있다)
    watch.destroy_node()
    rclpy.shutdown()
print(f'\n[obstacle-test] {sum(results)}/{len(results)} passed  (기록: {out_dir})')
sys.exit(0 if all(results) else 1)
