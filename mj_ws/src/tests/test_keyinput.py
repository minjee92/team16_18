"""
[빠름: ROS 환경 필요 (주행 노드 모듈 import), 모델 불필요] 터미널 키 입력 (KeyInput, D2)

pty(가짜 터미널)로 실제 키 바이트를 보낸다. SSH 로 접속한 터미널도 같은 바이트를 보낸다:
  Space → start, Enter → quit, ESC 한 번 → quit
  ← ESC [ D, → ESC [ C, ↑ ESC [ A (보통 모드) / ESC O D·C·A (응용 커서 모드 터미널)
  화살표를 빠르게 두 번 → 둘 다
  터미널이 아니면(표준 입력이 파이프) 키를 받지 않음 → `ssh 로봇 명령` 처럼 tty 없이 띄우면 안 되는 이유
"""

import os
import pty
import subprocess
import sys
import time

import testlib

T = testlib.Checker('keyinput')

CHILD = r'''
import sys, time
sys.path[:0] = [{core!r}, {drive!r}]
from mj_lane_drive.lane_follow_node import KeyInput
with KeyInput() as keys:
    print('enabled', keys.enabled, flush=True)
    end = time.monotonic() + {secs}
    while time.monotonic() < end:
        for k in keys.read():
            print('key', k, flush=True)
        time.sleep(0.01)
'''.format(core=str(testlib.SRC_DIR / 'mj_lane_core'), drive=str(testlib.SRC_DIR / 'mj_lane_drive'), secs=4.0)


def run_pty(chunks):
    master, slave = pty.openpty()
    proc = subprocess.Popen([sys.executable, '-c', CHILD], stdin=slave, stdout=slave, stderr=subprocess.STDOUT)
    os.close(slave)
    out = b''
    deadline = time.monotonic() + 3
    while b'enabled' not in out and time.monotonic() < deadline:
        out += os.read(master, 4096)
    for data, pause in chunks:
        os.write(master, data)
        time.sleep(pause)
    proc.wait(timeout=10)
    try:
        while True:
            more = os.read(master, 4096)
            if not more:
                break
            out += more
    except OSError:
        pass
    os.close(master)
    text = out.decode(errors='ignore')
    return text, [line.split()[1] for line in text.splitlines() if line.startswith('key ')]


text, keys = run_pty([(b' ', 0.2), (b'\x1b[D', 0.2), (b'\x1b[C', 0.2), (b'\x1b[A', 0.2),
                      (b'\x1bOD', 0.2), (b'\x1bOC', 0.2), (b'\x1b[C\x1b[C', 0.2), (b'\x1b[B', 0.2),
                      (b'\x1b', 0.3), (b'\r', 0.2)])
T.check('터미널이면 키를 받음', 'enabled True' in text, text.splitlines()[0] if text else '')
expected = ['start', 'left', 'right', 'up', 'left', 'right', 'right', 'right', 'down', 'quit', 'quit']
T.check('Space·화살표(두 형식)·빠른 두 번·ESC·Enter 순서대로', keys == expected, f'{keys}')

proc = subprocess.run([sys.executable, '-c', CHILD.replace('{secs}', '0.5').replace('end = time.monotonic() + 4.0', 'end = time.monotonic() + 0.5')],
                      input=b' \x1b[D\r', capture_output=True, timeout=10)
T.check('터미널이 아니면 키를 받지 않음 (ssh 를 -t 없이 쓴 경우)', b'enabled False' in proc.stdout
        and b'key ' not in proc.stdout, proc.stdout.decode(errors='ignore').strip())

T.finish()
