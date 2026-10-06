"""
[느림: ncnn 모델 + 영상 + ROS 환경, 약 1분] step2 강제 종료 후에도 녹화 영상이 열리는지

노드를 --record 로 띄우고 (dry-run 아님: cmd_vel 을 실제로 publish 하는 경로, 격리 도메인) 몇 초 뒤 끝낸다.

  ESC 키 / SIGINT / SIGTERM / SIGHUP(SSH 끊김) / SIGINT 두 번(정리 중 두 번째 신호) / 루프 안 예외

이전에는 SIGINT·SIGTERM 에서 rclpy 가 context 를 먼저 내려 정지 명령 publish 가 실패하고, SIGHUP 은 즉시 죽어서
VideoWriter 가 release 되지 않아 mp4 가 "moov atom not found" 로 열리지 않았다.

확인: mp4 가 열리고 프레임 수 = "영상 저장 완료 (N프레임)", CSV 줄 수 = 출력값, "종료 완료",
      "Exception in thread"·"정리 실패" 없음, 종료 코드 (예외만 1, 나머지 0)
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'                        # 실제 로봇과 분리 (cmd_vel 을 publish 함)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import csv           # noqa: E402
import pty           # noqa: E402
import re            # noqa: E402
import signal        # noqa: E402
import subprocess    # noqa: E402
import sys           # noqa: E402
import tempfile      # noqa: E402
import time          # noqa: E402
from pathlib import Path  # noqa: E402

import cv2           # noqa: E402

import testlib       # noqa: E402

T = testlib.Checker('step2_shutdown')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')
testlib.require(testlib.VIDEO, '입력 영상')

NODE = str(testlib.SRC_DIR / 'robot' / 'step2_lane_follow.py')
INJECT = (                                                   # 30번째 추론 결과 처리에서 예외
    'import sys; sys.argv = sys.argv[1:]; '
    f'sys.path.insert(0, {str(testlib.SRC_DIR / "robot")!r}); '
    'import step2_lane_follow as S2; orig = S2.build_masks; calls = [0]\n'
    'def boom(*a, **k):\n'
    '    calls[0] += 1\n'
    '    if calls[0] == 30:\n'
    '        raise RuntimeError("주입한 예외 (테스트)")\n'
    '    return orig(*a, **k)\n'
    'S2.build_masks = boom; sys.exit(S2.main())'
)


def run(name, action, inject=False):
    out_dir = Path(tempfile.mkdtemp(prefix=f'step2_shutdown_{name}_'))
    log_path = out_dir / 'node.log'
    args = ['--video', str(testlib.VIDEO), '--record', '--out-dir', str(out_dir)]
    cmd = [sys.executable, '-u', '-c', INJECT, NODE] + args if inject else [sys.executable, '-u', NODE] + args
    master, slave = pty.openpty()
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(cmd, stdin=slave, stdout=log, stderr=subprocess.STDOUT, cwd=str(testlib.MJ_DIR))
    os.close(slave)

    deadline = time.monotonic() + 90                  # 시작 배너(파일 경로 출력)까지 기다림
    while '주행 로그:' not in log_path.read_text() and proc.poll() is None and time.monotonic() < deadline:
        time.sleep(0.3)
    time.sleep(3.0)
    action(proc, master)
    try:
        code = proc.wait(timeout=60)
    except subprocess.TimeoutExpired:
        proc.kill()
        code = 'timeout'
    os.close(master)
    return code, log_path.read_text(), out_dir


def signal_once(sig):
    return lambda proc, master: proc.send_signal(sig)


def signal_twice(gap):
    def act(proc, master):
        proc.send_signal(signal.SIGINT)
        time.sleep(gap)
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
    return act


cases = [
    ('esc', lambda proc, master: os.write(master, b'\x1b'), False, 0),
    ('sigint', signal_once(signal.SIGINT), False, 0),
    ('sigterm', signal_once(signal.SIGTERM), False, 0),
    ('sighup', signal_once(signal.SIGHUP), False, 0),
    ('sigint_x2_fast', signal_twice(0.05), False, 0),
    ('sigint_x2_late', signal_twice(0.3), False, 0),
    ('exception', lambda proc, master: None, True, 1),      # 예외가 나서 스스로 끝남
]

for name, action, inject, expected_code in cases:
    code, text, out_dir = run(name, action, inject)
    mp4 = sorted(out_dir.glob('result_step2_lane_follow_*.mp4'))
    csv_files = sorted(out_dir.glob('result_step2_lane_follow_*.csv'))
    frames, opened = 0, False
    if mp4:
        cap = cv2.VideoCapture(str(mp4[0]))
        opened = cap.isOpened()
        while cap.read()[0]:
            frames += 1
    m_video = re.search(r'영상 저장 완료: .*\((\d+)프레임\)', text)
    m_log = re.search(r'주행 로그: .*\((\d+)줄\)', text)
    rows = len(list(csv.DictReader(open(csv_files[0])))) if csv_files else -1

    T.check(f'[{name}] 종료 코드 {expected_code}', code == expected_code, f'exit {code}, log {out_dir}/node.log')
    T.check(f'[{name}] mp4 가 열리고 프레임 수 = 출력값',
            opened and m_video is not None and frames == int(m_video[1]) > 0,
            f'opened {opened}, frames {frames}, 출력 {m_video[1] if m_video else None}')
    T.check(f'[{name}] CSV 줄 수 = 출력값', m_log is not None and rows == int(m_log[1]) > 0,
            f'{rows} vs {m_log[1] if m_log else None}')
    T.check(f'[{name}] 정리 완료, 스레드 예외·정리 실패 없음',
            '종료 완료' in text and 'Exception in thread' not in text and '정리 실패' not in text)
    if name == 'exception':
        T.check('[exception] 주입한 예외 traceback 출력, 종료 이유 "예외"',
                '주입한 예외' in text and '종료: 예외' in text)

T.finish()
