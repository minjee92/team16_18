"""
[느림: ncnn 모델 + 영상 + ROS 환경, 약 30초] step2 주행 기록 (프레임별 CSV + --record 영상)

노드를 --video --record --dry-run --out-dir <임시 폴더> 로 띄우고, 가짜 초음파 → Space → 몇 초 주행 → Enter.
  - 파일: 실행 시각을 붙인 같은 이름의 .csv / .mp4
  - 개수: CSV 줄 수 = 스텝 수, 영상 프레임 수 = recorded=1 줄 수 = 녹화 프레임 수, 녹화 + 버림 = 스텝 수
  - CSV 내부 일관성 (로그만 보고 역추적할 수 있는지):
      valid 이면 target_x = 행별 cx 의 ROW_WEIGHTS 가중 평균, 아니면 target_x·src 빈칸
      src=both ⇒ lx·rx 둘 다, left ⇒ lx 만, right ⇒ rx 만
      left_det=0 ⇒ 모든 행 lx 빈칸 (right 도 같음), det ⇔ conf 있음
  - 상태: WAIT 다음에 DRIVE 가 기록됨
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'                        # 실제 로봇과 분리
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import csv           # noqa: E402
import pty           # noqa: E402
import re            # noqa: E402
import subprocess    # noqa: E402
import sys           # noqa: E402
import tempfile      # noqa: E402
import threading     # noqa: E402
import time          # noqa: E402
from pathlib import Path  # noqa: E402

import cv2           # noqa: E402

import testlib       # noqa: E402

T = testlib.Checker('step2_record')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')
testlib.require(testlib.VIDEO, '입력 영상')

import rclpy                                        # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node                         # noqa: E402
from sensor_msgs.msg import Range                   # noqa: E402

import step2_lane_follow as S2                      # noqa: E402  (상수 비교용)


class FakeSonar(Node):
    def __init__(self):
        super().__init__('step2_record_sonar')
        self.on = False
        self.pub = self.create_publisher(Range, '/us_sensor/range', 10)
        self.create_timer(0.05, self.tick)

    def tick(self):
        if self.on:
            msg = Range()
            msg.min_range, msg.max_range, msg.range = 0.02, 3.0, 0.5
            self.pub.publish(msg)


rclpy.init()
sonar = FakeSonar()
executor = SingleThreadedExecutor()
executor.add_node(sonar)
spin = threading.Thread(target=executor.spin, daemon=True)
spin.start()

out_dir = Path(tempfile.mkdtemp(prefix='step2_record_'))
log_path = out_dir / 'node.log'
log = open(log_path, 'w')
master, slave = pty.openpty()
proc = subprocess.Popen(
    [sys.executable, '-u', str(testlib.SRC_DIR / 'robot' / 'step2_lane_follow.py'),
     '--video', str(testlib.VIDEO), '--record', '--dry-run', '--out-dir', str(out_dir)],
    stdin=slave, stdout=log, stderr=subprocess.STDOUT, cwd=str(testlib.MJ_DIR))
os.close(slave)

deadline = time.monotonic() + 90
while '주행 로그:' not in log_path.read_text() and proc.poll() is None and time.monotonic() < deadline:
    time.sleep(0.5)
sonar.on = True
time.sleep(2.0)                    # WAIT 상태도 기록되게
os.write(master, b' ')             # 출발
time.sleep(6.0)
os.write(master, b'\r')            # 종료
try:
    proc.wait(timeout=60)
except subprocess.TimeoutExpired:
    proc.kill()
executor.shutdown()
spin.join(timeout=2)
sonar.destroy_node()
rclpy.shutdown()

text = log_path.read_text()
T.check('노드 정상 종료 (exit 0)', proc.returncode == 0, f'exit {proc.returncode}, log {log_path}')

csv_files = sorted(out_dir.glob('result_step2_lane_follow_*.csv'))
mp4_files = sorted(out_dir.glob('result_step2_lane_follow_*.mp4'))
T.check('CSV·영상 각 1개', len(csv_files) == 1 and len(mp4_files) == 1, f'{[p.name for p in out_dir.iterdir()]}')
if not (csv_files and mp4_files):
    T.finish()
csv_path, mp4_path = csv_files[0], mp4_files[0]
T.check('이름에 실행 시각, CSV·영상 이름 같음',
        re.fullmatch(r'result_step2_lane_follow_\d{8}_\d{6}', csv_path.stem) is not None
        and csv_path.stem == mp4_path.stem, csv_path.stem)

m_log = re.search(r'주행 로그: .*\((\d+)줄\)', text)
m_rec = re.search(r'주행 영상: .*\(녹화 (\d+)프레임, 버림 (\d+), 오류 (\d+)\)', text)
T.check('종료 시 로그 줄 수·녹화 수 출력', m_log is not None and m_rec is not None)
n_log = int(m_log[1]) if m_log else -1
written, dropped, failed = (int(x) for x in m_rec.groups()) if m_rec else (-1, -1, -1)

rows = list(csv.DictReader(open(csv_path)))
tags = [f'{round(r * 100)}' for r in S2.SAMPLE_ROWS]
T.check('CSV 줄 수 = 출력된 줄 수', len(rows) == n_log, f'{len(rows)} vs {n_log}')
T.check('step 이 0 부터 빠짐없이', [int(r['step']) for r in rows] == list(range(len(rows))))
for col in ['t', 'state', 'valid', 'lost_time', 'left_det', 'left_conf', 'right_det', 'right_conf',
            'target_x', 'error', 'steer', 'v', 'w', 'sonar_range', 'estop_reason', 'recorded'] + \
           [f'{k}_{tag}' for tag in tags for k in ('lx', 'rx', 'src', 'cx', 'hw')]:
    if col not in rows[0]:
        T.check(f'열 {col} 있음', False)
T.check('필요한 열이 모두 있음', all(c in rows[0] for c in ('target_x', f'hw_{tags[0]}', f'lx_{tags[-1]}')))

cap = cv2.VideoCapture(str(mp4_path))
frames, size = 0, None
while True:
    ok, frame = cap.read()
    if not ok:
        break
    size = frame.shape[1::-1]
    frames += 1
recorded_rows = sum(int(r['recorded']) for r in rows)
T.check('영상 프레임 수 = recorded=1 줄 수 = 녹화 수', frames == recorded_rows == written,
        f'영상 {frames}, recorded {recorded_rows}, 녹화 {written}')
T.check('녹화 + 버림 = 스텝 수, 오류 0', written + dropped == len(rows) and failed == 0,
        f'녹화 {written} + 버림 {dropped} vs {len(rows)}, 오류 {failed}')
T.check('영상 크기 = FRAME_SIZE', size == S2.FRAME_SIZE, f'{size}')

states = [r['state'] for r in rows]
first_drive = states.index('DRIVE') if 'DRIVE' in states else None
T.check('WAIT 다음 DRIVE 가 기록됨', first_drive is not None and first_drive > 0 and states[0] == 'WAIT')

bad = []
for r in rows:
    step = r['step']
    cxs = [(int(r[f'cx_{tag}']), w) for tag, w in zip(tags, S2.ROW_WEIGHTS) if r[f'cx_{tag}'] != '']
    if r['valid'] == '1':
        expected = sum(c * w for c, w in cxs) / sum(w for _, w in cxs)
        if abs(expected - float(r['target_x'])) > 0.006:              # target_x 는 소수 2자리로 기록
            bad.append((step, 'target_x', expected, r['target_x']))
    elif r['target_x'] != '' or cxs:
        bad.append((step, 'invalid row has target'))
    for tag in tags:
        src, lx, rx = r[f'src_{tag}'], r[f'lx_{tag}'], r[f'rx_{tag}']
        ok = {'both': lx != '' and rx != '', 'left': lx != '' and rx == '',
              'right': lx == '' and rx != '', '': lx == '' and rx == ''}.get(src, False)
        if not ok:
            bad.append((step, tag, src, lx, rx))
        if r['left_det'] == '0' and lx != '':
            bad.append((step, tag, 'lx without left_det'))
        if r['right_det'] == '0' and rx != '':
            bad.append((step, tag, 'rx without right_det'))
    if (r['left_det'] == '1') != (r['left_conf'] != '') or (r['right_det'] == '1') != (r['right_conf'] != ''):
        bad.append((step, 'det/conf'))
T.check('CSV 일관성 (target_x 재계산, src↔lx/rx, det↔lx/rx/conf)', not bad, f'{len(bad)}건, 예: {bad[:3]}')
T.check('valid 프레임이 충분히 있음 (검사가 실제로 돌았는지)', sum(r['valid'] == '1' for r in rows) > 20)
ages = [float(r['sonar_age']) for r in rows if r['sonar_age'] != '']
T.check('sonar_age 는 0 이상 (측정 후 지난 시간)', ages and min(ages) >= 0, f'최소 {min(ages) if ages else None}')

T.finish()
