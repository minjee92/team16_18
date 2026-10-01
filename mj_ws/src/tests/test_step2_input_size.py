"""
[느림: ncnn 모델 + ROS 환경, 약 20초] step2 시작 시 모델 입력 크기 검사

ncnn 은 export 크기(320)에 고정된다. INFER_SIZE 가 이와 다르면 두 번째 프레임부터 검출이 0 이 되므로
step2_lane_follow.py 는 시작할 때 이를 잡아 메시지를 내고 exit 2 로 끝나야 한다.

  1) verify_input_size(): ncnn + 320 → 통과, ncnn + 640 → 실패 (메시지에 고정 크기·INFER_SIZE 표시),
     .pt 는 크기 고정이 아니므로 320 / 640 모두 통과
  2) 실제 노드를 INFER_SIZE = 640 으로 바꿔 띄우면 주행 루프에 들어가지 않고 exit 2
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'                        # 실제 로봇과 분리 (test_step2_ros_e2e.py 와 같음)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import subprocess   # noqa: E402
import sys          # noqa: E402

import testlib      # noqa: E402

T = testlib.Checker('step2_input_size')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')
testlib.require(testlib.VIDEO, '입력 영상')
PT_MODEL = testlib.MJ_DIR / 'models' / '260928_yolon_best.pt'

import step2_lane_follow as S2      # noqa: E402
from ultralytics import YOLO        # noqa: E402

ok, message = S2.verify_input_size(YOLO(str(testlib.NCNN_MODEL), task='segment'), 320)
T.check('ncnn + 320 → 통과', ok, message)

ok, message = S2.verify_input_size(YOLO(str(testlib.NCNN_MODEL), task='segment'), 640)
T.check('ncnn + 640 → 실패', not ok)
T.check('메시지에 고정 크기와 INFER_SIZE 표시', '320x320' in message and 'INFER_SIZE = 640' in message,
        message.splitlines()[0])

if PT_MODEL.exists():
    for size in (320, 640):
        ok, message = S2.verify_input_size(YOLO(str(PT_MODEL), task='segment'), size)
        T.check(f'.pt + {size} → 통과 (고정 크기 아님)', ok, message)
else:
    print(f'건너뜀: .pt 모델 없음 ({PT_MODEL})')

# 실제 노드: INFER_SIZE 만 640 으로 바꿔 main() 실행
runner = (
    'import sys; sys.argv = ["step2_lane_follow.py", "--video", sys.argv[1]]; '
    f'sys.path.insert(0, {str(testlib.SRC_DIR / "robot")!r}); '
    'import step2_lane_follow as S2; S2.INFER_SIZE = 640; sys.exit(S2.main())'
)
proc = subprocess.run([sys.executable, '-c', runner, str(testlib.VIDEO)], stdin=subprocess.DEVNULL,
                      capture_output=True, text=True, timeout=120, cwd=str(testlib.MJ_DIR))
output = proc.stdout + proc.stderr
T.check('INFER_SIZE 640 으로 노드 실행 → exit 2', proc.returncode == 2, f'exit {proc.returncode}')
T.check('오류 메시지 출력', '모델 입력 크기 불일치' in output)
T.check('주행 루프에 들어가지 않음 (배너·상태 로그 없음)', 'Space     : 출발' not in output and 'fps |' not in output)
T.check('정리 후 종료 ("종료 완료")', '종료 완료' in output)

T.finish()
