"""
[빠름: ROS 환경 필요, 모델 불필요] Picamera2 가 없는 PC 에서도 주행 노드가 뜨는지

PC 에서 시뮬을 돌릴 때는 picamera2 가 없다. 주행 노드(mj_lane_drive)는 Picamera2 입력원을 고를 때만
picamera2 를 import 해야 한다. 이 시험은 picamera2 를 "없는 것"으로 막아 놓고(설치돼 있어도) 확인한다.
"""

import ast
import sys

import testlib

sys.modules['picamera2'] = None          # import picamera2 → ImportError (설치 여부와 상관없이)

T = testlib.Checker('step2_no_picamera')

try:
    import mj_lane_drive.lane_follow_node as S2
    from mj_lane_drive.sources import PiCameraSource, VideoSource
    imported = True
except ImportError as error:
    imported = False
    print(f'  import 실패: {error}')
T.check('picamera2 없이 주행 노드 모듈 import', imported)
if not imported:
    T.finish()

top_level = []
for path in (testlib.SRC_DIR / 'mj_lane_drive' / 'mj_lane_drive').glob('*.py'):
    for node in ast.parse(path.read_text()).body:           # 모듈 맨 위 import 만 (함수 안 import 는 제외)
        if isinstance(node, ast.Import):
            top_level += [(path.name, a.name) for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            top_level.append((path.name, node.module or ''))
T.check('mj_lane_drive 어느 모듈도 맨 위에서 picamera2 를 import 하지 않음',
        not [m for _, m in top_level if m.split('.')[0] == 'picamera2'], f'{[t for t in top_level if "picamera" in t[1]]}')

testlib.require(testlib.VIDEO, '입력 영상')
video = VideoSource(str(testlib.VIDEO), S2.FRAME_SIZE)
frame = video.read()
video.close()
T.check('picamera2 없이 영상 입력원 동작 (640x480 BGR)', frame is not None and frame.shape == (480, 640, 3),
        f'{None if frame is None else frame.shape}')

try:
    PiCameraSource(S2.FRAME_SIZE)
    raised = None
except ImportError as error:
    raised = error
T.check('Picamera2 입력원을 고르면 그때 ImportError (조용히 넘어가지 않음)', raised is not None, f'{raised}')

T.finish()
