"""
[느림: ncnn 모델 + 영상, 약 40초] robot/lane_mission_drive.py 의 LaneTracker 복제본 ↔ 공통 LaneTracker 동등성

lane_mission_drive.py 에서 상수와 x_at_row / pick_main / build_masks / LaneTracker 소스만 AST 로 꺼내
(ROS import 없이) 실행하고, 같은 ncnn 추론 결과를 공통 경로(group_masks + LaneTracker, 로봇 파라미터)와
나란히 넣어 프레임마다 (centers, steer, valid) 가 완전히 같은지 본다.

복제본은 상실을 제어 스텝 수(LOST_STOP_AFTER)로 세고 공통 모듈은 초로 센다.
now = 스텝 ÷ CONTROL_HZ, lost_stop_sec = LOST_STOP_AFTER ÷ CONTROL_HZ 로 넣으면 둘이 같아야 한다
(= "2.0초" 가 기존 "20스텝 @ 10Hz" 와 같은 동작이라는 확인).
lane_mission_drive.py 가 공통 모듈로 바뀌면 (4단계) 이 테스트는 지운다.
"""

import ast

import cv2
import numpy as np

import testlib
from common.lane_postprocess import LaneTracker, group_masks, pick_main

T = testlib.Checker('tracker_parity')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')
testlib.require(testlib.VIDEO, '입력 영상')

from ultralytics import YOLO  # noqa: E402  (느린 import 는 파일 확인 뒤에)

source = (testlib.SRC_DIR / 'robot' / 'lane_mission_drive.py').read_text()
want_consts = {'SAMPLE_ROWS', 'ROW_WEIGHTS', 'ROAD_HALF_INIT', 'ROAD_HALF_SMOOTH', 'STEER_GAIN', 'STEER_D_GAIN',
               'STEER_SMOOTH', 'LOST_STOP_AFTER', 'FRAME_SIZE', 'INFER_SIZE', 'CONF', 'CONTROL_HZ'}
want_defs = {'x_at_row', 'pick_main', 'build_masks', 'LaneTracker'}
robot = {'np': np, 'cv2': cv2}
for node in ast.parse(source).body:
    if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', None) in want_consts:
        exec(ast.unparse(node), robot)
    elif isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in want_defs:
        exec(ast.unparse(node), robot)
missing = (want_consts | want_defs) - robot.keys()
T.check('lane_mission_drive.py 에서 필요한 정의를 모두 찾음', not missing, f'없음: {missing}')
if missing:
    T.finish()

W, H = robot['FRAME_SIZE']
hz = robot['CONTROL_HZ']
old = robot['LaneTracker'](W, H)
new = LaneTracker(W, H, sample_rows=robot['SAMPLE_ROWS'], row_weights=robot['ROW_WEIGHTS'],
                  road_half_init=robot['ROAD_HALF_INIT'], road_half_smooth=robot['ROAD_HALF_SMOOTH'],
                  steer_gain=robot['STEER_GAIN'], steer_smooth=robot['STEER_SMOOTH'],
                  lost_stop_sec=robot['LOST_STOP_AFTER'] / hz,
                  steer_d_gain=robot['STEER_D_GAIN'], steer_clip=1.0)

model = YOLO(str(testlib.NCNN_MODEL), task='segment')
cap = cv2.VideoCapture(str(testlib.VIDEO))
n = mismatch = resets = clipped = lost = steer_zeroed = 0
first_mismatch = None
while True:
    ok, frame = cap.read()
    if not ok:
        break
    frame = cv2.resize(frame, (W, H))
    r = model.predict(source=frame, conf=robot['CONF'], imgsz=robot['INFER_SIZE'], verbose=False)[0]

    rm = robot['build_masks'](r, W, H)                          # 로봇 경로: 배열 목록
    o_centers, o_steer, o_valid = old.update(robot['pick_main'](rm.get('left_lane')),
                                             robot['pick_main'](rm.get('right_lane')))
    if r.masks is None or r.boxes is None:                     # 공통 경로: dict 목록
        cm = {}
    else:
        cm = group_masks(r.masks.data.cpu().numpy(), [r.names[int(c)] for c in r.boxes.cls.cpu().numpy()],
                         r.boxes.conf.cpu().numpy(), W, H)
    c_centers, _, c_steer, c_valid = new.update(pick_main(cm.get('left_lane')), pick_main(cm.get('right_lane')),
                                                now=n / hz)

    if (o_centers, o_steer, o_valid) != (c_centers, c_steer, c_valid):
        mismatch += 1
        first_mismatch = n if first_mismatch is None else first_mismatch
    clipped += abs(o_steer) == 1.0
    lost += not o_valid
    steer_zeroed += (not o_valid) and old.lost_frames > robot['LOST_STOP_AFTER']
    n += 1
    if n % 300 == 0:                                           # reset() 도 같이 비교
        old.reset()
        new.reset()
        resets += 1

T.check('전 프레임 (centers, steer, valid) 일치', mismatch == 0, f'{n}프레임 중 불일치 {mismatch}, 첫 불일치 {first_mismatch}')
T.check('D항·clip·상실·reset·상실 후 조향 0 경로를 모두 지남',
        robot['STEER_D_GAIN'] > 0 and clipped > 0 and lost > 0 and resets > 0 and steer_zeroed > 0,
        f'D {robot["STEER_D_GAIN"]}, clip {clipped}, 상실 {lost}, reset {resets}, 조향 0 복귀 {steer_zeroed}')

T.finish()
