"""
[느림: ncnn 모델 + 영상 + ROS 환경, 약 40초] step2 판단 로직을 저장 영상 시간으로 재생해 기대값과 비교

robot/step2_lane_follow.py 의 상수·build_masks 를 그대로 import 해서 (실제 배포 코드)
카메라 대신 영상 프레임을, 시계 대신 영상 시간(프레임 ÷ fps)을 넣는다.
프레임마다 FOLLOW / LOST(70% 속도) / SLOW / STOP 단계를 매기고 아래 EXPECTED 와 비교한다.

EXPECTED 는 이 PC(ncnn CPU 추론)에서 c899633 기준으로 기록한 값이다.
정지·속도 로직을 일부러 바꿨으면 (예: 3단계 crosswalk 정지) 결과를 확인한 뒤 EXPECTED 를 갱신한다.
step2_lane_follow 가 rclpy 를 import 하므로 ROS 환경(source /opt/ros/jazzy/setup.bash)이 필요하다.
"""

import collections

import cv2

import testlib

T = testlib.Checker('step2_replay')
testlib.require(testlib.NCNN_MODEL, 'ncnn 모델')
testlib.require(testlib.VIDEO, '입력 영상')

import step2_lane_follow as S2                                   # noqa: E402
from ultralytics import YOLO                                     # noqa: E402
from common.lane_postprocess import LaneTracker, pick_main       # noqa: E402
from drive_control import LaneFollowController                   # noqa: E402

EXPECTED = {
    'tiers': {'FOLLOW': 1365, 'LOST': 32, 'SLOW': 43, 'STOP': 29},
    # 0.5초를 넘긴 상실 구간: (시작, 끝, SLOW 시작, STOP 시작)
    'long_runs': [(955, 977, 965, None), (1400, 1468, 1410, 1440)],
}

model = YOLO(S2.MODEL_PATH, task='segment')
cap = cv2.VideoCapture(str(testlib.VIDEO))
fps = cap.get(cv2.CAP_PROP_FPS)
W, H = S2.FRAME_SIZE
tracker = LaneTracker(W, H, sample_rows=S2.SAMPLE_ROWS, row_weights=S2.ROW_WEIGHTS,
                      road_half_init=S2.ROAD_HALF_INIT, road_half_smooth=S2.ROAD_HALF_SMOOTH,
                      steer_gain=S2.STEER_GAIN, steer_smooth=S2.STEER_SMOOTH,
                      lost_stop_sec=S2.LOST_STOP_SEC, steer_d_gain=S2.STEER_D_GAIN, steer_clip=S2.STEER_CLIP)
controller = LaneFollowController(S2.BASE_SPEED, S2.MIN_SPEED, S2.MAX_ANGULAR, S2.CORNER_SLOWDOWN,
                                  S2.STEER_TO_ANGULAR, S2.LOST_SLOW_SEC, S2.LOST_STOP_SEC)

rows = []
i = 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    frame = cv2.resize(frame, (W, H))
    r = model.predict(source=frame, conf=S2.CONF, imgsz=S2.INFER_SIZE, verbose=False)[0]
    masks = S2.build_masks(r, W, H)
    _, _, steer, valid = tracker.update(pick_main(masks.get('left_lane')), pick_main(masks.get('right_lane')),
                                        now=i / fps)
    v, w, note = controller.command(steer * S2.STEER_SIGN, valid, tracker.lost_time)
    if valid:
        tier = 'FOLLOW'
    elif note.endswith('STOP'):
        tier = 'STOP'
    elif note.endswith('SLOW'):
        tier = 'SLOW'
    else:
        tier = 'LOST'
    rows.append((i, tier, v, w))
    i += 1

tiers = dict(collections.Counter(t for _, t, _, _ in rows))

runs = []
for i, tier, _, _ in rows:
    if tier == 'FOLLOW':
        continue
    if runs and runs[-1][1] == i - 1:
        runs[-1][1] = i
    else:
        runs.append([i, i, None, None])
    if tier == 'SLOW' and runs[-1][2] is None:
        runs[-1][2] = i
    if tier == 'STOP' and runs[-1][3] is None:
        runs[-1][3] = i
long_runs = [tuple(r) for r in runs if r[2] is not None]

T.check('프레임 수', len(rows) == sum(EXPECTED['tiers'].values()), f'{len(rows)}')
T.check('단계별 프레임 수', tiers == EXPECTED['tiers'], f'{tiers}')
T.check('0.5초 넘는 상실 구간 (시작, 끝, SLOW, STOP)', long_runs == EXPECTED['long_runs'], f'{long_runs}')
T.check('STOP 은 v, w 모두 0', all(v == 0.0 and w == 0.0 for _, t, v, w in rows if t == 'STOP'))
T.check('FOLLOW 속도는 MIN~BASE, |w| ≤ MAX_ANGULAR',
        all(S2.MIN_SPEED <= v <= S2.BASE_SPEED and abs(w) <= S2.MAX_ANGULAR for _, t, v, w in rows if t == 'FOLLOW'))

T.finish()
