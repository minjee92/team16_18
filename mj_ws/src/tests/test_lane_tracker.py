"""
[빠름] common/lane_postprocess.py LaneTracker 의 시간 기준 차선 상실 판정 (모델 불필요)

  - 20fps 영상 시간(프레임 ÷ fps)에서 40프레임(정확히 2.0초) 상실은 조향 유지, 41프레임(2.05초)에서 0 복귀
  - 시작 위치를 바꿔도 경계 판정이 부동소수점 오차로 흔들리지 않음
  - reset() 후 시간 기준점이 다시 잡힘
"""

import numpy as np

import testlib
from common.lane_postprocess import LaneTracker

T = testlib.Checker('lane_tracker')

W, H, FPS = 640, 480, 20.0
LANE = np.zeros((H, W), np.uint8)
LANE[:, 400:410] = 1                 # 오른쪽으로 치우친 left_lane → steer > 0


def run(lost_frames, start_index):
    """차선을 30프레임 본 뒤 lost_frames 동안 놓친다. 반환: (직전 steer, 조향 0 이 된 상실 프레임 번호, lost_time)."""
    tracker = LaneTracker(W, H, lost_stop_sec=2.0)
    i = start_index
    for _ in range(30):
        tracker.update(LANE, None, now=i / FPS)
        i += 1
    held, reset_at = tracker.steer, None
    for k in range(1, lost_frames + 1):
        tracker.update(None, None, now=i / FPS)
        i += 1
        if tracker.steer == 0.0 and reset_at is None:
            reset_at = k
    return held, reset_at, tracker.lost_time


held, reset_at, lost_time = run(40, 1368)
T.check('40프레임(2.0초) 상실 → 조향 유지', held > 0 and reset_at is None and lost_time == 2.0,
        f'steer {held:+.3f}, lost_time {lost_time}')
held, reset_at, lost_time = run(41, 1368)
T.check('41프레임(2.05초) 상실 → 41번째에 조향 0', reset_at == 41, f'reset_at {reset_at}, lost_time {lost_time}')

unstable = [s for s in range(0, 3000, 7) if run(40, s)[1] is not None or run(41, s)[1] != 41]
T.check('시작 위치 429곳에서 경계 판정 동일', not unstable, f'흔들린 시작 위치 {unstable[:5]}')

tracker = LaneTracker(W, H)
tracker.update(LANE, None, now=0.0)
for k in range(10):
    tracker.update(None, None, now=0.05 * (k + 1))
T.check('상실 중 lost_time 누적', tracker.lost_time == 0.5, f'{tracker.lost_time}')
tracker.reset()
tracker.update(None, None, now=5.0)
T.check('reset 후 첫 상실 프레임 lost_time = 0', tracker.lost_time == 0.0, f'{tracker.lost_time}')
T.check('reset 후 steer 0', tracker.steer == 0.0)

# rows_obs: 로그·영상용 행별 관측이 실제 계산(centers, target_x)과 맞는지
partial = np.zeros((H, W), np.uint8)
partial[int(H * 0.75):, 400:410] = 1          # 아래쪽 두 행(0.90, 0.80)에만 left_lane
tracker = LaneTracker(W, H)
centers, _, _, valid = tracker.update(partial, None, now=0.0)
obs = tracker.rows_obs
T.check('rows_obs 는 샘플 행마다 하나', [o['y'] for o in obs] == tracker.rows)
T.check('읽은 행: source=left, lx 있음, rx 없음, cx = int(lx + 반폭)',
        all(o['source'] == 'left' and o['lx'] is not None and o['rx'] is None
            and o['cx'] == int(o['lx'] + o['half_width']) for o in obs[:2]))
T.check('못 읽은 행: lx/rx/source/cx 모두 None', all(o['lx'] is None and o['source'] is None and o['cx'] is None
                                                  for o in obs[2:]))
T.check('rows_obs 의 cx 가 centers 와 같음', [(o['cx'], o['y'], o['source']) for o in obs[:2]] == centers)
weighted = sum(o['cx'] * w for o, w in zip(obs, tracker.row_weights) if o['cx'] is not None)
T.check('rows_obs 로 다시 계산한 목표점 = target_x',
        valid and weighted / sum(tracker.row_weights[:2]) == tracker.target_x)

T.finish()
