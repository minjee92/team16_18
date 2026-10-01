"""
[빠름] tools/step1_check_lane_detection.py DetectionStats 단위 테스트 (가짜 마스크, 추론 없음)

  미검출 연속 구간이 영상 처음·중간·끝에 있을 때 시작 프레임과 길이를 맞게 세는지,
  좌우 차선 양쪽/한쪽/미검출 집계가 맞는지 확인한다.
"""

import testlib
import step1_check_lane_detection as step1

T = testlib.Checker('step1_stats')

LEFT = {'left_lane': [1]}
BOTH = {'left_lane': [1], 'right_lane': [1]}
NONE = {}

# 미검출 2 | 한쪽 1 | 미검출 15 | 양쪽 3 | 미검출 16 (끝까지)
seq = [NONE] * 2 + [LEFT] + [NONE] * 15 + [BOTH] * 3 + [NONE] * 16

stats = step1.DetectionStats()
for masks in seq:
    stats.update(masks)
stats._close_run(stats.total)

T.check('처리 프레임 수', stats.total == 37)
T.check('미검출 구간 (시작, 길이)', stats.miss_runs == [(0, 2), (3, 15), (21, 16)], f'{stats.miss_runs}')
T.check('양쪽/한쪽/미검출 집계', stats.lanes == {'both': 3, 'one': 1, 'none': 33}, f'{stats.lanes}')
T.check('클래스별 검출 수', stats.counts == {'left_lane': 4, 'right_lane': 3, 'crosswalk': 0}, f'{stats.counts}')

T.finish()
