"""
[빠름] robot/drive_control.py 단위 테스트 (ROS·모델 불필요)

  FrontStop           : 초음파 비상 정지 판정 경계, latch, 수동 재개 거부, 무효값/끊김 fail-safe
  LaneFollowController: 추종 속도·각속도, 차선 상실 단계(0.5초 감속 / 2.0초 정지) 경계
"""

import testlib
from drive_control import FrontStop, LaneFollowController

T = testlib.Checker('drive_control')


def front():
    return FrontStop(stop_distance=0.15, timeout=0.3, min_range=0.02)


# ---------------- FrontStop ----------------

f = front()
T.check('측정 없음 → NO SONAR 로 정지', f.check(0.0) and f.trip_reason == 'NO SONAR')
T.check('측정 없음에서 재개 거부', f.clear(0.0) == (False, 'NO SONAR'))
f.on_range(0.50, 1.0)
T.check('정상 측정 후 재개 성공', f.clear(1.0) == (True, ''))
T.check('정상 거리 → 주행', not f.check(1.05))
f.on_range(0.16, 1.10)
T.check('0.16 m → 주행', not f.check(1.10))
f.on_range(0.15, 1.15)
T.check('0.15 m (경계, 이하) → OBSTACLE 정지', f.check(1.15) and f.trip_reason == 'OBSTACLE')
f.on_range(0.50, 1.20)
T.check('장애물이 치워져도 자동 재개 안 함 (latch)', f.check(1.20))
T.check('치운 뒤 재개 → 성공', f.clear(1.20) == (True, ''))
f.on_range(-0.03, 1.25)
T.check('I2C 실패값 −0.03 → SONAR INVALID 정지', f.check(1.25) and f.trip_reason == 'SONAR INVALID')
T.check('무효값 상태에서 재개 거부', f.clear(1.25) == (False, 'SONAR INVALID'))

for bad in (float('nan'), float('inf'), float('-inf')):
    g = front()
    g.on_range(0.5, 0.0)
    g.clear(0.0)
    g.on_range(bad, 0.05)
    T.check(f'{bad} → SONAR INVALID 정지', g.check(0.05) and g.trip_reason == 'SONAR INVALID')

g = front()
g.on_range(0.5, 0.0)
g.clear(0.0)
T.check('측정 후 0.30 s (경계) → 아직 주행', not g.check(0.30))
T.check('측정 후 0.31 s → SONAR TIMEOUT 정지', g.check(0.31) and g.trip_reason == 'SONAR TIMEOUT')
T.check('끊긴 상태에서 재개 거부', g.clear(0.4) == (False, 'SONAR TIMEOUT'))

h = front()
h.on_range(0.10, 0.0)
h.check(0.0)
h.on_range(-0.03, 0.05)
h.check(0.05)
T.check('정지 중 다른 조건이 와도 처음 이유를 유지', h.trip_reason == 'OBSTACLE')


# ---------------- LaneFollowController ----------------

C = LaneFollowController(base_speed=0.10, min_speed=0.05, max_angular=1.5, corner_slowdown=0.5,
                         steer_to_angular=2.0, lost_slow_sec=0.5, lost_stop_sec=2.0)

T.check('차선 보임, 직진 → v 0.10, w 0', C.command(0.0, True, 0.0)[:2] == (0.10, 0.0))
v, w, _ = C.command(0.5, True, 0.0)
T.check('steer +0.5 → v 0.075, w −1.0 (우회전)', abs(v - 0.075) < 1e-12 and w == -1.0)
v, w, _ = C.command(1.0, True, 0.0)
T.check('steer +1.0 → v 0.05(최소), w −1.5(제한)', v == 0.05 and w == -1.5)
v, w, _ = C.command(-1.0, True, 0.0)
T.check('steer −1.0 → w +1.5 (좌회전, 제한)', w == 1.5)
T.check('상실 0.5 s (경계) → 70% 속도', C.command(0.2, False, 0.5)[0] == 0.10 * 0.7)
T.check('상실 0.55 s → MIN_SPEED', C.command(0.2, False, 0.55)[0] == 0.05)
T.check('상실 2.0 s (경계) → 아직 MIN_SPEED', C.command(0.2, False, 2.0)[0] == 0.05)
T.check('상실 2.05 s → 정지 (v, w 모두 0)', C.command(0.2, False, 2.05)[:2] == (0.0, 0.0))
T.check('상실 중에도 직전 조향 유지', C.command(0.5, False, 0.3)[1] == -1.0)

T.finish()
