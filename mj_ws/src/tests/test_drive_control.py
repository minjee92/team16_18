"""
[빠름] robot/drive_control.py 단위 테스트 (ROS·모델 불필요)

  FrontStop           : 초음파 비상 정지 판정 경계, latch, 수동 재개 거부, 무효값/끊김 fail-safe
  LaneFollowController: 추종 속도·각속도, 차선 상실 단계(0.5초 감속 / 2.0초 정지) 경계
"""

import testlib
from drive_control import FrontStop, LaneFollowController

T = testlib.Checker('drive_control')


INVALID_COUNT = 3


def front():
    return FrontStop(stop_distance=0.15, timeout=0.3, min_range=0.02, invalid_count=INVALID_COUNT)


def started(range_m=0.5, now=0.0):
    """정상 측정을 받고 출발한 상태."""
    f = front()
    f.on_range(range_m, now)
    assert f.clear(now) == (True, '')
    return f


# ---------------- FrontStop: 기본 판정 ----------------

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

g = started()
T.check('측정 후 0.30 s (경계) → 아직 주행', not g.check(0.30))
T.check('측정 후 0.31 s → SONAR TIMEOUT 정지', g.check(0.31) and g.trip_reason == 'SONAR TIMEOUT')
T.check('끊긴 상태에서 재개 거부', g.clear(0.4) == (False, 'SONAR TIMEOUT'))

h = started()
h.on_range(0.10, 0.05)
h.check(0.05)
for k in range(INVALID_COUNT):
    h.on_range(-0.03, 0.10 + 0.05 * k)
h.check(0.25)
T.check('정지 중 다른 조건이 와도 처음 이유를 유지', h.trip_reason == 'OBSTACLE')


# ---------------- FrontStop: 무효값 연속 판정 ----------------

g = started()
g.on_range(-0.03, 0.05)
T.check('무효 1회 → 무시하고 주행', not g.check(0.05))
g.on_range(-0.03, 0.10)
T.check(f'무효 2회 연속 → 아직 주행 (기준 {INVALID_COUNT}회)', not g.check(0.10))
g.on_range(0.50, 0.15)
T.check('유효값으로 끊긴 2회 튐 → 무시 2회, 튐 1번', g.invalid_summary() == (2, 1, 0), f'{g.invalid_summary()}')
g.on_range(-0.03, 0.20)
g.on_range(-0.03, 0.25)
T.check('연속 수는 유효값에서 0 부터 다시 (2+2 회여도 정지 안 함)', not g.check(0.25) and g.invalid_streak == 2)
g.on_range(-0.03, 0.30)
T.check(f'무효 {INVALID_COUNT}회 연속 → SONAR INVALID 정지', g.check(0.30) and g.trip_reason == 'SONAR INVALID')
T.check('정지로 이어진 무효값은 무시 수에 안 셈', g.invalid_summary() == (2, 1, 1), f'{g.invalid_summary()}')
T.check('무효 연속 중에는 재개 거부', g.clear(0.30) == (False, 'SONAR INVALID'))
g.on_range(0.50, 0.35)
T.check('유효값이 오면 재개 가능', g.clear(0.35) == (True, ''))
T.check('정지로 끝난 연속은 튐으로 안 셈', g.invalid_summary() == (2, 1, 1), f'{g.invalid_summary()}')

for bad in (float('nan'), float('inf'), float('-inf'), 0.019, -0.03):
    b = started()
    for k in range(INVALID_COUNT - 1):
        b.on_range(bad, 0.05 * (k + 1))
    single = not b.check(0.05 * INVALID_COUNT)
    b.on_range(bad, 0.05 * INVALID_COUNT)
    T.check(f'{bad}: {INVALID_COUNT - 1}회까지 주행, {INVALID_COUNT}회째 SONAR INVALID',
            single and b.check(0.05 * INVALID_COUNT) and b.trip_reason == 'SONAR INVALID')

o = started(0.50)
o.on_range(0.12, 0.05)
o.check(0.05)
T.check('장애물 판정은 마지막 유효 거리 (0.12 → 정지)', o.tripped and o.trip_reason == 'OBSTACLE')
p = started(0.50)
p.on_range(0.12, 0.05)
p.on_range(-0.03, 0.10)
T.check('튐 사이에서도 마지막 유효 거리(0.12)로 장애물 판정', p.check(0.10) and p.trip_reason == 'OBSTACLE')
q = started(0.50)
q.on_range(-0.03, 0.05)
T.check('튐 중 마지막 유효 거리가 멀면(0.5) 주행 유지', not q.check(0.05) and q.valid_range == 0.5)

r = front()
r.on_range(-0.03, 0.0)
T.check('유효 거리를 한 번도 못 받았으면 정지 (출발 전 fail-safe)', r.check(0.0) and r.clear(0.0)[0] is False)
r.on_range(0.5, 0.05)
T.check('첫 유효 거리 뒤 출발 가능', r.clear(0.05) == (True, ''))

s = started()
s.on_range(-0.03, 0.05)
T.check('끝나지 않은 짧은 연속도 요약에서는 무시로 셈', s.invalid_summary() == (1, 1, 0), f'{s.invalid_summary()}')
t = started()
for k in range(INVALID_COUNT):
    t.on_range(-0.03, 0.05 * (k + 1))
t.on_range(-0.03, 0.05 * (INVALID_COUNT + 1))
t.check(0.2)
T.check('정지 뒤 이어지는 무효값도 무시 수에 안 셈', t.invalid_summary() == (0, 0, 1), f'{t.invalid_summary()}')
T.check('무효값도 수신으로 쳐서 끊김(timeout) 아님', t.reason(0.21) == 'SONAR INVALID')


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
