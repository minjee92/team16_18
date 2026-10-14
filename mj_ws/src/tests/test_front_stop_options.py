"""
[빠름: 모델·ROS 불필요] 전방 정지의 선택 기능 (B: 속도에 맞춘 정지 거리, D: 갈림길 회전 단계의 제자리 회전 허용)

  - 기본(둘 다 끔): 속도와 상관없이 0.15 m, spin_ok 는 늘 False → 지금 동작 그대로
  - B (0.06, 0.9, 0.10): 0.10 m/s → 0.15 m, 0.05 m/s → 0.105 m, 0 → 0.10 m (바닥값). 수동 재개는 0.15 m 기준
  - D (spin_min 0.05): 장애물이 0.05 m 보다 멀면 회전 허용, 0.05 m 이하면 회전도 막음
    센서 고장(끊김·무효값 연속·측정 없음)이면 회전도 허용하지 않음
  - 진짜 장애물: 어느 설정에서도 정지 조건이면 멈추고(latch), 장애물이 남아 있으면 재개 거부
"""

import testlib

from mj_lane_core.drive_control import FrontStop

T = testlib.Checker('front_stop_options')
B = (0.06, 0.9, 0.10)


def front(**kw):
    f = FrontStop(0.15, 0.3, 0.02, 3, **kw)
    return f


def at(f, rng, now=0.0):
    f.on_range(rng, now)
    return f


# ---------- 기본 ----------
f = at(front(), 0.12)
T.check('기본: 0.12 m 는 속도와 상관없이 OBSTACLE', f.reason(0.0, 0.05) == 'OBSTACLE' and f.reason(0.0, 0.0) == 'OBSTACLE'
        and f.reason(0.0) == 'OBSTACLE')
T.check('기본: 정지 거리 0.15 고정', front().stop_distance_for(0.05) == 0.15 and front().stop_distance_for(None) == 0.15)
T.check('기본: spin_ok 는 늘 False (회전도 막음)', not at(front(), 0.12).spin_ok(0.0))

# ---------- B ----------
f = front(speed_scaled=B)
d = {v: round(f.stop_distance_for(v), 4) for v in (0.10, 0.07, 0.05, 0.0, 0.2)}
T.check('B: 0.10→0.15, 0.07→0.123, 0.05→0.105, 0→0.10(바닥), 0.2→0.15(위로 자름)',
        d == {0.10: 0.15, 0.07: 0.123, 0.05: 0.105, 0.0: 0.10, 0.2: 0.15}, f'{d}')
f = at(front(speed_scaled=B), 0.12)
T.check('B: 0.12 m 에서 0.10 m/s 는 정지, 0.05 m/s 는 통과', f.reason(0.0, 0.10) == 'OBSTACLE' and f.reason(0.0, 0.05) == '')
f = at(front(speed_scaled=B), 0.09)
T.check('B: 0.09 m 는 아주 느려도 정지 (바닥 0.10 m)', f.reason(0.0, 0.0) == 'OBSTACLE' and f.reason(0.0, 0.05) == 'OBSTACLE')
f = at(front(speed_scaled=B), 0.12)
T.check('B: 수동 재개는 0.15 m 기준이라 0.12 m 면 거부', f.clear(0.0) == (False, 'OBSTACLE'))
f = at(front(speed_scaled=B), 0.09)
f.check(0.0, 0.05)
f.on_range(0.12, 0.05)
T.check('B: 한 번 멈추면 장애물이 조금 멀어져도 정지 유지 (latch)', f.check(0.05, 0.05) and f.trip_reason == 'OBSTACLE')

# ---------- D ----------
f = at(front(spin_min=0.05), 0.12)
T.check('D: 0.12 m 장애물이면 회전 허용 (전진은 여전히 정지 조건)', f.spin_ok(0.0) and f.reason(0.0, 0.0) == 'OBSTACLE')
T.check('D: 0.05 m 이하면 회전도 막음', not at(front(spin_min=0.05), 0.05).spin_ok(0.0)
        and not at(front(spin_min=0.05), 0.03).spin_ok(0.0))
f = at(front(spin_min=0.05), 0.12)
T.check('D: 측정이 끊기면(0.3초) 회전 허용 안 함', not f.spin_ok(0.5))
f = front(spin_min=0.05)
for k in range(3):
    f.on_range(-0.03, 0.01 * k)
T.check('D: 무효값 연속(센서 이상)이면 회전 허용 안 함', not f.spin_ok(0.03))
T.check('D: 측정이 없으면 회전 허용 안 함', not front(spin_min=0.05).spin_ok(0.0))

T.finish()
