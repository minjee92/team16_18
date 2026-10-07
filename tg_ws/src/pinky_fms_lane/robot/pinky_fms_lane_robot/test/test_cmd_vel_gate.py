"""cmd_vel_gate 판정 시험 (ROS 없이 gate() 함수)."""
import math

from pinky_fms_lane_robot.cmd_vel_gate import gate
from pinky_fms_lane_robot.front_stop import FrontStop

P = {'input_timeout': 0.5, 'link_timeout': 2.0, 'max_linear': 0.25, 'max_angular': 1.5, 'allow_reverse': False}


def state(now, sonar=True):
    s = FrontStop(0.0, 1.0, 0.02, 3) if sonar else None
    if s is not None:
        s.on_range(0.5, now)
    return {'in_t': now, 'hb_t': now, 'sonar': s, 'publishers': 1}


def test_passes_and_clamps():
    st = state(10.0)
    assert gate(0.1, 0.3, 10.1, st, P) == (0.1, 0.3, [])
    assert gate(0.9, -3.0, 10.1, st, P) == (0.25, -1.5, [])            # 빠르게 만들지 않고 자른다
    assert gate(-0.1, 0.0, 10.1, st, P)[0] == 0.0                       # 후진 금지
    assert gate(-0.1, 0.0, 10.1, st, dict(P, allow_reverse=True))[0] == -0.1


def test_stops_for_each_reason():
    st = state(10.0)
    assert gate(0.1, 0.0, 10.6, dict(st, hb_t=10.6, sonar=None), P)[2] == ['STALE_INPUT']
    assert gate(0.1, 0.0, 10.1, dict(st, hb_t=7.0), P)[2] == ['FMS_LINK']
    assert gate(0.1, 0.0, 10.1, dict(st, hb_t=None), P)[2] == ['FMS_LINK']          # 신호를 한 번도 못 받음
    assert gate(0.1, 0.0, 10.1, dict(st, hb_t=None), dict(P, link_timeout=0.0))[2] == []   # 0 이면 확인 안 함
    assert gate(0.1, 0.0, 10.1, dict(st, publishers=2), P)[:2] == (0.0, 0.0)
    assert gate(0.1, 0.0, 10.1, dict(st, publishers=2), P)[2] == ['CMD_VEL_CONFLICT']
    assert gate(float('nan'), 0.0, 10.1, st, P)[2] == ['BAD_INPUT']


def test_sonar_fail_closed_but_not_distance():
    st = state(10.0)
    s = st['sonar']
    assert gate(0.1, 0.0, 11.2, dict(st, in_t=11.2, hb_t=11.2), P)[2] == ['SONAR_TIMEOUT']
    for _ in range(2):
        s.on_range(-0.03, 10.2)                                        # 단발 무효값(I2C 실패 −0.03) 은 무시
    assert gate(0.1, 0.0, 10.25, st, P)[2] == []
    s.on_range(float('nan'), 10.3)                                     # 3번 연속 → 정지
    assert gate(0.1, 0.0, 10.35, st, P)[2] == ['SONAR_INVALID']
    s.on_range(0.05, 10.4)                                             # 가까운 사물은 게이트가 아니라 차선 노드가 선다
    assert gate(0.1, 0.0, 10.45, st, P)[2] == []
    assert gate(0.1, 0.0, 10.1, dict(state(10.0), sonar=FrontStop(0.0, 1.0, 0.02)), P)[2] == ['NO_SONAR']


def test_reason_names_have_no_spaces():
    st = state(10.0)
    st['sonar'] = FrontStop(0.0, 1.0, 0.02)
    v, w, r = gate(0.1, 0.0, 10.1, st, P)
    assert (v, w) == (0.0, 0.0) and all(' ' not in x for x in r) and not math.isnan(v)
