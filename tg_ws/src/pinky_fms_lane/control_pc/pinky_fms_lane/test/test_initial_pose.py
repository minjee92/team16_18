"""initial_pose.py 의 계산 부분 시험 (ROS 없이 실행). ROS 를 쓰는 부분은 tests/run_initial_pose_test.sh 로 시험한다."""
import copy
import math

import numpy as np
import pytest

from pinky_fms_lane.course import Course
from pinky_fms_lane.initial_pose import (
    axis_summary, chain, compose, covariance, diagnosis, direction_offsets, evaluate, expected_ranges,
    parse_targets, scan_match, start_pose, stds)

COURSE = {
    'points': {'J': {'x': 0.0, 'y': 1.0}, 'TE': {'x': -1.0, 'y': 1.0}, 'NE': {'x': 1.0, 'y': 1.0},
               'SE': {'x': 1.0, 'y': 0.0}, 'SW': {'x': 0.0, 'y': 0.0},
               'R1': {'x': -0.5, 'y': 1.0, 'yaw': 0.0}, 'NOYAW': {'x': 0.5, 'y': 0.0}},
    'edges': {'tail': {'points': ['J', 'TE']}, 'loop': {'points': ['J', 'NE', 'SE', 'SW', 'J'], 'oneway': True}},
    'junctions': {'J': {'moves': [
        {'from': 'tail', 'to': 'loop', 'turn': 'STRAIGHT', 'follow': 'LEFT', 'follow_dist': 0.3},
        {'from': 'loop', 'to': 'tail', 'turn': 'LEFT', 'follow': 'LEFT', 'follow_dist': 0.3},
        {'from': 'loop', 'to': 'loop', 'turn': 'RIGHT', 'follow': 'RIGHT', 'follow_dist': 0.3}]}},
}
LIMITS = {'max_xy_std': 0.10, 'max_yaw_std': math.radians(10), 'min_match': 0.7, 'need_match': True}


def test_parse_targets():
    assert parse_targets(['amr_01:R1', '/amr_02:R2']) == [('amr_01', 'R1'), ('amr_02', 'R2')]
    for bad in (['amr_01'], ['amr_01:'], [':R1']):
        with pytest.raises(ValueError):
            parse_targets(bad)


def test_start_pose_reads_course_points():
    c = Course(copy.deepcopy(COURSE))
    assert start_pose(c, 'R1') == (-0.5, 1.0, 0.0, 'estimate')      # 출처도 함께 돌려준다
    with pytest.raises(ValueError, match='yaw'):
        start_pose(c, 'NOYAW')                      # 방향이 없는 점은 출발점이 될 수 없다
    with pytest.raises(ValueError, match='없음'):
        start_pose(c, 'R9')


def test_covariance_round_trip():
    cov = covariance(0.05, math.radians(5))
    assert len(cov) == 36 and cov[0] == cov[7] == pytest.approx(0.0025)
    sxy, syaw = stds(cov)
    assert sxy == pytest.approx(0.05) and math.degrees(syaw) == pytest.approx(5)


def test_chain_of_static_transforms():
    static = {'base_link': ('base_footprint', (0.0, 0.0, 0.0)),
              'laser_link': ('base_link', (0.03, 0.0, math.pi))}
    x, y, yaw = chain(static, 'base_footprint', 'laser_link')
    assert (x, y) == pytest.approx((0.03, 0.0)) and abs(yaw) == pytest.approx(math.pi)    # +π 와 -π 는 같은 방향
    assert chain(static, 'base_footprint', 'base_footprint') == (0.0, 0.0, 0.0)
    assert chain(static, 'base_footprint', 'camera') is None
    # 로봇이 (1, 2) 에서 북쪽을 보면 라이다는 (1, 2.03), 남쪽을 본다
    x, y, yaw = compose((1.0, 2.0, math.pi / 2), (0.03, 0.0, math.pi))
    assert (x, y) == pytest.approx((1.0, 2.03)) and abs(abs(yaw) - math.pi / 2) < 1e-9


def test_scan_match_counts_beam_ends_near_walls():
    # 벽 = x 가 1.0 인 세로선. 원점에서 동쪽(0°)으로 쏜 빔만 벽에 닿는다
    dist = lambda xs, ys: np.abs(np.asarray(xs) - 1.0)        # noqa: E731
    ranges = [1.0, 1.0, float('inf'), 0.5]                    # 0°, 90°(벽 아님), 범위 밖, 270°(벽 아님)
    score, n = scan_match(ranges, 0.0, math.pi / 2, 0.05, 3.5, (0.0, 0.0, 0.0), dist, 0.05)
    assert n == 3 and score == pytest.approx(1 / 3)
    score, _ = scan_match([1.0], 0.0, 0.1, 0.05, 3.5, (0.3, 0.0, 0.0), dist, 0.05)   # 30 cm 틀린 위치
    assert score == 0.0
    assert scan_match([float('nan')], 0.0, 0.1, 0.05, 3.5, (0, 0, 0), dist, 0.05) == (0.0, 0)


def test_evaluate():
    good = covariance(0.03, math.radians(3))
    assert evaluate((0, 0, 0), good, 0.9, LIMITS) == (True, [])
    ok, why = evaluate(None, None, None, LIMITS)
    assert not ok and 'amcl_pose' in why[0]
    ok, why = evaluate((0, 0, 0), covariance(0.2, math.radians(3)), 0.9, LIMITS)
    assert not ok and '위치 표준편차' in why[0]
    ok, why = evaluate((0, 0, 0), good, 0.4, LIMITS)          # 확신은 하지만 스캔이 지도와 안 맞음 = 위치가 틀림
    assert not ok and '스캔 일치율' in why[0]
    assert evaluate((0, 0, 0), good, None, dict(LIMITS, need_match=False)) == (True, [])
    assert not evaluate((0, 0, 0), good, None, LIMITS)[0]


def room(width, height, res=0.01):
    """가로 width, 세로 height (m) 방. 벽 안쪽 면이 x=0, x=width, y=0, y=height. 원점 (-0.1, -0.1)."""
    w, h = int(round((width + 0.2) / res)), int(round((height + 0.2) / res))
    occ = np.zeros((h, w), bool)
    occ[:10, :] = occ[-10:, :] = True
    occ[:, :10] = occ[:, -10:] = True
    return occ, res, (-0.1, -0.1)


ANGLES = np.radians(np.arange(0, 360, 1.0))


def scan_from(occ, res, origin, x, y):
    return expected_ranges(occ, res, origin, x, y, ANGLES, 3.5)


def test_expected_ranges_in_a_room():
    occ, res, origin = room(2.0, 1.0)
    r = expected_ranges(occ, res, origin, 0.5, 0.4, np.array([0.0, np.pi / 2, np.pi, -np.pi / 2]), 3.5)
    assert r == pytest.approx([1.5, 0.6, 0.5, 0.4], abs=0.006)


def test_map_too_small_shows_as_size_difference():
    real = room(2.0, 1.08)                                  # 실제 방은 세로 108 cm
    occ, res, origin = room(2.0, 1.0)                        # 지도는 세로 100 cm (위 벽이 8 cm 안쪽)
    measured = scan_from(*real, 0.5, 0.4)
    off = direction_offsets(ANGLES, measured, scan_from(occ, res, origin, 0.5, 0.4), 0.05, 3.5)
    assert off['up'][0] == pytest.approx(0.08, abs=0.01) and off['down'][0] == pytest.approx(0.0, abs=0.01)
    size, shift = axis_summary(off)['vertical']
    assert size == pytest.approx(0.08, abs=0.01)             # 로봇 위치와 상관없이 크기 차이만 남는다
    assert axis_summary(off)['horizontal'][0] == pytest.approx(0.0, abs=0.01)
    assert any('세로 지도 크기 차이' in n for n in diagnosis(axis_summary(off)))


def test_robot_offset_shows_as_shift_not_size():
    occ, res, origin = room(2.0, 1.0)
    measured = scan_from(occ, res, origin, 0.5, 0.45)        # 실제 로봇은 5 cm 위에 있는데
    amcl_view = scan_from(occ, res, origin, 0.5, 0.40)       # AMCL 은 0.40 이라고 봄
    off = direction_offsets(ANGLES, measured, amcl_view, 0.05, 3.5)
    size, shift = axis_summary(off)['vertical']
    assert size == pytest.approx(0.0, abs=0.01) and shift == pytest.approx(0.05, abs=0.01)
    notes = diagnosis(axis_summary(off))
    assert any('로봇 위치 어긋남' in n for n in notes) and not any('지도 크기' in n for n in notes)


def test_direction_offsets_ignore_obstacles_and_missing_sides():
    occ, res, origin = room(2.0, 1.0)
    exp = scan_from(occ, res, origin, 0.5, 0.4)
    measured = exp.copy()
    measured[80:100] = 0.2                                   # 위쪽 빔 일부를 다른 로봇이 가림 → 빼야 한다
    off = direction_offsets(ANGLES, measured, exp, 0.05, 3.5)
    assert off['up'][0] == pytest.approx(0.0, abs=0.005) and off['up'][1] < 61
    off = direction_offsets(ANGLES, np.full(360, np.inf), exp, 0.05, 3.5)
    assert off['up'] == (None, 0) and axis_summary(off)['vertical'] == (None, None)
    assert diagnosis(axis_summary(off)) == []
