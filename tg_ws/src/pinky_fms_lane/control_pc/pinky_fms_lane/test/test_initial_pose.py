"""initial_pose.py 의 계산 부분 시험 (ROS 없이 실행). ROS 를 쓰는 부분은 tests/run_initial_pose_test.sh 로 시험한다."""
import copy
import math

import numpy as np
import pytest

from pinky_fms_lane.course import Course
from pinky_fms_lane.initial_pose import (
    chain, compose, covariance, evaluate, parse_targets, scan_match, start_pose, stds)

COURSE = {
    'points': {'J': {'x': 0.0, 'y': 1.0}, 'TE': {'x': -1.0, 'y': 1.0}, 'NE': {'x': 1.0, 'y': 1.0},
               'SE': {'x': 1.0, 'y': 0.0}, 'SW': {'x': 0.0, 'y': 0.0},
               'R1': {'x': -0.5, 'y': 1.0, 'yaw': 0.0, 'source': 'measured'}, 'NOYAW': {'x': 0.5, 'y': 0.0}},
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
    assert start_pose(c, 'R1') == (-0.5, 1.0, 0.0, 'measured')
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
