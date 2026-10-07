"""record_points.py 의 계산·파일 부분 시험 (ROS 없이). ROS 부분은 tests/run_record_test.sh."""
import datetime
import math
import os

import pytest
import yaml

from pinky_fms_lane.course import Course, Point
from pinky_fms_lane.record_points import (
    is_still, next_name, robot_entry, robot_file_path, save_robot_point, summarize, warnings_for)

HERE = os.path.dirname(os.path.abspath(__file__))
REAL_COURSE = os.path.join(HERE, '..', 'course', 'mission4_3_clean_1cm.course.yaml')
LIMITS = {'min_samples': 2, 'max_xy_std': 0.05, 'max_yaw_std': math.radians(10), 'max_spread': 0.02,
          'min_match': 0.7, 'max_road_dist': 0.083, 'max_diff': 0.05, 'max_yaw_diff': math.radians(30)}


def good(**kw):
    rec = {'x': 1.0, 'y': 0.5, 'yaw': 0.0, 'spread': 0.005, 'yaw_spread': 0.01, 'samples': 3,
           'std_xy': 0.02, 'std_yaw': math.radians(4), 'match': 0.9, 'road_dist': 0.01}
    rec.update(kw)
    return rec


def test_summarize_uses_circular_mean_for_yaw():
    s = summarize([(1.0, 0.0, math.pi - 0.1), (1.02, 0.0, -math.pi + 0.1)])    # ±180° 근처
    assert s['x'] == pytest.approx(1.01) and abs(abs(s['yaw']) - math.pi) < 1e-9
    assert s['yaw_spread'] == pytest.approx(0.1) and s['spread'] == pytest.approx(0.02)
    drift = summarize([(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.03, 0.0, 0.0)])    # 한쪽으로 흐름
    assert drift['spread'] == pytest.approx(0.03)                              # 첫·마지막 차이를 놓치지 않는다
    with pytest.raises(ValueError):
        summarize([])


def test_warnings():
    ref = Point('P', 1.0, 0.5, None, 'tape')
    assert warnings_for(good(), ref, LIMITS) == []
    cases = [(good(std_xy=0.08), '위치 표준편차'), (good(spread=0.04), '흐르는'), (good(match=0.4), '일치율'),
             (good(road_dist=0.12), '차선 위'), (good(x=1.08), '지금 값(tape)과 8.0 cm'),
             (good(samples=1), '1개만'), (good(std_yaw=math.radians(15)), '방향 표준편차')]
    for rec, text in cases:
        w = warnings_for(rec, ref, LIMITS)
        assert len(w) == 1 and text in w[0], (text, w)
    start = Point('R1', 1.0, 0.5, math.pi, 'estimate')
    assert '방향이' in warnings_for(good(yaw=0.0), start, LIMITS)[0]             # 반대로 놓인 출발점
    assert warnings_for(good(match=None), None, LIMITS) == []                   # 지도 없이, 새 점


def test_robot_entry_keeps_yaw_only_for_start_points():
    e = robot_entry(good(x=1.23456, yaw=3.0), True, '2026-10-07T10:00:00')
    assert e['x'] == 1.2346 and e['yaw'] == 3.0 and e['samples'] == 3 and e['match'] == 0.9
    assert 'yaw' not in robot_entry(good(), False, 's') and 'match' not in robot_entry(good(match=None), False, 's')


def test_save_robot_point_backs_up_and_course_reads_it(tmp_path):
    course = tmp_path / 'c.course.yaml'
    data = yaml.safe_load(open(REAL_COURSE, encoding='utf-8'))
    data.pop('tape')
    course.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
    out = robot_file_path(str(course))
    assert out == str(tmp_path / 'mission4_3_clean_1cm.robot.yaml')
    assert save_robot_point(out, 'J', robot_entry(good(x=1.15, y=0.95), False, 't1')) == ''      # 처음엔 백업 없음
    t = datetime.datetime(2026, 10, 7, 12, 0, 0)
    backup = save_robot_point(out, 'R1', robot_entry(good(x=0.79, y=0.25, yaw=3.1), True, 't2'), now=t)
    assert backup.endswith('.bak.20261007_120000') and 'R1' not in yaml.safe_load(open(backup))['points']
    text = open(out, encoding='utf-8').read()
    assert text.startswith('# 로봇 기록') and not os.path.exists(out + '.tmp')
    c = Course.load(str(course))
    assert (c.points['J'].x, c.points['J'].source, c.points['J'].stamp) == (1.15, 'robot', 't1')
    assert (c.points['R1'].yaw, c.points['R1'].source) == (3.1, 'robot')
    assert c.points['NE'].source == 'estimate'


def test_is_still():
    moving = [(t / 10, 0.05, 0.0) for t in range(20)]
    still = [(t / 10, 0.0, 0.001) for t in range(20)]
    assert is_still(still, 1.0) and not is_still(moving, 1.0)
    assert not is_still(still[:5], 1.0)                                         # 데이터가 1 s 를 덮지 못함
    assert not is_still(still[:15] + [(1.5, 0.0, 0.3)] + still[16:], 1.0)        # 제자리 회전 중
    assert not is_still([], 1.0)


def test_record_order_of_real_course():
    c = Course.load(REAL_COURSE)
    assert c.record_order[-1] == 'R1' and 'T_END' not in c.record_order        # R1 은 마지막, 벽 면은 기록 안 함
    assert next_name(c.record_order, {'T_D'}, {'T_C'}) == 'T_B'
    assert next_name(['A'], {'A'}, set()) is None
