"""lane_route.plan_lane_cmd 시험 (ROS 없이). 좌표에 기대지 않게 test_course 의 시험용 코스를 쓴다."""
import copy
import math
import os

import pytest

from pinky_fms_lane.course import Course
from pinky_fms_lane.lane_route import Reject, plan_lane_cmd

from test_course import BASE, EAST, NORTH, WEST  # noqa: I100

HERE = os.path.dirname(os.path.abspath(__file__))
REAL_COURSE = os.path.join(HERE, '..', 'course', 'mission4_3_clean_1cm.course.yaml')


@pytest.fixture
def course():
    return Course(copy.deepcopy(BASE))


def test_go_has_route_and_route_back(course):
    # 꼬리 윗길(-0.5, 1.0)에서 동쪽(J 쪽)을 보고, 고리 오른변 목표
    msg, res = plan_lane_cmd(course, (-0.5, 1.0, EAST), (1.02, 0.5), 'go', 7)
    assert msg['id'] == 7 and msg['cmd'] == 'go' and res['ok']
    assert (msg['x'], msg['y']) == (1.0, 0.5)                    # 길 위로 맞춘 목표
    r = msg['route']
    assert r['uturn'] is False and r['s_goal'] == pytest.approx(0.5 + 1.0 + 0.5)
    assert [(m['node'], m['turn'], m['follow']) for m in r['maneuvers']] == [('J', 'STRAIGHT', 'LEFT')]
    m = r['maneuvers'][0]
    assert m['s_at'] == pytest.approx(0.5) and m['radius'] == 0.2 and (m['x'], m['y']) == (0.0, 1.0)
    assert r['points'][0] == [-0.5, 1.0] and r['points'][-1] == [1.0, 0.5]
    b = msg['route_back']                                       # 일방통행 고리를 마저 돌아 J 에서 좌회전해 꼬리로
    assert [(m['node'], m['turn']) for m in b['maneuvers']] == [('J', 'LEFT')]
    assert b['points'][-1] == [-0.5, 1.0] and b['s_goal'] == pytest.approx(0.5 + 1.0 + 1.0 + 0.5)
    assert msg['arrive_tol'] == 0.05


def test_home_is_one_way(course):
    msg, _ = plan_lane_cmd(course, (-0.5, 1.0, EAST), (1.0, 0.5), 'home', 8)
    assert 'route_back' not in msg


def test_start_facing_dead_end_needs_uturn(course):
    msg, _ = plan_lane_cmd(course, (-0.5, 1.0, WEST), (1.0, 0.5), 'home', 9)
    assert msg['route']['uturn'] is True


@pytest.mark.parametrize('pose, goal, cmd, why', [
    (None, (1.0, 0.5), 'go', '위치를 모름'),
    ((-0.5, 1.5, EAST), (1.0, 0.5), 'go', '차선 위에 있지 않음'),
    ((-0.5, 1.0, NORTH), (1.0, 0.5), 'go', '수직'),
    ((-0.5, 1.0, EAST), (0.5, 0.5), 'go', '목표를 둘 수 없음'),           # 길에서 0.5 m
    ((-0.5, 1.0, EAST), (1.0, 0.5), 'cancel', '명령'),
])
def test_rejects_with_reason(course, pose, goal, cmd, why):
    with pytest.raises(Reject, match=why):
        plan_lane_cmd(course, pose, goal, cmd, 1)


def test_goal_in_junction_zone_is_moved(course):
    msg, res = plan_lane_cmd(course, (-0.5, 1.0, EAST), (0.05, 1.0), 'go', 2)
    assert math.hypot(msg['x'] - 0.0, msg['y'] - 1.0) > 0.2 and '옮김' in res['note']


def test_home_dock_is_not_moved_out_of_zones(course):
    msg, res = plan_lane_cmd(course, (-0.5, 1.0, EAST), (-0.3, 1.01), 'home', 4)    # 횡단보도 C1 구역 안의 도크
    assert (msg['x'], msg['y']) == (-0.3, 1.0) and res['note'] == ''


def test_uturn_refused_where_walls_are_close(course):
    with pytest.raises(Reject, match='U턴 불가'):
        plan_lane_cmd(course, (-0.5, 1.0, WEST), (1.0, 0.5), 'home', 3, clearance_fn=lambda x, y: 0.05)


def test_real_course_demo_a_routes():
    c = Course.load(REAL_COURSE)
    r1, r2 = c.points['R1'], c.points['R2']
    loop, tail = c.edges['loop'], c.edges['tail']
    g1 = loop.point_at(0.30 * loop.length)
    g2 = tail.point_at(0.55 * tail.length)
    m1, _ = plan_lane_cmd(c, (r1.x, r1.y, r1.yaw), g1, 'go', 1)
    m2, _ = plan_lane_cmd(c, (r2.x, r2.y, r2.yaw), g2, 'go', 2)
    assert [(m['node'], m['turn']) for m in m1['route']['maneuvers']] == [('J', 'STRAIGHT')]
    assert [(m['node'], m['turn']) for m in m2['route']['maneuvers']] == [('J', 'LEFT')]
    assert not m1['route']['uturn'] and not m2['route']['uturn']
    assert m1['route_back']['maneuvers'][-1]['turn'] == 'LEFT'   # 고리를 마저 돌아 J 에서 좌회전해 꼬리로
