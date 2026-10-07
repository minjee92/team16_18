"""course.py 시험 (ROS 없이 실행: 패키지 폴더에서 python3 -m pytest test).

좌표에 기대지 않도록 시험용 코스를 따로 쓴다: 1 m 정사각형 고리(J 가 왼쪽 위, 시계방향) + ㄱ자 꼬리(막다른 길).
실제 코스 파일(course/*.course.yaml)은 읽히고 검사를 통과하는지만 본다.
"""
import copy
import math
import os

import numpy as np
import pytest
import yaml

from pinky_fms_lane.course import Course, CourseError, PlanError

HERE = os.path.dirname(os.path.abspath(__file__))
REAL_COURSE = os.path.join(HERE, '..', 'course', 'mission4_3_clean_1cm.course.yaml')

BASE = {
    'params': {'snap_max': 0.15, 'dead_end_clearance': 0.15, 'wait_keepout': 0.12, 'uturn_clearance': 0.10,
               'lane_width': 0.14},
    'points': {
        'J': {'x': 0.0, 'y': 1.0},
        'NE': {'x': 1.0, 'y': 1.0}, 'SE': {'x': 1.0, 'y': 0.0}, 'SW': {'x': 0.0, 'y': 0.0},
        'TA': {'x': -1.0, 'y': 1.0}, 'TB': {'x': -1.0, 'y': 0.0}, 'TE': {'x': -0.4, 'y': 0.0},
        'WL': {'x': 0.0, 'y': 0.6}, 'WT': {'x': -0.6, 'y': 1.0},
        'C1': {'x': -0.3, 'y': 1.0},
    },
    'edges': {
        'tail': {'points': ['J', 'TA', 'TB', 'TE'], 'oneway': False},     # 길이 2.6 (J→TA 1.0, TA→TB 1.0, TB→TE 0.6)
        'loop': {'points': ['J', 'NE', 'SE', 'SW', 'J'], 'oneway': True},  # 길이 4.0, 시계방향
    },
    'junctions': {'J': {'radius': 0.2, 'moves': [
        {'from': 'tail', 'to': 'loop', 'turn': 'STRAIGHT', 'follow': 'LEFT', 'follow_dist': 0.3},
        {'from': 'loop', 'to': 'tail', 'turn': 'LEFT', 'follow': 'LEFT', 'follow_dist': 0.3},
        {'from': 'loop', 'to': 'loop', 'turn': 'RIGHT', 'follow': 'RIGHT', 'follow_dist': 0.3},
    ]}},
    'waits': {'W_LOOP': {'point': 'WL'}, 'W_TAIL': {'point': 'WT'}},
    'crosswalks': {'C1': {'point': 'C1', 'radius': 0.08}},
}
EAST, NORTH, WEST, SOUTH = 0.0, math.pi / 2, math.pi, -math.pi / 2


def make(**changes):
    data = copy.deepcopy(BASE)
    for path, value in changes.items():
        node = data
        keys = path.split('__')
        for k in keys[:-1]:
            node = node[k]
        if value is None:
            del node[keys[-1]]
        else:
            node[keys[-1]] = value
    return Course(data)


@pytest.fixture
def course():
    return Course(copy.deepcopy(BASE))


def polyline_length(pts):
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))


# ---------- 읽기 / 검사 ----------
def test_loads_edges_and_waits(course):
    assert course.edges['tail'].length == pytest.approx(2.6)
    assert course.edges['loop'].length == pytest.approx(4.0)
    assert (course.waits['W_LOOP'].edge, course.waits['W_LOOP'].s) == ('loop', pytest.approx(3.6))
    assert (course.waits['W_TAIL'].edge, course.waits['W_TAIL'].s) == ('tail', pytest.approx(0.6))
    assert len(course.estimate_points()) == len(BASE['points'])     # source 를 안 적으면 추정값


TAPE = {                                       # 시험용 줄자 파일: 벽 x=-1.2(왼쪽), y=1.2(위). J 를 줄자로 잰다
    'walls': {'left': {'value': -1.2, 'probe': {'x': 0, 'y': 0.5, 'dir': 'left'}},
              'top': {'value': 1.2, 'probe': {'x': 0, 'y': 0.5, 'dir': 'up'}}},
    'roads': {'tail_top': [{'num': 1, 'wall': 'top', 'a': 12.0, 'b': 26.0}]},        # 가운데 y = 1.2 - 0.19 = 1.01
    'marks': {'jx': {'num': 2, 'wall': 'left', 'dist': 115.0, 'length': 10.0}},     # 가운데 x = -1.2 + 1.2 = 0.0
    'params': {'lane_width': 0.16},
    'points': {'J': {'x': 'jx', 'y': 'tail_top'}, 'TA': {'x': -1.0, 'y': 'tail_top'},
               'R1': {'x': -0.7, 'y': 'tail_top'}},
}


def write_course(tmp_path, tape=None, robot=None, **extra):
    data = copy.deepcopy(BASE)
    data['points']['R1'] = {'x': -0.7, 'y': 1.0, 'yaw': WEST}
    data.update(tape='c.tape.yaml', robot='c.robot.yaml', **extra)
    if tape is not None:
        (tmp_path / 'c.tape.yaml').write_text(yaml.safe_dump(tape, allow_unicode=True))
    if robot is not None:
        (tmp_path / 'c.robot.yaml').write_text(yaml.safe_dump(robot, allow_unicode=True))
    (tmp_path / 'c.yaml').write_text(yaml.safe_dump(data, allow_unicode=True))
    return str(tmp_path / 'c.yaml')


def test_tape_file_overrides_estimates(tmp_path):
    c = Course.load(write_course(tmp_path, tape=TAPE))
    j, ta, r1 = c.points['J'], c.points['TA'], c.points['R1']
    assert (j.x, j.y, j.source) == (pytest.approx(0.0), pytest.approx(1.01), 'tape')
    assert (ta.x, ta.y, ta.source) == (-1.0, pytest.approx(1.01), 'estimate')    # 숫자(추정)가 섞이면 estimate
    assert r1.yaw == WEST                                     # 줄자는 방향을 재지 않음: 코스 설정의 방향 유지
    assert (c.params['lane_width'], c.param_sources['lane_width']) == (0.16, 'tape')
    assert c.source_summary()['tape'] == ['J'] and 'TA' in c.source_summary()['estimate']


def test_robot_file_overrides_tape(tmp_path):
    robot = {'points': {'J': {'x': 0.01, 'y': 1.02, 'stamp': '2026-10-07T10:00:00'}}, 'params': {'lane_width': 0.17}}
    c = Course.load(write_course(tmp_path, tape=TAPE, robot=robot))
    j = c.points['J']
    assert (j.x, j.y, j.source, j.stamp) == (0.01, 1.02, 'robot', '2026-10-07T10:00:00')
    assert c.points['R1'].source == 'estimate'                # R1 은 줄자 파일에서 x 가 숫자(추정)
    assert (c.params['lane_width'], c.param_sources['lane_width']) == (0.17, 'robot')
    assert list(c.source_summary()) == ['robot', 'tape', 'estimate']          # 우선순위 높은 것부터


def test_missing_tape_and_robot_files_are_fine(tmp_path):
    c = Course.load(write_course(tmp_path))
    assert c.points['J'].source == 'estimate' and c.param_sources['lane_width'] == 'estimate'


@pytest.mark.parametrize('tape, robot, message', [
    (dict(TAPE, points={'NOPE': {'x': 0.0, 'y': 'tail_top'}}), None, '줄자 파일.*NOPE'),
    (None, {'points': {'NOPE': {'x': 0, 'y': 0}}}, '로봇 기록 파일.*NOPE'),
    (dict(TAPE, points={'J': {'x': 'tail_top', 'y': 1.0}}), None, 'y 좌표라 x 에'),
    (None, {'params': {'speed': 1.0}}, 'params.speed'),
])
def test_override_file_errors(tmp_path, tape, robot, message):
    with pytest.raises(CourseError, match=message):
        Course.load(write_course(tmp_path, tape=tape, robot=robot))


def test_course_file_points_must_be_estimates():
    with pytest.raises(CourseError, match='추정값'):
        make(points__J={'x': 0.0, 'y': 1.0, 'source': 'tape'})


@pytest.mark.parametrize('changes, message', [
    ({'edges__tail__points': ['J', 'TA', 'NOWHERE']}, 'NOWHERE'),
    ({'edges__loop__oneway': False}, '일방통행'),
    ({'junctions__J__moves': BASE['junctions']['J']['moves'][:1] + BASE['junctions']['J']['moves'][2:]},
     'tail 로 나가는 moves'),
    ({'points__WL': {'x': 0.0, 'y': 0.9}}, 'W_LOOP: 갈림길 J'),
    ({'points__WL': {'x': 0.3, 'y': 0.6}}, '길 가운데 선'),
    ({'junctions__J__moves': [dict(BASE['junctions']['J']['moves'][0], turn='straight'),
                              *BASE['junctions']['J']['moves'][1:]]}, 'turn 은'),
    ({'params__speed': 1.0}, 'params.speed'),
    ({'points__J': {'x': 0.0, 'y': 1.0, 'source': 'guess'}}, 'source'),
    ({'junctions__J': None}, 'junctions 에 있어야 함'),
])
def test_validation_errors(changes, message):
    with pytest.raises(CourseError, match=message):
        make(**changes)


# ---------- 위치 ----------
def test_locate_sets_direction_from_yaw(course):
    loc = course.locate(-0.5, 1.02, EAST)                 # 꼬리 윗길에서 동쪽(J 쪽) = 꼬리 - 방향
    assert (loc.edge, loc.dir) == ('tail', -1)
    assert loc.s == pytest.approx(0.5) and loc.dist == pytest.approx(0.02) and loc.cross < 0
    assert course.locate(-0.5, 1.02, WEST).dir == 1
    assert course.locate(-0.5, 1.02, NORTH).dir == 0      # 길과 수직이면 방향을 모름
    assert course.locate(-0.5, 1.5) is None               # 길에서 멀면 없음


def test_locate_near_junction_prefers_edge_matching_heading(course):
    loc = course.locate(0.0, 0.9, NORTH)                  # 왼변을 따라 J 로 올라가는 중
    assert (loc.edge, loc.dir) == ('loop', 1) and loc.s == pytest.approx(3.9)
    assert course.locate(0.05, 1.0, EAST).edge in ('loop', 'tail')


# ---------- 목표 ----------
def test_snap_goal_on_road(course):
    loc, note = course.snap_goal(0.5, 0.05)
    assert (loc.edge, note) == ('loop', '') and loc.s == pytest.approx(2.5)


def test_snap_goal_rejects_far_click(course):
    loc, note = course.snap_goal(0.5, 0.5)
    assert loc is None and '0.50 m' in note


def test_snap_goal_moves_out_of_junction_to_nearest_allowed_point(course):
    loc, note = course.snap_goal(0.05, 1.0)               # J 에서 5 cm
    assert loc.edge == 'loop' and loc.s == pytest.approx(0.21)
    assert '갈림길 J' in note
    assert math.hypot(loc.x - 0.0, loc.y - 1.0) > 0.2


def test_snap_goal_moves_away_from_dead_end(course):
    loc, note = course.snap_goal(-0.42, 0.0)
    assert loc.edge == 'tail' and loc.s == pytest.approx(2.6 - 0.15 - 0.01)
    assert '막다른 끝' in note


def test_snap_goal_result_is_never_inside_a_zone(course):
    for x, y in [(0.0, 0.62), (-0.3, 1.0), (-0.6, 1.0), (0.0, 0.85), (-0.15, 1.0), (1.0, 0.5)]:
        loc, _ = course.snap_goal(x, y)
        assert loc is not None
        assert all(not (lo - 1e-9 <= loc.s <= hi + 1e-9) for lo, hi, _ in course.zones(loc.edge)), (x, y, loc)


# ---------- 경로 ----------
def test_plan_tail_to_loop_goes_straight_at_junction(course):
    start = course.locate(-0.7, 0.0, WEST)                # 꼬리 아랫길, 출구 쪽을 봄
    goal, _ = course.snap_goal(1.0, 0.5)
    r = course.plan(start, goal)
    assert not r.uturn and r.length == pytest.approx(2.3 + 1.5)
    assert [(lg.edge, lg.dir) for lg in r.legs] == [('tail', -1), ('loop', 1)]
    assert [(m.node_id, m.turn, m.follow) for m in r.maneuvers] == [('J', 'STRAIGHT', 'LEFT')]
    assert r.maneuvers[0].s_at == pytest.approx(2.3)
    assert np.allclose(r.points, [(-0.7, 0), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0.5)])
    assert polyline_length(r.points) == pytest.approx(r.length)


def test_plan_loop_to_tail_turns_left(course):
    r = course.plan(course.locate(0.5, 0.0, WEST), course.location_at('tail', 1.5))
    assert [(m.turn, m.follow) for m in r.maneuvers] == [('LEFT', 'LEFT')]
    assert r.length == pytest.approx(1.5 + 1.5) and r.maneuvers[0].s_at == pytest.approx(1.5)


def test_plan_goal_behind_on_loop_goes_around_through_junction(course):
    r = course.plan(course.locate(0.5, 0.0, WEST), course.location_at('loop', 0.5))
    assert r.length == pytest.approx(1.5 + 0.5)
    assert [(m.turn, m.follow) for m in r.maneuvers] == [('RIGHT', 'RIGHT')]


def test_plan_goal_ahead_on_same_edge(course):
    r = course.plan(course.locate(0.2, 1.0, EAST), course.location_at('loop', 0.8))
    assert r.length == pytest.approx(0.6) and r.maneuvers == [] and len(r.legs) == 1


def test_plan_needs_uturn_when_facing_dead_end(course):
    start = course.locate(-0.7, 0.0, EAST)                # 꼬리 안쪽(막다른 끝)을 봄
    goal = course.location_at('loop', 1.5)
    with pytest.raises(PlanError, match='U턴'):
        course.plan(start, goal, allow_uturn=False)
    r = course.plan(start, goal, allow_uturn=True)
    assert r.uturn and r.legs[0].dir == -1


def test_plan_without_uturn_may_go_around_loop_instead(course):
    start = course.locate(-1.0, 0.5, NORTH)               # 꼬리 왼쪽 세로 구간, 출구 쪽을 봄
    goal = course.location_at('tail', 2.3)                # 등 뒤 (더 안쪽)
    r = course.plan(start, goal, allow_uturn=False)       # U턴 없이: 나가서 고리를 한 바퀴 돌고 다시 들어온다
    assert not r.uturn and r.length == pytest.approx(1.5 + 4.0 + 2.3)
    assert [m.turn for m in r.maneuvers] == ['STRAIGHT', 'LEFT']
    r = course.plan(start, goal, allow_uturn=True)        # U턴이 되면 그 자리에서 돌아 0.8 m
    assert r.uturn and r.length == pytest.approx(0.8)


def test_plan_wrong_way_on_one_way_loop(course):
    start = course.locate(0.5, 0.0, EAST)                 # 아랫변에서 반시계 방향을 봄
    with pytest.raises(PlanError):
        course.plan(start, course.location_at('loop', 3.0), allow_uturn=False)
    assert course.plan(start, course.location_at('loop', 3.0), allow_uturn=True).uturn


def test_plan_needs_known_direction(course):
    with pytest.raises(PlanError, match='방향'):
        course.plan(course.locate(-0.5, 1.0, NORTH), course.location_at('loop', 1.0))


def test_route_maps_between_route_and_edge_positions(course):
    r = course.plan(course.locate(-0.7, 0.0, WEST), course.location_at('loop', 1.5))
    assert r.locate_s(0.0) == ('tail', pytest.approx(2.3))
    assert r.locate_s(3.0) == ('loop', pytest.approx(0.7))
    assert r.s_of('loop', 1.0) == pytest.approx(3.3)
    assert r.s_of('tail', 2.0) == pytest.approx(0.3)
    assert r.s_of('tail', 2.5) is None                    # 출발점보다 안쪽은 지나지 않음


def test_can_uturn(course):
    assert course.can_uturn(0.05, 1.0)[0] is False        # 갈림길 구역 안
    assert course.can_uturn(-0.7, 0.0, lambda x, y: 0.05)[0] is False
    assert course.can_uturn(-0.7, 0.0, lambda x, y: 0.2) == (True, '')
    assert course.can_uturn(-0.7, 0.0) == (True, '')


# ---------- 실제 코스 파일 ----------
def test_real_course_file_loads_and_plans():
    c = Course.load(REAL_COURSE)
    assert set(c.edges) == {'tail', 'loop'} and set(c.waits) == {'W_LOOP', 'W_TAIL'}
    assert len(c.junctions['J'].moves) == 3
    r1, r2 = c.points['R1'], c.points['R2']
    start1 = c.locate(r1.x, r1.y, r1.yaw)
    start2 = c.locate(r2.x, r2.y, r2.yaw)
    assert start1.edge == 'tail' and start2.edge == 'loop'
    route1 = c.plan(start1, c.location_at('loop', 0.3 * c.edges['loop'].length), allow_uturn=True)
    assert [(m.node_id, m.turn) for m in route1.maneuvers] == [('J', 'STRAIGHT')]
    route2 = c.plan(start2, c.location_at('tail', 0.55 * c.edges['tail'].length), allow_uturn=True)
    assert [(m.node_id, m.turn) for m in route2.maneuvers] == [('J', 'LEFT')]


def test_real_course_uses_tape_measurements():
    c = Course.load(REAL_COURSE)
    src = c.source_summary()
    assert set(src['tape']) == {'J', 'T_END', 'NE', 'SE', 'SW', 'C1', 'C2'}
    assert {'T_A', 'T_B', 'T_C', 'T_D', 'W_LOOP', 'W_TAIL', 'R1', 'R2'} <= set(src['estimate'])
    assert (c.params['lane_width'], c.param_sources['lane_width']) == (0.166, 'tape')
    # 설계값 규칙: 출발점은 목표로 써도 옮겨지지 않고(구역 밖), 대기 지점은 갈림길·횡단보도 구역 밖
    for name in ('R1', 'R2'):
        p = c.points[name]
        goal, note = c.snap_goal(p.x, p.y)
        assert goal is not None and note == '' and goal.dist < 0.005, (name, note)
    for name, loc in c.waits.items():
        for x, y, r in c.crosswalks.values():
            assert math.hypot(loc.x - x, loc.y - y) > r + 0.06, name
