"""tape.py(줄자 → 좌표)와 lane_map.py(GUI 표시용 지도) 시험 (ROS 없이 실행)."""
import copy
import os

import numpy as np
import pytest

from pinky_fms_lane.course import Course
from pinky_fms_lane.lane_map import OCCUPIED, ROAD, UNKNOWN, lane_image, rounded
from pinky_fms_lane.tape import Tape, TapeError, load_raw, measure_wall

HERE = os.path.dirname(os.path.abspath(__file__))
REAL_TAPE = os.path.join(HERE, '..', 'course', 'mission4_3_clean_1cm.tape.yaml')

# 방: 벽 안쪽 면 x = 0 ~ 2, y = 0 ~ 1
WALLS = {'left': {'value': 0.0, 'probe': {'x': 1.0, 'y': 0.5, 'dir': 'left'}},
         'right': {'value': 2.0, 'probe': {'x': 1.0, 'y': 0.5, 'dir': 'right'}},
         'top': {'value': 1.0, 'probe': {'x': 1.0, 'y': 0.5, 'dir': 'up'}},
         'bottom': {'value': 0.0, 'probe': {'x': 1.0, 'y': 0.5, 'dir': 'down'}}}
DATA = {
    'walls': WALLS,
    'roads': {'top_road': [{'num': 1, 'wall': 'top', 'a': 10.0, 'b': 26.0}],           # y = 1 - 0.18 = 0.82, 폭 16
              'right_road': [{'num': 2, 'wall': 'right', 'a': 12.0, 'b': 28.0}],       # x = 2 - 0.20 = 1.80
              'left_road': [{'num': 3, 'wall': 'left', 'a': 10.0, 'b': 24.0, 'at': 0.8},   # x = 0.17 (y 0.8)
                            {'num': 4, 'wall': 'left', 'a': 12.0, 'b': 26.0, 'at': 0.4}]},  # x = 0.19 (y 0.4)
    'marks': {'cw': {'num': 5, 'wall': 'left', 'dist': 100.0, 'length': 12.0}},       # x = 1.06
    'points': {'NE': {'x': 'right_road', 'y': 'top_road'},
               'CW': {'x': 'cw', 'y': 'top_road'},
               'L1': {'x': 'left_road', 'y': 0.6},                                     # 기울어진 길: y 로 x 를 구함
               'L2': {'x': 'left_road', 'y': 'top_road'}},
}


def test_points_from_raw_measurements():
    t = Tape(copy.deepcopy(DATA))
    assert t.points['NE'] == {'x': 1.8, 'y': 0.82, 'source': 'tape'}
    assert t.points['CW'] == {'x': 1.06, 'y': 0.82, 'source': 'tape'}
    assert t.points['L1'] == {'x': pytest.approx(0.18), 'y': 0.6, 'source': 'estimate'}
    assert t.points['L2']['x'] == pytest.approx(0.17 - 0.02 / 0.4 * 0.02) and t.points['L2']['source'] == 'tape'
    assert t.roads['top_road'][3][0][3] == pytest.approx(0.16)                 # 길 폭 = b - a


def test_mixed_list_is_rejected_unless_same_axis():
    with pytest.raises(TapeError, match='y 좌표라 x 에'):
        Tape(dict(copy.deepcopy(DATA), points={'P': {'x': 'top_road', 'y': 0.5}}))


@pytest.mark.parametrize('change, message', [
    ({'roads': {'r': [{'num': 1, 'wall': 'nowall', 'a': 1, 'b': 2}]}}, 'walls 에 있는 wall'),
    ({'roads': {'r': [{'num': 1, 'wall': 'top', 'a': 20, 'b': 10}]}}, 'b > a'),
    ({'roads': {'r': [{'num': 1, 'wall': 'left', 'a': 1, 'b': 2}, {'num': 2, 'wall': 'left', 'a': 1, 'b': 3}]}}, 'at'),
    ({'walls': {'w': {'value': 1.0, 'probe': {'dir': 'sideways'}}}}, 'probe.dir'),
    ({'points': {'P': {'x': 'nothing', 'y': 0.0}}}, 'nothing'),
    ({'points': {'P': {'x': 'left_road', 'y': 'left_road'}}}, '기울어진'),
])
def test_errors(change, message):
    data = copy.deepcopy(DATA)
    for k, v in change.items():
        data[k] = dict(data.get(k) or {}, **v)
    with pytest.raises(TapeError, match=message):
        Tape(data)


def test_measure_wall_finds_inner_faces():
    res, origin = 0.01, (-0.1, -0.1)                  # 방 2 x 1 m, 벽 두께 10 cm
    occ = np.zeros((120, 220), bool)
    occ[:10, :] = occ[-10:, :] = True
    occ[:, :10] = occ[:, -10:] = True
    for name, w in WALLS.items():
        assert measure_wall(occ, res, origin, w['probe']) == pytest.approx(w['value'], abs=1e-9), name
    assert measure_wall(np.zeros_like(occ), res, origin, WALLS['top']['probe']) is None


def test_real_tape_file_matches_hand_check():
    """사용자가 손으로 검산한 course_tape.json 값과 1 mm 안으로 같다 (2026-10-07)."""
    t = Tape(load_raw(REAL_TAPE))
    want = {'J': (1.152, 0.949), 'NE': (2.0, 0.946), 'SE': (2.0, 0.135), 'SW': (1.152, 0.135),
            'T_END': (0.95, 0.253), 'C1': (0.943, 0.952), 'C2': (2.0, 0.609),
            'T_A': (0.199, 0.952), 'T_B': (0.055, 0.822), 'T_C': (0.074, 0.328), 'T_D': (0.197, 0.253)}
    for name, (x, y) in want.items():
        p = t.points[name]
        assert abs(p['x'] - x) <= 0.0031 and abs(p['y'] - y) <= 0.0011, (name, p)   # T_END 만 x 3 mm (벽 면 0.953)
    assert {n for n, p in t.points.items() if p['source'] == 'tape'} == {'J', 'NE', 'SE', 'SW', 'T_END', 'C1', 'C2'}
    assert t.params['lane_width'] == 0.166
    assert [r[0] for r in t.roads['tail_left'][3]] == [3, 4]                   # 측정 번호가 남는다 (YAML 'no' 함정)


# ---------- GUI 표시용 지도 ----------
COURSE = {
    'params': {'lane_width': 0.16},
    'points': {'J': {'x': 1.0, 'y': 0.8}, 'NE': {'x': 1.8, 'y': 0.8}, 'SE': {'x': 1.8, 'y': 0.2},
               'SW': {'x': 1.0, 'y': 0.2}, 'TE': {'x': 0.3, 'y': 0.8}, 'C': {'x': 1.8, 'y': 0.5}},
    'edges': {'tail': {'points': ['J', 'TE']}, 'loop': {'points': ['J', 'NE', 'SE', 'SW', 'J'], 'oneway': True}},
    'junctions': {'J': {'moves': [
        {'from': 'tail', 'to': 'loop', 'turn': 'STRAIGHT', 'follow': 'LEFT', 'follow_dist': 0.3},
        {'from': 'loop', 'to': 'tail', 'turn': 'LEFT', 'follow': 'LEFT', 'follow_dist': 0.3},
        {'from': 'loop', 'to': 'loop', 'turn': 'RIGHT', 'follow': 'RIGHT', 'follow_dist': 0.3}]}},
    'crosswalks': {'C': {'point': 'C', 'radius': 0.1}},
}


def pixel(img, x, y, res=0.01, origin=(0.0, 0.0)):
    return img[img.shape[0] - 1 - int((y - origin[1]) / res), int((x - origin[0]) / res)]


def test_lane_image_paints_road_unknown_walls_and_stripes():
    occ = np.zeros((100, 200), bool)                  # 2 x 1 m, 원점 (0, 0), 행 0 = 맨 위
    occ[0, :] = True                                  # 맨 위 줄 = 벽
    img = lane_image(Course(copy.deepcopy(COURSE)), occ, 0.01, (0.0, 0.0), 0.12)
    assert pixel(img, 0.6, 0.8) == ROAD and pixel(img, 0.6, 0.87) == ROAD     # 꼬리 가운데, 폭 안
    assert pixel(img, 0.6, 0.9) == UNKNOWN and pixel(img, 0.5, 0.5) == UNKNOWN  # 길 밖
    assert pixel(img, 0.25, 0.8) == UNKNOWN           # 막다른 끝은 평평하게 끝난다 (둥글게 튀어나오지 않음)
    assert pixel(img, 1.85, 0.85) == ROAD             # 고리 모서리 바깥은 둥글게 이어진다
    assert pixel(img, 1.0, 0.99) == OCCUPIED          # 벽은 그대로
    col = [pixel(img, x, 0.5) for x in np.arange(1.73, 1.875, 0.01)]          # 횡단보도 C 를 가로질러 보면 줄무늬
    assert col.count(UNKNOWN) >= 5 and col.count(ROAD) >= 5


def test_rounded_corner_stays_near_corner():
    pts = rounded([(0, 0), (1, 0), (1, 1)], 0.08)
    assert np.allclose(pts[0], (0, 0)) and np.allclose(pts[-1], (1, 1))
    assert np.max(np.hypot(pts[1:-1, 0] - 1, pts[1:-1, 1])) <= 0.08 + 1e-9             # 둥근 부분은 꼭짓점 0.08 m 안
    assert not any(np.allclose(p, (1, 0)) for p in pts)                       # 꼭짓점은 잘려 나간다
    assert len(rounded([(0, 0), (1, 0), (1, 1)], 0.0)) == 3                   # 0 이면 그대로
