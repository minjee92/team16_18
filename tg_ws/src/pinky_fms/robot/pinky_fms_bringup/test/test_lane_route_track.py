"""lane_route_track.RouteTrack 시험 (ROS 없음)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from lane_route_track import RouteError, RouteTrack  # noqa: E402

# ㄱ자 경로: (0,0) → (1,0) → (1,1). 갈림길 J 는 (1,0) (s_at 1.0, 반지름 0.2)
ROUTE = {'points': [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], 's_goal': 1.8, 'uturn': False,
         'maneuvers': [{'node': 'J', 'x': 1.0, 'y': 0.0, 's_at': 1.0, 'radius': 0.2, 'turn': 'LEFT',
                        'follow': 'LEFT', 'follow_dist': 0.3}]}


def test_progress_and_lateral():
    t = RouteTrack(ROUTE)
    assert t.length == pytest.approx(2.0)
    s, lat = t.update(0.5, 0.03)
    assert s == pytest.approx(0.5) and lat == pytest.approx(0.03)
    s, lat = t.update(1.02, 0.5)
    assert s == pytest.approx(1.5) and lat == pytest.approx(0.02)


def test_maneuver_starts_at_zone_entry_and_skips_passed():
    t = RouteTrack(ROUTE)
    t.update(0.7, 0.0)
    assert t.due_maneuver() is None
    t.update(0.85, 0.0)
    assert t.due_maneuver()['turn'] == 'LEFT'          # 구역(1.0 - 0.2) 안
    t.done_maneuver()
    assert t.due_maneuver() is None and t.next_node() is None
    t2 = RouteTrack(ROUTE)
    t2.update(0.5, 0.0)
    t2.update(1.0, 0.5)                                # 양보 뒤 갈림길을 지나친 자리로 복귀 (전체에서 다시 찾음)
    assert t2.s == pytest.approx(1.5) and t2.due_maneuver() is None   # 늦게 꺾지 않는다


def test_window_prefers_nearby_segment():
    # 왕복처럼 같은 선을 두 번 지나는 경로: 지금 진행 근처를 고른다
    t = RouteTrack({'points': [[0, 0], [1, 0], [1, 0.05], [0, 0.05]], 's_goal': 2.05})
    t.update(0.5, 0.0)
    assert t.s == pytest.approx(0.5)
    t.update(0.6, 0.02)
    assert t.s == pytest.approx(0.6, abs=0.01)         # 0.05 m 옆 돌아오는 구간(s≈1.45)으로 튀지 않는다


def test_arrival_by_progress():
    t = RouteTrack(ROUTE, arrive_tol=0.05)
    t.update(1.0, 0.70)
    assert not t.arrived()
    t.update(1.0, 0.76)
    assert t.arrived()


@pytest.mark.parametrize('bad', [
    {'points': [[0, 0]]},
    {'points': [[0, 0], [1, 'x']]},
    {'points': [[0, 0], [1, 0]], 'maneuvers': [{'turn': 'UP', 'follow': 'LEFT', 's_at': 0.5}]},
    {'points': [[0, 0], [float('nan'), 0]]},
    'not a dict',
])
def test_bad_route_is_rejected(bad):
    with pytest.raises(RouteError):
        RouteTrack(bad)
