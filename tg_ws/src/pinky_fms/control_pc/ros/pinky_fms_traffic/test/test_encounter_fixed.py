"""고정 로봇(차선 주행) vs Nav2 로봇 마주침 시험 (ROS 없이 시뮬레이터, encounter 방식)."""
import os

import numpy as np
import pytest

from pinky_fms_traffic.encounter import EncounterManager
from pinky_fms_traffic.gridmap import GridMap
from pinky_fms_traffic.sim import SimRobot, run
from pinky_fms_traffic.traffic import Agent

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'maps', 'mission4_3_clean_1cm.yaml')
E, W = (1.95, 1.0), (0.25, 1.0)          # scenarios.py 와 같은 통로 양 끝 (지도 좌표)


@pytest.fixture(scope='module')
def gm():
    return GridMap(MAP)


@pytest.mark.parametrize('lane_first', [True, False])
def test_nav_robot_yields_to_fixed_lane_robot(gm, lane_first):
    # S1 과 같은 정면 마주침인데 한 대가 차선 로봇(관제 명령을 따르지 않음)
    lane = SimRobot('lane', E, W, fixed=True) if lane_first else SimRobot('lane', W, E, fixed=True)
    nav = SimRobot('nav', W, E) if lane_first else SimRobot('nav', E, W)
    events = []
    res = run(gm, EncounterManager(gm), [lane, nav], T=200, log=lambda t, i, m: events.append((t, i, m)))
    assert res['result'] == 'OK' and res['collision_frames'] == 0, res
    assert res['waited']['nav'] > res['waited']['lane']          # 기다린(비킨) 쪽은 Nav2 로봇
    assert not any(i == 'lane' for _, i, _ in events)             # 고정 로봇에게 내린 명령은 없다


def test_fixed_robot_always_wins_and_is_never_swapped(gm):
    em = EncounterManager(gm)
    a = Agent('lane', np.array(E), np.linspace(E, W, 80), fixed=True)
    b = Agent('nav', np.array([1.45, 1.0]), np.linspace((1.45, 1.0), E, 30))
    for t in np.arange(0.0, 3.0, 0.5):
        cmds = em.decide([a, b], t)
    e = next(iter(em.enc.values()))
    assert e.winner == 'lane' and e.loser == 'nav'
    assert cmds['nav'].kind in ('YIELD', 'BACKUP', 'HOLD')


def test_two_fixed_robots_are_left_to_lane_traffic(gm):
    em = EncounterManager(gm)
    a = Agent('l1', np.array(E), np.linspace(E, W, 80), fixed=True)
    b = Agent('l2', np.array([1.5, 1.0]), np.linspace((1.5, 1.0), E, 30), fixed=True)
    em.decide([a, b], 0.0)
    assert em.enc == {}
