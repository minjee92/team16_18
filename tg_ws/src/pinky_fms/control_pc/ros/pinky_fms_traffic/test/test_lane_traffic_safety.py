"""lane_traffic 의 관제 살아있음 신호와 '주인 없는 HOLD' 경고 시험 (지도 없이, ROS 노드만)."""
import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from pinky_fms_traffic.lane_traffic import LaneRobot, LaneTraffic  # noqa: E402


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'orphan_warn:=0.0'])
    n = LaneTraffic()
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def test_heartbeat_counts_up(node):
    sent = []
    node.hb_pub = type('P', (), {'publish': lambda self, m: sent.append(json.loads(m.data))})()
    node._heartbeat()
    node._heartbeat()
    assert [d['seq'] for d in sent] == [1, 2]


def test_orphan_hold_is_reported_not_released(node):
    r = LaneRobot('amr_09')
    r.st, r.t = {'state': 'WAITING', 'traffic': 'HOLD', 'pose': [0.0, 0.0, 0.0]}, time.monotonic()
    node.robots['amr_09'] = r
    node._check_orphans(time.monotonic())
    assert 'amr_09' in node.orphans and r.cmd is None           # 경고만, resume 을 보내지 않는다
    r.st['traffic'] = None
    node._check_orphans(time.monotonic())
    assert 'amr_09' not in node.orphans
