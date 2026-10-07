"""fms_lane_mission 시험 (PC 에서, 카메라·YOLO 는 가짜). 로봇에서만 되는 차선 인식·주행은 시험하지 않는다.

    source /opt/ros/jazzy/setup.bash && source <tg_ws>/install/setup.bash && python3 -m pytest test
(pinky_autonomous 가 set_lamp 서비스를 5 s 기다리므로 노드 하나 만드는 데 몇 초 걸린다)
"""
import importlib.util
import json
import os
import sys
import time
import types

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('pinky_autonomous.autonomous_drive_node')
from std_msgs.msg import String  # noqa: E402

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'fms_lane_mission.py')


class FakeCamera:
    def create_video_configuration(self, **kw):
        return kw

    create_preview_configuration = create_video_configuration

    def configure(self, cfg):
        pass

    def start(self):
        pass

    def capture_array(self):
        return np.zeros((480, 640, 3), np.uint8)

    def stop(self):
        pass

    def close(self):
        pass


def load_module():
    spec = importlib.util.spec_from_file_location('fms_lane_mission_under_test', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def mod():
    return load_module()


def make_node(mod, monkeypatch, **params):
    monkeypatch.setitem(sys.modules, 'ultralytics', types.SimpleNamespace(YOLO=lambda *a, **k: object()))
    monkeypatch.setitem(sys.modules, 'picamera2', types.SimpleNamespace(Picamera2=FakeCamera))
    args = ['--ros-args', '-r', '__ns:=/amr_98', '-p', 'enable_lcd:=false', '-p', 'camera_warmup:=0.0']
    for k, v in params.items():
        args += ['-p', f'{k}:={v}']
    rclpy.init(args=args)
    n = mod.FmsLaneMission()
    n.destroy_timer(n.drive_timer)          # 시험에서는 루프를 직접 부른다
    sent = []
    n.cmd_pub = types.SimpleNamespace(publish=sent.append, get_subscription_count=lambda: 0)
    n._sent = sent
    return n


@pytest.fixture
def cleanup():
    nodes = []
    yield nodes
    for n in nodes:
        n.destroy_node()
    rclpy.try_shutdown()


def start_mission(n):
    n.goal, n.result = (1.0, 0.5), None
    n.start_pose = (0.0, 0.0, 0.0)


def test_defaults_keep_old_behavior_except_requested(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch)
    cleanup.append(n)
    assert n.link_timeout == 0.0             # 노드 기본: 관제 신호 검사 끔 (robot_lane.launch.xml 이 2.0 으로 켬)
    assert n.traffic_cmd_timeout == 0.0      # 10 s 자동 재개 끔 (사용자 결정 2026-10-08)


def test_link_lost_holds_then_resumes_and_shifts_timers(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch, fms_link_timeout=0.5)
    cleanup.append(n)
    called = []
    monkeypatch.setattr(mod.AutonomousDriveNode, '_drive_loop', lambda self: called.append('drive'))
    start_mission(n)
    n.leg_t0 = n.return_t0 = 100.0
    n._drive_loop()                          # 신호를 한 번도 못 받음 → 정지
    assert called == [] and n.link_hold_t is not None and n._sent and n._sent[-1].linear.x == 0.0
    st = []
    n.status_pub = types.SimpleNamespace(publish=lambda m: st.append(json.loads(m.data)))
    n._publish_status()
    assert st[-1]['state'] == 'WAITING' and st[-1]['detail'] == 'fms link lost' and st[-1]['link_ok'] is False
    n.link_hold_t -= 3.0                     # 3 s 동안 멈춰 있었던 것으로
    n._on_heartbeat(String(data='{"seq": 1}'))
    n._drive_loop()                          # 신호 복구 → 이어서 진행
    assert called == ['drive'] and n.link_hold_t is None
    assert n.leg_t0 == pytest.approx(103.0, abs=0.5) and n.return_t0 == pytest.approx(103.0, abs=0.5)
    assert n.ramp_t0 is None                 # 다시 천천히 출발


def test_link_check_ignored_when_idle_or_disabled(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch, fms_link_timeout=0.5)
    cleanup.append(n)
    assert n._link_check(time.monotonic()) is False      # 미션이 없으면 검사하지 않는다
    n.link_timeout = 0.0
    start_mission(n)
    assert n._link_check(time.monotonic()) is False      # 꺼져 있으면 검사하지 않는다


def test_hold_is_not_released_by_timeout_by_default(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch)
    cleanup.append(n)
    start_mission(n)
    n.traffic, n.tr_rx = 'HOLD', time.monotonic() - 60.0
    n._traffic_step()
    assert n.traffic == 'HOLD'               # 60 s 명령이 없어도 그대로 (resume·cancel 로만 풀림)
    n.traffic_cmd_timeout = 10.0             # 예전 동작
    n._traffic_step()
    assert n.traffic is None


def test_hold_released_by_resume_and_cancel(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch)
    cleanup.append(n)
    start_mission(n)
    n._on_traffic_cmd(String(data=json.dumps({'id': 1, 'cmd': 'hold'})))
    assert n.traffic == 'HOLD'
    n._on_traffic_cmd(String(data=json.dumps({'id': 2, 'cmd': 'resume', 'role': 'priority'})))
    assert n.traffic is None
    n._on_traffic_cmd(String(data=json.dumps({'id': 3, 'cmd': 'hold'})))
    n._on_cmd(String(data=json.dumps({'id': 9, 'cmd': 'cancel'})))
    assert n.traffic is None and n.goal is None
