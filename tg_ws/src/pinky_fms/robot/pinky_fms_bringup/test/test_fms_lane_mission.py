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
from sensor_msgs.msg import Range  # noqa: E402
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
    assert n.sonar_zero_stop == 3


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


def sonar(n, r, k=1):
    for _ in range(k):
        m = Range()
        m.range, m.max_range = r, 2.0
        n._sonar_callback(m)


def test_sonar_zero_ignored_once_but_stops_after_n(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch, sonar_zero_stop_count=3)
    cleanup.append(n)
    n.lidar_t = 0.0                          # 라이다 없음 (초음파만 본다)
    sonar(n, 0.5, 3)
    now = time.monotonic()
    assert n._sonar_distance(now) == pytest.approx(0.5 * n.sonar_scale + n.sonar_offset)
    sonar(n, 0.0, 2)                         # 중앙값도 0 이 되지만 연속 2번 → 아직 무시
    assert n._sonar_distance(time.monotonic()) == float('inf')
    sonar(n, 0.0, 1)                         # 3번 연속 → 사물로 보고 0 m
    assert n._sonar_distance(time.monotonic()) == 0.0
    sonar(n, 0.5, 3)                         # 정상 값이 오면 풀린다
    assert n.sonar_zero_streak == 0 and n._sonar_distance(time.monotonic()) > 0.5
    n.sonar_zero_stop = 0                    # 예전 동작: 항상 무시
    sonar(n, 0.0, 5)
    assert n._sonar_distance(time.monotonic()) == float('inf')


# ---------- 관제 경로 모드 ----------
ROUTE = {'points': [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], 's_goal': 1.8, 'uturn': False,
         'maneuvers': [{'node': 'J', 'x': 1.0, 'y': 0.0, 's_at': 1.0, 'radius': 0.2, 'turn': 'LEFT',
                        'follow': 'RIGHT', 'follow_dist': 0.3}]}
BACK = {'points': [[1.0, 0.8], [1.0, 0.0], [0.0, 0.0]], 's_goal': 1.8, 'uturn': True, 'maneuvers': []}


def route_node(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch)
    cleanup.append(n)
    monkeypatch.setattr(mod.AutonomousDriveNode, '_drive_loop', lambda self: None)
    n._on_cmd(String(data=json.dumps({'id': 5, 'cmd': 'go', 'x': 1.0, 'y': 0.8, 'route': ROUTE,
                                      'route_back': BACK, 'arrive_tol': 0.05})))
    n.start_pose = (0.0, 0.0, 0.0)
    return n


def test_route_fields_are_used(mod, monkeypatch, cleanup):
    n = route_node(mod, monkeypatch, cleanup)
    assert n.track is n.route_out and n.route_back is not None and n.goal == (1.0, 0.8)
    turns = []
    monkeypatch.setattr(n, '_begin_turn', lambda choice, pose, now: turns.append(choice))
    n.departing = True
    n._plan_exit(time.monotonic())               # 출발 방향: 경로의 uturn=False → 그대로 출발 (지도 판단 안 함)
    assert turns == ['straight']


def test_route_maneuver_starts_on_zone_entry_and_cross_lane_only_confirms(mod, monkeypatch, cleanup):
    n = route_node(mod, monkeypatch, cleanup)
    pose = [(0.5, 0.0, 0.0)]
    monkeypatch.setattr(n, '_current_pose', lambda: pose[0])
    n.departing, n.return_phase, n.junction_latched = False, 'following', False
    n._drive_loop()
    assert n.lane_bias is None                   # 아직 J 구역 밖
    pose[0] = (0.85, 0.0, 0.0)
    n._drive_loop()
    assert n.exit_choice == 'left' and n.lane_bias == 'right'          # 동작은 LEFT, 따라갈 테이프는 RIGHT
    assert n.bias_dist == pytest.approx(0.15 + 0.3) and n.return_phase == 'following'
    # cross_lane 이 보여도 갈림길 절차(정지·판단)를 시작하지 않고 확인만 기록
    n.junction_frames = 0
    for _ in range(5):
        n._decide_state(time.monotonic(), 0.0, 0.9)
    assert n.junction_latched is False and n.route_maneuver['confirmed'] is True


def test_route_arrival_switches_to_route_back(mod, monkeypatch, cleanup):
    n = route_node(mod, monkeypatch, cleanup)
    pose = [(1.0, 0.5, 1.57)]
    monkeypatch.setattr(n, '_current_pose', lambda: pose[0])
    n.departing, n.return_phase = False, 'following'
    n.track.update(0.9, 0.0)
    n.track.done_maneuver()
    n._drive_loop()
    assert n.leg == 'outbound'                   # s=1.5, 아직 (반경 0.30 이면 이미 도착으로 봤을 거리)
    pose[0] = (1.0, 0.77, 1.57)
    n._drive_loop()
    assert n.leg == 'inbound' and n.track is n.route_back and n.route_turn_pending
    turns = []
    monkeypatch.setattr(n, '_begin_turn', lambda choice, p, now: turns.append(choice))
    n._plan_exit(time.monotonic())               # 돌아오는 길 uturn=True → 제자리 U턴
    assert turns == ['back'] and not n.route_turn_pending


def test_without_route_fields_old_behavior(mod, monkeypatch, cleanup):
    n = make_node(mod, monkeypatch)
    cleanup.append(n)
    n._on_cmd(String(data=json.dumps({'id': 6, 'cmd': 'go', 'x': 1.0, 'y': 0.8})))
    assert n.track is None and n.route_out is None
    n.wall_grid = None
    assert n._arrived_at((1.0, 0.6, 0.0), (1.0, 0.8), 0.30)            # 예전 반경 판정
    n._on_cmd(String(data=json.dumps({'id': 7, 'cmd': 'go', 'x': 1.0, 'y': 0.8, 'route': {'points': 'bad'}})))
    assert n.track is None and n.goal == (1.0, 0.8)                    # 잘못된 경로는 버리고 예전 방식
