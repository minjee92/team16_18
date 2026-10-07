"""camera_view(녹화용 상속 노드) 시험. 로봇 전용 모듈(ultralytics, picamera2)은 가짜로 바꿔 PC 에서 돌린다.

    source /opt/ros/jazzy/setup.bash && source <tg_ws>/install/setup.bash && python3 -m pytest test
(set_lamp 서비스를 5 s 기다리므로 시험 하나에 몇 초 걸린다)
"""
import sys
import types

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('pinky_autonomous.autonomous_drive_node')


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


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setitem(sys.modules, 'ultralytics', types.SimpleNamespace(YOLO=lambda *a, **k: object()))
    monkeypatch.setitem(sys.modules, 'picamera2', types.SimpleNamespace(Picamera2=FakeCamera))
    from pinky_fms_lane_robot.camera_view import CameraViewNode
    rclpy.init(args=['--ros-args', '-r', '__ns:=/amr_99', '-p', 'enable_lcd:=false',
                     '-p', 'enable_return_home:=false', '-p', 'camera_warmup:=0.0'])
    n = CameraViewNode()
    n.destroy_timer(n.drive_timer)                 # 시험에서는 루프를 직접 부른다
    yield n
    n.shutdown_hardware()
    n.destroy_node()
    rclpy.try_shutdown()


def test_no_cmd_vel_publisher(node):
    names = [t for t, _ in node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())]
    assert '/amr_99/cmd_vel' not in names                       # 키보드 조종과 겹치지 않는다
    assert '/amr_99/camera/compressed' in names                 # 인식 화면은 그대로 낸다
    assert node.count_publishers('/amr_99/cmd_vel') == 0
    node.cmd_pub.publish(object())                               # 아무것도 하지 않는다 (예외 없음)


def test_drive_loop_clears_junction_latch(node, monkeypatch):
    from pinky_autonomous.autonomous_drive_node import AutonomousDriveNode
    seen = []
    monkeypatch.setattr(AutonomousDriveNode, '_drive_loop', lambda self: seen.append(self.junction_latched))
    node.junction_latched = True                                 # cross_lane 을 본 뒤라도
    node._drive_loop()
    assert seen == [False]                                       # 복귀 동작으로 넘어가지 않고 인식을 계속한다


def test_lamp_is_left_alone(node, monkeypatch):
    calls = []
    monkeypatch.setattr(node.lamp_cli, 'call_async', lambda req: calls.append(req))
    monkeypatch.setattr(node.lamp_cli, 'service_is_ready', lambda: True)
    node._apply_lamp((0, 255, 0, 1, 0))
    assert calls == []
