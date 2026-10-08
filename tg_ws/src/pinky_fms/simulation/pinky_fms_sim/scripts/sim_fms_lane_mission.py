#!/usr/bin/env python3
"""Gazebo 에서 실물 차선 미션 노드(pinky_fms_bringup fms_lane_mission.py)를 그대로 돌리는 래퍼.

실물 노드는 Picamera2 로 카메라를 읽는다. 여기서는 import 전에 가짜 picamera2 모듈을 끼워
/<ns>/camera/image_raw (Gazebo 카메라) 를 Picamera2.capture_array() 로 돌려준다. 실물 노드 코드는 수정하지 않는다.
  - 실물 노드는 카메라 영상을 180° 돌려서 쓰므로(로봇에 거꾸로 달림) 가짜 카메라는 미리 180° 돌려서 준다.
  - LED(set_lamp)·초음파·LCD 는 없다: LED 는 서비스가 없다는 경고만 나오고, 초음파는 '데이터 없음'(장애물 없음)으로 보고
    앞 사물 정지는 라이다로만 한다 (fms_lane_mission 이 라이다 앞 통로를 같이 본다). enable_lcd:=false 로 띄운다.
  - YOLO(ultralytics)는 이 스크립트를 돌리는 파이썬에 있어야 한다 (launch 의 python 인자, 예: --system-site-packages 가상환경).

    ros2 launch pinky_fms_sim sim_lane_mission.launch.xml namespace:=amr_01 model_path:=<best.pt> python:=<venv>/bin/python
"""
import os
import runpy
import sys
import threading
import time
import types

import numpy as np


def _install_fake_picamera2():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class Picamera2:
        """Picamera2 흉내: ROS 카메라 토픽의 마지막 영상을 RGB 배열로 준다."""

        def __init__(self, *args, **kwargs):
            ns = os.environ.get('SIM_CAMERA_NS', '')
            topic = os.environ.get('SIM_CAMERA_TOPIC', f'{ns}/camera/image_raw' if ns else 'camera/image_raw')
            self._frame, self._lock, self._t = None, threading.Lock(), 0.0
            self._node = rclpy.create_node('sim_camera_feed')
            self._node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
            self._exec = SingleThreadedExecutor()
            self._exec.add_node(self._node)
            self._thread = threading.Thread(target=self._exec.spin, daemon=True)
            self._thread.start()
            self._node.get_logger().info(f'가짜 Picamera2: {topic} 영상을 씀')

        def _on_image(self, m):
            ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(m.encoding)
            if ch is None:
                return
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * ch].reshape(m.height, m.width, ch)
            if m.encoding.startswith('bgr'):
                a = a[..., 2::-1]
            a = np.ascontiguousarray(a[:, :, :3][::-1, ::-1])      # RGB, 180° 회전 (실물 노드가 다시 돌린다)
            with self._lock:
                self._frame, self._t = a, time.monotonic()

        def create_video_configuration(self, **kw):
            return kw

        create_preview_configuration = create_video_configuration

        def configure(self, cfg):
            pass

        def start(self):
            pass

        def capture_array(self, *args):
            end = time.monotonic() + 10.0
            while True:
                with self._lock:
                    if self._frame is not None:
                        return self._frame
                if time.monotonic() > end:
                    raise RuntimeError('카메라 영상이 10 s 동안 오지 않음 (sim_robot camera:=true, 토픽 이름 확인)')
                time.sleep(0.05)

        def stop(self):
            pass

        def close(self):
            try:
                self._exec.shutdown(timeout_sec=1.0)
                self._node.destroy_node()
            except Exception:
                pass

    sys.modules['picamera2'] = types.SimpleNamespace(Picamera2=Picamera2)


def main():
    _install_fake_picamera2()
    from ament_index_python.packages import get_package_prefix
    script = os.path.join(get_package_prefix('pinky_fms_bringup'), 'lib', 'pinky_fms_bringup', 'fms_lane_mission.py')
    sys.argv[0] = script
    runpy.run_path(script, run_name='__main__')


if __name__ == '__main__':
    main()
