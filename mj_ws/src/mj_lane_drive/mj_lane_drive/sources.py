"""
주행 노드의 영상 입력원 (BGR 프레임을 read() 로 돌려준다, 끝나면 None).

  PiCameraSource : 로봇 카메라 (Picamera2). 이 입력원을 고를 때만 picamera2 를 import 한다.
                   mj_lane_real 을 만들 때 카메라 발행 노드로 옮기고 여기서는 지운다 (계획).
  VideoSource    : 저장 영상 (PC 검증용)
  RosImageSource : ROS 카메라 토픽 (sensor_msgs/Image). 시뮬, 나중에 실물 카메라 발행 노드도 이것으로 받는다.
                   영상이 잠깐 끊기면 None(끝) 대신 NO_FRAME 을 돌려준다.
"""

import threading
import time

import cv2
import numpy as np

NO_FRAME = object()     # RosImageSource: 기다려도 새 영상이 오지 않음 (끝난 것은 아님)

# 토픽 인코딩 → (채널 수, BGR 로 바꾸는 cv2 변환. None 이면 그대로)
ENCODINGS = {
    'rgb8': (3, cv2.COLOR_RGB2BGR),
    'bgr8': (3, None),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
}


class PiCameraSource:
    """로봇 카메라 (Picamera2)."""

    def __init__(self, size, rotate_180=True, mirror=False, swap_rb=False):
        from picamera2 import Picamera2      # 이 입력원을 고를 때만 import (PC 에서는 없어도 됨)
        self.rotate_180, self.mirror, self.swap_rb = rotate_180, mirror, swap_rb
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_video_configuration(
            main={'size': size, 'format': 'RGB888'}))
        self.camera.start()
        time.sleep(1.0)

    def read(self):
        frame = self.camera.capture_array()
        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if self.swap_rb:
            frame = frame[:, :, ::-1].copy()
        if self.mirror:
            frame = cv2.flip(frame, 1)
        return frame

    def close(self):
        try:
            self.camera.stop()
            self.camera.close()
        except Exception:
            pass


class VideoSource:
    """PC 테스트용: 저장 영상 (이미 모델 입력 방향이므로 전처리 없음)."""

    def __init__(self, path, size):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f'영상을 열 수 없습니다: {path}')
        self.size = size

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return cv2.resize(frame, self.size)

    def close(self):
        self.cap.release()


def ros_image_to_bgr(height, width, encoding, step, data):
    """sensor_msgs/Image 내용 → BGR 배열 (cv_bridge 없이. yolo venv 에 cv_bridge 가 없다).
    step 은 한 줄의 바이트 수로, 줄 끝 여백이 있어도 된다."""
    if encoding not in ENCODINGS:
        raise ValueError(f'지원하지 않는 영상 인코딩: {encoding} (지원: {", ".join(ENCODINGS)})')
    channels, conversion = ENCODINGS[encoding]
    raw = np.frombuffer(data, dtype=np.uint8)
    if step < width * channels or raw.size < height * step:
        raise ValueError(f'영상 크기가 맞지 않음: {width}x{height} {encoding}, step {step}, {raw.size} 바이트')
    image = raw[:height * step].reshape(height, step)[:, :width * channels].reshape(height, width, channels)
    return image.copy() if conversion is None else cv2.cvtColor(image, conversion)


class RosImageSource:
    """
    ROS 카메라 토픽 (A4 규격: 바로 선 영상, rgb8/bgr8/rgba8/bgra8, 640x480, sensor_data QoS).
    콜백은 최신 메시지 하나만 남기고, read() 가 새 메시지를 기다려 BGR 로 바꾼다 (오래된 프레임은 버림).
    크기가 다르면 size 로 줄이고 경고는 한 번만 한다. 회전·좌우 반전은 하지 않는다 (발행하는 쪽 일).
    """

    def __init__(self, node, topic, size, timeout, warn=print):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.size, self.timeout, self.warn = size, timeout, warn
        self.cond = threading.Condition()
        self.latest, self.seq, self.read_seq = None, 0, 0
        self.resize_warned = False
        self.sub = node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.topic = self.sub.topic_name                 # 네임스페이스를 붙인 실제 이름

    def _on_image(self, msg):
        with self.cond:
            self.latest, self.seq = msg, self.seq + 1
            self.cond.notify_all()

    def read(self):
        """새 프레임(BGR). timeout 초 동안 새 메시지가 없으면 NO_FRAME."""
        with self.cond:
            if not self.cond.wait_for(lambda: self.seq > self.read_seq, timeout=self.timeout):
                return NO_FRAME
            msg, self.read_seq = self.latest, self.seq
        frame = ros_image_to_bgr(msg.height, msg.width, msg.encoding, msg.step, msg.data)
        if (msg.width, msg.height) != tuple(self.size):
            if not self.resize_warned:
                self.warn(f'카메라 토픽 크기 {msg.width}x{msg.height} → {self.size[0]}x{self.size[1]} 로 줄여 씀 '
                          f'(규격은 {self.size[0]}x{self.size[1]}. 이 경고는 한 번만)')
                self.resize_warned = True
            frame = cv2.resize(frame, tuple(self.size))
        return frame

    def close(self):
        pass                                             # 구독은 노드와 함께 정리된다
