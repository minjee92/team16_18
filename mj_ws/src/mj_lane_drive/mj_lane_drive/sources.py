"""
주행 노드의 영상 입력원 (BGR 프레임을 read() 로 돌려준다, 끝나면 None).

  PiCameraSource : 로봇 카메라 (Picamera2). 이 입력원을 고를 때만 picamera2 를 import 한다.
                   mj_lane_real 을 만들 때 카메라 발행 노드로 옮기고 여기서는 지운다 (계획).
  VideoSource    : 저장 영상 (PC 검증용)
"""

import time

import cv2


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
