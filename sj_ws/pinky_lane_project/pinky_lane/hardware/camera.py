import time
import cv2
import numpy as np
from pinky_lane.config import VISION

class PinkyCamera:

    def __init__(self):
        self._camera = None

    def start(self):
        from picamera2 import Picamera2
        self._camera = Picamera2()
        self._camera.configure(self._camera.create_video_configuration(
            main={"size": (VISION.frame_width, VISION.frame_height), "format": "RGB888"}))
        self._camera.start()
        time.sleep(1.0)

    def capture(self) -> np.ndarray:
        frame = self._camera.capture_array()

        if VISION.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Picamera2 RGB888 is already a BGR byte array (libcamera naming).

        if VISION.swap_rb:
            frame = frame[:, :, ::-1].copy()

        if VISION.mirror_input:
            frame = cv2.flip(frame, 1)

        return frame

    def stop(self):
        if self._camera is not None:
            self._camera.stop()

    def close(self):
        if self._camera is not None:
            self._camera.close()
