import cv2
import numpy as np
from pinky_lane.config import VISION

class PinkyDisplay:
    def __init__(self, enabled: bool, every_n_frames: int) -> None:
        self._lcd = None
        self._image_type = None
        self._counter = 0
        self._every = max(1, every_n_frames)

        if not enabled:
            return

        try:
            from pinky_lcd import LCD
            from PIL import Image

            self._lcd = LCD()
            self._image_type = Image
        except Exception as exc:
            print(f"[LCD] disabled: {exc}")
            self._lcd = None

    def _show_rgb(self, rgb: np.ndarray) -> None:
        if self._lcd is None:
            return
        try:
            self._lcd.img_show(self._image_type.fromarray(rgb))
        except Exception as exc:
            print(f"[LCD] output stopped: {exc}")
            self._lcd = None

    def frame(self, bgr: np.ndarray, force: bool = False) -> None:
        if self._lcd is None:
            return

        self._counter += 1
        if not force and self._counter % self._every:
            return

        self._show_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

    @property
    def available(self) -> bool:
        return self._lcd is not None

    def text(self, message: str, color=(255, 255, 255)) -> None:
        if self._lcd is None:
            return

        canvas = np.zeros((VISION.frame_height, VISION.frame_width, 3), np.uint8)
        cv2.putText(
            canvas,
            message,
            (24, VISION.frame_height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            color,
            3,
            cv2.LINE_AA,
        )
        self._show_rgb(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
