import time
from typing import Optional, List
import numpy as np
from pinky_lane.config import LANE, MOTION
from pinky_lane.core.types import LanePoint, LaneEstimate
from pinky_lane.core.math_utils import clamp

def robust_x_at_row(mask: Optional[np.ndarray], y: int, band: int) -> Optional[float]:
    """한 줄 대신 y±band 영역을 사용해 segmentation 끊김에 덜 민감하게 x를 추정."""
    if mask is None:
        return None

    h, _ = mask.shape[:2]
    y0 = max(0, y - band)
    y1 = min(h, y + band + 1)
    if y0 >= y1:
        return None

    _, xs = np.nonzero(mask[y0:y1])
    if xs.size == 0:
        return None

    # mean보다 outlier에 강한 median 사용
    return float(np.median(xs))

class LaneEstimator:
    def __init__(self, width: int, height: int) -> None:
        if len(LANE.sample_rows) != len(LANE.row_weights):
            raise ValueError("sample_rows and row_weights must have the same length")

        self.width = width
        self.height = height
        self.rows = [int(height * ratio) for ratio in LANE.sample_rows]
        self.half_width = {
            row: width * LANE.initial_half_width_ratio for row in self.rows
        }

        self._smoothed_target: Optional[float] = None
        self._steer = 0.0
        self._prev_error = 0.0
        self._last_update_time: Optional[float] = None
        self.lost_frames = 0
        self._lost_since = None
        self._ever_valid = False

    def reset_control(self) -> None:
        self._steer = 0.0
        self._prev_error = 0.0
        self._last_update_time = None
        self._smoothed_target = None
        self.lost_frames = 0
        self._lost_since = None
        self._ever_valid = False

    def _valid_lane_width(self, lx: float, rx: float) -> bool:
        width = rx - lx
        return (
            self.width * LANE.min_lane_width_ratio
            <= width
            <= self.width * LANE.max_lane_width_ratio
        )

    def update(
        self,
        left_mask: Optional[np.ndarray],
        right_mask: Optional[np.ndarray],
        now: Optional[float] = None,
    ) -> LaneEstimate:
        now = time.monotonic() if now is None else now

        points: List[LanePoint] = []
        used_weights: List[float] = []
        both_count = 0

        for row, weight in zip(self.rows, LANE.row_weights):
            lx = robust_x_at_row(left_mask, row, LANE.sample_band_half_height)
            rx = robust_x_at_row(right_mask, row, LANE.sample_band_half_height)

            center: Optional[float] = None
            source: Optional[str] = None

            if lx is not None and rx is not None and self._valid_lane_width(lx, rx):
                center = (lx + rx) * 0.5
                measured_half = abs(rx - lx) * 0.5
                old = self.half_width[row]
                self.half_width[row] = old + LANE.half_width_alpha * (measured_half - old)
                source = "both"
                both_count += 1
            elif lx is not None and rx is not None:
                continue # contradictory pair: do not pretend only the left lane exists
            elif lx is not None:
                center = lx + self.half_width[row]
                source = "left"
            elif rx is not None:
                center = rx - self.half_width[row]
                source = "right"

            if center is None:
                continue

            center = clamp(center, 0.0, float(self.width - 1))
            points.append(LanePoint(int(round(center)), row, source))
            used_weights.append(weight)

        if len(points) < LANE.min_valid_rows:
            self.lost_frames += 1
            if self._lost_since is None:
                self._lost_since = now
            lost_seconds = max(0.0, now - self._lost_since)
            if lost_seconds >= LANE.lost_stop_s:
                self._steer = 0.0
            return LaneEstimate(points, self._steer, 0.0, False,
                                self.lost_frames, None, lost_seconds,
                                self._ever_valid and lost_seconds < LANE.lost_stop_s)

        self.lost_frames = 0
        self._lost_since = None
        self._ever_valid = True
        total_weight = sum(used_weights)
        raw_target = sum(p.x * w for p, w in zip(points, used_weights)) / total_weight

        if self._smoothed_target is None:
            self._smoothed_target = raw_target
        else:
            self._smoothed_target += LANE.target_alpha * (raw_target - self._smoothed_target)

        error = (self._smoothed_target - self.width * 0.5) / (self.width * 0.5)

        if self._last_update_time is None:
            dt = 1.0 / MOTION.control_hz
        else:
            dt = max(now - self._last_update_time, 1e-3)
        self._last_update_time = now

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        # derivative가 FPS 변화로 과도하게 커지지 않도록 약하게 제한
        derivative = clamp(derivative, -3.0, 3.0)
        raw_steer = LANE.kp * error + LANE.kd * derivative
        raw_steer = clamp(raw_steer, -1.0, 1.0)

        self._steer += LANE.steer_alpha * (raw_steer - self._steer)
        self._steer = clamp(self._steer, -1.0, 1.0)

        row_ratio = len(points) / len(self.rows)
        both_ratio = both_count / len(self.rows)
        confidence = clamp(0.70 * row_ratio + 0.30 * both_ratio, 0.0, 1.0)
        valid = len(points) >= LANE.min_valid_rows

        return LaneEstimate(
            points=points,
            steering=self._steer,
            confidence=confidence,
            valid=valid,
            lost_frames=0,
            target_x=self._smoothed_target,
        )
