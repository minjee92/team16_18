import cv2
import numpy as np
from pinky_lane.core.types import LaneEstimate

class DebugOverlay:

    def render(
        self,
        frame: np.ndarray,
        lane: LaneEstimate,
        crosswalk_ratio: float,
        linear: float,
        angular: float,
        state, note: str, drive_output: bool,
    ) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]
        mid = w // 2

        cv2.line(out, (mid, 0), (mid, h), (110, 110, 110), 1)

        for point in lane.points:
            color = (0, 255, 0) if point.source == "both" else (0, 200, 255)
            cv2.circle(out, (point.x, point.y), 6, color, -1)

        if len(lane.points) >= 2:
            pts = np.array([[p.x, p.y] for p in lane.points], dtype=np.int32)
            cv2.polylines(out, [pts], False, (0, 255, 0), 2)

        if lane.target_x is not None:
            tx = int(round(lane.target_x))
            cv2.line(out, (tx, h - 90), (tx, h), (255, 255, 0), 2)

        cv2.putText(
            out,
            state.name,
            (14, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            f"steer {lane.steering:+.2f} conf {lane.confidence:.2f}",
            (14, 66),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            f"v {linear:.2f}  w {angular:+.2f}  cross {crosswalk_ratio:.3f}",
            (14, 94),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if note:
            cv2.putText(
                out,
                note,
                (14, 124),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

        if not drive_output:
            cv2.putText(
                out,
                "DRY RUN",
                (w - 170, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )

        return out
