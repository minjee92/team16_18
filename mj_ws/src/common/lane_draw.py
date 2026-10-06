"""
차선 추적 시각화 (모델 무관)

step1 검증 도구와 step2 주행 기록 영상이 같은 그림을 쓰도록 공통으로 둔다.
입력은 LaneTracker 결과와 화면(BGR)뿐이다.

  draw_tracking : 중앙점·목표점·조향 막대·상태 글자 (step1 결과 영상과 동일)
"""

import cv2
import numpy as np


def draw_tracking(frame, centers, error, steer, valid, events, tracker):
    """LaneTracker.update() 결과를 frame 위에 그린다 (frame 을 직접 수정하고 돌려준다)."""

    height, width = frame.shape[:2]
    mid = width // 2

    cv2.line(frame, (mid, 0), (mid, height), (120, 120, 120), 1)

    for x, y, source in centers:
        color = {
            'both': (0, 255, 0),
            'left': (0, 200, 255),
            'right': (0, 200, 255),
        }[source]
        cv2.circle(frame, (x, y), 5, color, -1)
        cv2.line(frame, (mid, y), (x, y), color, 1)

    if centers:
        pts = np.array([[x, y] for x, y, _ in centers], np.int32)
        cv2.polylines(frame, [pts], False, (0, 255, 0), 2)

    if valid:
        # 다시 계산하지 않고 LaneTracker가 조향에 실제로 쓴 목표점을 그린다
        target_x = int(tracker.target_x)
        target_y = tracker.rows[0]
        cv2.drawMarker(frame, (target_x, target_y), (0, 0, 255),
                       cv2.MARKER_CROSS, 22, 2)

    # 조향 막대
    bar_y = height - 24
    cv2.rectangle(frame, (mid - 150, bar_y - 8), (mid + 150, bar_y + 8),
                  (60, 60, 60), -1)
    tip = int(mid + np.clip(steer, -1, 1) * 150)
    cv2.rectangle(frame, (mid, bar_y - 8), (tip, bar_y + 8), (0, 165, 255), -1)
    cv2.line(frame, (mid, bar_y - 12), (mid, bar_y + 12), (255, 255, 255), 1)

    status = 'LOST' if not valid else f'error {error:+.3f}'
    cv2.putText(frame, f'{status}   steer {steer:+.3f}', (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255) if not valid else (255, 255, 255), 2)

    for i, text in enumerate(events):
        cv2.putText(frame, text, (12, 58 + i * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    return frame
