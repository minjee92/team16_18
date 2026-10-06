"""
차선 추적 시각화 (모델 무관)

step1 검증 도구와 step2 주행 기록 영상이 같은 그림을 쓰도록 공통으로 둔다.
입력은 LaneTracker 결과와 화면(BGR)뿐이다.

  draw_tracking : 중앙점·목표점·조향 막대·상태 글자 (step1 결과 영상과 동일)
  draw_debug    : 행별 좌/우 차선 x·도로 반폭, 정보 줄 (step2 주행 기록용)
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


LEFT_COLOR = (255, 0, 0)        # 파랑 (BGR) — left_lane
RIGHT_COLOR = (0, 165, 255)     # 주황 — right_lane


def _panel(frame, x0, y0, x1, y1, alpha=0.55):
    """반투명 검은 바탕 (글자를 읽기 쉽게)."""
    x0, y0 = max(x0, 0), max(y0, 0)
    x1, y1 = min(x1, frame.shape[1]), min(y1, frame.shape[0])
    if x1 > x0 and y1 > y0:
        roi = frame[y0:y1, x0:x1]
        roi[:] = (roi * (1 - alpha)).astype(roi.dtype)


def draw_debug(frame, rows_obs, lines, top=48):
    """
    LaneTracker.rows_obs 의 행별 관측과 정보 줄을 그린다 (frame 을 직접 수정하고 돌려준다).

      - 각 샘플 행에 tracker 가 읽은 left_lane x(파랑)·right_lane x(주황) 눈금
      - 오른쪽 끝에 그 행의 도로 반폭 추정값(px). 눈금이 가려지지 않게 글자를 먼저 그린다
      - lines: 왼쪽 위(top 부터) 반투명 바탕 위에 한 줄씩. 샘플 행(화면 아래쪽)을 가리지 않는 자리
    """
    font, scale = cv2.FONT_HERSHEY_SIMPLEX, 0.45
    height, width = frame.shape[:2]

    for obs in rows_obs:
        y = obs['y']
        _panel(frame, width - 62, y - 10, width, y + 6)
        cv2.putText(frame, f'hw {obs["half_width"]:.0f}', (width - 58, y + 2), font, 0.4,
                    (255, 255, 255), 1, cv2.LINE_AA)

    for obs in rows_obs:
        y = obs['y']
        if obs['lx'] is not None:
            x = int(obs['lx'])
            cv2.line(frame, (x, y - 9), (x, y + 9), LEFT_COLOR, 3)
        if obs['rx'] is not None:
            x = int(obs['rx'])
            cv2.line(frame, (x, y - 9), (x, y + 9), RIGHT_COLOR, 3)

    if lines:
        line_h = 18
        text_w = max(cv2.getTextSize(text, font, scale, 1)[0][0] for text in lines)
        _panel(frame, 4, top - 14, 4 + text_w + 10, top - 14 + line_h * len(lines) + 6)
        for i, text in enumerate(lines):
            cv2.putText(frame, text, (9, top + line_h * i), font, scale, (255, 255, 255), 1, cv2.LINE_AA)

    return frame
