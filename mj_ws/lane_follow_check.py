#!/usr/bin/env python3
"""
차선 추종 검증기 (PC용)

저장된 영상으로 모델의 차선 인식과 조향값 계산을 눈으로 확인한다.
로봇에 올리기 전에 여기서 먼저 동작을 확인할 것.

  - 마스크 픽셀에서 여러 높이의 x좌표를 뽑아 차선 중앙 계산
  - 한쪽 차선만 보일 때는 기억해둔 도로 폭으로 목표점 추정
  - 조향값(error, angular)을 화면과 터미널에 표시

실행:
    python3 lane_follow_check.py
"""

import cv2
import numpy as np
from ultralytics import YOLO


# ----------------------------- 설정 -----------------------------

INPUT_SOURCE = 'pinky_20260919_182009.mp4'   # 영상 경로 (웹캠은 0)
MODEL_PATH = 'best.pt'
OUTPUT_VIDEO = 'result_lane_follow.mp4'

CONF = 0.5
INFER_SIZE = 640          # 로봇에서는 320으로 낮출 값

# 차선을 읽을 높이들 (화면 높이 대비 비율). 아래쪽이 로봇에 가까움
SAMPLE_ROWS = (0.90, 0.80, 0.70, 0.60)

# 각 높이의 가중치. 가까운 쪽을 더 신뢰하되 먼 쪽도 반영해 커브를 미리 본다
ROW_WEIGHTS = (0.40, 0.30, 0.20, 0.10)

ROAD_HALF_INIT = 0.28     # 도로 반폭 초깃값 (화면 너비 대비)
ROAD_HALF_SMOOTH = 0.1    # 도로 폭 갱신 속도 (0~1, 클수록 빠르게 반영)

STEER_GAIN = 1.0          # 조향 민감도
STEER_SMOOTH = 0.35       # 조향값 부드럽게 (0~1, 작을수록 부드러움)
LOST_LIMIT = 15           # 차선을 못 본 프레임이 이보다 많으면 조향 0으로

LANE_CLASSES = ('left_lane', 'right_lane')


# --------------------------- 마스크 처리 ---------------------------

def build_masks(result, width, height):
    """클래스 이름별로 원본 크기의 이진 마스크 목록을 만든다."""

    out = {}

    if result.masks is None or result.boxes is None:
        return out

    names = result.names
    data = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    confs = result.boxes.conf.cpu().numpy()

    for mask, cls_id, conf in zip(data, classes, confs):
        name = names[cls_id]

        resized = cv2.resize(
            mask.astype(np.uint8),
            (width, height),
            interpolation=cv2.INTER_NEAREST
        )

        out.setdefault(name, []).append({
            'mask': resized,
            'conf': float(conf),
            'area': int(resized.sum()),
        })

    return out


def pick_main(instances):
    """같은 클래스가 여럿이면 면적이 가장 큰 것을 고른다."""
    if not instances:
        return None
    return max(instances, key=lambda d: d['area'])['mask']


def x_at_row(mask, y):
    """주어진 행에서 마스크가 차지하는 x의 평균. 없으면 None."""
    if mask is None or not (0 <= y < mask.shape[0]):
        return None

    xs = np.flatnonzero(mask[y])
    if xs.size == 0:
        return None

    return float(xs.mean())


# --------------------------- 목표점 계산 ---------------------------

class LaneTracker:
    """행마다 좌우 차선 위치를 읽어 목표점과 조향값을 만든다."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.rows = [int(height * r) for r in SAMPLE_ROWS]

        # 행별 도로 반폭. 한쪽 차선만 보일 때 쓴다
        self.half_width = {
            y: width * ROAD_HALF_INIT for y in self.rows
        }

        self.steer = 0.0
        self.lost_frames = 0

    def update(self, left_mask, right_mask):
        """
        반환: (centers, error, steer, valid)
          centers : [(x, y, source)] 행별 중앙점
          error   : -1.0 ~ 1.0 (양수면 로봇이 왼쪽으로 치우침)
          steer   : 부드럽게 처리한 조향값
          valid   : 차선을 하나라도 봤는지
        """

        centers = []
        weights = []
        seen = False

        for y, w in zip(self.rows, ROW_WEIGHTS):

            lx = x_at_row(left_mask, y)
            rx = x_at_row(right_mask, y)

            if lx is not None and rx is not None:
                # 양쪽 다 보임 → 중앙이 확실하고, 도로 폭도 갱신
                cx = (lx + rx) / 2.0
                measured = abs(rx - lx) / 2.0
                self.half_width[y] += ROAD_HALF_SMOOTH * (measured - self.half_width[y])
                source = 'both'

            elif lx is not None:
                cx = lx + self.half_width[y]
                source = 'left'

            elif rx is not None:
                cx = rx - self.half_width[y]
                source = 'right'

            else:
                continue

            seen = True
            centers.append((int(cx), y, source))
            weights.append(w)

        if not seen:
            self.lost_frames += 1
            if self.lost_frames > LOST_LIMIT:
                self.steer = 0.0            # 오래 못 보면 직진으로 되돌림
            return centers, None, self.steer, False

        self.lost_frames = 0

        total = sum(weights)
        target_x = sum(c[0] * w for c, w in zip(centers, weights)) / total

        # 화면 중심 기준 오차. 양수면 목표가 오른쪽 → 로봇이 왼쪽으로 치우침
        error = (target_x - self.width / 2.0) / (self.width / 2.0)

        raw = STEER_GAIN * error
        self.steer += STEER_SMOOTH * (raw - self.steer)

        return centers, error, self.steer, True


# ----------------------------- 시각화 -----------------------------

def draw(frame, centers, error, steer, valid, events, tracker):

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
        target_x = int(np.average(
            [c[0] for c in centers], weights=ROW_WEIGHTS[:len(centers)]
        ))
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


# ------------------------------ 메인 ------------------------------

def main():

    model = YOLO(MODEL_PATH)
    print('클래스:', model.names)

    missing = [c for c in LANE_CLASSES if c not in model.names.values()]
    if missing:
        print(f'경고: 모델에 {missing} 클래스가 없습니다. 이름을 확인하세요.')

    cap = cv2.VideoCapture(INPUT_SOURCE)
    if not cap.isOpened():
        print(f'입력 소스를 열 수 없습니다: {INPUT_SOURCE}')
        return

    ok, first = cap.read()
    if not ok:
        print('프레임을 읽을 수 없습니다.')
        return

    height, width = first.shape[:2]
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    fps = cap.get(cv2.CAP_PROP_FPS)
    fps = fps if fps and fps > 0 else 20.0

    writer = cv2.VideoWriter(
        OUTPUT_VIDEO, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height)
    )

    tracker = LaneTracker(width, height)

    print(f'{width}x{height} @ {fps:.1f}fps   종료: q')

    while True:

        ok, frame = cap.read()
        if not ok:
            break

        result = model.predict(
            source=frame, conf=CONF, imgsz=INFER_SIZE, verbose=False
        )[0]

        masks = build_masks(result, width, height)

        left_mask = pick_main(masks.get('left_lane'))
        right_mask = pick_main(masks.get('right_lane'))

        centers, error, steer, valid = tracker.update(left_mask, right_mask)

        events = []
        if masks.get('crosswalk'):
            events.append('CROSSWALK')
        if masks.get('cross_lane'):
            events.append('CROSS LANE')

        view = result.plot()
        view = draw(view, centers, error, steer, valid, events, tracker)

        writer.write(view)
        cv2.imshow('Lane Follow Check', view)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    writer.release()
    cv2.destroyAllWindows()
    print(f'저장 완료: {OUTPUT_VIDEO}')


if __name__ == '__main__':
    main()
