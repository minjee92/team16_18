"""
차선 후처리 (모델 무관)

세그멘테이션 결과(마스크 + 클래스 이름 + conf)만 받아 차선 중앙점과
조향값을 계산한다. ultralytics / ncnn 등 모델 종류에 의존하지 않는다.
모델 결과를 numpy 배열로 꺼내는 일은 호출하는 쪽에서 한다.
"""

import cv2
import numpy as np


# --------------------------- 마스크 처리 ---------------------------

def group_masks(masks, class_names, confs, width, height):
    """
    클래스 이름별로 원본 크기의 이진 마스크 목록을 만든다.

      masks       : (N, h, w) 0/1 배열 (모델 출력 해상도)
      class_names : 길이 N, 각 마스크의 클래스 이름
      confs       : 길이 N, 각 마스크의 신뢰도
      반환        : {이름: [{'mask': (height, width) uint8, 'conf', 'area'}, ...]}
    """

    out = {}

    for mask, name, conf in zip(masks, class_names, confs):

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
    """
    행마다 좌우 차선 위치를 읽어 목표점과 조향값을 만든다.

      sample_rows      : 차선을 읽을 높이들 (화면 높이 대비 비율). 아래쪽이 로봇에 가까움
      row_weights      : 각 높이의 가중치
      road_half_init   : 도로 반폭 초깃값 (화면 너비 대비)
      road_half_smooth : 도로 폭 갱신 속도 (0~1, 클수록 빠르게 반영)
      steer_gain       : 조향 민감도
      steer_smooth     : 조향값 부드럽게 (0~1, 작을수록 부드러움)
      lost_limit       : 차선을 못 본 프레임이 이보다 많으면 조향 0으로
    """

    def __init__(self, width, height,
                 sample_rows=(0.90, 0.80, 0.70, 0.60),
                 row_weights=(0.40, 0.30, 0.20, 0.10),
                 road_half_init=0.28,
                 road_half_smooth=0.1,
                 steer_gain=1.0,
                 steer_smooth=0.35,
                 lost_limit=15):
        self.width = width
        self.height = height
        self.rows = [int(height * r) for r in sample_rows]
        self.row_weights = row_weights
        self.road_half_smooth = road_half_smooth
        self.steer_gain = steer_gain
        self.steer_smooth = steer_smooth
        self.lost_limit = lost_limit

        # 행별 도로 반폭. 한쪽 차선만 보일 때 쓴다
        self.half_width = {
            y: width * road_half_init for y in self.rows
        }

        self.steer = 0.0
        self.lost_frames = 0
        self.target_x = None     # 마지막 update에서 조향에 쓴 목표점 x (못 봤으면 None)

    def update(self, left_mask, right_mask):
        """
        반환: (centers, error, steer, valid)
          centers : [(x, y, source)] 행별 중앙점
          error   : -1.0 ~ 1.0 (양수면 로봇이 왼쪽으로 치우침)
          steer   : 부드럽게 처리한 조향값
          valid   : 차선을 하나라도 봤는지
        조향에 쓴 목표점 x는 self.target_x 에 남는다 (시각화용).
        """

        centers = []
        weights = []
        seen = False

        for y, w in zip(self.rows, self.row_weights):

            lx = x_at_row(left_mask, y)
            rx = x_at_row(right_mask, y)

            if lx is not None and rx is not None:
                # 양쪽 다 보임 → 중앙이 확실하고, 도로 폭도 갱신
                cx = (lx + rx) / 2.0
                measured = abs(rx - lx) / 2.0
                self.half_width[y] += self.road_half_smooth * (measured - self.half_width[y])
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
            self.target_x = None
            self.lost_frames += 1
            if self.lost_frames > self.lost_limit:
                self.steer = 0.0            # 오래 못 보면 직진으로 되돌림
            return centers, None, self.steer, False

        self.lost_frames = 0

        total = sum(weights)
        target_x = sum(c[0] * w for c, w in zip(centers, weights)) / total
        self.target_x = target_x

        # 화면 중심 기준 오차. 양수면 목표가 오른쪽 → 로봇이 왼쪽으로 치우침
        error = (target_x - self.width / 2.0) / (self.width / 2.0)

        raw = self.steer_gain * error
        self.steer += self.steer_smooth * (raw - self.steer)

        return centers, error, self.steer, True
