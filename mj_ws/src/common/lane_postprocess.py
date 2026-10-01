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
      lost_stop_sec    : 차선을 못 본 시간이 이보다 길면 조향 0으로 (초)
      steer_d_gain     : 오차 변화량(D항) 가중치. 0이면 D항 없음
      steer_clip       : 조향값 절댓값 상한. None이면 제한 없음

    차선 상실은 프레임 수가 아니라 시간으로 판단한다 (로봇과 PC의 fps가 달라서).
    update()에 넘기는 now 는 초 단위 시각: 로봇은 time.monotonic(), 저장 영상은 프레임 번호 ÷ fps.
    """

    def __init__(self, width, height,
                 sample_rows=(0.90, 0.80, 0.70, 0.60),
                 row_weights=(0.40, 0.30, 0.20, 0.10),
                 road_half_init=0.28,
                 road_half_smooth=0.1,
                 steer_gain=1.0,
                 steer_smooth=0.35,
                 lost_stop_sec=2.0,
                 steer_d_gain=0.0,
                 steer_clip=None):
        self.width = width
        self.height = height
        self.rows = [int(height * r) for r in sample_rows]
        self.row_weights = row_weights
        self.road_half_smooth = road_half_smooth
        self.steer_gain = steer_gain
        self.steer_smooth = steer_smooth
        self.lost_stop_sec = lost_stop_sec
        self.steer_d_gain = steer_d_gain
        self.steer_clip = steer_clip

        # 행별 도로 반폭. 한쪽 차선만 보일 때 쓴다
        self.half_width = {
            y: width * road_half_init for y in self.rows
        }

        self.steer = 0.0
        self.prev_error = 0.0    # D항 계산용 직전 오차
        self.lost_frames = 0     # 연속으로 못 본 프레임 수 (표시용)
        self.lost_time = 0.0     # 마지막으로 차선을 본 뒤 지난 시간 (초)
        self.last_valid_time = None
        self.target_x = None     # 마지막 update에서 조향에 쓴 목표점 x (못 봤으면 None)

    def reset(self):
        """조향 상태를 초기화한다 (교차로 회전 뒤 등). 도로 반폭 추정은 유지한다."""
        self.steer = 0.0
        self.prev_error = 0.0
        self.lost_frames = 0
        self.lost_time = 0.0
        self.last_valid_time = None
        self.target_x = None

    def update(self, left_mask, right_mask, now):
        """
        now: 이 프레임의 시각 (초)

        반환: (centers, error, steer, valid)
          centers : [(x, y, source)] 행별 중앙점
          error   : -1.0 ~ 1.0 (양수면 로봇이 왼쪽으로 치우침)
          steer   : 부드럽게 처리한 조향값
          valid   : 차선을 하나라도 봤는지
        조향에 쓴 목표점 x는 self.target_x, 차선을 못 본 시간은 self.lost_time 에 남는다.
        """

        if self.last_valid_time is None:
            self.last_valid_time = now      # 시작(또는 reset) 시점을 기준으로 센다

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
            # µs 단위 반올림: 저장 영상(프레임 번호 ÷ fps)에서 정확히 경계값일 때 부동소수점 오차로 판정이 흔들리지 않게
            self.lost_time = round(now - self.last_valid_time, 6)
            if self.lost_time > self.lost_stop_sec:
                self.steer = 0.0            # 오래 못 보면 직진으로 되돌림
            return centers, None, self.steer, False

        self.lost_frames = 0
        self.lost_time = 0.0
        self.last_valid_time = now

        total = sum(weights)
        target_x = sum(c[0] * w for c, w in zip(centers, weights)) / total
        self.target_x = target_x

        # 화면 중심 기준 오차. 양수면 목표가 오른쪽 → 로봇이 왼쪽으로 치우침
        error = (target_x - self.width / 2.0) / (self.width / 2.0)

        derivative = error - self.prev_error
        self.prev_error = error

        raw = self.steer_gain * error
        if self.steer_d_gain:
            raw += self.steer_d_gain * derivative
        self.steer += self.steer_smooth * (raw - self.steer)
        if self.steer_clip is not None:
            self.steer = max(-self.steer_clip, min(self.steer_clip, self.steer))

        return centers, error, self.steer, True
