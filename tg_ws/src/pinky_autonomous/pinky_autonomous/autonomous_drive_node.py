#!/usr/bin/env python3
"""
Pinky Pro 자율주행 노드 (RPi5에서 실행)

  상태 → 동작/LED 규칙 (우선순위 순)
    COLLISION      초음파 <= collision_dist        정지  / 빨강 ON
    OBSTACLE_STOP  초음파 <= ultrasonic_stop_dist  정지  / 빨강 ON
    JUNCTION       cross_lane 감지 → junction_advance_dist 만큼 더 전진한 뒤 정지, 방향 판단, 출발점 복귀 시작
                   (전진 중에는 주행 상태: 초록 깜빡 / 멈춘 뒤에는 빨강 ON)
    RETURN_HOME    후진(U턴) 선택 시 제자리 180° 회전 중              회전 / 초록 깜빡
    (복귀 주행)    SLAM 맵으로 출발점까지 최단 경로 쪽(좌/우/후진)을 고른다.
                   좌/우: 제자리에서 돌지 않고, 고른 쪽 차선만 따라가며(차선 편향) 갈림길을 빠져나간다.
                   후진: U턴 후 좌우 차선 역할을 바꿔 추종. SLAM 위치가 출발점 근처면 도착
    HOME           출발점 도착                                    정지 / 빨강 ON
    CW_BLOCKED     횡단보도 확인 중 초음파 <= crosswalk_stop_dist  정지 / 빨강 ON
    CAUTION        횡단보도 확인 중 or 초음파 <= obstacle_dist     감속 / 주황 깜빡
    DRIVING        그 외                           정상  / 초록 깜빡
    (차선 소실 정지)                               정지  / 빨강 ON

  횡단보도: 감지(면적 >= crosswalk_min_area) → 감속하며 앞을 확인 →
            crosswalk_check_time 동안 물체가 없으면 정속 복귀. 물체가 있으면 정지, 사라지면 다시 확인.
"""

import math
import signal
import time
import numpy as np
import cv2
from PIL import Image as PILImage
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Range
from pinky_interfaces.srv import SetLamp
from pinky_autonomous.route_planner import choose_exit


# ──────────────────────────────────────────────
# 상태 정의
# ──────────────────────────────────────────────
STATE_DRIVING       = 'driving'
STATE_CAUTION       = 'caution'
STATE_OBSTACLE_STOP = 'obstacle_stop'
STATE_COLLISION     = 'collision'
STATE_CW_BLOCKED    = 'crosswalk_blocked'   # 횡단보도 위 물체 → 정지
STATE_JUNCTION      = 'junction'            # cross_lane(갈림길/끝) 감지: 잠시 정지
STATE_RETURN_HOME   = 'return_home'         # U턴 회전 중
STATE_HOME          = 'home'                # 출발점 도착
STATE_RETURN_FAILED = 'return_failed'       # 복귀 실패 → 정지

# ──────────────────────────────────────────────
# LED 정의: (r, g, b, mode, time_ms)   mode 0=off 1=on 2=blink
# ──────────────────────────────────────────────
LAMP_DRIVE   = (0.0, 1.0,  0.0, 2, 500)   # 초록 깜빡
LAMP_CAUTION = (1.0, 0.35, 0.0, 2, 300)   # 주황 깜빡
LAMP_STOP    = (1.0, 0.0,  0.0, 1, 0)     # 빨강 고정
LAMP_OFF     = (0.0, 0.0,  0.0, 0, 0)
LAMP_RESEND_SEC = 2.0                     # 램프 노드 재시작 대비 주기적 재전송

# ──────────────────────────────────────────────
# 차선 추적 상수
# ──────────────────────────────────────────────
SAMPLE_ROWS = (0.90, 0.80, 0.70, 0.60)
ROW_WEIGHTS = (0.40, 0.30, 0.20, 0.10)
FAR_ROWS = (0.55, 0.45, 0.35)  # 추종 행에서 차선을 놓쳤을 때만 보는 위쪽 행 (코너 앞 가로 띠 등)
FAR_HORIZON = 0.25             # 위쪽 행의 차선 폭을 원근으로 줄일 때 쓰는 지평선 위치(화면 높이 비율)
FAR_SPEED_FACTOR = 0.7         # 위쪽 행만 보일 때의 속도 배율
FAR_STEER_MAX = 0.7            # 위쪽 행만 보일 때의 조향 상한 (멀리 보이는 차선에 최대 조향을 걸지 않는다)
ROAD_HALF_INIT = 0.46
ROAD_HALF_SMOOTH = 0.1
LOST_SLOW_AFTER = 5        # 잘 주행하던 코드와 같은 값
LOST_STOP_AFTER = 20       # 차선을 놓친 채 이 프레임 수가 지나면 정지 (초음파가 안 될 때 벽으로 계속 가지 않도록 짧게)
TARGET_HISTORY_SIZE = 3
CAMERA_OFFSET = 0
STEER_D_GAIN = 0.2
STEER_SMOOTH = 0.7
STEER_TO_ANGULAR = 1.5

# 한쪽 차선만 보일 때의 진행 방향(heading) 보정
HEADING_ROI_TOP = 0.35         # 화면 위쪽(먼 벽 등)은 제외하고 이 비율 아래만 사용
HEADING_MIN_PIXELS = 400       # 이보다 작은 마스크는 방향 추정에 쓰지 않음
HEADING_MAX_SAMPLES = 3000     # PCA 계산용 픽셀 샘플 상한 (연산량 제한)
HEADING_GAIN = 0.6             # 각도 차이(rad) → 조향 오차 환산
HEADING_CLIP = 1.2             # 각도 차이 상한(rad)
HEADING_REF_ALPHA = 0.05       # 직진 기준각 학습 속도
HEADING_REF_INIT = 0.35        # 기준각 초기값(rad, 왼쪽 -, 오른쪽 +). 양쪽 차선이 보이는 직진 구간에서 자동 학습

BASE_SPEED = 0.08
MAX_ANGULAR = 1.5
CORNER_SLOWDOWN = 0.5
CROSSWALK_MIN_AREA = 0.05      # (기본값) 횡단보도 접근으로 볼 화면 면적 비율
JUNCTION_CONFIRM_FRAMES = 3   # cross_lane이 연속 몇 프레임 보여야 갈림길로 인정할지
SONAR_TIMEOUT = 1.0        # 이 시간 동안 초음파 수신이 없으면 값을 무효 처리
FLIP_IOU = 0.30           # 직전 프레임의 반대편 차선과 이만큼 겹치면 라벨이 뒤바뀐 것으로 본다
FLIP_MARGIN = 0.15         # 그리고 같은 편 차선과의 겹침보다 이만큼 더 커야 한다
STOP_HYSTERESIS = 1.2      # 정지 상태 해제 시 임계거리 배수 (경계에서 깜빡임 방지)


# ========================== 마스크 처리 ==========================
def build_masks(result, width, height, class_conf):
    out = {}
    if result.masks is None or result.boxes is None:
        return out
    names = result.names
    data = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    confs = result.boxes.conf.cpu().numpy()
    for mask, cls_id, conf in zip(data, classes, confs):
        if conf < class_conf.get(names[cls_id], 1.1):
            continue
        resized = cv2.resize(mask.astype(np.uint8), (width, height),
                             interpolation=cv2.INTER_NEAREST)
        out.setdefault(names[cls_id], []).append(resized)
    return out

def pick_main(masks):
    return max(masks, key=lambda m: int(m.sum())) if masks else None


def _small(mask):
    return mask[::4, ::4].astype(bool)


def _iou(a, b):
    if a is None or b is None:
        return 0.0
    union = int((a | b).sum())
    return int((a & b).sum()) / union if union else 0.0


def assign_lanes(lefts, rights, prev_left, prev_right):
    """왼쪽/오른쪽 후보 마스크 목록에서 각각 하나를 고른다 (직전 프레임과의 연속성 사용).

    모델이 코너에서 같은 차선을 프레임마다 left/right로 다르게 부르는 경우가 있다.
    직전 프레임의 반대편 차선과 거의 같은 마스크가 이쪽 라벨로 들어오면 반대편 후보로 옮긴다.
    반환: (left_mask, right_mask, 뒤바뀜 보정 횟수)
    """
    flips = 0
    new_l, new_r = [], []
    for m in lefts:
        sm = _small(m)
        if prev_right is not None and _iou(sm, prev_right) >= FLIP_IOU \
                and _iou(sm, prev_right) > _iou(sm, prev_left) + FLIP_MARGIN:
            new_r.append(m)
            flips += 1
        else:
            new_l.append(m)
    for m in rights:
        sm = _small(m)
        if prev_left is not None and _iou(sm, prev_left) >= FLIP_IOU \
                and _iou(sm, prev_left) > _iou(sm, prev_right) + FLIP_MARGIN:
            new_l.append(m)
            flips += 1
        else:
            new_r.append(m)

    def choose(cands, prev):
        if not cands:
            return None
        if prev is not None:
            # 직전 위치와 가장 많이 겹치는 후보를 우선하되, 가장 큰 후보의 절반 이상인 것만 대상으로 한다
            # (작은 조각이 직전 위치와 겹친다는 이유로 큰 차선 대신 선택되는 것을 막는다)
            biggest = int(pick_main(cands).sum())
            eligible = [m for m in cands if int(m.sum()) * 2 >= biggest]
            best = max(eligible, key=lambda m: _iou(_small(m), prev))
            if _iou(_small(best), prev) > 0.0:
                return best
        return pick_main(cands)
    return choose(new_l, prev_left), choose(new_r, prev_right), flips

def x_at_row(mask, y):
    if mask is None or not (0 <= y < mask.shape[0]):
        return None
    xs = np.flatnonzero(mask[y])
    return float(xs.mean()) if xs.size else None


def lane_angle(mask):
    """차선 마스크의 주방향이 수직(화면 위아래)에서 얼마나 기울었는지(rad).

    행 평균 x는 차선이 수평에 가까워지면 의미가 없어서(코너), 마스크 픽셀 전체의 PCA로 방향을 구한다.
    아래로 갈수록 x가 커지면(+), 작아지면(-). 수평에 가까우면 ±π/2.
    """
    if mask is None:
        return None
    y0 = int(mask.shape[0] * HEADING_ROI_TOP)
    ys, xs = np.nonzero(mask[y0:])
    if ys.size < HEADING_MIN_PIXELS:
        return None
    step = max(1, ys.size // HEADING_MAX_SAMPLES)
    pts = np.stack((xs[::step], ys[::step])).astype(np.float32)
    pts -= pts.mean(axis=1, keepdims=True)
    cov = pts @ pts.T / pts.shape[1]
    _, vecs = np.linalg.eigh(cov)
    dx, dy = vecs[:, 1]                  # 가장 큰 고유값의 방향
    if dy < 0:
        dx, dy = -dx, -dy
    if abs(dy) < 1e-6:
        return None                      # 완전 수평은 좌우 방향을 구분할 수 없음
    return float(np.arctan2(dx, dy))


# ========================== 차선 추적기 ==========================
class LaneTracker:
    def __init__(self, width, height, heading_gain=HEADING_GAIN, far_fallback=True):
        self.heading_gain = heading_gain
        self.far_fallback = far_fallback
        self.width = width
        self.height = height
        self.rows = [int(height * r) for r in SAMPLE_ROWS]
        self.half_width = {y: width * ROAD_HALF_INIT for y in self.rows}
        self.target_history = deque(maxlen=TARGET_HISTORY_SIZE)
        self.steer = 0.0
        self.prev_error = 0.0
        self.lost_frames = 0
        self.ref_angle = {'left': -HEADING_REF_INIT, 'right': HEADING_REF_INIT}
        self.heading_term = 0.0
        self.far = False

    def _far_half(self, y):
        """위쪽 행의 도로 반폭: 추종 행(0.6H)에서 배운 값을 원근에 맞춰 줄인다."""
        y_ref = self.rows[-1]
        y_hor = FAR_HORIZON * self.height
        return self.half_width[y_ref] * max(y - y_hor, 1.0) / max(y_ref - y_hor, 1.0)

    def update(self, left_mask, right_mask):
        centers = []
        weights = []
        both_count = 0

        for y, w in zip(self.rows, ROW_WEIGHTS):
            lx = x_at_row(left_mask, y)
            rx = x_at_row(right_mask, y)

            if lx is not None and rx is not None:
                cx = (lx + rx) / 2.0
                measured = abs(rx - lx) / 2.0
                if measured > self.width * 0.15:
                    self.half_width[y] += ROAD_HALF_SMOOTH * (measured - self.half_width[y])
                self.half_width[y] = max(self.half_width[y], self.width * 0.35)
                source = 'both'
                both_count += 1
            elif lx is not None:
                cx = lx + self.half_width[y]
                source = 'left'
            elif rx is not None:
                cx = rx - self.half_width[y]
                source = 'right'
            else:
                continue

            centers.append((int(cx), y, source))
            weights.append(w)

        self.far = False
        if self.far_fallback and not centers and (left_mask is not None or right_mask is not None):
            # 추종 행에는 없지만 차선이 화면 위쪽에 보이는 경우(코너 앞 띠 등): 위쪽 행으로 방향을 잡는다
            for fy in FAR_ROWS:
                y = int(self.height * fy)
                lx, rx = x_at_row(left_mask, y), x_at_row(right_mask, y)
                half = self._far_half(y)
                if lx is not None and rx is not None:
                    cx, source = (lx + rx) / 2.0, 'both'
                elif lx is not None:
                    cx, source = lx + half, 'left'
                elif rx is not None:
                    cx, source = rx - half, 'right'
                else:
                    continue
                centers.append((int(cx), y, source))
                weights.append(1.0)
            self.far = bool(centers)

        if not centers:
            self.lost_frames += 1
            if self.lost_frames > LOST_STOP_AFTER:
                self.steer = 0.0
            return centers, self.steer, False, 'LOST'

        self.lost_frames = 0
        total = sum(weights)
        current_target_x = sum(c[0] * w for c, w in zip(centers, weights)) / total

        self.target_history.append(current_target_x)
        smoothed_target_x = sum(self.target_history) / len(self.target_history)

        mode = 'BOTH' if both_count > 0 else ('FAR' if self.far else 'SINGLE')

        error = (smoothed_target_x - (self.width / 2.0 + CAMERA_OFFSET)) / (self.width / 2.0)

        # 차선 기울기(heading) 보정.
        # 직진 구간에서 양쪽이 보일 때 각 차선의 기준각을 학습하고, 한쪽만 보일 때는
        # 기준각 대비 얼마나 더 꺾였는지를 오차에 더해 코너를 미리 감지한다.
        # (왼쪽 차선이 더 오른쪽으로 눕거나, 오른쪽 차선이 더 세워지면 우회전 쪽 +)
        ang_l, ang_r = lane_angle(left_mask), lane_angle(right_mask)
        self.heading_term = 0.0
        if both_count > 0:
            if ang_l is not None and ang_r is not None and abs(self.steer) < 0.2:
                for side, ang in (('left', ang_l), ('right', ang_r)):
                    self.ref_angle[side] += HEADING_REF_ALPHA * (ang - self.ref_angle[side])
        else:
            side, ang = ('left', ang_l) if left_mask is not None else ('right', ang_r)
            if ang is not None:
                diff = float(np.clip(self.ref_angle[side] - ang, -HEADING_CLIP, HEADING_CLIP))
                self.heading_term = self.heading_gain * diff
                error = float(np.clip(error + self.heading_term, -1.5, 1.5))
        dynamic_gain = 0.4 + (0.8 * abs(error))

        derivative = error - self.prev_error
        self.prev_error = error
        raw = dynamic_gain * error + STEER_D_GAIN * derivative

        self.steer += STEER_SMOOTH * (raw - self.steer)
        self.steer = float(np.clip(self.steer, -1.0, 1.0))
        if self.far:
            self.steer = float(np.clip(self.steer, -FAR_STEER_MAX, FAR_STEER_MAX))

        return centers, self.steer, True, mode

    def reset(self):
        self.steer = 0.0
        self.prev_error = 0.0
        self.lost_frames = 0
        self.heading_term = 0.0
        self.target_history.clear()


# ============================ 시각화 ============================
def draw_hud(frame, centers, steer, valid, state_text, mode_text):
    height, width = frame.shape[:2]
    mid = width // 2

    cv2.line(frame, (mid, 0), (mid, height), (110, 110, 110), 1)

    for x, y, source in centers:
        color = (0, 255, 0) if source == 'both' else (0, 200, 255)
        cv2.circle(frame, (x, y), 6, color, -1)

    if len(centers) >= 2:
        pts = np.array([[x, y] for x, y, _ in centers], np.int32)
        cv2.polylines(frame, [pts], False, (0, 255, 0), 3)

    bar_y = height - 30
    cv2.rectangle(frame, (mid - 160, bar_y - 12), (mid + 160, bar_y + 12), (50, 50, 50), -1)
    tip = int(mid + np.clip(steer, -1, 1) * 160)
    cv2.rectangle(frame, (mid, bar_y - 12), (tip, bar_y + 12), (0, 165, 255), -1)
    cv2.line(frame, (mid, bar_y - 16), (mid, bar_y + 16), (255, 255, 255), 2)

    color = (255, 255, 255) if valid else (0, 0, 255)
    cv2.putText(frame, state_text, (14, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
    
    if mode_text:
        cv2.putText(frame, mode_text, (14, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2, cv2.LINE_AA)

    return frame


class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive')

        # ── 파라미터 ──────────────────────────────
        self.declare_parameter('model_path',           '/home/pinky/yolo_drive/best_ncnn_model')
        self.declare_parameter('drive_speed',          BASE_SPEED)
        self.declare_parameter('crosswalk_min_area',   CROSSWALK_MIN_AREA)  # 이 면적 비율 이상이면 횡단보도 접근으로 판단
        self.declare_parameter('crosswalk_check_time', 1.5)    # 감속하며 '물체 없음'을 확인하는 시간(s)
        self.declare_parameter('crosswalk_stop_dist',  0.30)   # 횡단보도 확인 중 이 거리 이하면 물체로 보고 정지
        self.declare_parameter('crosswalk_block_timeout', 6.0) # 횡단보도 앞 '물체'가 이 시간 넘게 그대로면 벽 등 고정물로 보고 감지를 해제(s)
        self.declare_parameter('crosswalk_rearm_time', 2.0)    # 횡단보도가 이 시간 안 보이면 다음 횡단보도 감지 재개
        self.declare_parameter('junction_conf',        0.35)   # cross_lane은 늦게 잡히므로 낮은 신뢰도도 허용
        self.declare_parameter('enable_return_home',   True)   # cross_lane 감지 시 U턴 후 출발점 복귀 (SLAM 위치 사용)
        self.declare_parameter('map_frame',            'map')
        self.declare_parameter('base_frame',           'base_footprint')
        self.declare_parameter('start_pose_wait',      15.0)   # 출발 위치(map 좌표)를 얻기 위해 정지한 채 기다리는 최대 시간(s)
        self.declare_parameter('junction_pause_time',  1.0)    # cross_lane에서 멈춘 뒤 복귀를 시작하기까지의 대기(s)
        self.declare_parameter('return_arrive_dist',   0.30)   # 출발 위치와 이 거리 이내면 도착(m)
        self.declare_parameter('turn_angular',         0.8)    # 회전 속도(rad/s)
        self.declare_parameter('junction_bias_dist',   0.8)    # 좌/우 선택 후 한쪽 차선만 따라갈 거리(m, 위치 정보 없으면 시간)
        self.declare_parameter('junction_bias_time',   8.0)
        self.declare_parameter('bias_search_angular',  0.4)    # 편향 중 차선을 놓치면 고른 쪽으로 도는 각속도(rad/s)
        self.declare_parameter('legacy_tracking',      True)   # True(기본): heading 보정/라벨 보정/FAR를 모두 끄고 예전(잘 되던) 추종과 같게 동작. False면 아래 보정을 켠다
        self.declare_parameter('far_fallback',         True)   # 추종 행에서 놓치면 위쪽 행을 보조로 사용
        self.declare_parameter('camera_config',        'video')  # 'video'(잘 되던 코드와 동일) | 'preview'
        self.declare_parameter('camera_warmup',        1.0)    # 카메라 시작 후 노출/화이트밸런스 안정을 기다리는 시간(s)
        self.declare_parameter('start_ramp_time',      2.0)    # 출발 후 이 시간 동안 직진 위주로 시작해 서서히 추종 (0이면 끔)
        self.declare_parameter('start_ramp_frames',    10)     # 그리고 최소 이 프레임 수 이상 처리한 뒤에 램프 종료
        self.declare_parameter('junction_advance_dist', 0.25)  # cross_lane에서 멈추기 전 더 전진할 거리(m, 0이면 바로 정지)
        self.declare_parameter('junction_advance_clear', 0.20) # 전진 중 앞이 이 거리 이하로 가까워지면 거기서 정지(m)
        self.declare_parameter('heading_gain',         HEADING_GAIN)   # 0.0이면 코너 기울기 보정 끔 (비교 시험용)
        self.declare_parameter('lane_flip_fix',        True)           # False면 left/right 라벨 뒤바뀜 보정 끔 (비교 시험용)
        self.declare_parameter('map_topic',            '/map') # 갈림길 판단에 쓰는 SLAM 맵 (PC에서 SLAM을 돌려도 됨)
        self.declare_parameter('map_wait_time',        5.0)    # 맵이 안 오면 이 시간 뒤 '왔던 길로 후진'으로 대체(s)
        self.declare_parameter('unknown_cost',         2.0)    # 미탐색 칸 비용 배수 (클수록 가본 길을 선호)
        self.declare_parameter('inflate_m',            0.08)   # 벽 주변 여유(m)
        self.declare_parameter('junction_min_width',   0.60)   # cross_lane이 화면 너비의 이 비율 이상 가로지르면 갈림길
        self.declare_parameter('caution_speed_factor', 0.5)    # 감속 배율
        self.declare_parameter('collision_dist',       0.05)   # 이하 → 충돌
        self.declare_parameter('ultrasonic_stop_dist', 0.10)   # 이하 → 정지
        self.declare_parameter('obstacle_dist',        0.35)   # 이하 → 감속
        self.declare_parameter('yolo_imgsz',           320)
        self.declare_parameter('yolo_conf',            0.50)
        self.declare_parameter('camera_width',         640)
        self.declare_parameter('camera_height',        480)
        self.declare_parameter('enable_lcd',           True)
        self.declare_parameter('lcd_every_n',          3)      # LCD는 N프레임에 한 번만 갱신 (리사이즈+SPI 전송이 주행 루프를 느리게 하므로)
        self.declare_parameter('lane_classes',         ['left_lane', 'right_lane'])
        self.declare_parameter('crosswalk_classes',    ['crosswalk'])
        self.declare_parameter('junction_classes',     ['cross_lane'])
        self.declare_parameter('ultrasonic_topic',     'us_sensor/range')   # ADC 보드 초음파(pinky_sensor_adc). GPIO 센서는 'ultrasonic/range'
        self.declare_parameter('ultrasonic_scale',     1.43)   # 센서 값 보정: 거리 = scale * range + offset (실측: 10cm→0.07, 20cm→0.14)
        self.declare_parameter('ultrasonic_offset',    0.0)

        p = self.get_parameter
        self.model_path        = p('model_path').value
        self.drive_speed       = p('drive_speed').value
        self.crosswalk_min_area = p('crosswalk_min_area').value
        self.cw_check_time     = p('crosswalk_check_time').value
        self.cw_stop_dist      = p('crosswalk_stop_dist').value
        self.cw_rearm_time     = p('crosswalk_rearm_time').value
        self.cw_block_timeout  = p('crosswalk_block_timeout').value
        self.junction_conf     = p('junction_conf').value
        self.return_enabled    = p('enable_return_home').value
        self.map_frame         = p('map_frame').value
        self.base_frame        = p('base_frame').value
        self.start_pose_wait   = p('start_pose_wait').value
        self.junction_pause    = p('junction_pause_time').value
        self.arrive_dist       = p('return_arrive_dist').value
        self.turn_angular      = p('turn_angular').value
        self.bias_dist         = p('junction_bias_dist').value
        self.bias_time         = p('junction_bias_time').value
        self.bias_search_w     = p('bias_search_angular').value
        self.legacy            = p('legacy_tracking').value
        self.lane_flip_fix     = p('lane_flip_fix').value and not self.legacy
        self.far_fallback      = p('far_fallback').value and not self.legacy
        self.camera_config     = p('camera_config').value
        self.camera_warmup     = p('camera_warmup').value
        self.ramp_time         = p('start_ramp_time').value
        self.ramp_frames       = p('start_ramp_frames').value
        self.advance_dist      = p('junction_advance_dist').value
        self.advance_clear     = p('junction_advance_clear').value
        self.map_topic         = p('map_topic').value
        self.map_wait          = p('map_wait_time').value
        self.unknown_cost      = p('unknown_cost').value
        self.inflate_m         = p('inflate_m').value
        self.junction_min_width = p('junction_min_width').value
        self.caution_factor    = p('caution_speed_factor').value
        self.collision_dist    = p('collision_dist').value
        self.stop_dist         = p('ultrasonic_stop_dist').value
        self.obstacle_dist     = p('obstacle_dist').value
        self.yolo_imgsz        = p('yolo_imgsz').value
        self.yolo_conf         = p('yolo_conf').value
        self.cam_w             = p('camera_width').value
        self.cam_h             = p('camera_height').value
        self.enable_lcd        = p('enable_lcd').value
        self.lcd_every_n       = max(1, int(p('lcd_every_n').value))
        self.lane_classes      = list(p('lane_classes').value)
        self.crosswalk_classes = list(p('crosswalk_classes').value)
        self.junction_classes  = list(p('junction_classes').value)
        self.sonar_scale       = p('ultrasonic_scale').value
        self.sonar_offset      = p('ultrasonic_offset').value
        self.wanted_classes    = set(self.lane_classes) | set(self.crosswalk_classes) | set(self.junction_classes)
        # 클래스별 최소 신뢰도: 모델에는 가장 낮은 값을 주고, 여기서 클래스별로 거른다
        self.class_conf = {c: self.yolo_conf for c in self.lane_classes + self.crosswalk_classes}
        self.class_conf.update({c: self.junction_conf for c in self.junction_classes})
        self.model_conf = min(self.class_conf.values())

        # ── 상태 변수 ─────────────────────────────
        self.state            = STATE_DRIVING
        self.ultrasonic_dist  = float('inf')
        self.sonar_stamp      = 0.0
        self.cw_armed         = True    # 횡단보도 감지 가능 여부 (통과 확인 후 해제, 안 보이면 재무장)
        self.cw_check_active  = False
        self.cw_clear_since   = None
        self.cw_block_t0      = None
        self.last_cw_seen     = 0.0
        self.junction_frames  = 0
        self.junction_latched = False
        self.start_pose       = None      # (x, y, yaw) in map frame
        self.pose_frame       = self.map_frame     # 위치 기준 프레임 (SLAM 'map', 없으면 'odom')
        self.return_phase     = None      # None → pause → turning → following → arrived / failed
        self.return_t0        = 0.0
        self.turn_target      = None
        self.turn_delta       = math.pi   # 이번 회전량(rad): 좌 +π/2, 우 −π/2, 후진 π
        self.exit_choice      = None
        self.lane_bias        = None      # None | 'left' | 'right': 갈림길 통과 중 이쪽 차선만 추종
        self.bias_t0          = 0.0
        self.bias_pose        = None
        self.ramp_t0          = None      # 출발 직진 램프 시작 시각 (첫 추론 프레임)
        self.ramp_n           = 0
        self.advance_pose     = None      # cross_lane 확정 후 전진 시작 위치
        self.advance_t0       = 0.0
        self.last_det         = ''        # 로그용: 이번 프레임 검출 요약
        self.map_sub          = None
        self.latest_map       = None
        self.tf_buffer        = None
        self.prev_lane        = {'left': None, 'right': None}   # 직전 프레임 차선 마스크(1/4 해상도)
        self.swap_lanes       = False     # U턴 후에는 화면의 왼쪽/오른쪽 차선 클래스가 뒤바뀐다
        self.sonar_msgs       = 0
        self.sonar_hist       = deque(maxlen=3)   # 아날로그/ADC 센서의 튀는 값을 거르는 중앙값 필터용
        self.last_sonar_warn  = 0.0
        self.boot_time        = time.monotonic()   # 주행 타이머 시작 시점으로 다시 설정됨
        self.camera           = None
        self.lcd              = None
        self._last_lamp       = None
        self._last_lamp_time  = 0.0
        self.tracker          = LaneTracker(self.cam_w, self.cam_h,
                                        heading_gain=0.0 if self.legacy else p('heading_gain').value,
                                        far_fallback=self.far_fallback)

        # ── ROS2 인터페이스 (무거운 초기화보다 먼저: 실행 직후 LED부터 켠다) ──
        self.cmd_pub   = self.create_publisher(Twist,           'cmd_vel',           10)
        self.video_pub = self.create_publisher(CompressedImage, 'camera/compressed', 10)
        self.sonar_sub = self.create_subscription(
            Range, p('ultrasonic_topic').value, self._sonar_callback, 10)

        self.lamp_cli = self.create_client(SetLamp, 'set_lamp')
        if not self.lamp_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('⚠️ set_lamp 서비스 미연결. 연결되면 자동으로 LED 제어를 재개합니다.')
        self._apply_lamp(LAMP_DRIVE)

        if self.return_enabled:
            self._init_return_home()

        # ── YOLO 모델 로드 ────────────────────────
        from ultralytics import YOLO
        self.get_logger().info(f'🧠 YOLO 모델 로드 중: {self.model_path}')
        self.model = YOLO(self.model_path, task='segment')
        self.get_logger().info('✅ YOLO 모델 로드 완료.')

        try:
            # ── 카메라 초기화 ─────────────────────────
            from picamera2 import Picamera2
            self.camera = Picamera2()
            make_cfg = (self.camera.create_video_configuration if self.camera_config == 'video'
                        else self.camera.create_preview_configuration)
            cfg = make_cfg(main={'size': (self.cam_w, self.cam_h), 'format': 'RGB888'})
            self.camera.configure(cfg)
            self.camera.start()
            time.sleep(self.camera_warmup)      # 노출/화이트밸런스 안정 (청록 차선이 흰색처럼 보이는 오인 완화)
            self.get_logger().info(f'📸 카메라 시작됨 ({self.camera_config}, 안정화 {self.camera_warmup:.1f}s).')

            # ── LCD 초기화 ────────────────────────────
            if self.enable_lcd:
                try:
                    from pinky_emotion.pinky_lcd import LCD
                    self.lcd = LCD()
                    self.get_logger().info('📺 LCD 초기화 완료.')
                except Exception as e:
                    self.get_logger().warn(f'⚠️ LCD 초기화 실패 (스킵): {e}')
        except Exception:
            self.shutdown_hardware()   # 초기화 도중 실패해도 카메라 점유를 풀고 종료
            raise

        self.frames = 0
        self.start_time = time.monotonic()
        self.boot_time = self.start_time        # 모델/카메라 로딩 시간은 '출발 위치 대기'에 포함하지 않는다

        # ── 주행 루프 타이머 (10 Hz) ──────────────
        self.drive_timer = self.create_timer(0.10, self._drive_loop)
        self.get_logger().info('🚀 자율주행 노드 준비 완료! (주행 시작)')

    # ──────────────────────────────────────────
    # 출발점 복귀: U턴 → 차선 추종 → SLAM 위치로 도착 판정
    #   (Nav2는 바닥 차선을 모르고 미탐색 영역을 가로지르는 경로를 만들 수 있어 쓰지 않는다)
    # ──────────────────────────────────────────
    def _init_return_home(self):
        try:
            from tf2_ros import Buffer, TransformListener
            from rclpy.time import Time
        except ImportError as e:
            self.get_logger().warn(f'⚠️ 출발점 복귀 비활성화 (tf2_ros 없음): {e}')
            self.return_enabled = False
            return
        self._Time = Time
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self._try_record_start_pose)
        self.get_logger().info('🗺️ 출발점 복귀 활성화: 출발 위치를 SLAM(map) 좌표로 기억합니다.')

    def _lookup_pose(self, frame):
        try:
            t = self.tf_buffer.lookup_transform(frame, self.base_frame, self._Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _current_pose(self):
        return self._lookup_pose(self.pose_frame) if self.tf_buffer is not None else None

    def _try_record_start_pose(self):
        """map→base 변환을 얻으면 그 첫 값을 출발 위치로 저장한다 (주행 시작 전, 정지 상태)."""
        if self.start_pose is not None or self.tf_buffer is None:
            return
        pose = self._lookup_pose(self.map_frame)
        if pose is None and time.monotonic() - self.boot_time > 5.0:
            pose = self._lookup_pose('odom')        # SLAM이 안 뜬 경우: 바퀴 오도메트리로 대체
            if pose is not None:
                self.pose_frame = 'odom'
                self.get_logger().warn('⚠️ SLAM(map) 변환이 없어 odom 기준으로 출발 위치를 기억합니다. (누적 오차 가능)')
        if pose is None:
            return
        self.start_pose = pose
        self.get_logger().info(
            f'📍 출발 위치 저장 ({self.pose_frame}): x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.0f}°')

    def _waiting_for_start_pose(self, now):
        """출발 위치를 얻을 때까지 정지. 제한 시간이 지나면 복귀 기능만 끄고 주행한다."""
        if not self.return_enabled or self.start_pose is not None:
            return False
        if now - self.boot_time > self.start_pose_wait:
            self.get_logger().warn('⚠️ 출발 위치를 얻지 못해 복귀 기능을 끄고 주행합니다. SLAM이 실행 중인지 확인하세요.')
            self.return_enabled = False
            return False
        return True

    def _return_step(self, now, sonar_blocked):
        """cross_lane 확정 직후: 정지 → 맵으로 방향 판단 → 회전 시작을 관리한다."""
        if not self.return_enabled or self.start_pose is None:
            if self.return_phase is None:
                self.return_phase = 'failed'
                self.get_logger().warn('cross_lane 감지: 출발점 복귀를 쓸 수 없어 정지합니다.')
            return
        if self.return_phase is None:
            self._ensure_map_sub()                    # 맵은 갈림길에서만 구독한다 (평소 부하 없음)
            if self.advance_dist > 0:
                # cross_lane 정지선 바로 앞에서 멈추면 갈림길 안쪽 차선이 안 보여 판단/진행이 어렵다.
                # 조금 더 전진해 교차로 안으로 들어간 뒤에 멈춘다.
                self.return_phase, self.advance_t0, self.advance_pose = 'advance', now, self._current_pose()
                self.get_logger().info(f'➡️ cross_lane 감지: {self.advance_dist:.2f}m 더 전진한 뒤 정지합니다.')
            else:
                self.return_phase, self.return_t0 = 'pause', now
        elif self.return_phase == 'pause' and now - self.return_t0 >= self.junction_pause and not sonar_blocked:
            self.return_phase, self.return_t0 = 'planning', now
        elif self.return_phase == 'planning':
            self._plan_exit(now)

    def _ensure_map_sub(self):
        if self.map_sub is not None:
            return
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        # slam_toolbox는 맵을 transient_local로 발행하므로 늦게 구독해도 마지막 맵을 바로 받는다
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, qos)

    def _map_callback(self, msg):
        self.latest_map = msg

    def _plan_exit(self, now):
        """SLAM 맵에서 출발점까지 최단 경로가 갈림길을 어느 쪽으로 떠나는지 보고 좌/우/후진을 정한다."""
        pose = self._current_pose()
        choice, reason = None, ''
        if self.pose_frame != self.map_frame:
            reason = 'SLAM(map) 위치가 없음'
        elif self.latest_map is not None and pose is not None:
            m = self.latest_map
            grid = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            t0 = time.monotonic()
            choice, info = choose_exit(grid, m.info.resolution,
                                       (m.info.origin.position.x, m.info.origin.position.y),
                                       pose, self.start_pose[:2], self.unknown_cost, self.inflate_m)
            self.get_logger().info(
                f'🧭 갈림길 판단: {choice} ({time.monotonic() - t0:.2f}s) {info}')
            if choice is None:
                reason = '맵에서 출발점까지 경로를 찾지 못함'
        elif now - self.return_t0 > self.map_wait:
            reason = f'{self.map_wait:.0f}초 안에 맵({self.map_topic})을 받지 못함'
        else:
            return                                    # 맵/위치를 기다리는 중
        if choice is None:
            choice = 'back'                           # 가장 안전한 기본값: 왔던 길로 돌아간다
            self.get_logger().warn(f'⚠️ {reason}: 왔던 길로 되돌아갑니다(후진 U턴).')
        self._begin_turn(choice, pose, now)

    def _begin_turn(self, choice, pose, now):
        self.exit_choice = choice
        self.turn_delta = {'left': math.pi / 2, 'right': -math.pi / 2, 'back': math.pi, 'straight': 0.0}[choice]
        if self.map_sub is not None:                  # 판단이 끝났으니 맵 구독 해제
            self.destroy_subscription(self.map_sub)
            self.map_sub, self.latest_map = None, None
        if choice in ('straight', 'left', 'right'):
            self._start_following(now, bias=choice if choice in ('left', 'right') else None)
            return
        self.turn_target = None if pose is None else self._wrap(pose[2] + self.turn_delta)
        self.return_phase, self.return_t0 = 'turning', now
        self.get_logger().info(f'↩️ {choice} 방향으로 회전 시작 ({math.degrees(self.turn_delta):+.0f}°)')

    def _start_following(self, now, bias=None):
        self.return_phase, self.return_t0 = 'following', now
        self.swap_lanes = self.exit_choice == 'back'  # U턴하면 화면의 좌우 차선 클래스가 뒤바뀐다
        self.prev_lane = {'left': None, 'right': None}
        self.tracker.reset()
        self.lane_bias, self.bias_t0, self.bias_pose = bias, now, self._current_pose()
        if bias:
            self.get_logger().info(
                f'✅ 갈림길 {bias} 선택: 제자리 회전 없이 {bias}쪽 차선만 따라 {self.bias_dist:.1f}m 진행합니다.')
        else:
            self.get_logger().info(
                f'✅ 차선을 따라 출발점으로 복귀합니다 ({self.exit_choice}, 좌우 교환={self.swap_lanes}).')

    def _update_bias(self, now):
        """갈림길을 빠져나갈 만큼 갔으면 다시 양쪽 차선을 모두 본다."""
        if self.lane_bias is None:
            return
        pose, moved = self._current_pose(), None
        if pose is not None and self.bias_pose is not None:
            moved = math.hypot(pose[0] - self.bias_pose[0], pose[1] - self.bias_pose[1])
        if (moved is not None and moved >= self.bias_dist) or now - self.bias_t0 >= self.bias_time:
            self.get_logger().info(f'↔️ 갈림길 통과 ({"%.2fm" % moved if moved is not None else "시간 기준"}): 양쪽 차선 추종으로 복귀.')
            self.lane_bias = None

    def _advance_speed(self, now):
        """cross_lane 이후 직진 속도. 목표 거리(위치 정보 없으면 시간)에 닿거나 앞이 가까우면 멈춤 단계로 넘어간다."""
        v = self.drive_speed * 0.5
        pose, moved = self._current_pose(), None
        if pose is not None and self.advance_pose is not None:
            moved = math.hypot(pose[0] - self.advance_pose[0], pose[1] - self.advance_pose[1])
        elapsed = now - self.advance_t0
        ahead = self._sonar_distance(now)
        done = ((moved is not None and moved >= self.advance_dist)
                or (moved is None and elapsed * v >= self.advance_dist)
                or elapsed > self.advance_dist / max(v, 0.02) + 3.0          # 위치가 멈춰 있어도 무한 전진 방지
                or ahead <= self.advance_clear)
        if done:
            self.return_phase, self.return_t0 = 'pause', now
            self.get_logger().info(
                f'⏹️ 전진 종료 ({"%.2fm" % moved if moved is not None else "%.1fs" % elapsed}, 앞 {ahead:.2f}m): 정지 후 방향을 판단합니다.')
            return 0.0
        return v

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _turn_command(self, now):
        """제자리 회전의 각속도. 목표 방향에 도달하면 차선 추종(following)으로 넘어간다."""
        pose = self._current_pose()
        if self.turn_target is not None and pose is not None:
            err = self._wrap(self.turn_target - pose[2])
            done = abs(err) < 0.15
            w = float(np.clip(1.5 * err, -self.turn_angular, self.turn_angular))
            w = math.copysign(max(abs(w), 0.3), err)      # 너무 느리면 멈춰버리므로 최소 속도 보장
        else:                                              # 위치 정보 없으면 시간으로 회전량을 맞춘다
            done = (now - self.return_t0) * self.turn_angular >= abs(self.turn_delta)
            w = math.copysign(self.turn_angular, self.turn_delta)
        if now - self.return_t0 > 20.0:
            self.return_phase = 'failed'
            self.get_logger().error('❌ 회전이 20초 안에 끝나지 않아 정지합니다.')
            return 0.0
        if done:
            self._start_following(now)
            return 0.0
        return w

    def _check_arrival(self, now):
        """복귀 주행 중 SLAM 위치가 출발점 근처면 도착."""
        if now - self.return_t0 < 3.0:                     # 막 U턴을 끝낸 직후의 오판 방지
            return
        pose = self._current_pose()
        if pose is None:
            return
        d = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])
        if d <= self.arrive_dist:
            self.return_phase = 'arrived'
            self.get_logger().info(f'🏠 출발점 도착 (거리 {d:.2f}m).')

    def _capture_bgr(self):
        frame_rgb = self.camera.capture_array()
        frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    # ──────────────────────────────────────────
    # 인식
    # ──────────────────────────────────────────
    def _detect(self):
        """카메라 프레임 → YOLO → (result, left_mask, right_mask, crosswalk_area, junction_width)"""
        frame_bgr = self._capture_bgr()

        result = self.model(frame_bgr, imgsz=self.yolo_imgsz, conf=self.model_conf, verbose=False)[0]
        try:                                         # 클래스별 기준에 못 미치는 박스는 HUD에도 그리지 않는다 (추종에 안 쓰이므로)
            names, confs = result.names, result.boxes.conf.cpu().numpy()
            clses = result.boxes.cls.cpu().numpy().astype(int)
            keep = [i for i, (c, p) in enumerate(zip(clses, confs)) if p >= self.class_conf.get(names[c], 1.1)]
            if len(keep) < len(confs):
                result = result[keep]
        except Exception:
            pass
        masks = build_masks(result, self.cam_w, self.cam_h, self.class_conf)

        if self.lane_flip_fix:
            left_mask, right_mask, flips = assign_lanes(
                masks.get(self.lane_classes[0], []), masks.get(self.lane_classes[1], []),
                self.prev_lane['left'], self.prev_lane['right'])
        else:
            left_mask, right_mask, flips = (pick_main(masks.get(self.lane_classes[0])),
                                            pick_main(masks.get(self.lane_classes[1])), 0)
        if flips:
            self.get_logger().info(f'🔁 left/right 라벨 뒤바뀜 {flips}건 보정', throttle_duration_sec=1.0)
        self.prev_lane['left'] = None if left_mask is None else _small(left_mask)
        self.prev_lane['right'] = None if right_mask is None else _small(right_mask)
        if self.swap_lanes:                          # U턴 후: 화면 왼쪽에 보이는 것은 right_lane 클래스
            left_mask, right_mask = right_mask, left_mask
        if self.lane_bias == 'left':                 # 갈림길 통과 중: 고른 쪽 차선만 따라간다
            right_mask = None
        elif self.lane_bias == 'right':
            left_mask = None
        n = {c: len(v) for c, v in masks.items()}
        self.last_det = (f'L{n.get(self.lane_classes[0], 0)}R{n.get(self.lane_classes[1], 0)}'
                         f'CW{sum(n.get(c, 0) for c in self.crosswalk_classes)}'
                         f'JN{sum(n.get(c, 0) for c in self.junction_classes)}')

        crosswalk_area = 0.0
        cw_mask = pick_main([m for c in self.crosswalk_classes for m in masks.get(c, [])])
        if cw_mask is not None:
            center_strip = cw_mask[:, self.cam_w // 3: self.cam_w * 2 // 3]
            if center_strip.sum() > 100:
                crosswalk_area = float(cw_mask.sum()) / (self.cam_w * self.cam_h)

        # cross_lane은 화면을 가로지르는 띠이므로 면적보다 가로 폭으로 판단한다
        junction_width = 0.0
        jn_mask = pick_main([m for c in self.junction_classes for m in masks.get(c, [])])
        if jn_mask is not None:
            junction_width = float(jn_mask.any(axis=0).sum()) / self.cam_w
        return result, left_mask, right_mask, crosswalk_area, junction_width

    # ──────────────────────────────────────────
    # 상태 결정
    # ──────────────────────────────────────────
    def _sonar_text(self, now):
        """로그/HUD용: 센서 수신 여부와 값을 구분해서 보여준다 (무한대와 '수신 없음'을 구분)."""
        if self.sonar_msgs == 0 or now - self.sonar_stamp > SONAR_TIMEOUT:
            return 'NO-DATA'
        d = self.ultrasonic_dist
        return 'clear' if math.isinf(d) else f'{d:.2f}m'

    def _sonar_distance(self, now):
        if now - self.sonar_stamp > SONAR_TIMEOUT:
            return float('inf')            # 센서 노드 죽음/미수신 → 무효
        return self.ultrasonic_dist

    def _update_crosswalk(self, now, crosswalk_area, dist):
        """횡단보도 접근 감지 → 감속하며 앞 확인 → 물체 없으면 정속 복귀. (확인 중 물체가 있으면 정지)"""
        seen = crosswalk_area >= self.crosswalk_min_area
        if seen:
            self.last_cw_seen = now
        elif not self.cw_armed and now - self.last_cw_seen > self.cw_rearm_time:
            self.cw_armed = True                      # 횡단보도를 지나쳤으니 다음 것을 감지할 수 있게 재무장

        if not self.cw_check_active:
            if seen and self.cw_armed:
                self.cw_check_active = True
                self.cw_clear_since = None
                self.get_logger().info(f'🚸 횡단보도 접근 ({crosswalk_area:.1%}): 감속하며 앞을 확인합니다.')
            return False

        hold = STOP_HYSTERESIS if self.state == STATE_CW_BLOCKED else 1.0
        blocked = dist <= self.cw_stop_dist * hold
        if blocked:
            self.cw_clear_since = None
            if self.cw_block_t0 is None:
                self.cw_block_t0 = now
            elif now - self.cw_block_t0 > self.cw_block_timeout:
                # 물체가 아니라 벽처럼 계속 그 자리에 있는 것: 영구 정지(교착)를 막고 일반 규칙(감속 후 0.10m에서 정지)으로 넘긴다
                self.cw_check_active = False
                self.cw_armed = False
                self.cw_block_t0 = None
                self.get_logger().warn(
                    f'⚠️ 횡단보도 앞 {dist:.2f}m의 물체가 {self.cw_block_timeout:.0f}초 넘게 그대로입니다: '
                    '고정물로 보고 횡단보도 확인을 해제합니다 (일반 감속/정지 규칙은 유지).')
                return False
            return blocked
        elif self.cw_clear_since is None:
            self.cw_block_t0 = None
            self.cw_clear_since = now
        elif now - self.cw_clear_since >= self.cw_check_time:
            self.cw_check_active = False
            self.cw_armed = False                     # 같은 횡단보도에서 다시 감속하지 않는다
            self.get_logger().info('✅ 횡단보도 위 물체 없음 확인: 정속 주행 복귀.')
        return blocked

    def _decide_state(self, now, crosswalk_area, junction_width):
        dist = self._sonar_distance(now)
        cw_blocked = self._update_crosswalk(now, crosswalk_area, dist)

        # cross_lane: 몇 프레임 연속으로 가로지를 때만 인정 (순간 오검출 방지), 한번 인정되면 유지
        self.junction_frames = self.junction_frames + 1 if junction_width >= self.junction_min_width else 0
        if self.junction_frames >= JUNCTION_CONFIRM_FRAMES:
            self.junction_latched = True

        # 이미 정지 상태면 해제 임계값을 키워 경계에서 상태가 떨리지 않게 한다
        k_col  = STOP_HYSTERESIS if self.state == STATE_COLLISION else 1.0
        k_stop = STOP_HYSTERESIS if self.state in (STATE_COLLISION, STATE_OBSTACLE_STOP) else 1.0

        if dist <= self.collision_dist * k_col:
            new = STATE_COLLISION
        elif dist <= self.stop_dist * k_stop:
            new = STATE_OBSTACLE_STOP
        elif self.junction_latched and self.return_phase != 'following':
            new = {'turning': STATE_RETURN_HOME, 'advance': STATE_RETURN_HOME, 'arrived': STATE_HOME,
                   'failed': STATE_RETURN_FAILED}.get(self.return_phase, STATE_JUNCTION)
        elif cw_blocked:
            new = STATE_CW_BLOCKED
        elif self.cw_check_active or dist <= self.obstacle_dist:
            new = STATE_CAUTION
        else:
            new = STATE_DRIVING

        if new != self.state:
            self.get_logger().info(
                f'상태 전환: {self.state} → {new} (초음파 {dist:.2f}m, 횡단보도 {crosswalk_area:.1%}, '
                f'cross_lane 폭 {junction_width:.0%})')
            self.state = new

    def _compute_command(self, steer, valid):
        """(speed, angular, stopped) 계산"""
        if self.state in (STATE_COLLISION, STATE_OBSTACLE_STOP, STATE_CW_BLOCKED, STATE_JUNCTION,
                          STATE_HOME, STATE_RETURN_FAILED):
            return 0.0, 0.0, True

        if valid:
            speed = max(self.drive_speed * (1.0 - CORNER_SLOWDOWN * min(abs(steer), 1.0)),
                        self.drive_speed / 2.0)
        else:
            lost = self.tracker.lost_frames
            if lost > LOST_STOP_AFTER:
                return 0.0, 0.0, True
            if self.lane_bias and lost > 3:          # 갈림길 통과 중: 고른 쪽으로 천천히 돌며 차선을 찾는다
                w = self.bias_search_w if self.lane_bias == 'left' else -self.bias_search_w
                return self.drive_speed * 0.5 * (self.caution_factor if self.state == STATE_CAUTION else 1.0), w, False
            speed = self.drive_speed / 2.0 if lost > LOST_SLOW_AFTER else self.drive_speed * 0.7

        # 감속할 때는 각속도도 같은 비율로 줄여 회전 곡률(각속도/속도)을 유지한다.
        # 속도만 줄이면 같은 조향이 더 급한 회전이 되어 코너에서 과하게 꺾인다.
        f = 1.0
        if self.state == STATE_CAUTION:
            f *= self.caution_factor
        if valid and self.tracker.far:
            f *= FAR_SPEED_FACTOR                   # 차선이 멀리서만 보이면 코너 직전이므로 감속
        return speed * f, -STEER_TO_ANGULAR * steer * f, False

    # ──────────────────────────────────────────
    # 메인 루프
    # ──────────────────────────────────────────
    def _output_view(self, view):
        if self.lcd is not None and self.frames % self.lcd_every_n == 0:
            try:
                self.lcd.img_show(PILImage.fromarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB)))
            except Exception as e:
                self.get_logger().warn(f'LCD 출력 에러 (무시됨): {e}', throttle_duration_sec=2.0)
        if self.video_pub.get_subscription_count() > 0:
            self._publish_compressed(view)

    def _return_loop(self, now):
        """cross_lane 확정 후 U턴이 끝날 때까지(또는 도착/실패 후): 추론 없이 정지/회전만 관리."""
        self._decide_state(now, 0.0, 0.0)
        blocked = self.state in (STATE_COLLISION, STATE_OBSTACLE_STOP)
        self._return_step(now, blocked)

        twist = Twist()
        if self.return_phase == 'turning' and not blocked:
            twist.angular.z = float(self._turn_command(now))
        elif self.return_phase == 'advance' and not blocked:
            twist.linear.x = float(self._advance_speed(now))
        self.cmd_pub.publish(twist)
        self._apply_lamp(LAMP_DRIVE if self.state == STATE_RETURN_HOME else LAMP_STOP)

        text = f'{self.state} ({self.return_phase})'
        if self.lcd is not None or self.video_pub.get_subscription_count() > 0:
            view = draw_hud(self._capture_bgr(), [], 0.0, True, text, f'sonar {self._sonar_text(now)}')
            self._output_view(view)

        self.frames += 1
        if self.frames % 20 == 0:
            self.get_logger().info(f'{text} | sonar {self._sonar_text(now)}')

    def _ramp_progress(self, now):
        """출발 직후 0→1. 시간(start_ramp_time)과 처리 프레임 수(start_ramp_frames)가 모두 차야 1이 된다.
        초반에는 모델 워밍업으로 루프가 느리므로 시간만으로는 부족하다."""
        if self.ramp_time <= 0:
            return 1.0
        if self.ramp_t0 is None:
            self.ramp_t0 = now
        self.ramp_n += 1
        return min(1.0, (now - self.ramp_t0) / self.ramp_time, self.ramp_n / max(self.ramp_frames, 1))

    def _drive_loop(self):
        now = time.monotonic()
        if self._waiting_for_start_pose(now):       # 출발 위치를 기억하기 전에는 움직이지 않는다
            self.cmd_pub.publish(Twist())
            return
        if self.return_phase == 'following':
            self._update_bias(now)
            self._check_arrival(now)
        if self.junction_latched and self.return_phase != 'following':
            self._return_loop(now)
            return

        try:
            result, left_mask, right_mask, crosswalk_area, junction_width = self._detect()
        except Exception as e:
            self.get_logger().error(f'인식 실패 → 정지: {e}', throttle_duration_sec=2.0)
            self.cmd_pub.publish(Twist())
            self._apply_lamp(LAMP_STOP)
            return

        now = time.monotonic()
        if self._sonar_text(now) == 'NO-DATA' and now - self.last_sonar_warn > 5.0:
            self.last_sonar_warn = now
            self.get_logger().warn('⚠️ 초음파 값이 들어오지 않습니다 (센서 노드/배선 확인). 초음파 정지는 동작하지 않습니다.')
        self._decide_state(now, crosswalk_area, junction_width)
        centers, steer, valid, lane_mode = self.tracker.update(left_mask, right_mask)
        if not valid and (left_mask is not None or right_mask is not None):
            lane_mode = 'LOST*'                      # 차선은 보이는데 추종 행(화면 60~90%)에 걸리지 않는 경우를 구분
        speed, angular, stopped = self._compute_command(steer, valid)
        ramp = self._ramp_progress(now)
        if ramp < 1.0 and not stopped:
            # 출발 직후: 속도를 서서히 올리고 조향 권한을 0→1로 올려, 목표점과 실제 중심이 안 맞는 초기 흔들림을 줄인다
            speed = min(speed, self.drive_speed * (0.5 + 0.5 * ramp))
            angular *= ramp
        angular = float(np.clip(angular, -MAX_ANGULAR, MAX_ANGULAR))

        # 로봇 제어 + LED (LED는 항상 실제 동작에서 파생: 정지=빨강, 감속=주황, 주행=초록)
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

        if stopped:
            self._apply_lamp(LAMP_STOP)
        elif self.state == STATE_CAUTION:
            self._apply_lamp(LAMP_CAUTION)
        else:
            self._apply_lamp(LAMP_DRIVE)

        state_text = 'STOPPED' if stopped else ('SLOW' if self.state == STATE_CAUTION else 'DRIVE')
        if self.return_phase == 'following' and not stopped:
            state_text = 'RETURN'
        note = (f'{self.state} sonar {self._sonar_text(now)} det {self.last_det}'
                + (f' bias={self.lane_bias}' if self.lane_bias else '')
                + (f' ramp={ramp:.1f}' if ramp < 1.0 else ''))

        # 영상 출력은 보는 사람이 있을 때만 그린다 (result.plot / JPEG 인코딩 비용 절약)
        if self.lcd is not None or self.video_pub.get_subscription_count() > 0:
            self._output_view(draw_hud(result.plot(), centers, steer, valid, state_text, f'{lane_mode} {note}'))

        self.frames += 1
        if self.frames % 20 == 0:
            fps = self.frames / (time.monotonic() - self.start_time)
            self.get_logger().info(
                f'{fps:4.1f}fps | {state_text:7s} | {lane_mode:6s} | '
                f'steer {steer:+.2f} | v {speed:.2f} w {angular:+.2f} | {note}'
            )

    # ──────────────────────────────────────────
    # 콜백 / 유틸
    # ──────────────────────────────────────────
    def _sonar_callback(self, msg: Range):
        self.sonar_msgs += 1
        self.sonar_stamp = time.monotonic()
        r = float(msg.range)
        if math.isnan(r) or (msg.max_range > 0 and r >= msg.max_range):
            r = float('inf')                         # 측정 범위 밖 = 장애물 없음
        else:
            # 센서 값 보정 후 음수는 0으로. min_range 이하처럼 아주 가까운 값도 유효한 근접 값으로 취급한다
            r = max(0.0, r * self.sonar_scale + self.sonar_offset)
        self.sonar_hist.append(r)
        # 최근 3개의 중앙값: 한 번 튀는 값으로 정지/충돌 상태가 되지 않게 한다
        self.ultrasonic_dist = float(sorted(self.sonar_hist)[len(self.sonar_hist) // 2])

    def _publish_compressed(self, frame_bgr: np.ndarray):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        ok, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            msg.data = buf.tobytes()
            self.video_pub.publish(msg)

    def _apply_lamp(self, lamp):
        """같은 설정은 중복 전송하지 않되, 서비스가 안 준비됐거나 일정 시간이 지나면 다시 보낸다."""
        now = time.monotonic()
        if not self.lamp_cli.service_is_ready():
            self._last_lamp = None          # 램프 노드가 (재)시작되면 바로 다시 보내도록
            return
        if lamp == self._last_lamp and now - self._last_lamp_time < LAMP_RESEND_SEC:
            return

        r, g, b, mode, time_ms = lamp
        req = SetLamp.Request()
        req.color.r, req.color.g, req.color.b, req.color.a = float(r), float(g), float(b), 1.0
        req.mode = mode
        req.time = time_ms
        self.lamp_cli.call_async(req)
        self._last_lamp = lamp              # 실제로 전송한 뒤에만 기록
        self._last_lamp_time = now

    def shutdown_hardware(self):
        """종료 시 하드웨어 해제. 다음 실행에서 카메라/LED가 점유되지 않도록 반드시 호출."""
        try:
            for _ in range(3):              # 한 번만 보내면 유실될 수 있어 반복 발행
                self.cmd_pub.publish(Twist())
                time.sleep(0.03)
            if self.lamp_cli.service_is_ready():
                req = SetLamp.Request()
                req.mode = 0
                self.lamp_cli.call_async(req)
        except Exception:
            pass

        if self.lcd is not None:
            try:
                self.lcd.clear()
            except Exception:
                pass
        if self.camera is not None:
            for fn in ('stop', 'close'):    # close()까지 해야 카메라 점유가 풀린다
                try:
                    getattr(self.camera, fn)()
                except Exception:
                    pass


def main(args=None):
    # rclpy 기본 시그널 핸들러는 컨텍스트를 먼저 닫아버려 정지 명령을 못 보낸다.
    # SIGINT/SIGTERM 모두 파이썬 예외로 받아 정리 후 종료한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    node = None
    try:
        node = AutonomousDriveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_hardware()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
