#!/usr/bin/env python3
"""
차선 추종 검증기 (PC용)

저장된 영상으로 모델의 차선 인식과 조향값 계산을 눈으로 확인한다.
로봇에 올리기 전에 여기서 먼저 동작을 확인할 것.

  - 마스크 픽셀에서 여러 높이의 x좌표를 뽑아 차선 중앙 계산
  - 한쪽 차선만 보일 때는 기억해둔 도로 폭으로 목표점 추정
  - 조향값(error, angular)을 화면과 터미널에 표시
  - 프레임별 클래스 검출/신뢰도와 마지막 검출률 요약을 터미널에 출력

후처리(목표점·조향 계산)는 모델과 무관한 common/lane_postprocess.py에 있다.

실행:
    python3 src/tools/step1_check_lane_detection.py
    python3 src/tools/step1_check_lane_detection.py --input inputs/xxx.mp4 --output outputs/yyy.mp4
    python3 src/tools/step1_check_lane_detection.py --input 0          # 웹캠
    python3 src/tools/step1_check_lane_detection.py --max-frames 150   # 앞 150프레임만
    python3 src/tools/step1_check_lane_detection.py --threads 4        # torch CPU 스레드 수
"""

import argparse
import sys
import unicodedata
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO
import torch   # ultralytics 뒤에 import (ultralytics가 torch 로드 전 OMP 설정을 먼저 잡음)

SRC_DIR = Path(__file__).resolve().parent.parent     # mj_ws/src/
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))                 # src/common 을 import 하기 위해

from common.lane_postprocess import LaneTracker, group_masks, pick_main  # noqa: E402


# ----------------------------- 설정 -----------------------------

BASE_DIR = SRC_DIR.parent   # mj_ws/

INPUT_SOURCE = str(BASE_DIR / 'inputs' / 'pinky_20260919_182009.mp4')   # 영상 경로 (웹캠은 0)
MODEL_PATH = str(BASE_DIR / 'models' / '260928_yolon_best.pt')
OUTPUT_VIDEO = str(BASE_DIR / 'outputs' / 'result_step1_check_lane_detection.mp4')

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

REPORT_CLASSES = ('left_lane', 'right_lane', 'crosswalk', 'cross_lane')   # 프레임별 출력
STAT_CLASSES = ('left_lane', 'right_lane', 'crosswalk')                   # 마지막 요약


# --------------------------- 모델 결과 변환 ---------------------------

def build_masks(result, width, height):
    """ultralytics Results → 클래스 이름별 원본 크기 마스크 (후처리는 common 모듈)."""

    if result.masks is None or result.boxes is None:
        return {}

    names = result.names
    data = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    confs = result.boxes.conf.cpu().numpy()

    return group_masks(data, [names[c] for c in classes], confs, width, height)


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


# ----------------------------- 리포트 -----------------------------

def frame_report(index, masks):
    """프레임별 클래스 검출 여부와 최고 신뢰도 한 줄."""
    parts = []
    for name in REPORT_CLASSES:
        found = masks.get(name)
        conf = f'{max(d["conf"] for d in found):.2f}' if found else ' -  '
        parts.append(f'{name} {conf}')
    return f'frame {index:5d} | ' + ' | '.join(parts)


def _pad(text, width):
    """한글(전각)을 2칸으로 세어 터미널 폭 기준으로 오른쪽을 채운다."""
    cells = sum(2 if unicodedata.east_asian_width(ch) in 'WF' else 1 for ch in text)
    return text + ' ' * max(0, width - cells)


class DetectionStats:
    """
    프레임별 검출 결과를 모아 마지막 요약을 만든다.
    프레임 번호는 frame 로그와 같이 0부터 센다.
    """

    def __init__(self):
        self.total = 0
        self.counts = {name: 0 for name in STAT_CLASSES}
        self.lanes = {'both': 0, 'one': 0, 'none': 0}   # 좌우 차선 검출 개수별 프레임 수
        self.miss_runs = []        # 좌우 차선 둘 다 미검출이 이어진 구간 [(시작 프레임, 길이)]
        self._run_start = None

    def update(self, masks):
        index = self.total
        self.total += 1

        for name in STAT_CLASSES:
            if masks.get(name):
                self.counts[name] += 1

        seen = bool(masks.get('left_lane')) + bool(masks.get('right_lane'))
        self.lanes[('none', 'one', 'both')[seen]] += 1

        if seen == 0:
            if self._run_start is None:
                self._run_start = index
        else:
            self._close_run(index)

    def _close_run(self, end):
        if self._run_start is not None:
            self.miss_runs.append((self._run_start, end - self._run_start))
            self._run_start = None

    def print_summary(self, fps, lost_limit):
        self._close_run(self.total)     # 마지막 프레임까지 이어진 구간 마감

        def pct(n):
            return 100.0 * n / self.total if self.total else 0.0

        def row(label, n):
            print(f'  {_pad(label, 14)}{n:5d} 프레임  {pct(n):5.1f}%')

        print()
        print(f'===== 요약 (처리 프레임 {self.total}) =====')
        print('클래스별 검출')
        for name in STAT_CLASSES:
            row(name, self.counts[name])

        print('좌우 차선 (left_lane / right_lane)')
        row('양쪽 다 검출', self.lanes['both'])
        row('한쪽만 검출', self.lanes['one'])
        row('둘 다 미검출', self.lanes['none'])

        print('둘 다 미검출 연속 구간')
        if self.miss_runs:
            start, length = max(self.miss_runs, key=lambda r: r[1])   # 길이가 같으면 먼저 나온 구간
            long_runs = sum(1 for _, n in self.miss_runs if n >= lost_limit)
            print(f'  {_pad("최장", 14)}{length:5d} 프레임  '
                  f'(frame {start} 부터, 약 {start / fps:.1f}초)')
            print(f'  {_pad(f"{lost_limit}프레임 이상", 14)}{long_runs:5d} 회  '
                  f'(LOST_LIMIT={lost_limit}, 전체 구간 {len(self.miss_runs)}개)')
        else:
            print('  없음')


# ------------------------------ 메인 ------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description='차선 추종 검증기 (저장 영상 / 웹캠)')
    parser.add_argument('--input', default=INPUT_SOURCE,
                        help='입력 영상 경로, 웹캠은 번호 (기본: %(default)s)')
    parser.add_argument('--output', default=OUTPUT_VIDEO,
                        help='결과 영상 경로 (기본: %(default)s)')
    parser.add_argument('--max-frames', type=int, default=None,
                        help='이 프레임 수만 처리하고 종료 (기본: 제한 없음)')
    parser.add_argument('--threads', type=int, default=None,
                        help='첫 추론 뒤 torch CPU 스레드 수로 설정 '
                             '(기본: ultralytics 기본값 유지)')
    args = parser.parse_args()

    if args.max_frames is not None and args.max_frames <= 0:
        parser.error('--max-frames 는 1 이상이어야 합니다')
    if args.threads is not None and args.threads <= 0:
        parser.error('--threads 는 1 이상이어야 합니다')

    if isinstance(args.input, str) and args.input.isdigit():
        args.input = int(args.input)          # '0' → 웹캠 0
    return args


def main():

    args = parse_args()

    model = YOLO(MODEL_PATH)
    print('클래스:', model.names)

    missing = [c for c in LANE_CLASSES if c not in model.names.values()]
    if missing:
        print(f'경고: 모델에 {missing} 클래스가 없습니다. 이름을 확인하세요.')

    cap = cv2.VideoCapture(args.input)
    if not cap.isOpened():
        print(f'입력 소스를 열 수 없습니다: {args.input}')
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
        args.output, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height)
    )

    tracker = LaneTracker(
        width, height,
        sample_rows=SAMPLE_ROWS,
        row_weights=ROW_WEIGHTS,
        road_half_init=ROAD_HALF_INIT,
        road_half_smooth=ROAD_HALF_SMOOTH,
        steer_gain=STEER_GAIN,
        steer_smooth=STEER_SMOOTH,
        lost_limit=LOST_LIMIT,
    )

    stats = DetectionStats()

    print(f'{width}x{height} @ {fps:.1f}fps   종료: q')

    while True:

        if args.max_frames is not None and stats.total >= args.max_frames:
            break

        ok, frame = cap.read()
        if not ok:
            break

        result = model.predict(
            source=frame, conf=CONF, imgsz=INFER_SIZE, verbose=False
        )[0]

        if args.threads is not None and stats.total == 0:
            # ultralytics가 첫 predict에서 torch 스레드를 min(8, 코어-1)로 덮어쓰므로 그 뒤에 설정
            before = torch.get_num_threads()
            torch.set_num_threads(args.threads)
            print(f'torch 스레드 {before} → {torch.get_num_threads()}')

        masks = build_masks(result, width, height)

        print(frame_report(stats.total, masks))
        stats.update(masks)

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
    print(f'저장 완료: {args.output}')

    stats.print_summary(fps, LOST_LIMIT)


if __name__ == '__main__':
    main()
