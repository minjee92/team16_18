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
    python3 src/tools/step1_check_lane_detection.py --model models/lane_model_ncnn/best_ncnn_model --imgsz 320
"""

import argparse
import sys
import unicodedata
from pathlib import Path

import cv2
from ultralytics import YOLO
import torch   # ultralytics 뒤에 import (ultralytics가 torch 로드 전 OMP 설정을 먼저 잡음)

SRC_DIR = Path(__file__).resolve().parent.parent     # mj_ws/src/
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))                 # src/common 을 import 하기 위해

from common.lane_draw import draw_tracking  # noqa: E402
from common.lane_postprocess import LaneTracker, group_masks, pick_main  # noqa: E402


# ----------------------------- 설정 -----------------------------

BASE_DIR = SRC_DIR.parent   # mj_ws/

INPUT_SOURCE = str(BASE_DIR / 'inputs' / 'pinky_20260919_182009.mp4')   # 영상 경로 (웹캠은 0)
MODEL_PATH = str(BASE_DIR / 'models' / '260928_yolon_best.pt')
OUTPUT_VIDEO = str(BASE_DIR / 'outputs' / 'result_step1_check_lane_detection.mp4')

CONF = 0.5
INFER_SIZE = 640          # 로봇에서는 320으로 낮출 값 (export 된 모델은 export 크기로 고정됨)

# 차선을 읽을 높이들 (화면 높이 대비 비율). 아래쪽이 로봇에 가까움
SAMPLE_ROWS = (0.90, 0.80, 0.70, 0.60)

# 각 높이의 가중치. 가까운 쪽을 더 신뢰하되 먼 쪽도 반영해 커브를 미리 본다
ROW_WEIGHTS = (0.40, 0.30, 0.20, 0.10)

ROAD_HALF_INIT = 0.28     # 도로 반폭 초깃값 (화면 너비 대비)
ROAD_HALF_SMOOTH = 0.1    # 도로 폭 갱신 속도 (0~1, 클수록 빠르게 반영)

STEER_GAIN = 1.0          # 조향 민감도
STEER_SMOOTH = 0.35       # 조향값 부드럽게 (0~1, 작을수록 부드러움)
LOST_STOP_SEC = 2.0       # 차선을 못 본 시간이 이보다 길면 조향 0으로 (초, 2단계 설계와 동일)

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

    def print_summary(self, fps, lost_stop_sec):
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
            long_runs = sum(1 for _, n in self.miss_runs if round(n / fps, 6) > lost_stop_sec)
            print(f'  {_pad("최장", 14)}{length:5d} 프레임  '
                  f'(frame {start} 부터, 약 {start / fps:.1f}초)')
            print(f'  {_pad(f"{lost_stop_sec}초 초과", 14)}{long_runs:5d} 회  '
                  f'(LOST_STOP_SEC={lost_stop_sec}, 전체 구간 {len(self.miss_runs)}개)')
        else:
            print('  없음')


def check_imgsz(model, requested):
    """
    첫 추론 직후 호출. 이후 추론에 쓸 입력 크기를 돌려준다.

    export 된 고정 크기 모델(ncnn 등)은 첫 predict 에서만 export 크기로 덮어쓰이고, 다음 predict 부터는
    요청한 크기가 다시 적용되어 엉뚱한 크기로 들어간다 (검출이 사라짐). 그래서 고정 크기를 계속 쓴다.
    """
    used = model.predictor.args.imgsz
    used = list(used) if isinstance(used, (list, tuple)) else [used, used]
    if used != [requested, requested]:
        print(f'주의: 이 모델은 입력 크기 {used[1]}x{used[0]} 로 고정되어 있어 --imgsz {requested} 대신 '
              f'{used[1]}x{used[0]} 로 추론합니다 (export 메타데이터)')
        return used
    print(f'입력 크기: {requested}')
    return requested


def print_timing(speeds):
    """프레임당 추론 시간 평균 (ultralytics 가 재는 전처리 / 추론 / 후처리)."""
    if not speeds:
        return
    pre, inf, post = (sum(s[k] for s in speeds) / len(speeds)
                      for k in ('preprocess', 'inference', 'postprocess'))
    print(f'추론 시간 (첫 프레임 제외 {len(speeds)}개 평균)')
    print(f'  전처리 {pre:.1f} / 추론 {inf:.1f} / 후처리 {post:.1f} ms  → 합계 {pre + inf + post:.1f} ms/프레임')


# ------------------------------ 메인 ------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description='차선 추종 검증기 (저장 영상 / 웹캠)')
    parser.add_argument('--input', default=INPUT_SOURCE,
                        help='입력 영상 경로, 웹캠은 번호 (기본: %(default)s)')
    parser.add_argument('--output', default=OUTPUT_VIDEO,
                        help='결과 영상 경로 (기본: %(default)s)')
    parser.add_argument('--max-frames', type=int, default=None,
                        help='이 프레임 수만 처리하고 종료 (기본: 제한 없음)')
    parser.add_argument('--model', default=MODEL_PATH,
                        help='모델 경로 (.pt 파일 또는 *_ncnn_model 폴더, 기본: %(default)s)')
    parser.add_argument('--imgsz', type=int, default=INFER_SIZE,
                        help='추론 입력 크기 (기본: %(default)s). export 된 모델은 export 크기로 고정된다')
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

    model = YOLO(args.model, task='segment')
    print('모델:', args.model)
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
        lost_stop_sec=LOST_STOP_SEC,
    )

    stats = DetectionStats()
    speeds = []               # 프레임별 ultralytics speed (ms), 첫 프레임(워밍업) 제외
    imgsz = args.imgsz        # 첫 추론 뒤 모델 고정 크기로 바뀔 수 있음 (check_imgsz)

    print(f'{width}x{height} @ {fps:.1f}fps   종료: q')

    while True:

        if args.max_frames is not None and stats.total >= args.max_frames:
            break

        ok, frame = cap.read()
        if not ok:
            break

        result = model.predict(
            source=frame, conf=CONF, imgsz=imgsz, verbose=False
        )[0]

        if stats.total == 0:
            imgsz = check_imgsz(model, args.imgsz)
        else:
            speeds.append(result.speed)

        if args.threads is not None and stats.total == 0:
            # ultralytics가 첫 predict에서 torch 스레드를 min(8, 코어-1)로 덮어쓰므로 그 뒤에 설정
            before = torch.get_num_threads()
            torch.set_num_threads(args.threads)
            print(f'torch 스레드 {before} → {torch.get_num_threads()}')

        masks = build_masks(result, width, height)

        index = stats.total
        print(frame_report(index, masks))
        stats.update(masks)

        left_mask = pick_main(masks.get('left_lane'))
        right_mask = pick_main(masks.get('right_lane'))

        centers, error, steer, valid = tracker.update(left_mask, right_mask, now=index / fps)

        events = []
        if masks.get('crosswalk'):
            events.append('CROSSWALK')
        if masks.get('cross_lane'):
            events.append('CROSS LANE')

        view = result.plot()
        view = draw_tracking(view, centers, error, steer, valid, events, tracker)

        writer.write(view)
        cv2.imshow('Lane Follow Check', view)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    writer.release()
    cv2.destroyAllWindows()
    print(f'저장 완료: {args.output}')

    stats.print_summary(fps, LOST_STOP_SEC)
    print_timing(speeds)


if __name__ == '__main__':
    main()
