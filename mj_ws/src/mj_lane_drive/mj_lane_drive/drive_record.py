"""
주행 기록: 프레임별 CSV 로그(DriveLog)와 오버레이 영상(Recorder, 별도 스레드).
"""

import csv
import queue
import threading
from datetime import datetime
from pathlib import Path

import cv2

from mj_lane_core.lane_draw import draw_debug, draw_tracking

LOG_FLUSH_EVERY = 10             # CSV 를 이 줄마다 디스크에 씀 (중간에 죽어도 남게)
RECORD_QUEUE = 20                # 녹화 대기 프레임 상한. 넘치면 그 프레임은 버린다 (제어 루프 보호)

REPORT_CLASSES = (('left_lane', 'L'), ('right_lane', 'R'), ('crosswalk', 'CW'), ('cross_lane', 'CL'))


def output_paths(out_dir):
    """실행 시각을 붙인 CSV/영상 경로 (같은 이름, 확장자만 다름). 같은 초에 다시 실행하면 _1, _2 …"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f'result_step2_lane_follow_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    name, n = stem, 1
    while (out_dir / f'{name}.csv').exists() or (out_dir / f'{name}.mp4').exists():
        name, n = f'{stem}_{n}', n + 1
    return out_dir / f'{name}.csv', out_dir / f'{name}.mp4'


def class_confs(masks):
    """클래스별 최고 conf (검출이 없으면 None)."""
    return {name: max((d['conf'] for d in masks[name]), default=None) if masks.get(name) else None
            for name, _ in REPORT_CLASSES}


def _fmt(value, digits):
    return '' if value is None else f'{value:.{digits}f}'


class DriveLog:
    """프레임(제어 스텝)별 주행 로그 CSV. 값이 없으면 빈칸."""

    def __init__(self, path, sample_rows):
        self.path = path
        self.tags = [f'{round(r * 100)}' for r in sample_rows]          # '90', '80', '70', '60'
        columns = ['step', 't', 'state', 'valid', 'lost_time',
                   'left_det', 'left_conf', 'right_det', 'right_conf', 'crosswalk_det', 'cross_lane_det']
        for tag in self.tags:
            columns += [f'lx_{tag}', f'rx_{tag}', f'src_{tag}', f'cx_{tag}', f'hw_{tag}']
        columns += ['target_x', 'error', 'steer', 'v', 'w',
                    'sonar_range', 'sonar_valid_range', 'sonar_age', 'sonar_invalid_streak',
                    'sonar_invalid_ignored', 'estop_reason', 'recorded']
        self.file = open(path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        self.writer.writeheader()
        self.rows = 0

    def write(self, step, t, state, valid, tracker, confs, error, steer, v, w, sonar, recorded):
        row = {
            'step': step, 't': _fmt(t, 3), 'state': state, 'valid': int(valid),
            'lost_time': _fmt(tracker.lost_time, 3),
            'left_det': int(confs['left_lane'] is not None), 'left_conf': _fmt(confs['left_lane'], 3),
            'right_det': int(confs['right_lane'] is not None), 'right_conf': _fmt(confs['right_lane'], 3),
            'crosswalk_det': int(confs['crosswalk'] is not None),
            'cross_lane_det': int(confs['cross_lane'] is not None),
            'target_x': _fmt(tracker.target_x, 2), 'error': _fmt(error, 4), 'steer': _fmt(steer, 4),
            'v': _fmt(v, 3), 'w': _fmt(w, 3),
            'sonar_range': _fmt(sonar['range'], 3), 'sonar_valid_range': _fmt(sonar['valid_range'], 3),
            'sonar_age': _fmt(sonar['age'], 3), 'sonar_invalid_streak': sonar['streak'],
            'sonar_invalid_ignored': sonar['ignored'], 'estop_reason': sonar['reason'],
            'recorded': int(recorded),
        }
        for tag, obs in zip(self.tags, tracker.rows_obs):
            row[f'lx_{tag}'] = _fmt(obs['lx'], 1)
            row[f'rx_{tag}'] = _fmt(obs['rx'], 1)
            row[f'src_{tag}'] = obs['source'] or ''
            row[f'cx_{tag}'] = '' if obs['cx'] is None else obs['cx']
            row[f'hw_{tag}'] = _fmt(obs['half_width'], 1)
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % LOG_FLUSH_EVERY == 0:
            self.file.flush()

    def close(self):
        self.file.close()


def render_frame(item):
    """녹화 한 프레임: 마스크 오버레이(ultralytics) + step1 과 같은 추적 그림 + 행별 관측·정보 줄."""
    view = item['result'].plot()
    view = draw_tracking(view, item['centers'], item['error'], item['steer'], item['valid'],
                         item['events'], item['tracker'])
    return draw_debug(view, item['rows_obs'], item['lines'], top=58 + 28 * len(item['events']))   # 상태·이벤트 글자 아래


class Recorder:
    """
    주행 기록 영상. 그리기와 인코딩은 별도 스레드에서 한다.
    제어 루프는 submit() 으로 넘기기만 하고, 큐가 차 있으면 그 프레임을 버린다 (기다리지 않음).
    """

    def __init__(self, path, size, fps):
        self.path = path
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        if not self.writer.isOpened():
            raise RuntimeError(f'녹화 파일을 열 수 없습니다: {path}')
        self.queue = queue.Queue(maxsize=RECORD_QUEUE)
        self.written = 0
        self.dropped = 0
        self.failed = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, item):
        """반환: 녹화 대기열에 들어갔는지 (False 면 버려짐)."""
        try:
            self.queue.put_nowait(item)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        while True:
            item = self.queue.get()
            if item is None:
                break
            try:
                self.writer.write(render_frame(item))
                self.written += 1
            except Exception as error:           # 녹화 실패가 주행을 멈추지 않게
                self.failed += 1
                if self.failed == 1:
                    print(f'녹화 오류 (이후 같은 오류는 세기만 함): {error}')

    def close(self):
        self.queue.put(None)                     # 남은 프레임을 다 쓴 뒤 끝남
        self.thread.join(timeout=60)
        self.writer.release()
