"""
갈림길 기록 (--junction-keys 일 때만). 나중에 갈림길 자동 판단을 만들 때 참고 자료로 쓴다.

  result_..._junctions.csv   갈림길 사건마다 한 줄: 대기 시작(wait) / 키(key) / 꺾기 결과(result)
                             그 순간의 odom 위치, 4개 클래스 검출·신뢰도, 행별 lx/rx, 상실 시간, 초음파
  result_..._junctions/      대기에 들어간 순간의 카메라 원본(<번호>_raw.jpg)과 검출 표시 그림(<번호>_debug.jpg)
                             사진 저장은 별도 스레드 (제어 루프를 기다리게 하지 않음)
"""

import csv
import threading
from pathlib import Path

import cv2

from mj_lane_drive.drive_record import REPORT_CLASSES, _fmt, render_frame


class JunctionLog:
    def __init__(self, main_csv_path, sample_rows):
        main_csv_path = Path(main_csv_path)
        self.path = main_csv_path.with_name(main_csv_path.stem + '_junctions.csv')
        self.image_dir = main_csv_path.with_name(main_csv_path.stem + '_junctions')
        self.tags = [f'{round(r * 100)}' for r in sample_rows]
        columns = ['junction', 'kind', 'step', 't', 'clock', 'direction', 'wait_reason', 'wait_sec', 'result',
                   'maneuver_sec', 'odom_x', 'odom_y', 'odom_yaw']
        for name, _ in REPORT_CLASSES:
            columns += [f'{name}_det', f'{name}_conf']
        for tag in self.tags:
            columns += [f'lx_{tag}', f'rx_{tag}']
        columns += ['lost_time', 'sonar_range', 'images']
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        self.writer.writeheader()
        self.junction = 0
        self.rows = 0
        self.savers = []

    def write(self, event, step, t, confs, tracker, sonar, item=None):
        """event: JunctionGuide.events 의 한 개. item: 대기 사진용 render_frame 재료 (wait 일 때만)."""
        if event['kind'] == 'wait':
            self.junction += 1
        pose = event.get('pose')
        row = {
            'junction': self.junction, 'kind': event['kind'], 'step': step, 't': _fmt(t, 3),
            'clock': _fmt(event['time'], 3), 'direction': event.get('direction') or '',
            'wait_reason': event.get('wait_reason', ''), 'wait_sec': _fmt(event.get('wait_sec'), 2),
            'result': event.get('result', ''), 'maneuver_sec': _fmt(event.get('maneuver_sec'), 2),
            'odom_x': _fmt(pose[0], 3) if pose else '', 'odom_y': _fmt(pose[1], 3) if pose else '',
            'odom_yaw': _fmt(pose[2], 3) if pose else '',
            'lost_time': _fmt(tracker.lost_time, 3), 'sonar_range': _fmt(sonar['range'], 3), 'images': '',
        }
        for name, _ in REPORT_CLASSES:
            row[f'{name}_det'] = int(confs[name] is not None)
            row[f'{name}_conf'] = _fmt(confs[name], 3)
        for tag, obs in zip(self.tags, tracker.rows_obs):
            row[f'lx_{tag}'] = _fmt(obs['lx'], 1)
            row[f'rx_{tag}'] = _fmt(obs['rx'], 1)
        if event['kind'] == 'wait' and item is not None:
            stem = f'{self.junction:02d}'
            row['images'] = f'{self.image_dir.name}/{stem}_raw.jpg {self.image_dir.name}/{stem}_debug.jpg'
            saver = threading.Thread(target=self._save_images, args=(stem, item), daemon=True)
            saver.start()
            self.savers.append(saver)
        self.writer.writerow(row)
        self.file.flush()
        self.rows += 1

    def _save_images(self, stem, item):
        try:
            self.image_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(self.image_dir / f'{stem}_raw.jpg'), item['frame'])
            cv2.imwrite(str(self.image_dir / f'{stem}_debug.jpg'), render_frame(item))
        except Exception as error:                 # 사진 실패가 주행을 멈추지 않게
            print(f'갈림길 사진 저장 실패: {error}', flush=True)

    def close(self):
        for saver in self.savers:
            saver.join(timeout=5)
        self.file.close()
