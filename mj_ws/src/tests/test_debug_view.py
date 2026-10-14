"""
[빠름: ROS 환경 필요, 모델 불필요] 검출 표시 영상 (D1)

  - draw_detections: 클래스별 색으로 칠하고, 오른쪽 위 범례에 검출된 클래스는 색이 찬 칸으로
  - render_frame: 원본 프레임을 건드리지 않음 (복사본에 그림)
  - DebugImagePublisher: 구독자가 없으면 wanted() False, 있으면 raw(bgr8)·compressed(JPEG) 발행
  - 그리기가 느려도(0.3초) submit() 은 기다리지 않음 → 제어 루프를 늦추지 않음, 밀린 것은 덮어씀
  - Recorder 의 on_rendered: 녹화 스레드가 그린 그림을 넘겨 줌 (두 번 그리지 않음)

안전: ROS_DOMAIN_ID=87, LOCALHOST (다른 시험과 같음).
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import tempfile      # noqa: E402
import threading     # noqa: E402
import time          # noqa: E402
from types import SimpleNamespace  # noqa: E402

import cv2           # noqa: E402
import numpy as np   # noqa: E402

import testlib       # noqa: E402

import rclpy                                         # noqa: E402
from rclpy.executors import SingleThreadedExecutor   # noqa: E402
from rclpy.node import Node                          # noqa: E402
from sensor_msgs.msg import CompressedImage, Image   # noqa: E402

from mj_lane_core.lane_draw import CLASS_COLORS, CROSSWALK_COLOR, LEFT_COLOR, RIGHT_COLOR, draw_detections  # noqa: E402
from mj_lane_core.lane_postprocess import LaneTracker, pick_main   # noqa: E402
from mj_lane_drive.debug_view import DebugImagePublisher            # noqa: E402
from mj_lane_drive.drive_record import Recorder, render_frame       # noqa: E402

T = testlib.Checker('debug_view')
W, H = 640, 480


def make_masks():
    left = np.zeros((H, W), np.uint8)
    left[300:470, 100:140] = 1
    right = np.zeros((H, W), np.uint8)
    right[300:470, 500:540] = 1
    return {'left_lane': [{'mask': left, 'conf': 0.91, 'area': int(left.sum())}],
            'right_lane': [{'mask': right, 'conf': 0.77, 'area': int(right.sum())}]}


def make_item():
    masks = make_masks()
    tracker = LaneTracker(W, H)
    centers, error, steer, valid = tracker.update(pick_main(masks.get('left_lane')), pick_main(masks.get('right_lane')), now=0.0)
    return {'frame': np.full((H, W, 3), 90, np.uint8), 'masks': masks, 'centers': centers, 'error': error,
            'steer': steer, 'valid': valid, 'events': [], 'rows_obs': tracker.rows_obs,
            'tracker': SimpleNamespace(rows=tracker.rows, target_x=tracker.target_x),
            'lines': ['step 0  t 0.0s  DRIVE', 'test']}


# ---------------- 1. 그리기 ----------------
frame = np.full((H, W, 3), 90, np.uint8)
view = draw_detections(frame, make_masks())
inside_left, inside_right, outside = view[400, 120], view[400, 520], view[400, 320]
T.check('left_lane 자리는 파랑 쪽으로 칠해짐', inside_left[0] > 120 and inside_left[2] < 90, f'{inside_left}')
T.check('right_lane 자리는 주황 쪽으로 칠해짐', inside_right[2] > 120 and inside_right[0] < 90, f'{inside_right}')
T.check('검출 없는 곳은 그대로', tuple(outside) == (90, 90, 90), f'{outside}')
legend = view[0:100, W - 220:W].reshape(-1, 3)
has = lambda color: bool(np.any(np.all(legend == np.array(color, np.uint8), axis=1)))   # noqa: E731
T.check('범례: 검출된 left·right 는 색이 찬 칸', has(LEFT_COLOR) and has(RIGHT_COLOR))
filled = (legend == np.array(CROSSWALK_COLOR, np.uint8)).all(axis=1).sum()
T.check('범례: 검출 안 된 crosswalk 는 테두리 칸만 (찬 칸보다 적은 픽셀)', 0 < filled < 12 * 12, f'{filled} 픽셀')
T.check('범례 클래스 4개 (left, right, crosswalk, cross_lane)', [n for n, _ in CLASS_COLORS] ==
        ['left_lane', 'right_lane', 'crosswalk', 'cross_lane'])
item = make_item()
before = item['frame'].copy()
out = render_frame(item)
T.check('render_frame: 원본 프레임은 그대로, 결과는 새 그림', np.array_equal(item['frame'], before) and out.shape == (H, W, 3)
        and not np.array_equal(out, before))

# ---------------- 2. 발행 ----------------
rclpy.init()
node = Node('debug_view_test')
pub = DebugImagePublisher(node, frame_id='test/front_camera_link')
probe = Node('debug_view_probe')
got = {'raw': [], 'jpeg': []}
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.add_node(probe)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
time.sleep(0.3)
T.check('구독자 없음 → wanted() False', not pub.wanted())
probe.create_subscription(Image, '/lane_debug/image', lambda m: got['raw'].append(m), 5)
probe.create_subscription(CompressedImage, '/lane_debug/image/compressed', lambda m: got['jpeg'].append(m), 5)
deadline = time.monotonic() + 5
while not pub.wanted() and time.monotonic() < deadline:
    time.sleep(0.05)
T.check('구독자 생김 → wanted() True', pub.wanted(), pub.topic)
for _ in range(3):
    pub.submit(make_item())
    time.sleep(0.2)
time.sleep(0.5)
raw = got['raw'][-1] if got['raw'] else None
T.check('raw 영상 받음 (bgr8 640x480, frame_id)', raw is not None and raw.encoding == 'bgr8' and (raw.width, raw.height) == (W, H)
        and raw.header.frame_id == 'test/front_camera_link', f'{len(got["raw"])}장')
jpeg = cv2.imdecode(np.frombuffer(bytes(got['jpeg'][-1].data), np.uint8), cv2.IMREAD_COLOR) if got['jpeg'] else None
T.check('compressed 영상 받음 (JPEG 로 풀림)', jpeg is not None and jpeg.shape == (H, W, 3), f'{len(got["jpeg"])}장')

# ---------------- 3. 느린 그리기도 기다리지 않음 ----------------
slow = DebugImagePublisher(node, render=lambda item: (time.sleep(0.3), render_frame(item))[1])
items = [make_item() for _ in range(20)]
durations = []
loop_start = time.monotonic()
for it in items:                                    # 제어 루프처럼 10 Hz 로 넘김
    t0 = time.monotonic()
    slow.submit(it)
    durations.append(time.monotonic() - t0)
    time.sleep(max(0.0, 0.1 - (time.monotonic() - t0)))
loop_time = time.monotonic() - loop_start
time.sleep(0.5)
T.check('그리기 0.3초여도 submit 은 1 ms 안 (최대)', max(durations) < 0.001, f'{max(durations) * 1000:.3f} ms')
T.check('20번 넘기는 데 약 2초 (10 Hz 유지)', loop_time < 2.2, f'{loop_time:.2f}s')
T.check('밀린 것은 덮어씀 (그린 수 < 넘긴 수)', slow.rendered < 20 and slow.overwritten > 0,
        f'그림 {slow.rendered}, 덮어씀 {slow.overwritten}')
slow.close()

# ---------------- 4. 녹화 그림을 같이 씀 ----------------
seen = []
path = os.path.join(tempfile.mkdtemp(prefix='debug_view_'), 'rec.mp4')
rec = Recorder(path, (W, H), 10.0, on_rendered=seen.append)
rec.submit(make_item())
rec.submit(make_item())
rec.close()
T.check('Recorder 가 그린 그림을 on_rendered 로 넘김 (2장, 640x480)', len(seen) == 2 and seen[0].shape == (H, W, 3),
        f'{len(seen)}장')

pub.close()
executor.shutdown()
spin_thread.join(timeout=5)        # spin 이 끝난 뒤에 노드를 지운다 (아니면 가끔 C++ 쪽이 abort)
probe.destroy_node()
node.destroy_node()
rclpy.shutdown()
T.finish()
