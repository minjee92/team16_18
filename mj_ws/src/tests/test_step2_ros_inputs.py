"""
[빠름: ROS 환경 필요, 모델 불필요] A4 입력 연결의 계산 부분

  - ROS 영상 메시지 → BGR 변환 (rgb8/bgr8/rgba8/bgra8, 줄 끝 여백), 지원 안 하는 인코딩은 오류
  - RosImageSource: 최신 프레임만, 새 프레임이 없으면 NO_FRAME, 크기가 다르면 줄이고 경고 한 번
  - 제어 주기 칸 수: 시계가 ±수 ms 흔들리거나 한 번 밀린 뒤에도 감속·정지 시작 프레임이 늘 같음 (6번째 / 21번째)
  - 실행 인자: --video 와 --camera-topic 은 함께 못 씀, --ros-args 는 무시

안전: ROS_DOMAIN_ID=87, LOCALHOST (다른 시험과 같음).
"""

import os

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import random        # noqa: E402
import threading     # noqa: E402
import time          # noqa: E402

import numpy as np   # noqa: E402

import testlib       # noqa: E402

import rclpy                                         # noqa: E402
from rclpy.executors import SingleThreadedExecutor   # noqa: E402
from rclpy.node import Node                          # noqa: E402
from rclpy.qos import qos_profile_sensor_data        # noqa: E402
from rclpy.utilities import remove_ros_args          # noqa: E402
from sensor_msgs.msg import Image                    # noqa: E402

import mj_lane_drive.lane_follow_node as S2                                   # noqa: E402
from mj_lane_drive.sources import NO_FRAME, RosImageSource, ros_image_to_bgr  # noqa: E402
from mj_lane_core.lane_postprocess import LaneTracker                         # noqa: E402
from mj_lane_core.drive_control import LaneFollowController                   # noqa: E402

T = testlib.Checker('step2_ros_inputs')

# ---------------- 1. 영상 변환 ----------------
H, W = 4, 5
bgr = np.random.default_rng(0).integers(0, 256, (H, W, 3), dtype=np.uint8)
alpha = np.full((H, W, 1), 200, np.uint8)
cases = {
    'rgb8': bgr[:, :, ::-1],
    'bgr8': bgr,
    'rgba8': np.concatenate([bgr[:, :, ::-1], alpha], axis=2),
    'bgra8': np.concatenate([bgr, alpha], axis=2),
}
for enc, img in cases.items():
    ch = img.shape[2]
    padded = np.zeros((H, W * ch + 3), np.uint8)              # 줄 끝 여백 3 바이트
    padded[:, :W * ch] = img.reshape(H, W * ch)
    out = ros_image_to_bgr(H, W, enc, W * ch + 3, padded.tobytes())
    T.check(f'{enc} → BGR (줄 끝 여백 포함)', out.shape == (H, W, 3) and np.array_equal(out, bgr))
for enc, step, data, what in (('mono8', W, bytes(H * W), '지원 안 하는 인코딩'),
                              ('rgb8', W * 3, bytes(W * 3), '데이터가 모자람')):
    try:
        ros_image_to_bgr(H, W, enc, step, data)
        T.check(f'{what} → 오류', False)
    except ValueError as error:
        T.check(f'{what} → 오류', True, str(error))

# ---------------- 2. RosImageSource ----------------
rclpy.init()
node = Node('ros_inputs_test')
pub = node.create_publisher(Image, 'cam_test/image_raw', qos_profile_sensor_data)
warnings = []
source = RosImageSource(node, 'cam_test/image_raw', (64, 48), timeout=0.3, warn=warnings.append)
executor = SingleThreadedExecutor()
executor.add_node(node)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()


def publish(width, height, value):
    msg = Image()
    msg.width, msg.height, msg.encoding, msg.step = width, height, 'rgb8', width * 3
    msg.data = bytes([value]) * (width * 3 * height)
    pub.publish(msg)


time.sleep(0.5)                                  # 구독 연결
T.check('영상이 없으면 NO_FRAME', source.read() is NO_FRAME)
publish(64, 48, 10)
publish(64, 48, 20)                              # 연달아 두 장 → 최신 것만
time.sleep(0.2)
frame = source.read()
T.check('새 프레임을 BGR 로 받음 (최신 한 장)', frame is not NO_FRAME and frame.shape == (48, 64, 3) and frame[0, 0, 0] == 20,
        'NO_FRAME' if frame is NO_FRAME else f'{frame.shape}, 값 {frame[0, 0, 0]}')
T.check('같은 프레임을 다시 주지 않음 (기다리다 NO_FRAME)', source.read() is NO_FRAME)
publish(128, 96, 30)
time.sleep(0.2)
frame = source.read()
publish(128, 96, 40)
time.sleep(0.2)
frame2 = source.read()
T.check('크기가 다르면 규격 크기로 줄임', frame is not NO_FRAME and frame.shape == (48, 64, 3) and frame2.shape == (48, 64, 3))
T.check('크기 경고는 한 번만', len(warnings) == 1, f'{warnings}')
T.check('토픽 이름 (노드 기준)', source.topic == '/cam_test/image_raw', source.topic)
executor.shutdown()
spin_thread.join(timeout=5)        # spin 이 끝난 뒤에 노드를 지운다 (아니면 가끔 C++ 쪽이 abort)
node.destroy_node()

# ---------------- 3. 제어 주기 칸 수 ----------------
period = 1.0 / S2.CONTROL_HZ


def tick_times(gaps):
    c = S2.ControlTicks(period)
    now, out = 1000.0, [c.time(1000.0)]
    for g in gaps:
        now += g
        out.append(round(c.time(now), 6))
    return out


T.check('칸 수: 보통 간격(0.06–0.14초)은 늘 1칸', tick_times([0.06, 0.14, 0.1, 0.104, 0.096]) == [0.0, 0.1, 0.2, 0.3, 0.4, 0.5])
T.check('칸 수: 정말 밀리면(0.26초) 그만큼 더 셈 (3칸), 너무 짧아도(0.01초) 1칸',
        tick_times([0.26, 0.01]) == [0.0, 0.3, 0.4], f'{tick_times([0.26, 0.01])}')


def first_slow_stop(gaps):
    """처음부터 차선을 못 보는 프레임을 주어진 간격으로 넣고, 감속·정지가 처음 나온 프레임 번호."""
    tracker = LaneTracker(640, 480, sample_rows=S2.SAMPLE_ROWS, row_weights=S2.ROW_WEIGHTS,
                          lost_stop_sec=S2.LOST_STOP_SEC)
    controller = LaneFollowController(S2.BASE_SPEED, S2.MIN_SPEED, S2.MAX_ANGULAR, S2.CORNER_SLOWDOWN,
                                      S2.STEER_TO_ANGULAR, S2.LOST_SLOW_SEC, S2.LOST_STOP_SEC)
    ticks = S2.ControlTicks(period)
    now = 5000.0
    slow = stop = None
    for k in range(len(gaps) + 1):
        tracker.update(None, None, now=ticks.time(now))
        _, _, note = controller.command(0.0, False, tracker.lost_time)
        if slow is None and note.endswith('SLOW'):
            slow = k
        if stop is None and note.endswith('STOP'):
            stop = k
        if k < len(gaps):
            now += gaps[k]
    return slow, stop


rng = random.Random(0)
jitter = {first_slow_stop([period + rng.uniform(-0.004, 0.004) for _ in range(30)]) for _ in range(200)}
T.check('시계가 ±4 ms 흔들려도 감속은 6번째, 정지는 21번째 놓친 프레임에서 (200번 모두 같음)',
        jitter == {(6, 21)}, f'{jitter}')
# A4 방식의 약점: 앞에서 한 번 0.14초 밀린 뒤 프레임이 칸 가운데(±0.05)에 찍히는 경우 (D1 기준 CSV 비교에서 실제로 나옴)
shifted = {first_slow_stop([0.14] + [period + rng.uniform(-0.004, 0.004) for _ in range(29)]) for _ in range(200)}
T.check('한 번 0.14초 밀린 뒤에도 감속 6번째, 정지 21번째로 고정 (200번 모두 같음)',
        shifted == {(6, 21)}, f'{shifted}')

# ---------------- 4. 실행 인자 ----------------
args = S2.parse_args(['--camera-topic', '--namespace', 'amr_01', '--sonar-topic', 'us_sensor/range', '--use-sim-time'])
T.check('--camera-topic 값 없이 → 규격 토픽', args.camera_topic == S2.CAMERA_TOPIC and args.namespace == 'amr_01'
        and args.sonar_topic == 'us_sensor/range' and args.use_sim_time)
args = S2.parse_args([])
T.check('기본값: 벽시계, 네임스페이스 없음, /us_sensor/range', not args.use_sim_time and args.namespace == ''
        and args.sonar_topic == '/us_sensor/range' and args.camera_topic is None and args.video is None)
try:
    import contextlib
    import io
    with contextlib.redirect_stderr(io.StringIO()):
        S2.parse_args(['--video', 'a.mp4', '--camera-topic'])
    T.check('--video 와 --camera-topic 함께 → 거부', False)
except SystemExit:
    T.check('--video 와 --camera-topic 함께 → 거부', True)
argv = remove_ros_args(['node', '--dry-run', '--ros-args', '-r', '__node:=x', '-p', 'a:=1'])
T.check('--ros-args 이후는 무시', S2.parse_args(argv[1:]).dry_run, f'{argv}')
rclpy.shutdown()

T.finish()
