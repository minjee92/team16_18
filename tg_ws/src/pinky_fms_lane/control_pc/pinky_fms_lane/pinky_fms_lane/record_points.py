"""로봇으로 코스 점 좌표 기록하기 (관제 PC, 로봇 1대).

    ros2 run pinky_fms_lane record_course_points amr_01 --course <소스 폴더의 코스.yaml> --map <지도.yaml>

  로봇을 키보드로 몰아 점 위에 세우고 Enter 를 누르면:
    1. 바퀴가 멈췄는지 확인한다 (/<ns>/odom 속도가 still_sec 동안 0 근처)
    2. AMCL 제자리 갱신(/<ns>/request_nomotion_update)을 몇 번 하고, 갱신마다 나온 amcl_pose 를 모은다
       (1.3 실험: 제자리 갱신을 많이 하면 트인 구간에서 위치가 흐른다 → 기본 3번)
    3. 평균 위치·방향(원형 평균), 공분산, 갱신 사이 흔들림, 스캔 일치율(--map), 지금 값(줄자·추정)과의 차이를 보여 준다
    4. 경고가 없으면 Enter 로 저장, 경고가 있으면 y 를 입력해야 저장한다
  저장 위치: 코스 yaml 의 robot: 파일 (예: mission4_3_clean_1cm.robot.yaml). 덮어쓰기 전에 <파일>.bak.<시각> 으로 백업한다.
  로봇 기록은 줄자·추정값보다 우선한다 (course.py). 방향(yaw)은 코스에서 방향이 있는 점(출발점)만 저장한다.
  설치 폴더(install/)의 코스 파일에는 쓰지 않는다 (다시 빌드하면 사라지므로 소스 폴더의 파일을 --course 로 준다).

  이 도구는 로봇을 움직이지 않는다 (cmd_vel 을 내지 않음). 키보드 조종은 다른 터미널에서 한다.
"""
import argparse
import datetime
import math
import os
import shutil
import sys
import time

import yaml

ROBOT_FILE_HEADER = ('# 로봇 기록 (record_course_points 가 씀). 같은 이름의 줄자·추정값보다 우선한다.\n'
                     '# 손으로 고쳐도 되지만 다음 기록 때 주석은 사라진다. 고치기 전 값은 같은 폴더의 *.bak.* 백업에 있다.\n')


# ---------- ROS 없이 시험할 수 있는 계산 ----------
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def summarize(poses):
    """[(x, y, yaw)] → 평균 x, y, 원형 평균 yaw, 흔들림(첫·마지막 차이와 평균에서 가장 먼 거리 중 큰 값, m), 방향 흔들림(rad)."""
    if not poses:
        raise ValueError('amcl_pose 가 없음')
    n = len(poses)
    mx = sum(p[0] for p in poses) / n
    my = sum(p[1] for p in poses) / n
    myaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
    drift = math.hypot(poses[-1][0] - poses[0][0], poses[-1][1] - poses[0][1])
    spread = max([math.hypot(p[0] - mx, p[1] - my) for p in poses] + [drift])
    yaw_spread = max(abs(wrap(p[2] - myaw)) for p in poses)
    return {'x': mx, 'y': my, 'yaw': myaw, 'spread': spread, 'yaw_spread': yaw_spread, 'samples': n}


def warnings_for(rec, ref, limits):
    """기록값 rec(summarize + std_xy, std_yaw, match, road_dist) 와 지금 코스 값 ref(Point) → 경고 문장 목록."""
    w = []
    if rec['samples'] < limits['min_samples']:
        w.append(f'amcl_pose {rec["samples"]}개만 받음 (필요 {limits["min_samples"]}개)')
    if rec['std_xy'] > limits['max_xy_std']:
        w.append(f'위치 표준편차 {rec["std_xy"] * 100:.1f} cm > {limits["max_xy_std"] * 100:.1f} cm (조금 더 몰고 와서 다시)')
    if rec['std_yaw'] > limits['max_yaw_std']:
        w.append(f'방향 표준편차 {math.degrees(rec["std_yaw"]):.1f}° > {math.degrees(limits["max_yaw_std"]):.1f}°')
    if rec['spread'] > limits['max_spread']:
        w.append(f'갱신 사이 위치가 {rec["spread"] * 100:.1f} cm 움직임 > {limits["max_spread"] * 100:.1f} cm '
                 '(트인 구간에서 흐르는 중일 수 있음)')
    if rec.get('match') is not None and rec['match'] < limits['min_match']:
        w.append(f'스캔 일치율 {rec["match"] * 100:.0f}% < {limits["min_match"] * 100:.0f}% (위치가 틀렸을 수 있음)')
    if rec['road_dist'] > limits['max_road_dist']:
        w.append(f'길 가운데 선에서 {rec["road_dist"] * 100:.1f} cm 떨어짐 > {limits["max_road_dist"] * 100:.1f} cm '
                 '(차선 위에 세웠는지 확인)')
    if ref is not None:
        d = math.hypot(rec['x'] - ref.x, rec['y'] - ref.y)
        if d > limits['max_diff']:
            w.append(f'지금 값({ref.source})과 {d * 100:.1f} cm 차이 > {limits["max_diff"] * 100:.1f} cm '
                     '(점 이름·로봇 위치 확인)')
        if ref.yaw is not None and abs(wrap(rec['yaw'] - ref.yaw)) > limits['max_yaw_diff']:
            w.append(f'방향이 지금 값과 {math.degrees(abs(wrap(rec["yaw"] - ref.yaw))):.0f}° 다름 '
                     f'> {math.degrees(limits["max_yaw_diff"]):.0f}° (로봇 방향 확인)')
    return w


def robot_entry(rec, keep_yaw, stamp):
    """robot 파일에 쓸 한 점. 좌표는 mm, 품질 값은 확인용."""
    e = {'x': round(rec['x'], 4), 'y': round(rec['y'], 4)}
    if keep_yaw:
        e['yaw'] = round(rec['yaw'], 4)
    e.update({'stamp': stamp, 'std_xy': round(rec['std_xy'], 4), 'std_yaw_deg': round(math.degrees(rec['std_yaw']), 2),
              'spread': round(rec['spread'], 4), 'samples': rec['samples']})
    if rec.get('match') is not None:
        e['match'] = round(rec['match'], 3)
    return e


def save_robot_point(path, name, entry, now=None):
    """robot 파일에 한 점을 넣고 저장. 이미 파일이 있으면 먼저 <파일>.bak.<시각> 으로 복사한다 → 백업 경로 또는 ''."""
    data = {}
    backup = ''
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        backup = f'{path}.bak.{(now or datetime.datetime.now()):%Y%m%d_%H%M%S}'
        shutil.copy2(path, backup)
    data.setdefault('points', {})[name] = entry
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(ROBOT_FILE_HEADER)
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    os.replace(tmp, path)                      # 쓰다가 끊겨도 원래 파일이 깨지지 않게
    return backup


def is_still(twists, still_sec, max_v=0.005, max_w=0.02):
    """[(시각, v, w)] (최근 것이 뒤) 의 마지막 still_sec 동안 모두 멈춰 있는가. 데이터가 그 시간을 덮지 못하면 False."""
    if not twists or twists[-1][0] - twists[0][0] < still_sec:
        return False
    t_end = twists[-1][0]
    return all(abs(v) <= max_v and abs(w) <= max_w for t, v, w in twists if t >= t_end - still_sec)


def next_name(order, done, skipped):
    for n in order:
        if n not in done and n not in skipped:
            return n
    return None


# ---------- ROS ----------
class Recorder:
    def __init__(self, args, course):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Empty
        from tf2_msgs.msg import TFMessage

        self.rclpy, self.Empty = rclpy, Empty
        self.args, self.course, self.ns = args, course, args.namespace
        self.node = Node('record_course_points')
        self.amcl = []                          # (도착 시각, x, y, yaw, 공분산)
        self.twists = []
        self.scan = None
        self.static = {}
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        ns = self.ns
        self.node.create_subscription(PoseWithCovarianceStamped, f'/{ns}/amcl_pose', self._on_amcl, 10)
        self.node.create_subscription(Odometry, f'/{ns}/odom', self._on_odom, 20)
        self.node.create_subscription(LaserScan, f'/{ns}/scan', lambda m: setattr(self, 'scan', m),
                                      QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(TFMessage, f'/{ns}/tf_static', self._on_static, latched)
        self.cli = self.node.create_client(Empty, f'/{ns}/request_nomotion_update')
        self.mm = None
        if args.map:
            from pinky_fms_lane.initial_pose import MapModel
            self.mm = MapModel(args.map)

    @staticmethod
    def _yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_amcl(self, m):
        p = m.pose.pose
        self.amcl.append((time.monotonic(), p.position.x, p.position.y, self._yaw(p.orientation),
                          list(m.pose.covariance)))

    def _on_odom(self, m):
        now = time.monotonic()
        self.twists.append((now, m.twist.twist.linear.x, m.twist.twist.angular.z))
        self.twists = [t for t in self.twists if t[0] >= now - 5.0]

    def _on_static(self, m):
        for t in m.transforms:
            self.static[t.child_frame_id] = (t.header.frame_id, (t.transform.translation.x, t.transform.translation.y,
                                                                 self._yaw(t.transform.rotation)))

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def check_ready(self):
        """시작 확인: odom, amcl 서비스. 문제 목록."""
        self.spin_for(1.5)
        bad = []
        if not self.twists:
            bad.append(f'/{self.ns}/odom 이 안 옴 (로봇 bringup 과 통신 설정 확인)')
        if not self.cli.wait_for_service(timeout_sec=3.0):
            bad.append(f'/{self.ns}/request_nomotion_update 서비스가 없음 (로봇에서 robot_localization 이 떠 있는지 확인)')
        return bad

    def measure(self):
        """멈춤 확인 → 제자리 갱신 → 측정값 dict. 실패하면 (None, 이유)."""
        a = self.args
        end = time.monotonic() + a.still_timeout
        while not is_still(self.twists, a.still_sec) and time.monotonic() < end:
            self.spin_for(0.1)
        if not is_still(self.twists, a.still_sec):
            return None, f'{a.still_timeout:.0f} s 안에 로봇이 {a.still_sec:.1f} s 동안 멈추지 않음 (odom 속도 확인)'
        poses, covs = [], []
        for _ in range(a.updates):
            n0 = len(self.amcl)
            fut = self.cli.call_async(self.Empty.Request())
            self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
            t_end = time.monotonic() + 2.0
            while len(self.amcl) == n0 and time.monotonic() < t_end:
                self.spin_for(0.05)
            if len(self.amcl) > n0:
                _, x, y, yaw, cov = self.amcl[-1]
                poses.append((x, y, yaw))
                covs.append(cov)
        if not is_still(self.twists, a.still_sec):
            return None, '측정하는 동안 로봇이 움직임 (키보드 조종을 멈춘 뒤 다시)'
        if not poses:
            return None, f'제자리 갱신 뒤 /{self.ns}/amcl_pose 가 안 옴'
        rec = summarize(poses)
        cov = covs[-1]
        rec['std_xy'] = math.sqrt(max(cov[0], cov[7], 0.0))
        rec['std_yaw'] = math.sqrt(max(cov[35], 0.0))
        rec['match'] = self._match((rec['x'], rec['y'], rec['yaw']))
        rec['road_dist'] = self.course.distance_to_road(rec['x'], rec['y'])
        return rec, ''

    def _match(self, pose):
        if self.mm is None or self.scan is None:
            return None
        from pinky_fms_lane.initial_pose import chain, compose, scan_match
        s = self.scan
        offset = chain(self.static, self.args.frame_prefix + 'base_footprint', s.header.frame_id) or (0.0, 0.0, 0.0)
        match, _ = scan_match(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
                              compose(pose, offset), self.mm.dist, self.args.match_tol)
        return match

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def describe(rec, ref, name):
    from pinky_fms_lane.course import SOURCE_LABELS
    lines = [f'  {name}: x {rec["x"]:.3f}  y {rec["y"]:.3f}  yaw {math.degrees(rec["yaw"]):.0f}°  '
             f'(갱신 {rec["samples"]}번, 흔들림 {rec["spread"] * 100:.1f} cm / {math.degrees(rec["yaw_spread"]):.1f}°)',
             ''.join([f'  표준편차 위치 {rec["std_xy"] * 100:.1f} cm, 방향 {math.degrees(rec["std_yaw"]):.1f}°',
                      '' if rec.get('match') is None else f', 스캔 일치율 {rec["match"] * 100:.0f}%',
                      f', 길 가운데 선까지 {rec["road_dist"] * 100:.1f} cm'])]
    if ref is not None:
        lines.append(f'  지금 값 ({SOURCE_LABELS.get(ref.source, ref.source)} {ref.source}): x {ref.x:.3f}  y {ref.y:.3f}'
                     f' → 차이 {math.hypot(rec["x"] - ref.x, rec["y"] - ref.y) * 100:.1f} cm')
    return '\n'.join(lines)


def print_list(course, order, done, skipped):
    from pinky_fms_lane.course import SOURCE_LABELS
    print('  순서  점        지금 출처        이번 기록')
    for i, n in enumerate(order, 1):
        p = course.points[n]
        mark = '기록함' if n in done else ('건너뜀' if n in skipped else '')
        print(f'  {i:>3}  {n:8s}  {SOURCE_LABELS.get(p.source, p.source) + "(" + p.source + ")":14s}  {mark}')


def robot_file_path(course_path):
    with open(course_path, encoding='utf-8') as f:
        name = (yaml.safe_load(f) or {}).get('robot')
    return os.path.join(os.path.dirname(os.path.abspath(course_path)), name) if name else ''


def default_src_course():
    ws = os.environ.get('FMS_WS', '')
    p = os.path.join(ws, 'src', 'pinky_fms_lane', 'control_pc', 'pinky_fms_lane', 'course',
                     'mission4_3_clean_1cm.course.yaml') if ws else ''
    return p if p and os.path.exists(p) else ''


def run(args, ask=input):
    import rclpy
    from pinky_fms_lane.course import Course
    course = Course.load(args.course)
    out = robot_file_path(args.course)
    order = args.points.split(',') if args.points else (course.record_order or list(course.points))
    unknown = [n for n in order if n not in course.points]
    if unknown:
        print(f'[record] 코스에 없는 점: {unknown}', file=sys.stderr)
        return 2
    limits = {'min_samples': max(1, args.updates - 1), 'max_xy_std': args.max_xy_std,
              'max_yaw_std': math.radians(args.max_yaw_std_deg), 'max_spread': args.max_spread,
              'min_match': args.min_match, 'max_road_dist': course.params['lane_width'] / 2,
              'max_diff': args.max_diff, 'max_yaw_diff': math.radians(args.max_yaw_diff_deg)}
    rclpy.init()
    rec_node = Recorder(args, course)
    try:
        bad = rec_node.check_ready()
        if bad:
            for b in bad:
                print(f'[record] 준비 안 됨: {b}')
            return 1
        print(f'[record] {args.namespace} 준비됨. 저장 파일: {out}')
        print('[record] 로봇을 다른 터미널에서 키보드로 몰아 점 위에 세운 뒤 여기서 Enter.'
              ' (이 도구는 로봇을 움직이지 않는다)')
        print('[record] 입력: Enter = 안내한 점 기록, 점 이름 = 그 점 기록, l = 목록, s = 건너뛰기, q = 끝')
        done, skipped = set(), set()
        while True:
            nxt = next_name(order, done, skipped)
            prompt = f'\n다음: {nxt} > ' if nxt else '\n안내할 점을 모두 지남 (점 이름 또는 q) > '
            cmd = ask(prompt).strip()
            if cmd == 'q':
                break
            if cmd == 'l':
                print_list(course, order, done, skipped)
                continue
            if cmd == 's':
                if nxt:
                    skipped.add(nxt)
                continue
            name = cmd or nxt
            if not name:
                continue
            if name not in course.points:
                print(f'  코스에 없는 점: {name}')
                continue
            print(f'  {name} 측정 중: 멈춤 확인 → 제자리 갱신 {args.updates}번 ...', flush=True)
            rec, why = rec_node.measure()
            if rec is None:
                print(f'  실패: {why}')
                continue
            ref = course.points[name]
            print(describe(rec, ref, name))
            warns = warnings_for(rec, ref, limits)
            for w in warns:
                print(f'  ⚠ {w}')
            ans = ask('  경고가 있음. 그래도 저장하려면 y > ' if warns else '  저장하려면 Enter (취소 n) > ').strip()
            if (warns and ans != 'y') or (not warns and ans not in ('', 'y')):
                print('  저장 안 함')
                continue
            entry = robot_entry(rec, ref.yaw is not None, datetime.datetime.now().isoformat(timespec='seconds'))
            backup = save_robot_point(out, name, entry)
            done.add(name)
            print(f'  저장: {out}' + (f' (이전 파일 백업 {os.path.basename(backup)})' if backup else ''))
        print(f'[record] 이번에 기록한 점 {len(done)}개: {", ".join(sorted(done)) or "없음"}')
        if done:
            print('[record] 다른 도구(draw_course 등)의 기본 코스에 반영하려면 colcon build 를 다시 하거나 --course 로 이 파일을 준다')
        return 0
    finally:
        rec_node.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description='로봇으로 코스 점 좌표를 기록한다 (로봇은 움직이지 않음, 키보드 조종은 따로)')
    ap.add_argument('namespace', help='로봇 ID (예: amr_01)')
    ap.add_argument('--course', default=default_src_course(),
                    help='소스 폴더의 코스 yaml (기본: $FMS_WS/src/.../course/mission4_3_clean_1cm.course.yaml)')
    ap.add_argument('--map', default='', help='지도 yaml. 주면 스캔 일치율도 확인한다 (권장)')
    ap.add_argument('--points', default='', help='기록할 점 순서 "A,B,C" (기본: 코스의 record_order)')
    ap.add_argument('--frame-prefix', default='', help="TF 프레임 접두어 (시뮬레이션: 'amr_01/')")
    ap.add_argument('--updates', type=int, default=3, help='점마다 제자리 갱신 횟수 (많이 하면 트인 구간에서 흐름)')
    ap.add_argument('--still-sec', type=float, default=1.0, help='이 시간 동안 odom 속도가 0 근처여야 측정 (s)')
    ap.add_argument('--still-timeout', type=float, default=10.0, help='멈춤을 기다리는 최대 시간 (s)')
    ap.add_argument('--max-xy-std', type=float, default=0.05, help='경고 기준: 위치 표준편차 (m)')
    ap.add_argument('--max-yaw-std-deg', type=float, default=10.0, help='경고 기준: 방향 표준편차 (°)')
    ap.add_argument('--max-spread', type=float, default=0.02, help='경고 기준: 갱신 사이 위치 흔들림 (m)')
    ap.add_argument('--min-match', type=float, default=0.7, help='경고 기준: 스캔 일치율 (0~1)')
    ap.add_argument('--match-tol', type=float, default=0.05, help='빔 끝이 벽에서 이 거리(m) 안이면 일치')
    ap.add_argument('--max-diff', type=float, default=0.05, help='경고 기준: 지금 값(줄자·추정)과 차이 (m)')
    ap.add_argument('--max-yaw-diff-deg', type=float, default=30.0, help='경고 기준: 출발점 방향 차이 (°)')
    args = ap.parse_args(argv)
    if not args.course or not os.path.exists(args.course):
        ap.error(f'코스 파일이 없습니다: {args.course!r}. 소스 폴더의 코스 yaml 을 --course 로 주거나 FMS_WS 를 설정하세요')
    if f'{os.sep}install{os.sep}' in os.path.abspath(args.course):
        ap.error('설치 폴더(install/)의 코스 파일에는 기록하지 않습니다 (다시 빌드하면 사라짐). 소스 폴더의 파일을 주세요')
    if not robot_file_path(args.course):
        ap.error('코스 yaml 에 robot: (로봇 기록 파일 이름)이 없습니다')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
