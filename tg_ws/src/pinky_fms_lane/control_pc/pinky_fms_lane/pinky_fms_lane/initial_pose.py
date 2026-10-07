"""출발점 초기 위치 도구: 코스 파일의 출발점(R1, R2 ...)으로 /<ns>/initialpose 를 보내고, 위치가 잡혔는지 확인한다.

    ros2 run pinky_fms_lane send_initial_pose amr_01:R1 amr_02:R2 --map <지도.yaml> [--course <코스.yaml>]

  로봇을 출발점 테이프 위에 코스 파일의 방향(yaw)대로 놓은 뒤 실행한다. GUI 로 손으로 찍는 것보다 정확하다.
  관제 스택과 같은 터미널 환경(ROS_DOMAIN_ID, DDS 설정: control_pc/fms_env.sh)에서 실행해야 로봇이 보인다.

  보낸 뒤 확인하는 것 (로봇마다 OK / 확인 필요):
    1. amcl_pose 를 받았는가, 출발점과의 차이
    2. AMCL 공분산(표준편차)이 기준 이하인가 → 위치를 얼마나 확신하는가
    3. 스캔 일치율: AMCL 위치에서 본 라이다 빔 끝이 지도의 벽에 얼마나 붙는가 → 위치가 실제로 맞는가
       (공분산만으로는 부족하다. 초기 위치가 틀린 채 서 있으면 AMCL 은 틀린 위치를 확신할 수 있다)
  하나라도 기준을 못 넘으면 종료 코드 1. 이때는 로봇이 출발점 위에 정확히 있는지 확인한다.

  원인 구분 (--map 을 주면, 판정에는 쓰지 않는 참고 숫자):
    4. 위·아래·왼쪽·오른쪽 방향 빔마다 "실제 거리 − 지도에서 예상한 거리"(중앙값)를 출력한다.
       위+아래 합 = 지도 세로 크기 차이 (로봇 위치와 무관), (아래−위)/2 = 로봇이 AMCL 위치보다 실제로 위쪽에 있는 거리.
       가로도 같다. 합이 크면 지도 문제, 합은 작은데 차가 크면 로봇 배치 문제다.
       스캔 일치율이 --diag-min-match(기본 50%)보다 낮으면 위치가 통째로 틀린 것이라 원인 안내를 하지 않는다.
    5. 스캔을 맵에 겹친 그림을 저장한다 (--out 폴더, 기본 FMS_OUTPUT_DIR 또는 $FMS_WS/outputs).
"""
import argparse
import math
import os
import sys
import time

DEFAULT_COURSE = 'mission4_3_clean_1cm.course.yaml'
DIRECTIONS = {'up': math.pi / 2, 'down': -math.pi / 2, 'left': math.pi, 'right': 0.0}   # map 좌표 방향
DIR_KO = {'up': '위', 'down': '아래', 'left': '왼쪽', 'right': '오른쪽'}
DIAG_LIMIT = 0.03      # 원인 구분 안내를 띄우는 차이 (m)


# ---------- ROS 없이 시험할 수 있는 계산 ----------
def parse_targets(items):
    """["amr_01:R1", ...] → [("amr_01", "R1"), ...]"""
    out = []
    for item in items:
        ns, sep, start = item.partition(':')
        ns = ns.strip().strip('/')
        if not sep or not ns or not start.strip():
            raise ValueError(f'"{item}" 는 "로봇ID:출발점" 형식이어야 함 (예: amr_01:R1)')
        out.append((ns, start.strip()))
    return out


def start_pose(course, name):
    """코스 파일의 출발점 → (x, y, yaw, source). yaw 가 없는 점은 출발점으로 쓸 수 없다."""
    p = course.points.get(name)
    if p is None:
        raise ValueError(f'코스 파일에 점 {name} 이 없음')
    if p.yaw is None:
        raise ValueError(f'점 {name} 에 yaw(로봇 방향)가 없어 출발점으로 쓸 수 없음')
    return p.x, p.y, p.yaw, p.source


def covariance(xy_std, yaw_std):
    """PoseWithCovariance 의 6x6 공분산 (x, y, yaw 만)."""
    cov = [0.0] * 36
    cov[0] = cov[7] = xy_std ** 2
    cov[35] = yaw_std ** 2
    return cov


def stds(cov):
    """공분산 → (위치 표준편차 m: x·y 중 큰 쪽, 방향 표준편차 rad)."""
    return math.sqrt(max(cov[0], cov[7], 0.0)), math.sqrt(max(cov[35], 0.0))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """2D 자세 합성 a∘b. 자세는 (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def chain(static, base, frame):
    """정적 TF {자식: (부모, (x, y, yaw))} 로 base → frame 2D 자세. 못 찾으면 None. 같은 프레임이면 원점."""
    path, f = [], frame
    while f != base:
        if f not in static or len(path) > 20:
            return None
        parent, pose = static[f]
        path.append(pose)
        f = parent
    out = (0.0, 0.0, 0.0)
    for pose in reversed(path):
        out = compose(out, pose)
    return out


def scan_match(ranges, angle_min, angle_inc, range_min, range_max, laser_pose, dist_fn, tol):
    """라이다 빔 끝 중 지도의 벽에서 tol 안에 있는 비율 → (비율, 빔 수). dist_fn(xs, ys) 는 벽까지 거리 배열."""
    import numpy as np
    r = np.asarray(ranges, float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    if not ok.any():
        return 0.0, 0
    x = laser_pose[0] + r[ok] * np.cos(laser_pose[2] + a[ok])
    y = laser_pose[1] + r[ok] * np.sin(laser_pose[2] + a[ok])
    d = dist_fn(x, y)
    return float(np.mean(d <= tol)), int(ok.sum())


def evaluate(amcl, cov, match, limits):
    """판정 → (OK 여부, 이유 목록). limits: max_xy_std, max_yaw_std, min_match, need_match(지도를 받았으면 True)."""
    reasons = []
    if amcl is None:
        return False, ['amcl_pose 를 받지 못함 (AMCL 이 떠 있고 initialpose 를 받았는지 확인)']
    sxy, syaw = stds(cov)
    if sxy > limits['max_xy_std']:
        reasons.append(f'위치 표준편차 {sxy * 100:.1f} cm > 기준 {limits["max_xy_std"] * 100:.0f} cm')
    if syaw > limits['max_yaw_std']:
        reasons.append(f'방향 표준편차 {math.degrees(syaw):.1f}° > 기준 {math.degrees(limits["max_yaw_std"]):.0f}°')
    if match is None:
        if limits['need_match']:
            reasons.append('스캔을 받지 못해 일치율을 계산하지 못함 (/<ns>/scan 확인)')
    elif match < limits['min_match']:
        reasons.append(f'스캔 일치율 {match * 100:.0f}% < 기준 {limits["min_match"] * 100:.0f}% '
                       '(위치가 틀렸을 수 있음: 로봇이 출발점·방향대로 놓였는지 확인)')
    return not reasons, reasons


def expected_ranges(occ, res, origin, x, y, angles, rmax, step=0.005):
    """지도에서 (x, y) 로부터 각 방향(map 기준 각도)으로 빔이 처음 닿는 거리. 닿지 않으면 inf. occ[행=y, 열=x]."""
    import numpy as np
    a = np.asarray(angles, float)
    r = np.arange(step, rmax, step)
    px = x + np.cos(a)[:, None] * r[None, :]
    py = y + np.sin(a)[:, None] * r[None, :]
    col = np.floor((px - origin[0]) / res).astype(int)
    row = np.floor((py - origin[1]) / res).astype(int)
    inside = (col >= 0) & (col < occ.shape[1]) & (row >= 0) & (row < occ.shape[0])
    hit = np.zeros(inside.shape, bool)
    hit[inside] = occ[row[inside], col[inside]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    return np.where(first >= 0, r[np.clip(first, 0, None)], np.inf)


def direction_offsets(angles, measured, expected, range_min, range_max, max_diff=0.30, cone_deg=30.0, min_beams=3):
    """방향(위·아래·왼쪽·오른쪽)별 "실제 − 지도" 벽 거리 차이 → {방향: (차이 m 또는 None, 빔 수)}.
    빔 길이 차이에 cos 를 곱해 그 축 방향 거리로 바꾼다. 차이가 max_diff 보다 큰 빔(다른 로봇·물체)은 뺀다."""
    import numpy as np
    a = np.asarray(angles, float)
    m = np.asarray(measured, float)
    e = np.asarray(expected, float)
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(m) & np.isfinite(e) & (m >= range_min) & (m <= range_max) & (np.abs(m - e) <= max_diff)
    out = {}
    for name, d in DIRECTIONS.items():
        off = (a - d + math.pi) % (2 * math.pi) - math.pi
        sel = ok & (np.abs(off) <= math.radians(cone_deg))
        n = int(sel.sum())
        out[name] = (float(np.median((m[sel] - e[sel]) * np.cos(off[sel]))) if n >= min_beams else None, n)
    return out


def axis_summary(off):
    """방향별 차이 → {'vertical': (지도 크기 차이, 로봇 위치 어긋남), 'horizontal': (...)} (m, 모르면 None).
    크기 차이 = 양쪽 합 (+ 이면 실제가 지도보다 넓다). 위치 어긋남 = 실제 − AMCL (+ 이면 실제 로봇이 위쪽/오른쪽)."""
    out = {}
    for axis, (pos, neg) in (('vertical', ('up', 'down')), ('horizontal', ('right', 'left'))):
        p, n = off[pos][0], off[neg][0]
        out[axis] = (None, None) if p is None or n is None else (p + n, (n - p) / 2)
    return out


def diagnosis(summary, limit=DIAG_LIMIT):
    """원인 구분 안내 문장 목록."""
    notes = []
    for axis, ko in (('vertical', '세로'), ('horizontal', '가로')):
        size, shift = summary[axis]
        if size is not None and abs(size) >= limit:
            notes.append(f'{ko} 지도 크기 차이 {size * 100:+.1f} cm: 지도 문제일 가능성 (로봇을 바르게 놓아도 일치율이 낮을 수 있음)')
        if shift is not None and abs(shift) >= limit and (size is None or abs(size) < limit):
            notes.append(f'{ko} 로봇 위치 어긋남 {shift * 100:+.1f} cm: 로봇 배치 문제일 가능성')
    return notes


# ---------- ROS ----------
def default_course_path():
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return ''
    try:
        return os.path.join(get_package_share_directory('pinky_fms_lane'), 'course', DEFAULT_COURSE)
    except PackageNotFoundError:
        return ''


class MapModel:
    """지도 → 벽(점유 칸)까지 거리, 빔이 닿을 거리 예상. 팀원 GridMap 재사용 (수정 없음)."""

    def __init__(self, map_yaml):
        from scipy import ndimage as ndi
        from pinky_fms_traffic.gridmap import GridMap
        self.gm = gm = GridMap(map_yaml, res=0.01)
        self.dist_field = ndi.distance_transform_edt(~gm.occ) * gm.res

    def dist(self, xs, ys):
        import numpy as np
        gm = self.gm
        r = np.clip(((ys - gm.oy) / gm.res).astype(int), 0, gm.H - 1)
        c = np.clip(((xs - gm.ox) / gm.res).astype(int), 0, gm.W - 1)
        return self.dist_field[r, c]

    def expected(self, x, y, angles, rmax):
        return expected_ranges(self.gm.occ, self.gm.res, (self.gm.ox, self.gm.oy), x, y, angles, rmax)


def save_scan_image(path, mm, title, robot, laser, ends, matched, expected_ends):
    """스캔을 지도에 겹친 그림. 초록 = 벽에 맞는 빔 끝, 빨강 = 안 맞는 빔 끝, 파란 점 = 지도에서 예상한 빔 끝."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from pinky_fms_lane.draw_course import korean_font
    font = korean_font()
    if font:
        plt.rcParams['font.family'] = font
    gm = mm.gm
    shade = np.where(gm.occ, 0.15, np.where(gm.free, 1.0, 0.75))
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.imshow(shade, cmap='gray', vmin=0, vmax=1, origin='lower', interpolation='nearest',
              extent=(gm.ox, gm.ox + gm.W * gm.res, gm.oy, gm.oy + gm.H * gm.res))
    if len(expected_ends):
        ax.scatter(expected_ends[:, 0], expected_ends[:, 1], s=4, color='#2c7fb8', label='지도에서 예상한 빔 끝')
    ax.scatter(ends[matched, 0], ends[matched, 1], s=7, color='#1b9e77', label='실제 빔 끝: 벽에 맞음')
    ax.scatter(ends[~matched, 0], ends[~matched, 1], s=7, color='#e41a1c', label='실제 빔 끝: 안 맞음')
    ax.annotate('', xy=(robot[0] + 0.12 * math.cos(robot[2]), robot[1] + 0.12 * math.sin(robot[2])),
                xytext=robot[:2], arrowprops={'arrowstyle': '-|>', 'color': '#111111', 'lw': 2.0})
    ax.plot([laser[0]], [laser[1]], 'k+', ms=8)
    pts = np.vstack([ends, expected_ends, [robot[:2]]]) if len(expected_ends) else np.vstack([ends, [robot[:2]]])
    ax.set_xlim(pts[:, 0].min() - 0.2, pts[:, 0].max() + 0.2)
    ax.set_ylim(pts[:, 1].min() - 0.2, pts[:, 1].max() + 0.2)
    ax.set_aspect('equal')
    ax.grid(True, lw=0.4, alpha=0.5)
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')
    ax.set_title(title, fontsize=11)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def report_walls(ns, name, scan, laser, robot, match, mm, args):
    """방향별 벽 거리 차이 출력 + 겹침 그림 저장 (판정에는 쓰지 않는다)."""
    import datetime
    import numpy as np
    r = np.asarray(scan.ranges, float)
    ang = laser[2] + scan.angle_min + np.arange(len(r)) * scan.angle_increment
    exp = mm.expected(laser[0], laser[1], ang, scan.range_max)
    off = direction_offsets(ang, r, exp, scan.range_min, scan.range_max)
    parts = [f'{DIR_KO[k]} ' + ('—' if v is None else f'{v * 100:+.1f} cm') + f' ({n}개)' for k, (v, n) in off.items()]
    print(f'[{ns}] 벽까지 거리 차이 (실제 − 지도, 빔 중앙값): ' + ', '.join(parts), flush=True)
    summ = axis_summary(off)
    fmt = lambda v: '—' if v is None else f'{v * 100:+.1f} cm'          # noqa: E731
    print(f'[{ns}]   세로: 지도 크기 차이 {fmt(summ["vertical"][0])}, 로봇 위치 어긋남 {fmt(summ["vertical"][1])}'
          f' / 가로: 지도 크기 차이 {fmt(summ["horizontal"][0])}, 로봇 위치 어긋남 {fmt(summ["horizontal"][1])}', flush=True)
    if match < args.diag_min_match:          # 위치가 통째로 틀리면 빔이 다른 벽에 닿아 방향별 차이로 원인을 나눌 수 없다
        print(f'[{ns}]   → 일치율 {match * 100:.0f}% 가 {args.diag_min_match * 100:.0f}% 보다 낮아 원인을 나눌 수 없음: '
              '로봇 배치(위치·방향)부터 확인', flush=True)
    else:
        for note in diagnosis(summ):
            print(f'[{ns}]   → {note}', flush=True)
    if args.no_image or not args.out:
        return
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    ends = np.stack([laser[0] + r[ok] * np.cos(ang[ok]), laser[1] + r[ok] * np.sin(ang[ok])], axis=1)
    matched = mm.dist(ends[:, 0], ends[:, 1]) <= args.match_tol
    fin = np.isfinite(exp)
    exp_ends = np.stack([laser[0] + exp[fin] * np.cos(ang[fin]), laser[1] + exp[fin] * np.sin(ang[fin])], axis=1)
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, f'result_initpose_{ns}_{datetime.datetime.now():%Y%m%d_%H%M%S}.png')
    title = (f'{ns} @ {name}: 스캔 일치율 {match * 100:.0f}%  |  세로 지도 크기 차이 {fmt(summ["vertical"][0])}, '
             f'가로 {fmt(summ["horizontal"][0])}')
    save_scan_image(path, mm, title, robot, laser, ends, matched, exp_ends)
    print(f'[{ns}] 그림: {path}', flush=True)


def run(args):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Empty
    from tf2_msgs.msg import TFMessage

    from pinky_fms_lane.course import Course

    targets = parse_targets(args.targets)
    course = Course.load(args.course)
    starts = {ns: (name,) + start_pose(course, name) for ns, name in targets}
    mm = MapModel(args.map) if args.map else None
    limits = {'max_xy_std': args.max_xy_std, 'max_yaw_std': math.radians(args.max_yaw_std_deg),
              'min_match': args.min_match, 'need_match': bool(args.map)}
    latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

    rclpy.init()
    node = Node('send_initial_pose')
    state = {ns: {'amcl': None, 'cov': None, 'scan': None, 'static': {}, 't_sent': None} for ns in starts}

    def on_amcl(ns, m):
        st = state[ns]
        if st['t_sent'] is not None and time.monotonic() > st['t_sent']:
            p = m.pose.pose
            q = p.orientation
            st['amcl'] = (p.position.x, p.position.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            st['cov'] = list(m.pose.covariance)

    def on_static(ns, m):
        for t in m.transforms:
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            state[ns]['static'][t.child_frame_id] = (t.header.frame_id,
                                                     (t.transform.translation.x, t.transform.translation.y, yaw))

    pubs = {}
    for ns in starts:
        pubs[ns] = node.create_publisher(PoseWithCovarianceStamped, f'/{ns}/initialpose', 10)
        node.create_subscription(PoseWithCovarianceStamped, f'/{ns}/amcl_pose', lambda m, ns=ns: on_amcl(ns, m), 10)
        node.create_subscription(LaserScan, f'/{ns}/scan', lambda m, ns=ns: state[ns].__setitem__('scan', m),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(TFMessage, f'/{ns}/tf_static', lambda m, ns=ns: on_static(ns, m), latched)

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    all_ok = True
    for ns, (name, x, y, yaw, source) in starts.items():
        print(f'[{ns}] 출발점 {name} ({source}): x {x:.3f}  y {y:.3f}  yaw {math.degrees(yaw):.0f}°', flush=True)
        end = time.monotonic() + args.timeout
        while node.count_subscribers(f'/{ns}/initialpose') < 1 and time.monotonic() < end:
            spin_for(0.2)
        if node.count_subscribers(f'/{ns}/initialpose') < 1:
            print(f'[{ns}] → 확인 필요: /{ns}/initialpose 를 받는 노드(AMCL)가 없음 '
                  '(로봇 위치 추정이 켜져 있는지, 같은 ROS_DOMAIN_ID·DDS 설정인지 확인)', flush=True)
            all_ok = False
            continue
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance = covariance(args.xy_std, math.radians(args.yaw_std_deg))
        for _ in range(2):                                   # GUI 처럼 두 번 보낸다 (처음 것이 유실될 수 있음)
            m.header.stamp = node.get_clock().now().to_msg()
            pubs[ns].publish(m)
            spin_for(0.5)
        state[ns]['t_sent'] = time.monotonic()
        cli = node.create_client(Empty, f'/{ns}/request_nomotion_update')
        n_upd = 0
        if cli.wait_for_service(timeout_sec=2.0):           # 제자리 갱신은 몇 번만 (1.3 실험: 반복하면 트인 구간에서 흐름)
            for _ in range(args.updates):
                fut = cli.call_async(Empty.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                n_upd += fut.result() is not None
                spin_for(0.3)
        node.destroy_client(cli)
        end = time.monotonic() + args.timeout
        while state[ns]['amcl'] is None and time.monotonic() < end:
            spin_for(0.2)
        spin_for(1.0)                                        # 마지막 갱신 결과와 스캔을 받는다

        st = state[ns]
        print(f'[{ns}] initialpose 보냄 (표준편차 {args.xy_std * 100:.0f} cm, {args.yaw_std_deg:.0f}°), '
              f'제자리 갱신 {n_upd}/{args.updates}번', flush=True)
        match = None
        if st['amcl'] is not None:
            ax, ay, ayaw = st['amcl']
            sxy, syaw = stds(st['cov'])
            print(f'[{ns}] amcl_pose: x {ax:.3f}  y {ay:.3f}  yaw {math.degrees(ayaw):.0f}° '
                  f'(출발점과 차이 {math.hypot(ax - x, ay - y) * 100:.1f} cm, {math.degrees(abs(wrap(ayaw - yaw))):.1f}°), '
                  f'표준편차 위치 {sxy * 100:.1f} cm, 방향 {math.degrees(syaw):.1f}°', flush=True)
            scan = st['scan']
            if mm is not None and scan is not None:
                base = args.frame_prefix + 'base_footprint'
                offset = chain(st['static'], base, scan.header.frame_id)
                if offset is None:
                    print(f'[{ns}] 주의: {base} → {scan.header.frame_id} 정적 TF 를 못 찾아 라이다를 로봇 중심으로 봄', flush=True)
                    offset = (0.0, 0.0, 0.0)
                laser = compose(st['amcl'], offset)
                match, n = scan_match(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                                      scan.range_max, laser, mm.dist, args.match_tol)
                print(f'[{ns}] 스캔 일치율 {match * 100:.0f}% (빔 {n}개 중 끝이 벽 {args.match_tol * 100:.0f} cm 안)', flush=True)
                report_walls(ns, name, scan, laser, st['amcl'], match, mm, args)
        if not args.map:
            print(f'[{ns}] 스캔 일치율 확인 건너뜀 (--map 을 주면 위치가 실제로 맞는지 확인한다)', flush=True)
        ok, reasons = evaluate(st['amcl'], st['cov'], match, limits)
        all_ok &= ok
        print(f'[{ns}] → ' + ('OK' if ok else '확인 필요: ' + '; '.join(reasons)), flush=True)

    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if all_ok else 1


def default_out_dir():
    from pinky_fms_lane.draw_course import default_out_dir as d
    return d()


def main(argv=None):
    ap = argparse.ArgumentParser(description='코스 파일의 출발점으로 /<ns>/initialpose 를 보내고 위치가 잡혔는지 확인한다')
    ap.add_argument('targets', nargs='+', help='로봇ID:출발점 (예: amr_01:R1 amr_02:R2)')
    ap.add_argument('--course', default=default_course_path(), help='코스 yaml (기본: 설치된 기본 코스)')
    ap.add_argument('--map', default='', help='지도 yaml. 주면 스캔 일치율로 위치가 맞는지 확인한다 (권장)')
    ap.add_argument('--frame-prefix', default='', help="TF 프레임 접두어 (시뮬레이션: 'amr_01/')")
    ap.add_argument('--xy-std', type=float, default=0.05, help='보낼 초기 위치 표준편차 (m)')
    ap.add_argument('--yaw-std-deg', type=float, default=5.0, help='보낼 초기 방향 표준편차 (°)')
    ap.add_argument('--updates', type=int, default=3, help='보낸 뒤 제자리 갱신 횟수 (많이 하면 트인 구간에서 흐를 수 있음)')
    ap.add_argument('--timeout', type=float, default=10.0, help='AMCL 을 기다리는 시간 (s)')
    ap.add_argument('--max-xy-std', type=float, default=0.10, help='OK 기준: 위치 표준편차 (m)')
    ap.add_argument('--max-yaw-std-deg', type=float, default=10.0, help='OK 기준: 방향 표준편차 (°)')
    ap.add_argument('--min-match', type=float, default=0.7, help='OK 기준: 스캔 일치율 (0~1, 실물에서 조정)')
    ap.add_argument('--match-tol', type=float, default=0.05, help='빔 끝이 벽에서 이 거리(m) 안이면 일치')
    ap.add_argument('--out', default=default_out_dir(), help='겹침 그림 폴더 (기본: FMS_OUTPUT_DIR, 없으면 $FMS_WS/outputs)')
    ap.add_argument('--no-image', action='store_true', help='겹침 그림을 저장하지 않는다')
    ap.add_argument('--diag-min-match', type=float, default=0.5,
                    help='스캔 일치율이 이보다 낮으면 원인(지도/배치) 안내를 하지 않는다 (위치가 통째로 틀린 경우)')
    args = ap.parse_args(argv)
    if not args.course or not os.path.exists(args.course):
        ap.error(f'코스 파일이 없습니다: {args.course!r} (--course 로 지정)')
    try:
        parse_targets(args.targets)
    except ValueError as e:
        ap.error(str(e))
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
