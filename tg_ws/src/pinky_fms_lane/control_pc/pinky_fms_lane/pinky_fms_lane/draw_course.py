"""코스 확인 그림: 지도 위에 코스(길·갈림길·대기 지점·횡단보도·출발점)와 예시 경로를 그려 PNG 로 저장한다.

    ros2 run pinky_fms_lane draw_course --map <지도.yaml> [--course <코스.yaml>] [--out <폴더>]
                                        [--example R1:loop:0.30,R2:tail:0.55]

  --course 를 생략하면 설치된 기본 코스(share/pinky_fms_lane/course/mission4_3_clean_1cm.course.yaml)를 쓴다.
  --out 을 생략하면 FMS_OUTPUT_DIR, 없으면 $FMS_WS/outputs 에 저장한다. 파일 이름: result_course_<YYYYmmdd_HHMMSS>.png
  --example 은 "출발점:간선:간선 길이 비율" 목록. 비율로 목표를 정하므로 좌표가 실측값으로 바뀌어도 그대로 쓸 수 있다. 빈 문자열이면 경로를 그리지 않는다.
  추정값(source: estimate) 점은 속이 빈 표시로, 실측값은 채운 표시로 그린다.
"""
import argparse
import datetime
import math
import os
import sys

import numpy as np
import yaml

from pinky_fms_lane.course import Course, PlanError

DEFAULT_COURSE = 'mission4_3_clean_1cm.course.yaml'
EDGE_COLORS = {'tail': '#1b9e77', 'loop': '#2c7fb8'}
ROUTE_COLORS = ['#d95f02', '#7570b3', '#e7298a', '#66a61e']


def default_course_path():
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:                                  # ROS 환경을 source 하지 않은 경우
        return ''
    try:
        return os.path.join(get_package_share_directory('pinky_fms_lane'), 'course', DEFAULT_COURSE)
    except PackageNotFoundError:
        return ''


def default_out_dir():
    if os.environ.get('FMS_OUTPUT_DIR'):
        return os.environ['FMS_OUTPUT_DIR']
    if os.environ.get('FMS_WS'):
        return os.path.join(os.environ['FMS_WS'], 'outputs')
    return ''


def load_map(map_yaml):
    """지도 → (회색 영상, extent[x0, x1, y0, y1], 벽까지 거리 함수 또는 None)."""
    from pinky_fms_traffic.gridmap import GridMap, read_pgm      # 팀원 코드 재사용 (수정 없음)
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta['image']))
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    extent = [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]
    gm = GridMap(map_yaml)
    return img, extent, (lambda x, y: float(gm.clearance_at([[x, y]])[0]))


def korean_font():
    from matplotlib import font_manager
    for f in font_manager.fontManager.ttflist:
        if 'CJK KR' in f.name or 'Nanum' in f.name:
            return f.name
    for f in font_manager.fontManager.ttflist:
        if 'CJK' in f.name:
            return f.name
    return None


def parse_examples(text, course):
    """예시 문자열 "R1:loop:0.30,R2:tail:0.55" → [(출발점 이름, 시작, 목표)]. 목표는 GUI 클릭처럼 snap_goal 을 거친다."""
    out = []
    for item in [t.strip() for t in (text or '').split(',') if t.strip()]:
        name, edge, frac = item.split(':')
        p = course.points[name]
        start = course.locate(p.x, p.y, p.yaw)
        q = course.location_at(edge, float(frac) * course.edges[edge].length)
        goal, note = course.snap_goal(q.x, q.y)
        if start is None or goal is None:
            print(f'[draw_course] 예시 {item}: 출발점이나 목표가 길 위에 없음 ({note})', file=sys.stderr)
            continue
        out.append((name, start, goal))
    return out


def draw(course, map_yaml, out_path, examples):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    font = korean_font()
    if font:
        plt.rcParams['font.family'] = font
    plt.rcParams['axes.unicode_minus'] = False

    img, extent, clearance = load_map(map_yaml)
    fig, ax = plt.subplots(figsize=(13, 7.6))
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper', alpha=0.55)
    pts = np.array([[p.x, p.y] for p in course.points.values()])
    pad = 0.25
    ax.set_xlim(pts[:, 0].min() - pad, pts[:, 0].max() + pad)
    ax.set_ylim(pts[:, 1].min() - pad, pts[:, 1].max() + pad)
    ax.set_aspect('equal')
    fig.canvas.draw()
    px_per_m = ax.transData.transform((1, 0))[0] - ax.transData.transform((0, 0))[0]
    lane_lw = course.params['lane_width'] * px_per_m * 72.0 / fig.dpi

    # 길: 폭(반투명 띠) + 가운데 선 + 방향 화살표
    for name, e in course.edges.items():
        color = EDGE_COLORS.get(name, '#555555')
        ax.plot(e.pts[:, 0], e.pts[:, 1], color=color, lw=lane_lw, alpha=0.22, solid_capstyle='round',
                solid_joinstyle='round', zorder=2)
        ax.plot(e.pts[:, 0], e.pts[:, 1], color=color, lw=1.6, zorder=3,
                label=f'{name}: {"일방통행" if e.oneway else "양방향"} ({e.length:.2f} m)')
        for s in np.arange(0.25, e.length, 0.45):
            a, b = e.point_at(s - 0.04), e.point_at(s + 0.04)
            style = '-|>' if e.oneway else '<|-|>'
            ax.annotate('', xy=b, xytext=a, arrowprops={'arrowstyle': style, 'color': color, 'lw': 1.4}, zorder=4)
        # 목표 금지 구간 (갈림길·대기·횡단보도·막다른 끝)
        for lo, hi, _ in course.zones(name):
            seg = np.array([e.point_at(s) for s in np.linspace(lo, hi, 12)])
            ax.plot(seg[:, 0], seg[:, 1], color='#e41a1c', lw=4.5, alpha=0.35, zorder=3)
        # 지도 기준 U턴 공간(벽까지 거리)이 모자란 곳
        samples = np.array([e.point_at(s) for s in np.arange(0.0, e.length, 0.02)])
        bad = samples[[clearance(x, y) < course.params['uturn_clearance'] for x, y in samples]]
        if len(bad):
            ax.scatter(bad[:, 0], bad[:, 1], s=9, color='#984ea3', zorder=5)

    for j in course.junctions.values():
        ax.add_patch(Circle((j.x, j.y), j.radius, fill=False, ls='--', lw=1.4, color='#e41a1c', zorder=6))
    for name, (x, y, r) in course.crosswalks.items():
        ax.add_patch(Circle((x, y), r, fill=True, alpha=0.25, color='#ff7f00', zorder=6))
    for name, loc in course.waits.items():
        ax.add_patch(Circle((loc.x, loc.y), course.params['wait_keepout'], fill=False, ls=':', lw=1.2,
                            color='#6a3d9a', zorder=6))

    # 이름 붙은 점: 추정값 = 빈 표시, 실측값 = 채운 표시
    kinds = {n: 'D' for n in course.waits}
    kinds.update({n: 's' for n in course.crosswalks})
    kinds.update({n: 'o' for n in course.junctions})
    for p in course.points.values():
        est = p.source == 'estimate'
        if p.yaw is not None:                              # 출발점: 방향 화살표
            ax.annotate('', xy=(p.x + 0.12 * math.cos(p.yaw), p.y + 0.12 * math.sin(p.yaw)), xytext=(p.x, p.y),
                        arrowprops={'arrowstyle': '-|>', 'color': '#111111', 'lw': 2.0}, zorder=8)
            marker = '^'
        else:
            marker = kinds.get(p.name, '.')
        ax.scatter([p.x], [p.y], s=46, marker=marker, zorder=9, linewidths=1.4, edgecolors='#111111',
                   facecolors='none' if est else '#111111')
        ax.annotate(p.name + (' (추정)' if est else ''), (p.x, p.y), xytext=(5, 6), textcoords='offset points',
                    fontsize=8.5, zorder=10,
                    bbox={'boxstyle': 'round,pad=0.15', 'fc': 'white', 'ec': 'none', 'alpha': 0.75})

    # 예시 경로
    for i, (name, start, goal) in enumerate(examples):
        color = ROUTE_COLORS[i % len(ROUTE_COLORS)]
        ok_u, _ = course.can_uturn(start.x, start.y, clearance)
        try:
            r = course.plan(start, goal, allow_uturn=ok_u)
        except PlanError as e:
            print(f'[draw_course] 예시 {name}: 경로 없음 ({e})', file=sys.stderr)
            continue
        off = (i + 1) * 0.012                              # 두 경로가 겹쳐도 보이게 조금 옮긴다
        ax.plot(r.points[:, 0] + off, r.points[:, 1] + off, color=color, lw=2.2, ls='--', zorder=7,
                label=f'예시 {name} → {goal.edge} ({r.length:.2f} m{", 출발 U턴" if r.uturn else ""})')
        ax.scatter([goal.x], [goal.y], marker='*', s=160, color=color, edgecolors='#111111', zorder=9)
        for m in r.maneuvers:
            edge, s_edge = r.locate_s(m.s_at)
            q = course.edges[edge].point_at(s_edge)
            ax.annotate(f'{name}@{m.node_id}: {m.turn} / {m.follow} 테이프 {m.follow_dist:.2f} m', q,
                        xytext=(12, -16 - 14 * i), textcoords='offset points', fontsize=8.5, color=color, zorder=10,
                        bbox={'boxstyle': 'round,pad=0.15', 'fc': 'white', 'ec': color, 'alpha': 0.85})

    n_est = len(course.estimate_points())
    title = f'코스 확인: {course.map_name or os.path.basename(map_yaml)}'
    title += f'  — 추정값 {n_est}개 포함 (속이 빈 점, 실측 전 확인용)' if n_est else '  — 모두 실측값'
    ax.set_title(title, fontsize=12)
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')
    ax.grid(True, lw=0.4, alpha=0.5)
    ax.plot([], [], color='#e41a1c', lw=4.5, alpha=0.35, label='목표 금지 구간 (갈림길·대기·횡단보도·막다른 끝)')
    ax.scatter([], [], s=9, color='#984ea3', label=f'U턴 공간 부족 (벽까지 < {course.params["uturn_clearance"]:.2f} m)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=8.5, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description='코스를 지도 위에 그려 PNG 로 저장한다')
    ap.add_argument('--map', required=True,
                    help='지도 yaml (예: <저장소>/tg_ws/src/pinky_fms/maps/mission4_3_clean_1cm.yaml)')
    ap.add_argument('--course', default=default_course_path(), help='코스 yaml (기본: 설치된 기본 코스)')
    ap.add_argument('--out', default=default_out_dir(), help='저장 폴더 (기본: FMS_OUTPUT_DIR, 없으면 $FMS_WS/outputs)')
    ap.add_argument('--example', default='R1:loop:0.30,R2:tail:0.55', help='예시 경로 "출발점:간선:비율,..." (빈 문자열이면 생략)')
    args = ap.parse_args(argv)
    if not args.course or not os.path.exists(args.course):
        ap.error(f'코스 파일이 없습니다: {args.course!r} (--course 로 지정)')
    if not args.out:
        ap.error('저장 폴더를 모릅니다. --out 을 주거나 FMS_OUTPUT_DIR 또는 FMS_WS 를 설정하세요')
    course = Course.load(args.course)
    os.makedirs(args.out, exist_ok=True)
    out_path = os.path.join(args.out, f'result_course_{datetime.datetime.now():%Y%m%d_%H%M%S}.png')
    draw(course, args.map, out_path, parse_examples(args.example, course))
    est = course.estimate_points()
    print(f'[draw_course] 저장: {out_path}')
    print(f'[draw_course] 추정값 {len(est)}개: {", ".join(est)}' if est else '[draw_course] 모두 실측값')
    return 0


if __name__ == '__main__':
    sys.exit(main())
