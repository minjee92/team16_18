"""GUI 표시용 차선 지도 만들기: 코스 좌표로 길을 칠한 지도(yaml + pgm)와 미리보기 PNG 를 저장한다.

    ros2 run pinky_fms_lane make_lane_map --map <지도.yaml> [--course <코스.yaml>] [--out <폴더>]

  ※ GUI 에 길을 보여 주기 위한 지도다. AMCL, Nav2, 시뮬레이션에는 쓰지 않는다.
    길 밖 바닥을 '미확인'으로 칠하므로, 위치 추정·경로 계획에 쓰면 길 밖을 벽처럼 다루게 된다.
  입력 지도와 원점·해상도·크기가 같다. 벽·상자(점유) = 0 그대로, 길 = 이동 가능(254),
  길 밖 바닥과 횡단보도 줄무늬 = 미확인(205).
  코스 좌표는 Course 가 읽은 값(robot > tape > estimate)을 쓰므로, 좌표가 바뀌면 이 명령을 다시 실행하면 된다.
  저장 파일: <지도 이름>_lanes_gui.yaml / .pgm / _preview.png (--out 생략 시 FMS_OUTPUT_DIR 또는 $FMS_WS/outputs)

  참고한 원본: /home/mindy/dev_ws/course_ref/make_lane_map.py (사용자 스크립트, shapely 사용).
  여기서는 shapely 없이 numpy 로 같은 모양을 칠한다: 꼬리 꺾이는 곳을 둥글게(간선 corner_round),
  막다른 끝은 평평하게, 갈림길 끝·꺾이는 곳은 둥글게 잇고, 횡단보도는 길 방향 줄무늬 5개.
"""
import argparse
import os
import sys

import numpy as np
import yaml

ROAD, UNKNOWN, OCCUPIED = 254, 205, 0
STRIPES = 5
# app.js buildMapBitmap 과 같은 3색
GUI_COLORS = {'occ': (15, 23, 42), 'free': (226, 232, 240), 'unknown': (100, 116, 139)}


def rounded(pts, t, n=12):
    """꺾은선의 안쪽 꼭짓점을 2차 베지어로 둥글게 (꼭짓점에서 최대 t m 떨어진 곳부터)."""
    pts = [np.asarray(p, float) for p in pts]
    if t <= 0 or len(pts) < 3:
        return np.array(pts)
    out = [pts[0]]
    for a, b, d in zip(pts, pts[1:], pts[2:]):
        u1, u2 = (a - b) / np.linalg.norm(a - b), (d - b) / np.linalg.norm(d - b)
        if abs(np.dot(u1, u2) + 1) < 1e-6:                   # 일직선
            out.append(b)
            continue
        tt = min(t, 0.45 * np.linalg.norm(a - b), 0.45 * np.linalg.norm(d - b))
        p1, p2 = b + u1 * tt, b + u2 * tt
        out += [(1 - k) ** 2 * p1 + 2 * (1 - k) * k * b + k ** 2 * p2 for k in np.linspace(0, 1, n)]
    out.append(pts[-1])
    return np.array(out)


def band_mask(X, Y, pts, half, round_start, round_end):
    """꺾은선 pts 를 가운데로 하는 폭 2*half 띠. 꺾이는 곳은 둥글게, 끝은 round_* 이면 둥글게 아니면 평평하게."""
    m = np.zeros(X.shape, bool)
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a
        L2 = float(ab @ ab)
        if L2 < 1e-12:
            continue
        t = ((X - a[0]) * ab[0] + (Y - a[1]) * ab[1]) / L2
        perp = np.abs((X - a[0]) * ab[1] - (Y - a[1]) * ab[0]) / np.sqrt(L2)
        m |= (t >= 0) & (t <= 1) & (perp <= half)
    joins = list(pts[1:-1])
    if round_start:
        joins.append(pts[0])
    if round_end:
        joins.append(pts[-1])
    for p in joins:
        m |= np.hypot(X - p[0], Y - p[1]) <= half
    return m


def stripe_mask(X, Y, center, u, lane_w, cw_len):
    """횡단보도 줄무늬: 길 방향(u)으로 길이 cw_len*0.84, 길 폭을 STRIPES 개로 나눈 띠 (make_lane_map.py 와 같은 모양)."""
    n = np.array([-u[1], u[0]])
    sw = lane_w / (2 * STRIPES)
    dx, dy = X - center[0], Y - center[1]
    along = dx * u[0] + dy * u[1]
    across = dx * n[0] + dy * n[1]
    m = np.zeros(X.shape, bool)
    for i in range(STRIPES):
        off = -lane_w / 2 + sw * (2 * i + 0.5)
        m |= (np.abs(along) <= cw_len * 0.42) & (across >= off) & (across <= off + sw)
    return m


def lane_image(course, base_occ, res, origin, cw_len):
    """코스 → GUI 표시용 지도 픽셀 (행 0 = 맨 위, 입력 지도와 같은 크기). base_occ = 입력 지도의 점유 칸 (같은 배치)."""
    H, W = base_occ.shape
    cols, rows = np.meshgrid(np.arange(W), np.arange(H))
    X = origin[0] + (cols + 0.5) * res
    Y = origin[1] + (H - rows - 0.5) * res
    half = course.params['lane_width'] / 2
    road = np.zeros((H, W), bool)
    for e in course.edges.values():
        closed = e.node_a == e.node_b
        pts = e.pts if closed else rounded(e.pts, e.corner_round)
        if closed:
            pts = np.vstack([pts, pts[1:2]])               # 한 바퀴 도는 간선: 시작 꼭짓점도 꺾이는 곳으로 잇는다
        road |= band_mask(X, Y, pts, half, closed or e.node_a in course.junctions,
                          closed or e.node_b in course.junctions)
    stripes = np.zeros((H, W), bool)
    for x, y, _ in course.crosswalks.values():
        loc = course.locate(x, y, max_dist=course.params['lane_width'])
        if loc is None:
            continue
        u = course.edges[loc.edge].tangent_at(loc.s)
        stripes |= stripe_mask(X, Y, (loc.x, loc.y), u, course.params['lane_width'], cw_len)
    out = np.full((H, W), UNKNOWN, np.uint8)
    out[road] = ROAD
    out[stripes] = UNKNOWN
    out[base_occ] = OCCUPIED                              # 벽·상자는 입력 지도 그대로
    return out


def gui_preview(img, occupied_thresh, free_thresh, scale=4):
    """GUI(app.js)가 그릴 3색 모습. 길 주변(길 + 15칸)만 잘라 scale 배로 키운다."""
    p = (255 - img.astype(float)) / 255
    rgb = np.empty(img.shape + (3,), np.uint8)
    rgb[:] = GUI_COLORS['unknown']
    rgb[p > occupied_thresh] = GUI_COLORS['occ']
    rgb[p < free_thresh] = GUI_COLORS['free']
    r, c = np.nonzero(img == ROAD)
    if len(r):
        rgb = rgb[max(r.min() - 15, 0):r.max() + 16, max(c.min() - 15, 0):c.max() + 16]
    return np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h) + np.ascontiguousarray(img, np.uint8).tobytes())


def main(argv=None):
    from pinky_fms_lane.course import Course
    from pinky_fms_lane.draw_course import default_course_path, default_out_dir
    from pinky_fms_traffic.gridmap import read_pgm      # 팀원 코드 재사용 (수정 없음)
    ap = argparse.ArgumentParser(description='GUI 표시용 차선 지도를 만든다 (AMCL·Nav2·시뮬레이션에는 쓰지 않는다)')
    ap.add_argument('--map', required=True, help='바탕 지도 yaml (벽·상자를 가져오고 원점·크기를 맞춘다)')
    ap.add_argument('--course', default=default_course_path(), help='코스 yaml (기본: 설치된 기본 코스)')
    ap.add_argument('--out', default=default_out_dir(), help='저장 폴더 (기본: FMS_OUTPUT_DIR, 없으면 $FMS_WS/outputs)')
    ap.add_argument('--crosswalk-len', type=float, default=0.12, help='횡단보도 길이 (m, 줄자 10·11번: 12 cm)')
    args = ap.parse_args(argv)
    if not args.course or not os.path.exists(args.course):
        ap.error(f'코스 파일이 없습니다: {args.course!r} (--course 로 지정)')
    if not args.out:
        ap.error('저장 폴더를 모릅니다. --out 을 주거나 FMS_OUTPUT_DIR 또는 FMS_WS 를 설정하세요')
    course = Course.load(args.course)
    with open(args.map, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    base = read_pgm(os.path.join(os.path.dirname(os.path.abspath(args.map)), meta['image']))
    p = base / 255.0 if meta.get('negate', 0) else (255 - base) / 255.0
    img = lane_image(course, p > meta['occupied_thresh'], meta['resolution'], meta['origin'][:2], args.crosswalk_len)

    stem = os.path.splitext(os.path.basename(args.map))[0] + '_lanes_gui'
    os.makedirs(args.out, exist_ok=True)
    write_pgm(os.path.join(args.out, stem + '.pgm'), img)
    out_meta = dict(meta, image=stem + '.pgm', negate=0)
    with open(os.path.join(args.out, stem + '.yaml'), 'w', encoding='utf-8') as f:
        f.write('# GUI 표시 전용 (pinky_fms_lane make_lane_map). AMCL, Nav2, 시뮬레이션에는 쓰지 말 것\n')
        f.write(f'# 바탕 지도: {os.path.basename(args.map)}, 코스: {os.path.basename(args.course)}\n')
        yaml.safe_dump(out_meta, f, sort_keys=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    preview = os.path.join(args.out, stem + '_preview.png')
    plt.imsave(preview, gui_preview(img, meta['occupied_thresh'], meta['free_thresh']))
    src = ', '.join(f'{k} {len(v)}개' for k, v in course.source_summary().items() if v)
    print(f'[make_lane_map] 저장: {os.path.join(args.out, stem)}.yaml / .pgm, 미리보기 {preview}')
    print(f'[make_lane_map] 길 폭 {course.params["lane_width"]:.3f} m ({course.param_sources["lane_width"]}), 점 {src}')
    print('[make_lane_map] GUI 표시 전용 지도입니다. AMCL, Nav2, 시뮬레이션에는 쓰지 마세요')
    return 0


if __name__ == '__main__':
    sys.exit(main())
