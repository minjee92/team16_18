"""줄자 실측 파일(*.tape.yaml) → 코스 점 좌표 (순수 Python, ROS 를 쓰지 않는다).

줄자 파일에는 원본 측정값(cm)과 계산에 쓴 벽 안쪽 면 위치(map 좌표, m)가 있다.
좌표는 읽을 때마다 원본에서 다시 계산한다. 그래서 지도가 바뀌면 walls 의 value 만 고치면 된다.

  walls  : 벽 안쪽 면. probe(지도에서 다시 잴 때 시작점과 방향)의 방향이 벽 쪽이다.
           up/down 이면 y 면, left/right 이면 x 면. 길은 probe 반대쪽(안쪽)에 있다.
  roads  : 길 하나를 잰 값 목록. 벽에서 길 양쪽 테이프 안쪽까지 a, b (cm).
           길 가운데 = 벽 면 + 안쪽으로 (a+b)/2, 길 폭 = b − a.
           두 곳 이상 쟀으면(at = 잰 곳의 다른 축 좌표, m) 직선으로 맞춰 기울기까지 쓴다.
  marks  : 벽에서 표시(횡단보도) 끝까지 dist, 표시 길이 length (cm). 가운데 = dist + length/2.
  points : x, y 마다 walls/roads/marks 이름(목록이면 평균) 또는 숫자(추정값).
           숫자가 하나라도 있으면 source 는 estimate, 모두 이름이면 tape.

ros2 run pinky_fms_lane course_tape [--course <코스.yaml>] [--map <지도.yaml>]
  계산한 좌표와 길 폭을 보여 준다. --map 을 주면 그 지도에서 벽 면을 다시 재서,
  바꿔야 할 walls 값과 그때 바뀌는 좌표를 보여 준다 (파일은 고치지 않는다. 사람이 쓰는 파일이라 주석을 지키기 위해).
"""
import argparse
import os
import sys

import numpy as np
import yaml

DIRS = {'up': ('y', 1), 'down': ('y', -1), 'right': ('x', 1), 'left': ('x', -1)}


class TapeError(ValueError):
    def __init__(self, problems):
        super().__init__('줄자 파일 오류:\n  - ' + '\n  - '.join(problems))
        self.problems = list(problems)


class Tape:
    def __init__(self, data):
        data = data or {}
        problems = []
        self.walls = {}                       # 이름 → (축, 면 위치, 안쪽 부호)
        self.probes = {}
        for name, v in (data.get('walls') or {}).items():
            probe = (v or {}).get('probe') or {}
            if probe.get('dir') not in DIRS or 'value' not in (v or {}):
                problems.append(f'walls.{name}: value 와 probe.dir(up/down/left/right) 가 필요함')
                continue
            axis, sign = DIRS[probe['dir']]
            self.walls[name] = (axis, float(v['value']), -sign)
            self.probes[name] = probe
        self.roads = {}                       # 이름 → (축, 기울기, 절편, [(번호, at, 가운데, 폭)])
        for name, samples in (data.get('roads') or {}).items():
            self._add_road(name, samples or [], problems)
        self.marks = {}
        for name, v in (data.get('marks') or {}).items():
            w = self.walls.get((v or {}).get('wall'))
            if w is None or 'dist' not in v or 'length' not in v:
                problems.append(f'marks.{name}: walls 에 있는 wall, dist, length 가 필요함')
                continue
            axis, face, sign = w
            self.marks[name] = (axis, face + sign * (float(v['dist']) + float(v['length']) / 2) / 100.0)
        self.params = {k: float(v) for k, v in (data.get('params') or {}).items()}
        self.points = {}
        for name, v in (data.get('points') or {}).items():
            try:
                self.points[name] = self._point(v or {})
            except ValueError as e:
                problems.append(f'points.{name}: {e}')
        if problems:
            raise TapeError(problems)

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            return cls(yaml.safe_load(f))

    def _add_road(self, name, samples, problems):
        rows, axes = [], set()
        for m in samples:
            w = self.walls.get(m.get('wall'))
            if w is None or 'a' not in m or 'b' not in m:
                problems.append(f'roads.{name}: 측정 {m.get("num", "?")} 에 walls 에 있는 wall, a, b 가 필요함')
                return
            axis, face, sign = w
            a, b = float(m['a']), float(m['b'])
            if b <= a:
                problems.append(f'roads.{name}: 측정 {m.get("num", "?")} 는 b > a 여야 함 (a = 가까운 테이프)')
                return
            axes.add(axis)
            rows.append((m.get('num', ''), m.get('at'), face + sign * (a + b) / 200.0, (b - a) / 100.0))
        if not rows or len(axes) != 1:
            problems.append(f'roads.{name}: 측정이 1개 이상, 모두 같은 축의 벽 기준이어야 함')
            return
        if len(rows) == 1:
            slope, icpt = 0.0, rows[0][2]
        elif any(r[1] is None for r in rows):
            problems.append(f'roads.{name}: 두 곳 이상 쟀으면 각 측정에 at(잰 곳의 다른 축 좌표)이 필요함')
            return
        else:
            slope, icpt = np.polyfit([float(r[1]) for r in rows], [r[2] for r in rows], 1)
        self.roads[name] = (axes.pop(), float(slope), float(icpt), rows)

    def _value(self, spec, axis, other):
        """좌표 하나: 숫자(추정) 또는 이름(목록이면 평균) → (값, 줄자인가)."""
        if isinstance(spec, (int, float)):
            return float(spec), False
        names = spec if isinstance(spec, list) else [spec]
        vals = []
        for n in names:
            if n in self.roads:
                ax, slope, icpt, _ = self.roads[n]
                if slope and other is None:
                    raise ValueError(f'기울어진 길 {n} 을 쓰려면 다른 축 좌표가 숫자이거나 기울지 않은 길이어야 함')
                v = icpt + slope * (other or 0.0)
            elif n in self.walls:
                ax, v, _ = self.walls[n]
            elif n in self.marks:
                ax, v = self.marks[n]
            else:
                raise ValueError(f'{n} 은 walls/roads/marks 에 없음')
            if ax != axis:
                raise ValueError(f'{n} 은 {ax} 좌표라 {axis} 에 쓸 수 없음')
            vals.append(v)
        if not vals:
            raise ValueError(f'{axis} 가 비어 있음')
        return float(np.mean(vals)), True

    def _sloped(self, spec):
        names = spec if isinstance(spec, list) else [spec]
        return any(n in self.roads and self.roads[n][1] for n in names if isinstance(n, str))

    def _point(self, v):
        if 'x' not in v or 'y' not in v:
            raise ValueError('x, y 가 필요함')
        if self._sloped(v['x']) and self._sloped(v['y']):
            raise ValueError('x, y 가 모두 기울어진 길이면 계산할 수 없음')
        if self._sloped(v['x']):                       # 기울어진 쪽은 다른 축 값을 먼저 구해 넣는다
            y, ty = self._value(v['y'], 'y', None)
            x, tx = self._value(v['x'], 'x', y)
        else:
            x, tx = self._value(v['x'], 'x', None)
            y, ty = self._value(v['y'], 'y', x)
        return {'x': round(x, 4), 'y': round(y, 4), 'source': 'tape' if tx and ty else 'estimate'}


def load_raw(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def measure_wall(occ, res, origin, probe, step=0.05, span=None):
    """지도(occ: 행 0 = y 최소)에서 probe 시작점부터 dir 방향으로 가며 처음 만나는 벽 칸의 가까운 면 (m).
    probe 와 수직으로 ±span 범위 여러 줄의 가운값. 못 찾으면 None."""
    axis, sign = DIRS[probe['dir']]
    span = float(probe.get('span', 0.1) if span is None else span)
    ox, oy = origin
    found = []
    for off in np.arange(-span, span + 1e-9, step):
        x, y = float(probe['x']), float(probe['y'])
        if axis == 'y':
            x += off
        else:
            y += off
        r, c = int((y - oy) / res), int((x - ox) / res)
        while 0 <= r < occ.shape[0] and 0 <= c < occ.shape[1]:
            if occ[r, c]:
                if axis == 'y':
                    found.append(oy + (r + (1 if sign < 0 else 0)) * res)
                else:
                    found.append(ox + (c + (1 if sign < 0 else 0)) * res)
                break
            if axis == 'y':
                r += sign
            else:
                c += sign
    return float(np.median(found)) if found else None


def read_occupancy(map_yaml):
    from pinky_fms_traffic.gridmap import read_pgm      # 팀원 코드 재사용 (수정 없음)
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta['image']))[::-1]
    p = img / 255.0 if meta.get('negate', 0) else (255 - img) / 255.0
    return p > meta['occupied_thresh'], meta['resolution'], tuple(meta['origin'][:2])


def tape_path(course_path):
    with open(course_path, encoding='utf-8') as f:
        name = (yaml.safe_load(f) or {}).get('tape')
    return os.path.join(os.path.dirname(os.path.abspath(course_path)), name) if name else ''


def main(argv=None):
    from pinky_fms_lane.draw_course import default_course_path
    ap = argparse.ArgumentParser(description='줄자 실측 → 코스 좌표 계산 결과를 보여 준다')
    ap.add_argument('--course', default=default_course_path(), help='코스 yaml (기본: 설치된 기본 코스)')
    ap.add_argument('--map', default='', help='이 지도에서 벽 면을 다시 재서 비교한다')
    args = ap.parse_args(argv)
    if not args.course or not os.path.exists(args.course):
        ap.error(f'코스 파일이 없습니다: {args.course!r} (--course 로 지정)')
    path = tape_path(args.course)
    if not path or not os.path.exists(path):
        ap.error(f'코스 파일에 줄자 파일(tape:)이 없거나 파일이 없습니다: {path!r}')
    raw = load_raw(path)
    tape = Tape(raw)
    print(f'[course_tape] 줄자 파일: {path}')
    print('  길 가운데·폭 (원본 측정 → 계산)')
    for name, (axis, slope, icpt, rows) in tape.roads.items():
        for no, at, center, width in rows:
            where = '' if at is None else f' ({"y" if axis == "x" else "x"}={float(at):.2f})'
            print(f'    {name:12s} {no:>3}번{where:12s} 가운데 {axis}={center:+.4f}  폭 {100 * width:.1f} cm')
        if slope:
            print(f'    {name:12s} 기울기 {slope:+.4f} (다른 축 1 m 당)')
    print('  점 (source: tape = 모두 줄자, estimate = 일부 추정)')
    for name, p in tape.points.items():
        print(f'    {name:6s} x={p["x"]:+.4f} y={p["y"]:+.4f}  {p["source"]}')
    if not args.map:
        return 0
    occ, res, origin = read_occupancy(args.map)
    new = {}
    print(f'  지도 {args.map} 에서 벽 면 다시 재기')
    for name, (axis, face, _) in tape.walls.items():
        v = measure_wall(occ, res, origin, tape.probes[name])
        if v is None:
            print(f'    {name:13s} 못 찾음 (probe 확인)')
            continue
        new[name] = round(v, 4)
        print(f'    {name:13s} 파일 {face:+.4f}  지도 {v:+.4f}  차이 {100 * (v - face):+.1f} cm')
    changed = {n: v for n, v in new.items() if abs(v - tape.walls[n][1]) > 1e-4}
    if not changed:
        print('  벽 면이 파일과 같다. 고칠 것 없음')
        return 0
    raw2 = dict(raw)
    raw2['walls'] = {n: dict(w, value=new.get(n, w['value'])) for n, w in raw['walls'].items()}
    tape2 = Tape(raw2)
    print('  walls 를 아래처럼 고치면 좌표가 이렇게 바뀐다 (파일은 직접 고친다)')
    for n, v in changed.items():
        print(f'    walls.{n}.value: {v}')
    for name, p in tape2.points.items():
        q = tape.points[name]
        d = np.hypot(p['x'] - q['x'], p['y'] - q['y'])
        if d > 1e-4:
            print(f'    {name:6s} ({q["x"]:+.4f}, {q["y"]:+.4f}) → ({p["x"]:+.4f}, {p["y"]:+.4f})  {100 * d:.1f} cm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
