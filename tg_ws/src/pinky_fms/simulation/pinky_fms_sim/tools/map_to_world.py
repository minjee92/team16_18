#!/usr/bin/env python3
"""점유 지도(yaml+pgm) → Gazebo 월드(SDF). 벽 칸을 상자로 세워서, 실물 코스와 같은 지형에서 시뮬레이션한다.
  python3 simulation/pinky_fms_sim/tools/map_to_world.py <map.yaml> <out.sdf> [벽 높이 m=0.22] [--block x0 y0 x1 y1 ...]
--block : 지도에는 비어 있지만 실제로는 막혀 있는 구역(m)을 벽으로 채운다 (여러 번 가능)
좌표계: 지도의 (x, y) = 월드의 (x, y). 그래서 같은 map.yaml 로 AMCL 을 돌리면 초기 위치만 맞추면 된다.
"""
import os
import sys

import numpy as np
import yaml


def read_pgm(path):
    raw = open(path, 'rb').read()
    toks, i = [], 0
    while len(toks) < 4:
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        toks.append(raw[i:j])
        i = j
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(raw[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


def boxes_from_grid(occ):
    """점유 칸을 가로 줄(run) 단위로 묶고, 같은 폭의 줄이 세로로 이어지면 하나로 합친다 (상자 수를 줄인다)."""
    H, W = occ.shape
    runs = {}                           # (c0, c1) -> [r_start, r_end]
    boxes = []
    for r in range(H):
        row = occ[r]
        cur = {}
        c = 0
        while c < W:
            if row[c]:
                c0 = c
                while c < W and row[c]:
                    c += 1
                key = (c0, c)
                if key in runs and runs[key][1] == r - 1:
                    runs[key][1] = r
                    cur[key] = runs[key]
                else:
                    cur[key] = [r, r]
            else:
                c += 1
        for key, rr in runs.items():
            if key not in cur:
                boxes.append((key[0], key[1], rr[0], rr[1] + 1))
        runs = cur
    for key, rr in runs.items():
        boxes.append((key[0], key[1], rr[0], rr[1] + 1))
    return boxes


def main():
    args = [a for a in sys.argv[1:]]
    blocks = []
    while '--block' in args:
        i = args.index('--block')
        blocks.append(tuple(float(v) for v in args[i + 1:i + 5]))
        del args[i:i + 5]
    yml, out = args[0], args[1]
    height = float(args[2]) if len(args) > 2 else 0.22
    meta = yaml.safe_load(open(yml))
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(yml)), meta['image']))[::-1]   # 행 0 = y 최소
    p = img / 255.0 if meta.get('negate', 0) else (255 - img) / 255.0
    occ = p > meta['occupied_thresh']
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    boxes = boxes_from_grid(occ)
    models = []
    for k, (x0, y0, x1, y1) in enumerate(blocks):        # 실제로 막혀 있는 구역: 상자 하나로 채운다
        models.append(f'''    <model name="block_{k}"><static>true</static><pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {height / 2:.3f} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>{abs(x1 - x0):.4f} {abs(y1 - y0):.4f} {height:.3f}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{abs(x1 - x0):.4f} {abs(y1 - y0):.4f} {height:.3f}</size></box></geometry>
      <material><diffuse>0.6 0.5 0.4 1</diffuse></material></visual></link></model>''')
    for k, (c0, c1, r0, r1) in enumerate(boxes):
        sx, sy = (c1 - c0) * res, (r1 - r0) * res
        cx, cy = ox + (c0 + c1) / 2 * res, oy + (r0 + r1) / 2 * res
        models.append(f'''    <model name="wall_{k}"><static>true</static><pose>{cx:.4f} {cy:.4f} {height / 2:.3f} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {height:.3f}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {height:.3f}</size></box></geometry>
      <material><diffuse>0.75 0.75 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular></material></visual></link></model>''')
    sdf = f'''<?xml version="1.0" ?>
<!-- {os.path.basename(yml)} 에서 map_to_world.py 로 자동 생성. 벽 상자 {len(boxes)}개 -->
<sdf version="1.8">
  <world name="fms_course">
    <physics name="4ms" type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>   <!-- 4 ms: /clock 250 Hz (1 ms 면 rclpy 노드들이 clock 처리에 CPU 를 다 쓴다) -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 5 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.3 0.2 -1</direction></light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
      <material><diffuse>0.92 0.92 0.92 1</diffuse></material></visual></link></model>
{chr(10).join(models)}
  </world>
</sdf>
'''
    open(out, 'w').write(sdf)
    print(f'{out}: 벽 상자 {len(boxes)}개, 막은 구역 {len(blocks)}개, 지도 {occ.shape[1] * res:.2f} x {occ.shape[0] * res:.2f} m, 원점 ({ox}, {oy})')


if __name__ == '__main__':
    main()
