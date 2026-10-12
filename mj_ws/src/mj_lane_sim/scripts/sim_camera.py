#!/usr/bin/env python3
# 원본: tg_ws/src/pinky_fms/simulation/pinky_fms_sim/scripts/sim_camera.py @ fb2f983 (origin/feat/tg-gazebo-course, GPT 작성 pinky_fms_sim). mj_lane_sim 으로 가져와 수정함
"""
생성된 URDF 의 시뮬 카메라를 실물에 맞춘다 (원본 모델 파일은 고치지 않음). stdin → stdout.

  width height   : 해상도 (원본 기능)
  --hfov-deg     : 수평 화각 (도). 실물 OV5647 = 53.5. 0 이하면 모델 값 그대로 (원본 1.1519 rad = 66°)
  --lens-height  : 바닥에서 렌즈(front_camera_link)까지 m. 음수면 모델 그대로 (약 0.063 m).
                   base_footprint(바닥)에서 카메라 링크까지 조인트를 따라 높이를 계산하고,
                   차이만큼 카메라 마운트 조인트(front_camera_mount)의 z 를 옮긴다 (기울기·앞뒤 위치는 그대로).
결과는 stderr 에 한 줄로 알린다.
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

CAMERA_LINK = 'front_camera_link'
MOUNT_LINK = 'front_camera_mount'


def _origin(joint):
    o = joint.find('origin')
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    return o, xyz, rpy


def _transform(xyz, rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = xyz
    return t


def lens_height(root):
    """base_footprint 에서 카메라 링크까지 조인트를 따라 올라가 렌즈 높이(z, m)를 구한다."""
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain, link = [], CAMERA_LINK
    while link in by_child:
        joint = by_child[link]
        chain.append(joint)
        link = joint.find('parent').get('link')
    t = np.eye(4)
    for joint in reversed(chain):
        _, xyz, rpy = _origin(joint)
        t = t @ _transform(xyz, rpy)
    return float(t[2, 3]), link, by_child


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('width', type=int)
    parser.add_argument('height', type=int)
    parser.add_argument('--hfov-deg', type=float, default=0.0)
    parser.add_argument('--lens-height', type=float, default=-1.0)
    args = parser.parse_args()
    if not (160 <= args.width <= 3840 and 120 <= args.height <= 2160):
        parser.error('Camera resolution outside supported limits')
    root = ET.parse(sys.stdin).getroot()
    cameras = root.findall(".//sensor[@type='camera']/camera")
    if len(cameras) != 1:
        raise ValueError(f'Expected one camera, found {len(cameras)}')
    camera = cameras[0]
    camera.find('image/width').text = str(args.width)
    camera.find('image/height').text = str(args.height)
    notes = [f'{args.width}x{args.height}']

    if args.hfov_deg > 0:
        camera.find('horizontal_fov').text = f'{math.radians(args.hfov_deg):.6f}'
        vfov = math.degrees(2 * math.atan(math.tan(math.radians(args.hfov_deg) / 2) * args.height / args.width))
        notes.append(f'화각 {args.hfov_deg:.1f}° (수직 {vfov:.1f}°)')

    before, base, by_child = lens_height(root)
    if args.lens_height >= 0:
        mount = by_child.get(MOUNT_LINK)
        if mount is None:
            raise ValueError(f'No joint with child {MOUNT_LINK}')
        o, xyz, rpy = _origin(mount)
        xyz[2] += args.lens_height - before
        o.set('xyz', ' '.join(f'{v:.6g}' for v in xyz))
        after, _, _ = lens_height(root)
        notes.append(f'렌즈 높이 {before:.4f} → {after:.4f} m ({base} 기준)')
    else:
        notes.append(f'렌즈 높이 {before:.4f} m (모델 그대로, {base} 기준)')
    print('[sim_camera] ' + ', '.join(notes), file=sys.stderr)
    print(ET.tostring(root, encoding='unicode'))


if __name__ == '__main__':
    main()
