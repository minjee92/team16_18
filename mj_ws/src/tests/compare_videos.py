"""
두 영상을 디코딩해 프레임 단위로 비교한다 (리팩터링 전후 결과 영상이 같은지 확인용).

    python src/tests/compare_videos.py before.mp4 after.mp4

출력: 프레임 수, 다른 프레임 수, 최대 픽셀 차, 다른 프레임 구간. 완전히 같으면 exit 0, 다르면 exit 1.
mp4v 는 앞 프레임을 참조해 압축하므로, 한 프레임이 바뀌면 뒤 몇 프레임에 작은 차이(코덱 전파)가 따라올 수 있다.
"""

import hashlib
import sys

import cv2
import numpy as np


def main():
    a_path, b_path = sys.argv[1], sys.argv[2]
    for path in (a_path, b_path):
        print(path, hashlib.sha256(open(path, 'rb').read()).hexdigest()[:16])

    a, b = cv2.VideoCapture(a_path), cv2.VideoCapture(b_path)
    n, diffs, max_diff = 0, [], 0
    while True:
        ok_a, frame_a = a.read()
        ok_b, frame_b = b.read()
        if not ok_a or not ok_b:
            if ok_a != ok_b:
                print(f'프레임 수 다름: 한쪽이 {n}에서 끝남')
                diffs.append(n)
            break
        d = int(np.abs(frame_a.astype(np.int16) - frame_b.astype(np.int16)).max())
        if d:
            diffs.append(n)
            max_diff = max(max_diff, d)
        n += 1

    ranges = []
    for i in diffs:
        if ranges and i == ranges[-1][1] + 1:
            ranges[-1][1] = i
        else:
            ranges.append([i, i])
    print(f'비교 프레임 {n}, 다른 프레임 {len(diffs)}, 최대 픽셀 차 {max_diff}')
    if ranges:
        print('다른 구간:', ', '.join(f'{s}' if s == e else f'{s}-{e}' for s, e in ranges[:20]),
              '...' if len(ranges) > 20 else '')
    sys.exit(0 if not diffs else 1)


if __name__ == '__main__':
    main()
