"""
화면 없이 스크립트 실행: cv2 GUI 호출(imshow / waitKey / destroyAllWindows)만 무력화하고 __main__ 으로 실행한다.
VideoWriter 등은 그대로 동작하므로 결과 영상 비교용으로 쓴다.

    python src/tests/headless_run.py src/tools/step1_check_lane_detection.py --max-frames 150 --output /tmp/a.mp4
"""

import os
import runpy
import sys

import cv2

cv2.imshow = lambda *a, **k: None
cv2.waitKey = lambda *a, **k: -1
cv2.destroyAllWindows = lambda *a, **k: None

script = sys.argv[1]
sys.argv = sys.argv[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(script)))   # python3 script.py 와 같게
runpy.run_path(script, run_name='__main__')
