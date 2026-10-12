#!/usr/bin/env python3
"""
옛 실행 경로 유지용 (빌드 없이 실행). 실제 코드: src/mj_lane_drive/mj_lane_drive/lane_follow_node.py

    python3 src/robot/step2_lane_follow.py [--video ...] [--dry-run] [--record]
"""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent          # mj_ws/src/
for pkg in ('mj_lane_core', 'mj_lane_drive'):
    if str(SRC / pkg) not in sys.path:
        sys.path.insert(0, str(SRC / pkg))

from mj_lane_drive.lane_follow_node import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
