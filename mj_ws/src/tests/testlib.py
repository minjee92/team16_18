"""
테스트 공용: 경로 설정과 결과 집계 (pytest 없이 python 으로 바로 실행)

각 테스트 파일은 `import testlib` 만 하면 src/, src/robot/, src/tools/ 를 import 할 수 있다.
"""

import sys
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
SRC_DIR = TESTS_DIR.parent
MJ_DIR = SRC_DIR.parent

for path in (SRC_DIR, SRC_DIR / 'robot', SRC_DIR / 'tools'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

VIDEO = MJ_DIR / 'inputs' / 'pinky_20260919_182009.mp4'
NCNN_MODEL = MJ_DIR / 'models' / 'lane_model_ncnn' / 'best_ncnn_model'


class Checker:
    """check() 로 항목을 모으고 finish() 에서 요약 후 실패가 있으면 exit 1."""

    def __init__(self, name):
        self.name = name
        self.results = []

    def check(self, label, condition, detail=''):
        ok = bool(condition)
        self.results.append(ok)
        print(f'{"PASS" if ok else "FAIL"}  {label}{f"  ({detail})" if detail else ""}')
        return ok

    def finish(self):
        passed = sum(self.results)
        print(f'\n[{self.name}] {passed}/{len(self.results)} passed')
        sys.exit(0 if passed == len(self.results) else 1)


def require(path, what):
    """테스트에 필요한 파일이 없으면 이유를 알리고 실패로 끝낸다 (models/, inputs/ 는 git 제외)."""
    if not Path(path).exists():
        print(f'FAIL  {what} 없음: {path}')
        sys.exit(1)
