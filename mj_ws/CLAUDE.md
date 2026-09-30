# mj_ws — 차선 세그멘테이션 프로젝트

자율주행 학습 MVP. 실내 트랙(회색 카펫 + 흰색 테이프)에서 촬영한 영상으로
차선 세그멘테이션 모델을 학습한다.

## 폴더 구조

| 경로 | 내용 | git |
|---|---|---|
| `src/common/` | 모델(.pt / ncnn)과 무관한 공통 모듈. `lane_postprocess.py`: 마스크 → 차선 중앙점·조향 계산 | 추적 |
| `src/tools/` | 개발용 (PC). `step1_check_lane_detection.py`: 저장 영상으로 차선 인식·조향 검증 | 추적 |
| `src/robot/` | 로봇에서 실행. `lane_mission_drive.py`(ncnn 주행), `model_ncnn.py`(ncnn 로드 테스트) | 추적 |
| `src/station/` | 관제 PC에서 실행. `mission_gui.py`(맵에서 목표 지점 지정) | 추적 |
| `src/legacy/` | 더 이상 쓰지 않는 스크립트 (`1_realtime.py`, `2_realtime.py`, `capture.py`, `find_target_point.py`, `find_target_point_dual.py`, `test_seg.py`) | 추적 |
| `maps/` | SLAM 맵 (`mission4_3.pgm`, `mission4_3.yaml`) | 추적 |
| `models/` | 모델 가중치와 `metadata.yaml`만 (코드 두지 말 것) | 폴더만 추적 (`.gitkeep`), 내용물 제외 |
| `inputs/` | 입력 영상 `pinky_*.mp4` | 폴더만 추적 (`.gitkeep`), 내용물 제외 |
| `outputs/` | 추론 결과 영상 `result_<스크립트명>.mp4` | 폴더만 추적 (`.gitkeep`), 내용물 제외 |
| `tmp/` | 쓰지 않는 파일 (초기 학습용 캡처 jpg, 중복 영상), `capture.py`가 저장하는 곳 | 통째로 제외 |
| `tmp/legacy_models/` | legacy 가중치 (`best.pt`, `lane_best.pt`, `260921_best.pt`, `old_pt/`) | 제외 |
| `dataset` | 데이터셋 심볼릭 링크 (아래 참고) | 제외 |

- 스크립트는 모두 `src/<역할>/` 아래에 있고, `BASE_DIR = Path(__file__).resolve().parent.parent.parent`
  (= `mj_ws/`)를 기준으로 경로를 잡으므로 어느 디렉터리에서 실행해도 된다.
- `common` 모듈을 쓰는 스크립트는 상단에서 `src/`를 `sys.path`에 넣은 뒤 import한다
  (`src/tools/step1_check_lane_detection.py` 참고). 실행은 `python3 src/tools/step1_check_lane_detection.py`처럼 경로로 한다.
- 입력 영상은 `inputs/`에서 읽고, 결과물은 `outputs/`에 저장한다.
- 모델·영상은 git에 올라가지 않으므로 팀원은 각자 `models/`, `inputs/`에 넣어야 한다.

## 모델

| 경로 | 용도 | 쓰는 스크립트 |
|---|---|---|
| `models/lane_model_ncnn/` | 로봇 실주행용 (NCNN, 320×320) | `robot/lane_mission_drive.py`, `robot/model_ncnn.py` |
| `models/260928_yolon_best.pt` | 저장 영상 검증용 (PC) | `tools/step1_check_lane_detection.py` (legacy의 `2_realtime.py`, `find_target_point.py`, `find_target_point_dual.py`, `test_seg.py`도 이 모델을 가리킴) |
| `tmp/legacy_models/*` | legacy (사용 안 함) | `legacy/1_realtime.py` (`old_pt/pouch_best.pt`) |

- ncnn 모델은 `260928_yolon_best.pt`에서 export했다
  (`models/lane_model_ncnn/best.pt`와 sha256 동일 확인).
  그래서 검증용 pt와 주행용 ncnn은 같은 가중치다.
- ultralytics NCNN은 폴더 경로를 받으므로 `MODEL_PATH`는 `models/lane_model_ncnn`을 가리킨다.
- 모델 경로는 각 스크립트 상단의 `MODEL_PATH` 상수 한 줄로 정해진다. 모델을 바꿀 때는 그 줄만 고친다.

## 출력 파일명 규칙

`outputs/result_<스크립트명>.mp4` (예: `step1_check_lane_detection.py` → `outputs/result_step1_check_lane_detection.mp4`)

## 하드웨어

### 로봇: Pinky

- 크기: 110(W) / 120(D) / 142(H) mm
- SBC: 라즈베리파이 5 (8GB)
- 구동: 다이나믹셀 XL330-M288-T × 2
  (무부하 103rpm @5V, 정지토크 0.52N·m, 감속비 288.4:1, Velocity Control Mode 지원)
- 바퀴 지름 50mm (±5mm), 바퀴 간격(트레드) 85mm
- 최고 속도 약 0.27 m/s (무부하 계산값), 권장 주행 0.10~0.15 m/s
- 센서: RPLiDAR C1, BNO055 9축 IMU, 초음파 US-016, IR TCRT5000

### 카메라

- Raspberry Pi Camera Module v1 (OV5647), 보드 Rev 1.3
- 수평 화각 53.5° / 수직 화각 41.4° (표준 사양, 촬영 모드에 따라 달라질 수 있음)
- 바닥에서 렌즈까지 높이 60mm
- 로봇 중심선 위에 장착
- 틸트각: 미측정

### 차동구동 변환식

```python
# 차동구동 변환 (Pinky)
# 부호 규칙: omega > 0 = 반시계 방향 = 좌회전 (ROS REP-103 관례)
import math

WHEEL_D = 0.05      # 바퀴 지름 [m]
TREAD   = 0.085     # 바퀴 간격 [m]
MAX_RPM = 103       # XL330-M288-T 무부하 최고 회전수 (5V)

def to_wheel_rpm(v, omega):
    """v [m/s], omega [rad/s] -> (좌, 우) 바퀴 rpm"""
    v_left  = v - omega * TREAD / 2
    v_right = v + omega * TREAD / 2
    k = 60.0 / (math.pi * WHEEL_D)   # m/s -> rpm
    return v_left * k, v_right * k
```

- 위 rpm을 다이나믹셀 Goal Velocity 레지스터 값으로 바꾸는 단위 변환은 별도다. XL330의 단위를 e-manual이나 사용 중인 라이브러리에서 확인해서 쓸 것
- 부호 규칙(어느 방향이 +인지)은 실제 로봇에서 반드시 검증할 것. 반대면 로봇이 거꾸로 꺾는다

### 실측 필요

- **카메라 거리 캘리브레이션** — 바닥 30/60/90/120cm 지점에 표시하고 촬영해, 거리↔이미지 y좌표
  대응표를 만들 것. 틸트각을 따로 재는 대신 이걸로 역산한다.
  (차선을 읽는 샘플 행 y=432/384/336/288(640×480 기준)이 바닥 몇 cm인지도 이 표로 확인한다)
- **로봇 중심선의 이미지상 x좌표** — 현재 코드는 화면 중앙(x=320)을 로봇 중심선으로 가정한다
  (`src/common/lane_postprocess.py`의 `error` 계산)

## 데이터셋

`mj_ws/dataset` → `/home/mindy/dev_ws/datasets/lane_seg.v5i.coco-segmentation`
(심볼릭 링크, git 제외. 팀원은 자기 환경에 맞게
`ln -s <데이터셋 경로> mj_ws/dataset`으로 만든다)

`lane_seg.v5i.coco-segmentation/` — Roboflow COCO Segmentation 포맷

- 842장 (train 589 / valid 160 / test 93), 전부 640×640
- 어노테이션 1,405개
- 전처리: Auto-orient + Resize 640×640 (Fit, black edges) → 위아래 검은 띠 있음
- 증강: 밝기 + 상하 이동 (좌우반전 증강은 **없음**)

### 클래스

| id | 이름 | 설명 |
|---|---|---|
| 0 | lane-seg | 더미 루트 (사용 안 함) |
| 2 | cross_lane | 교차로에서 가로지르는 차선 |
| 3 | crosswalk | 횡단보도 |
| 4 | left_lane | 로봇 기준 왼쪽 차선 |
| 5 | right_lane | 로봇 기준 오른쪽 차선 |

**left/right 판별 기준은 화면 위치가 아니라 프레임 하단에서 로봇 기준 좌·우다.**
선이 화면 중앙~오른쪽에 보여도, 로봇이 그 선의 오른쪽을 주행하면 `left_lane`이다.

`categories` 목록의 id 1(이름 `"2"`)은 Roboflow가 만든 오류 클래스이며
어노테이션은 모두 제거했다. **목록 자체는 지우지 말 것** — id 매핑이 깨진다.

## 중요: 좌우반전 이력

원본 데이터는 좌우반전된 영상으로 라벨링되어 있었다. 2026-09 기준으로
이미지·좌표·클래스를 모두 반전 처리해 **정방향으로 교정 완료**했다.

- 이미지: `cv2.flip(img, 1)`
- 폴리곤: `x' = W - x`, 점 순서 역순 (감김 방향 보존)
- bbox: `x' = W - (x + w)`, y·w·h·area 불변
- 클래스: annotation의 `category_id` 4↔5 교체 (categories 목록은 원본 유지)

**`flip_dataset.py` / `flip_images.py`를 다시 실행하지 말 것.** 한 번 더 뒤집혀
원상복구된다. 각 split 폴더의 `.images_flipped` 표식이 처리 완료를 뜻한다.
상태 확인은 `python3 flip_images.py . --check`.

json이 반전된 상태인지는 `info.description`에 `horizontally flipped`
문자열이 있는지로 판별한다.

## 알려진 이슈 (미해결)

**split 누수** — 같은 원본 프레임에서 증강된 이미지들이 train/valid/test에
흩어져 있다. 65개 프레임이 2개 이상 split에 걸쳐 있다. 이대로 학습하면
valid/test 점수가 실제 성능보다 높게 나온다. 재분할이 필요하다.

## 스크립트

- `flip_dataset.py` — 원본 → 반전본 전체 변환 (이미지 + json + 클래스)
- `flip_images.py` — 이미지만 반전, json 상태 자동 판별
- `auto_label_lanes.py` — 흰색 테이프 자동 검출 → COCO 폴리곤 생성.
  기존 GT 대비 IoU 중앙값 0.87. `crosswalk` / `cross_lane`은 자동 배정하지
  않으므로 사람이 지정해야 한다.

## 작업 규칙

- 데이터셋 파일을 제자리에서 수정하기 전에 반드시 백업하거나 새 폴더로 출력할 것
- 어노테이션을 변경했으면 오버레이 이미지를 렌더링해 눈으로 검증할 것
  (파란색 left_lane이 왼쪽, 주황색 right_lane이 오른쪽에 붙어야 정상)
- 좌표 변환 후에는 왕복 복원 테스트로 오차가 0인지 확인할 것

### 동시 실행 금지

추론 작업을 동시에 두 개 이상 돌리지 말 것. i7-1165G7(물리 4코어)에서 ultralytics가
7스레드를 잡기 때문에, 두 개를 동시에 돌리면 프레임당 59ms → 2,600ms로 약 45배 느려짐.
반드시 순차로 실행할 것.

### 커밋 메시지 규칙

커밋 메시지의 요약(첫 줄)은 영어로, 본문(설명)은 한국어로 작성한다.
요약은 Conventional Commits 형식(`type(scope): summary`)을 따른다.

예시:

```
refactor(mj_ws): reorganize src into role-based folders

- 옛 실습 코드를 legacy/ 로 분리
- tools/ robot/ station/ common/ 으로 실행 위치별 재편
- lane_follow_check.py → tools/step1_check_lane_detection.py
```

### 파일 이름 규칙

`step1_` ~ `step4_` 접두어는 `PLAN.md`의 4단계 계획 결과물에만 붙인다.
