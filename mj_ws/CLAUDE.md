# mj_ws — 차선 세그멘테이션 프로젝트

자율주행 학습 MVP. 실내 트랙(회색 카펫 + 흰색 테이프)에서 촬영한 영상으로
차선 세그멘테이션 모델을 학습한다.

## 폴더 구조

| 경로 | 내용 | git |
|---|---|---|
| `src/mj_lane_core/` | **ROS 패키지.** 판단만 (ROS·하드웨어 없음). `lane_postprocess.py`: 마스크 → 차선 중앙점·조향, `drive_control.py`: 초음파 비상 정지·속도, `lane_draw.py`: 추적 결과 그리기 | 추적 |
| `src/mj_lane_drive/` | **ROS 패키지.** 시뮬·실물 공용 주행 노드 `lane_follow_node.py` (step2), 입력원 `sources.py` (영상 파일·Picamera2·ROS 카메라 토픽), 주행 기록 `drive_record.py` | 추적 |
| `src/mj_lane_sim/` | **ROS 패키지 (PC 전용).** Gazebo 월드·바닥 그림·지도·로봇 launch, 라이다로 초음파 흉내, `scripts/run_mj_sim.sh`. 출처는 `SOURCES.md` | 추적 |
| `src/tools/` | 개발용 (PC). `step1_check_lane_detection.py`: 저장 영상으로 차선 인식·조향 검증 | 추적 |
| `src/robot/` | `step2_lane_follow.py`: 옛 실행 경로 (빌드 없이 `mj_lane_drive` 노드를 실행), `lane_mission_drive.py`(4단계 기반 미션 주행, 나중에 정리) | 추적 |
| `src/station/` | 관제 PC에서 실행. `mission_gui.py`(맵에서 목표 지점 지정) | 추적 |
| `src/legacy/` | 더 이상 쓰지 않는 스크립트 (`1_realtime.py`, `2_realtime.py`, `capture.py`, `find_target_point.py`, `find_target_point_dual.py`, `test_seg.py`, `model_ncnn.py`) | 추적 |
| `maps/` | SLAM 맵 (`mission4_3.pgm`, `mission4_3.yaml`) | 추적 |
| `models/` | 모델 가중치와 `metadata.yaml`만 (코드 두지 말 것) | 폴더만 추적 (`.gitkeep`), 내용물 제외 |
| `inputs/` | 입력 영상 `pinky_*.mp4` | 폴더만 추적 (`.gitkeep`), 내용물 제외 |
| `outputs/` | 추론 결과 영상 `result_<스크립트명>.mp4` | 폴더만 추적 (`.gitkeep`), 내용물 제외 |
| `tmp/` | 쓰지 않는 파일 (초기 학습용 캡처 jpg, 중복 영상), `capture.py`가 저장하는 곳 | 통째로 제외 |
| `tmp/legacy_models/` | legacy 가중치 (`best.pt`, `lane_best.pt`, `260921_best.pt`, `old_pt/`) | 제외 |
| `dataset` | 데이터셋 심볼릭 링크 (아래 참고) | 제외 |
| `build/` `install/` `log/` | colcon 빌드 결과 | 제외 |

- **mj_ws 는 colcon 작업공간이다.** 패키지는 `src/mj_*/` 에만 있고 의존 방향은 `core ← drive ← sim/real` 로만 간다.
  `package.xml` 이 없는 `src/tools`·`src/tests`·`src/legacy`·`src/station` 은 colcon 이 무시한다.
  빌드: `cd mj_ws && colcon build --base-paths src --symlink-install` (`~/pinky/install` 위, tg_ws 는 source 하지 않음)
- **빌드 없이도 실행된다.** 스크립트·시험은 `src/mj_lane_core`, `src/mj_lane_drive` 를 `sys.path` 에 넣고 import 한다
  (`src/robot/step2_lane_follow.py`, `src/tools/step1_check_lane_detection.py`, `src/tests/testlib.py`).
- 경로는 `mj_ws/` 를 기준으로 잡는다. 주행 노드는 파일 위치에서 계산하고, 복사 설치했을 때는 환경 변수 `MJ_WS` 로 알려 준다.
- 입력 영상은 `inputs/`에서 읽고, 결과물은 `outputs/`에 저장한다.
- 모델·영상은 git에 올라가지 않으므로 팀원은 각자 `models/`, `inputs/`에 넣어야 한다.

## 모델

| 경로 | 용도 | 쓰는 스크립트 |
|---|---|---|
| `models/lane_model_ncnn/best_ncnn_model/` | 로봇 실주행용 (NCNN, 320×320) | `mj_lane_drive` 주행 노드 (`robot/step2_lane_follow.py`), `robot/lane_mission_drive.py`, `legacy/model_ncnn.py` |
| `models/260928_yolon_best.pt` | 저장 영상 검증용 (PC) | `tools/step1_check_lane_detection.py` (legacy의 `2_realtime.py`, `find_target_point.py`, `find_target_point_dual.py`, `test_seg.py`도 이 모델을 가리킴) |
| `tmp/legacy_models/*` | legacy (사용 안 함) | `legacy/1_realtime.py` (`old_pt/pouch_best.pt`) |

- ncnn 모델은 `260928_yolon_best.pt`에서 export했다
  (`models/lane_model_ncnn/best.pt`와 sha256 동일 확인).
  그래서 검증용 pt와 주행용 ncnn은 같은 가중치다.
- ultralytics NCNN은 폴더 경로를 받고, **폴더 이름에 `_ncnn_model` 이 들어 있어야 NCNN 으로 인식한다**
  (`ultralytics/nn/autobackend.py` 의 형식 판별). 그래서 `MODEL_PATH`는 `models/lane_model_ncnn/best_ncnn_model`을 가리킨다.
  **이 폴더 이름을 바꾸지 말 것** — 바꾸면 "not a supported model format" 오류로 로드되지 않는다.
  `models/lane_model_ncnn/` 에는 export 원본 `best.pt` 와 `best_ncnn_model/`(metadata.yaml, model.ncnn.bin, model.ncnn.param)이 있다.
- 모델 경로는 각 스크립트 상단의 `MODEL_PATH` 상수 한 줄로 정해진다. 모델을 바꿀 때는 그 줄만 고친다.

## 출력 파일명 규칙

`outputs/result_<스크립트명>.mp4` (예: `step1_check_lane_detection.py` → `outputs/result_step1_check_lane_detection.mp4`)

주행 기록(step2)은 매번 남겨야 하므로 실행 시각을 붙인다:
`outputs/result_<스크립트명>_<YYYYmmdd_HHMMSS>.csv` (프레임별 로그, 항상) / `.mp4` (오버레이 영상, `--record` 일 때만). 두 파일은 이름이 같다

## 하드웨어

### 로봇: Pinky

- 크기: 110(W) / 120(D) / 142(H) mm
- SBC: 라즈베리파이 5 (8GB)
- 구동: 다이나믹셀 XL330-M288-T × 2
  (무부하 103rpm @5V, 정지토크 0.52N·m, 감속비 288.4:1, Velocity Control Mode 지원)
- 바퀴 지름 54mm, 바퀴 간격(트레드) 96.1mm — pinky_bringup 값
  (`wheel_radius 0.027`, `wheel_separation 0.0961`, `pinky_bringup/config/pinky_params.yaml`).
  `cmd_vel` → rpm 변환과 odom 계산에 실제로 쓰이는 값이다
  - 참고(측정 기준 불명확): 실측 지름 50mm(±5mm), 간격 85mm.
    트레드는 접지점 중심 사이 거리인데 바퀴 안쪽 면 사이를 쟀을 가능성이 높고(타이어 폭 약 10mm를 더하면 96mm),
    지름도 고무 타이어 바깥까지 재면 54mm가 나올 수 있다
- 최고 속도 약 0.28 m/s (bringup 100 rpm 제한 기준. 모터 무부하 103 rpm 이면 0.29 m/s), 권장 주행 0.10–0.15 m/s
- 센서: RPLiDAR C1, BNO055 9축 IMU, 초음파 US-016, IR TCRT5000

### 카메라

- Raspberry Pi Camera Module v1 (OV5647), 보드 Rev 1.3
- 수평 화각 53.5° / 수직 화각 41.4° (표준 사양, 촬영 모드에 따라 달라질 수 있음)
- 바닥에서 렌즈까지 높이 60mm
- 로봇 중심선 위에 장착
- 틸트각: 미측정 (재는 법: 화면 가운데(y=240)에 보이는 바닥 점까지 렌즈에서의 수평 거리 d 를 재서
  `틸트 = atan(0.060 / d)`. 지금 시뮬 가정 8° 이면 d ≈ 42.7 cm)
  - 주행 코드(`mj_lane_core`·`mj_lane_drive`)는 틸트를 쓰지 않는다. 시뮬 카메라에만 쓰인다
  - 바꾸는 곳 (시뮬, 지금 기본 8°): `mj_lane_sim/launch/sim_robot.launch.xml` 의 `camera_tilt` 인자 기본값.
    실행할 때만 바꾸려면 `camera_tilt:=값`. `scripts/sim_urdf.sh` 의 4번째 인자 기본값(8)도 같이 맞춘다

### 차동구동 변환식

**실제 제어 코드가 아니라, "이 속도면 바퀴가 몇 rpm인지" 가늠하는 참고용 계산이다.**
실제 변환은 bringup 이 한다 (아래 참고).

```python
# 차동구동 변환 (Pinky)
# 부호 규칙: omega > 0 = 반시계 방향 = 좌회전 (ROS REP-103 관례)
import math

WHEEL_D = 0.054     # 바퀴 지름 [m] (bringup wheel_radius 0.027)
TREAD   = 0.0961    # 바퀴 간격 [m] (bringup wheel_separation)
MAX_RPM = 103       # XL330-M288-T 무부하 최고 회전수 (5V)

def to_wheel_rpm(v, omega):
    """v [m/s], omega [rad/s] -> (좌, 우) 바퀴 rpm"""
    v_left  = v - omega * TREAD / 2
    v_right = v + omega * TREAD / 2
    k = 60.0 / (math.pi * WHEEL_D)   # m/s -> rpm
    return v_left * k, v_right * k
```

- **2단계 코드는 `cmd_vel`(m/s, rad/s)만 publish 한다.** 그 뒤 처리는 bringup(`pinky_pro/pinky_bringup`, 제조사 코드)이 모두 한다
  - 좌우 바퀴 rpm 계산: `bringup.py` 의 `twist_callback` (위와 같은 식, bringup 치수 사용)
  - 오른쪽 바퀴 부호 반전
  - 한쪽이라도 100 rpm 을 넘으면 두 바퀴를 같은 비율로 축소
  - 다이나믹셀 Goal Velocity 단위 변환: `dynamixel_driver.py` 의 `RPM_TO_VALUE_SCALE = 1 / 0.229` (0.229 rpm/단위)
- 회전 방향(ω > 0 이 실제로 좌회전인지)은 첫 주행에서 눈으로 확인할 것. 반대면 로봇이 거꾸로 꺾는다

### 실측 필요

- **카메라 거리 캘리브레이션** — 바닥 30/60/90/120cm 지점에 표시하고 촬영해, 거리↔이미지 y좌표
  대응표를 만들 것. 틸트각을 따로 재는 대신 이걸로 역산한다.
  (차선을 읽는 샘플 행 y=432/384/336/288(640×480 기준)이 바닥 몇 cm인지도 이 표로 확인한다)
- **로봇 중심선의 이미지상 x좌표** — 현재 코드는 화면 중앙(x=320)을 로봇 중심선으로 가정한다
  (`src/mj_lane_core/mj_lane_core/lane_postprocess.py`의 `error` 계산)
- **바퀴 지름** — bringup 값(54mm)을 채택했다. PLAN.md "2단계 테스트 항목"의 직진 검증으로 확인한다

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
선이 화면 중앙에서 오른쪽에 보여도, 로봇이 그 선의 오른쪽을 주행하면 `left_lane`이다.

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

## 테스트

회귀 테스트: `bash src/tests/run_tests.sh` (빠른 것 6개, 몇 초) / `bash src/tests/run_tests.sh all` (13개, ncnn 모델·입력 영상·ROS 포함, 약 3분, 순차 실행) — 정지·속도 로직을 일부러 바꿨으면 결과를 확인한 뒤 `src/tests/test_step2_replay.py` 의 `EXPECTED` 를 갱신한다.

## 시뮬레이션 (mj_lane_sim)

PC 에서 Gazebo 로 mission4_3 코스를 띄우고, 실물과 같은 주행 노드(`mj_lane_drive`)를 붙인다.
**내 도메인 95 + `GZ_PARTITION=mj_sim` + LOCALHOST 로만 돈다.** tg_ws 는 source 하지 않는다.

터미널 4개에서 하나씩 (스크립트가 source·환경 변수를 알아서 한다. 직접 `ros2 topic` 을 칠 터미널만 따로 설정):
```
R=~/dev_ws/wt_mj/mj_ws/src/mj_lane_sim/scripts/run_mj_sim.sh
bash $R world            # ① Gazebo 창. 10초쯤 기다림
bash $R robot            # ② 출발점 (1.80, 0.10, 서쪽). "Entity creation successful" 확인
bash $R camera           # ③ 카메라 화면 (검출 표시 화면은 `bash $R camera debug`, drive 가 떠 있어야 나옴)
bash $R drive --record   # ④ Space 출발, Enter 종료. teleop 과 동시에 켜지 말 것
                         #    갈림길에서 키로 고르려면 drive --record --junction-keys (대기 중 ← → ↑)
bash $R stop             # 끝난 뒤 Gazebo 가 남아 있으면
```
- 주행 노드 옵션: `--camera-topic --namespace amr_01 --sonar-topic us_sensor/range --use-sim-time` (run_mj_sim.sh 가 붙임).
  시뮬 시간(/clock)으로 판단하므로 시뮬이 느려도 0.5초·2.0초 기준이 시뮬 초로 맞다
- 검출 표시 영상: 주행 노드가 `lane_debug/image` (+ `/compressed`, JPEG) 를 낸다. 녹화(`--record`)와 같은 그림 (클래스별 색·범례·신뢰도). **구독자가 있을 때만** 별도 스레드에서 그려 제어 루프를 늦추지 않는다. 끄려면 `--no-debug-image`
- 갈림길 화살표 키 (`--junction-keys`, 기본 꺼짐): 차선 상실 정지나 "cross_lane + 상실" 이면 멈추고 기다린다.
  ← 좌회전 / → 우회전 / ↑ 직진. 앞으로 20 cm → odom 으로 70° 돌기 → 곧게 가며 좌우 차선을 다시 잡으면 보통 추종.
  꺾는 중에도 초음파 정지가 우선이고, 8초 안에 못 끝내면 멈추고 다시 기다린다. 판단은 `mj_lane_core/junction.py`.
  기록: `result_..._junctions.csv` (갈림길 사건·odom 위치·검출 상태) + 대기 순간 사진. 실물 SSH 는 `ssh -t` 로 접속한 터미널에서
- 카메라는 실물 값(640×480, 화각 53.5°, 렌즈 높이 60 mm)에 맞췄다. 기울기만 근사 (8°, 위 "카메라" 항목)
- 초음파는 라이다로 흉내 낸다 (10 Hz, 화각 0.26 rad, 상한 0.97 m). 실물의 무효값(−0.03) 같은 고장 모양은 없다
- **바닥 그림의 아래쪽 구간(고리 아랫변, 꼬리 아랫길)은 줄자와 다르다** (고리 아랫변 폭 11.2 cm, 줄자 16.6 cm). 이 구간 결과는 "실물과 다름" 으로 본다
- Wi-Fi 가 끊기면 Gazebo 끼리 서로를 못 찾아 로봇이 생성되지 않는다 ("Network is unreachable")
- 첫 주행 결과는 `PLAN.md` "시뮬레이션 결과" 에 있다

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

`step1_`–`step4_` 접두어는 `PLAN.md`의 4단계 계획 결과물에만 붙인다.
