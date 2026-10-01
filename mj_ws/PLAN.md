# 개발 계획 및 진행 상황

차선 인식 기반 자율주행. 저장 영상 검증부터 관제 GUI까지 4단계로 진행한다.

## 단계

| 단계 | 내용 | 모델 | 실행 위치 | 상태 |
|---|---|---|---|---|
| 1 | 저장 영상으로 left/right 차선 인식 확인 | `260928_yolon_best.pt` | PC | 완료 |
| 2 | 차선 가운데로 주행 | `lane_model_ncnn` | 로봇 | 대기 |
| 3 | 2번 + crosswalk 감지 시 3초 정지 후 재출발 | `lane_model_ncnn` | 로봇 | 대기 |
| 4 | 3번 + GUI로 지도에서 목표지점 클릭해 이동 | `lane_model_ncnn` | 로봇 + 관제 PC | 대기 |

4단계의 주행은 객체인식으로, 좌/우회전 판단은 좌표 계산으로 한다.

## 진행 이력

- **이관 완료** — `~/dev_ws/yolo_mission` → `mj_ws`, git 이력 관리 시작 (`aafea7e`)
- **정리 완료** — 폴더 구조 정리, 모델 역할 확정, legacy 분리 (`4e091a9`)
- **1단계 완료** — `src/lane_follow_check.py`(현 `src/tools/step1_check_lane_detection.py`) 로 차선 인식 확인. 결과 영상 육안 검증 통과
- **후처리 분리 완료** — 검증 기능 보강 + 후처리 분리 (`src/common/lane_postprocess.py`) (`b903714`)
- **폴더 재편 + --threads** — `src/` 를 common/robot/station/tools/legacy 로 재편, `--threads N` 추가 (`4f9f99a`)
- **빨간 십자 버그 수정 완료** — 목표점 마커를 `LaneTracker` 의 실제 `target_x` 로 그림 (`71a92f7`)
- **ncnn 모델 경로 수정** — 폴더 이름에 `_ncnn_model` 이 없으면 ultralytics 가 NCNN 으로 인식하지 못해 `best_ncnn_model/` 중첩 복원 (`585947f`)
- **2단계 준비** — 공통 `LaneTracker` 에 D항·clip·`reset()` 추가 (`7b3d2c9`), 차선 상실을 초 단위로 (`e7b0e54`)
- **2단계 코드 작성** — `src/robot/step2_lane_follow.py` + `src/robot/drive_control.py`. PC 검증 완료, 로봇 미검증
- **진행 중** — 2단계 로봇 테스트

## 1단계 전체 영상 통계

`inputs/pinky_20260919_182009.mp4` 전체 1469프레임, `260928_yolon_best.pt`, CONF 0.5.
`tools/step1_check_lane_detection.py` 의 요약 출력 기준.

| 항목 | 프레임 | 비율 |
|---|---|---|
| left_lane 검출 | 1119 | 76.2% |
| right_lane 검출 | 905 | 61.6% |
| crosswalk 검출 | 281 | 19.1% |
| 좌우 양쪽 다 보임 | 643 | 43.8% |
| 한쪽만 보임 | 738 | 50.2% |
| 둘 다 미검출 | 88 | 6.0% |

둘 다 미검출 연속 구간은 4개 (frame 814부터 3, 955부터 14, 986부터 2, 1400부터 69).
LOST_LIMIT(15) 이상은 1400부터 끝까지 이어진 69프레임 1회뿐인데, 이 구간은 로봇이 벽 바로
앞까지 다가간 영상 끝부분이라 화면에 차선이 없다 (frame 1395·1400·1430·1468 육안 확인).
주행 중 가장 긴 미검출은 frame 955부터 14프레임이다.

## 완료: 후처리 분리 (`b903714`)

1단계와 2단계를 잇는 준비 작업이었다.

`lane_follow_check.py`(현 `tools/step1_check_lane_detection.py`) 안에서 "마스크를 받아
차선 중심·조향값을 계산하는" 부분을 `src/common/lane_postprocess.py` 로 떼어냈다.
이 계산은 모델 종류와 무관하므로, 2단계 ncnn 주행 코드에서 그대로 재사용한다.

검증 기준: **분리 전후 결과 영상이 픽셀 단위로 동일해야 한다.**
→ 결과: 앞 150프레임 픽셀 단위 동일 (mp4 sha256 일치). 동작을 바꾸는 수정은 섞지 않았다.

함께 추가한 것:

- 프레임별 클래스 검출 여부와 신뢰도 출력
- 요약 통계 (left/right/crosswalk 검출률, 양쪽 다 놓친 프레임 수)
- 입출력 경로 argparse (`--input`, `--output`, `--max-frames`)

## 완료: 빨간 십자 버그 수정 (`71a92f7`)

`draw()` 가 목표점 마커를 `ROW_WEIGHTS[:len(centers)]` 로 다시 계산하고 있었다. 가까운 행이
비고 먼 행만 보이면 가중치가 한 칸씩 밀려서, 화면의 빨간 십자가 실제 조향에 쓰는 `target_x` 와
달랐다. `LaneTracker` 가 계산한 `target_x` 를 그대로 그리도록 고쳤다. 조향 계산은 원래 올바른
가중치를 쓰고 있었으므로 동작 변화는 없다 (요약 통계 동일).

결과 (전체 1469프레임, 인코딩 전 렌더 비교):

- 마커가 그려진 1375프레임 중 **590프레임(43%)에서 위치가 달랐다**
- 어긋난 것은 **가장 가까운 샘플 행(0.90)이 비었을 때만**이다
  (행 패턴 0011 319 · 0111 256 · 0110 14프레임. 1110 1프레임(frame 531)은 부동소수점 반올림 1px로 버그와 무관)
- 이동량 중앙값 9px, 최대 49px (frame 812)
- 프레임별 데이터: `tmp/marker_diff.csv` (git 제외)

## 현재 작업: 2단계 로봇 테스트

코드: `src/robot/step2_lane_follow.py` (ROS 노드), `src/robot/drive_control.py` (ROS 무관 판단 로직: `FrontStop`, `LaneFollowController`).
아래 "2단계 실행 절차" 대로 `--dry-run` 부터 시작하고, "2단계 테스트 항목" 순서로 확인한다.

PC 에서 확인한 것 (로봇에서는 아직 안 돌려 봄):

- 단위 테스트 23개 통과: 초음파 판정(측정 없음·끊김·무효값 −0.03/NaN/inf·0.15 m 경계·latch·재개 거부), 속도/상실 단계 경계(0.5초·2.0초)
- 저장 영상 재생 (ncnn 320, 영상 시간 기준): 주행 중 상실 구간 frame 955~977(1.15초)에서 감속만, 정지는 영상 끝 벽 구간(frame 1440~)뿐.
  ncnn 320 은 .pt 640 보다 이 구간을 길게 놓친다 (.pt 는 954~, 15프레임)
- ROS 연동 (격리 도메인, 영상 입력, 가짜 초음파): 장애물·무효값 → 0.02~0.05초 안에 정지 (추론 루프를 기다리지 않음),
  초음파 끊김 → 0.38초 안에 정지, 정지 후 Space 전까지 주행 명령 0개, 정지 조건이 남아 있으면 Space 거부, 종료 시 0 명령

### 주행 속도 메모

권장 주행 속도 **0.10~0.15 m/s**. 근거는 CLAUDE.md "하드웨어"의 `to_wheel_rpm` (참고용 계산),
bringup 치수 기준 (지름 54mm, 간격 96.1mm → k = 60 / (π × 0.054) ≈ 353.7 rpm per m/s):

- 최고 속도 0.283 m/s ← bringup 의 100 rpm 제한 (모터 무부하 103 rpm 이면 0.291 m/s)
- 직진 0.10 m/s → 35.4 rpm, 0.15 m/s → 53.1 rpm (100 rpm 제한의 35~53%)
- 회전 여유: 바깥 바퀴 속도 = v + ω × 0.0961 / 2. 현재 `robot/lane_mission_drive.py` 의
  `MAX_ANGULAR` 1.5 rad/s 로 최대로 꺾으면
  v = 0.10 일 때 바깥 바퀴 0.172 m/s (60.9 rpm), v = 0.15 일 때 0.222 m/s (78.5 rpm)
  → 권장 범위 안에서는 최대 조향에도 100 rpm 제한에 걸리지 않는다 (bringup 의 비율 축소가 일어나지 않음)
- 현재 `BASE_SPEED` 0.10 m/s 는 권장 범위 안이다

## 2단계 설계

### 제어 방식 (결정)

2단계 코드는 **`cmd_vel`(`linear.x` m/s, `angular.z` rad/s)만 publish 한다.**
좌우 바퀴 rpm 계산, 오른쪽 바퀴 부호 반전, 100 rpm 비율 제한, 다이나믹셀 단위 변환은 모두 bringup 이 처리한다.
CLAUDE.md 의 `to_wheel_rpm` 은 속도 감을 잡기 위한 참고용 계산이며 제어 코드에 넣지 않는다.

### 2단계 실행 절차

> **초음파 노드는 bringup 에 들어 있지 않다. 안 띄우면 비상 정지가 조용히 꺼진 채로 주행한다.**

1. `ros2 launch pinky_bringup bringup_robot.launch.xml`
2. `ros2 run pinky_sensor_adc main_node` ← **필수**. bringup 이 띄우지 않는다
3. 확인: `ros2 topic hz /us_sensor/range` 가 약 20 Hz 로 나오는지 본 뒤에 다음 단계로 간다
4. `python3 src/robot/step2_lane_follow.py --dry-run` (cmd_vel 없이 판단만 확인) → 이상 없으면 `--dry-run` 없이 실행.
   Space 로 출발 (초음파가 정상이고 장애물이 없을 때만 받아들임), Enter/ESC 로 종료.
   로봇에 mj_ws 구조가 없으면 `--model <best_ncnn_model 경로>` 로 모델 위치를 넘긴다 (`src/robot/` 와 `src/common/` 을 함께 복사)

- 4단계 미션 흐름(localization, `station/mission_gui.py`)은 `robot/lane_mission_drive.py` docstring 의 실행 순서를 따르되,
  2번(`pinky_sensor_adc`)을 bringup 바로 뒤에 넣는다
- 비상 정지 설계의 "메시지가 끊기면 정지" 판정이 코드에 들어가면 이 노드를 빠뜨려도 출발하지 않게 된다.
  **그 전까지는 이 절차가 유일한 안전장치다**

### 차선 상실 처리 (결정)

`robot/lane_mission_drive.py` 의 단계적 대응(감속 → 정지)을 채택한다. 단 기준을 **프레임 수가 아니라
초 단위**로 바꾼다. 로봇의 fps가 PC와 다르기 때문에 프레임 기준은 실행 환경마다 의미가 흔들린다.

차선을 못 본 시간 t 기준. 현재 코드의 구간별 동작과 값은 그대로 두고, 표현만 초 단위로 바꾼다.

| 구간 | 조건 | 동작 (현재 코드) | 기본값 |
|---|---|---|---|
| 초기 상실 | 0 < t ≤ 감속 기준 | `BASE_SPEED × 0.7`, 직전 조향 유지 | — |
| 감속 | t > 감속 기준 | `MIN_SPEED`, 직전 조향 유지 | 0.5초 |
| 정지 | t > 정지 기준 | 정지, 조향 0으로 초기화 | 2.0초 |

- 근거: 이 값은 `lane_mission_drive.py` 에서 실제로 쓰이던 값(`LOST_SLOW_AFTER = 5` / `LOST_STOP_AFTER = 20`
  스텝 @ `CONTROL_HZ = 10`)이며, 첫 주행 테스트에서 관찰 후 조정한다
- 스텝 기준인 현재 코드는 추론이 10Hz를 못 따라가면 실제 시간이 이보다 길어진다. 초 단위로 바꾸면 이 흔들림이 없어진다
- 구현 시 고려:
  - 경과 시간은 로봇에서는 마지막으로 차선을 본 시각부터의 실제 시간, 저장 영상 검증(step1)에서는
    프레임 수 ÷ 영상 fps로 계산해야 두 환경의 기준이 같아진다
  - step1 의 `LOST_LIMIT`(15프레임, 조향만 0으로 되돌림)도 이 기준으로 맞췄다 → `LOST_STOP_SEC = 2.0`.
    공통 `LaneTracker` 가 `update(..., now)` 로 시각을 받아 `lost_time`(초)을 계산한다.
    1단계 영상에서 달라진 것은 영상 끝 벽 구간의 조향 0 복귀 시점뿐이다 (frame 1414 → 1439, 25프레임)
- 검증 메모: 1단계 영상(20fps, `LaneTracker` 기준 상실 구간)에 이 기준을 대입하면 주행 중 감속은
  1회(frame 954부터 15프레임 = 0.75초), 정지는 영상 끝 벽 구간(frame 1399부터 70프레임 = 3.5초)뿐이다.
  **주행 중 정지 경로는 검증되지 않았으므로** 2단계에서 실제 로봇으로 따로 확인한다

### 전방 초음파 비상 정지 (추가)

전방 초음파(US-016)로 거리를 재서, **차선 검출과 무관하게** 전방 거리가 임계값 이하이면 즉시 정지한다.
카메라·추론이 정상이어야 동작하는 차선 상실 타이머보다 확실한 안전장치다.

- 임계 거리는 상수 `FRONT_STOP_DISTANCE = 0.15` m (결정). 첫 주행 테스트에서 관찰 후 조정
- 모든 주행 상태(차선 추종, 교차로 진입·회전 포함)보다 우선한다. `control()` 결과와 상관없이
  `send()` 직전에 정지 명령으로 덮어쓰는 위치가 자연스럽다
- 현재 상태: `lane_mission_drive.py` 는 **초음파를 쓰지 않는다**. 구독은 `/mission/goal_pose`, `/mission/enable`,
  `odom` 과 TF 뿐이고 `sensor_msgs` import 도 없다. 코드의 "distance" 는 모두 odom 기반 목표·진입 거리다

#### 드라이버 확인 결과

`/home/mindy/pinky_new/src/pinky_pro/pinky_sensor_adc/src/main_node.cpp` (제조사 코드, 읽기만 함)

| 항목 | 내용 |
|---|---|
| 노드 | `pinky_sensor_adc` (패키지 `pinky_sensor_adc`, 실행 파일 `main_node`). I2C `/dev/i2c-1`, 주소 0x08 의 ADC 를 읽는다 |
| 토픽 | `us_sensor/range` (상대 이름 → 기본 네임스페이스에서 `/us_sensor/range`) |
| 메시지 | `sensor_msgs/msg/Range`, `radiation_type = ULTRASOUND`, `frame_id = ultrasonic_link`, `field_of_view = 0.26` rad |
| 단위 | m (`min_range = 0.02`, `max_range = 3.0`) |
| 값 계산 | `range = adc / 4096 × 1.0 − 0.03` (ADC 12bit, 채널 3). 가능한 값은 **−0.03 ~ 약 0.97 m** |
| 측정 실패 / 범위 밖 | 드라이버에 유효성 검사가 없다. **inf·NaN·0 같은 표식 값은 오지 않는다.** I2C 읽기 실패 시 반환값을 확인하지 않고 버퍼 초깃값 0 을 그대로 써서 **−0.03 m** 가 온다. 먼 물체는 계산식상 상한 약 0.97 m 로 포화된다 (`max_range` 3.0 과 다름. 실제 포화 동작은 실측 필요) |
| 센서 개수 | 초음파는 ADC 채널 1개만 읽어 **토픽 하나**로 publish 한다. 좌/우 별도 토픽 없음. 같은 노드가 IR 3채널(`ir_sensor/range`, `UInt16MultiArray` 원시 ADC 값)과 `batt_state` 도 publish 한다 |
| 갱신 주기 | 파라미터 `rate` 기본 **20 Hz** (코드 주석의 "100Hz" 는 실제 값과 다름). 한 주기에 5채널 × 6 ms 대기가 있어 약 33 Hz 가 상한 |
| 기동 | **`pinky_bringup/launch/bringup_robot.launch.xml` 은 이 노드를 띄우지 않는다.** `ros2 run pinky_sensor_adc main_node` 로 따로 실행해야 한다 |

#### 설계에 반영할 점

- 구독: `/us_sensor/range` (`sensor_msgs/msg/Range`), 값은 m
- **fail-safe 판정**: 다음 중 하나면 정지한다
  - `range ≤ FRONT_STOP_DISTANCE`
  - `range < min_range` (0.02 m 미만, I2C 실패 시의 −0.03 m 포함). 무효값을 걸러 무시하면 센서 고장 시 비상 정지가 꺼지므로, 무시하지 말고 정지로 취급한다
  - 마지막 메시지가 일정 시간 이상 오지 않음 (노드 미실행·중단). 제한 시간은 `SONAR_TIMEOUT = 0.3` 초 (센서 주기 0.05초의 6배. 구현 때 정한 값이라 조정 가능)
- `FRONT_STOP_DISTANCE` 는 상한 약 0.97 m 보다 충분히 작아야 한다. 그 이상이면 항상 정지 상태가 된다
- 반응 지연: 센서 주기 최대 0.05초 + 메인 루프 한 바퀴(`spin_once()` → `step()`, 최대 0.1초 + 추론 지연).
  0.15 m/s 면 0.15초 동안 약 2.3cm 이동하므로 임계 거리에 이 여유를 포함한다
- `pinky_sensor_adc` 는 bringup 이 띄우지 않으므로 따로 실행해야 한다 → 위 "2단계 실행 절차" 2번
- 해제 조건: **수동 재개** (결정). 정지 상태를 유지하다가 Space 를 누르면 재개하되, 그 순간 정지 조건이 하나라도 남아 있으면 거부한다.
  출발도 같은 방식 (키 입력으로 시작, 초음파 노드가 없으면 출발 불가)
- 구현: 초음파 콜백은 별도 executor 스레드에서 돌고 걸리는 즉시 0 명령을 낸다. 메인 루프 발행과 같은 lock 으로 묶어
  정지 직후 주행 명령이 끼어들지 않게 했다. 끊김(timeout)은 콜백이 오지 않으므로 메인 루프에서 판정한다

### 2단계 테스트 항목

- **직진 검증 (바퀴 지름)** — 1m 직진 명령 후 실제 이동 거리를 줄자로 잰다
  - odom 거리와 실제 거리가 다르면 bringup 의 `wheel_radius` 를 보정한다:
    `새 wheel_radius = 0.027 × (실제 거리 ÷ odom 거리)`
    (bringup odom 거리 = 엔코더 회전수 × 2π × `wheel_radius` 라서 비례 보정이 성립한다)
  - 직진 중 한쪽으로 휘면 좌우 바퀴 지름 차이를 의심한다
- **회전 검증 (바퀴 간격)** — 직진 검증으로 `wheel_radius` 를 확정한 **뒤에** 한다 (회전각 계산에도 바퀴 지름이 들어가므로)
  - 제자리 회전을 **5바퀴(1800°)** 시킨다. 한 바퀴(360°)로는 오차가 작아 구분이 안 되므로 누적시킨다
  - 로봇과 바닥에 표시를 해 실제 회전각을 재고, odom 회전각과 비교한다
  - 제자리 회전은 바퀴 미끄러짐의 영향이 커서 직진 검증보다 신뢰도가 낮다.
    **차이가 10% 이상(1800° 기준 180°, 반 바퀴 이상)일 때만** `wheel_separation` 을 보정하고, 그 이하는 미끄러짐으로 본다
  - 보정식: `새 wheel_separation = 0.0961 × (odom 회전각 ÷ 실제 회전각)`
    (bringup 회전각 = (오른쪽 거리 − 왼쪽 거리) ÷ `wheel_separation` 이라 간격과 반비례.
    odom 이 실제보다 많이 돌았다고 하면 간격을 늘린다)
- **차선 상실 정지 경로** — 1단계 영상에서는 주행 중 한 번도 발동하지 않았다 (위 "차선 상실 처리" 검증 메모)

## 미뤄둔 것

| 항목 | 내용 | 언제 |
|---|---|---|
| 가까운 행 공백 | 가장 가까운 샘플 행(0.90)에 차선 중앙점이 없는 프레임이 **56%(770/1375)**, 마커가 그려진 프레임 기준. 목표점이 먼 행에 기대는 경우가 많다 | 2단계 속도 설정 시 고려 |
| 후처리 복제본 | 공통 `LaneTracker` 가 D항·clip·`reset()` 을 파라미터로 지원한다 (`7b3d2c9`, 복사본과 1469프레임 결과 일치 확인). `step2_lane_follow.py` 는 공통 모듈을 쓴다. `lane_mission_drive.py` 는 아직 자체 복사본(프레임 기준 상실)을 쓴다 | 4단계에서 공통 모듈로 교체 |
| 로봇 배포 경로 | 현재 모든 경로가 `BASE_DIR`(= `mj_ws/`) 기준이다. `step2_lane_follow.py` 는 `--model` 로 모델 경로를 받는다. `lane_mission_drive.py` 는 아직 상수 | 4단계에서 `lane_mission_drive.py` 정리 시 |
| lane_mission_drive 콜백 지연 | 메인 루프가 `spin_once()` 를 한 번만 부르는데, `spin_once` 는 콜백을 하나만 실행한다. `odom` 이 10 Hz 보다 자주 오면 큐에 쌓여 회전 판단에 오래된 odom 을 쓰게 된다. `step2_lane_follow.py` 는 executor 스레드로 해결 | 4단계에서 같은 방식으로 수정 |
| step2 LCD 미리보기 | `lane_mission_drive.py` 의 LCD 미리보기를 `step2_lane_follow.py` 에는 넣지 않았다 (터미널 로그만) | 필요하면 2단계 테스트 중 추가 |

## 코드 배치 원칙

실행 위치로 나눈다. 로봇에 배포할 때 해당 폴더만 복사하면 되도록.

```
src/
├── common/     양쪽에서 쓰는 순수 계산 (차선 후처리, 좌표 변환)
│   └── lane_postprocess.py
├── robot/      로봇에서 실행. ncnn 추론 + 주행
│   ├── lane_mission_drive.py
│   └── model_ncnn.py
├── station/    관제 PC에서 실행. GUI
│   └── mission_gui.py
├── tools/      개발용. 저장 영상 검증 등
│   └── step1_check_lane_detection.py
└── legacy/     안 쓰지만 보관
    ├── 1_realtime.py
    ├── 2_realtime.py
    ├── capture.py
    ├── find_target_point.py
    ├── find_target_point_dual.py
    └── test_seg.py
```

이름 규칙:

- `step1_` ~ `step4_` 접두어는 위 4단계 계획의 결과물에만 붙인다
  (1단계 → `tools/step1_check_lane_detection.py`)
- 아직 기반 코드인 `robot/lane_mission_drive.py`, `station/mission_gui.py` 는 접두어 없이 이름을 유지한다

## 작업 규칙

- 한 단계가 **실제로 동작하는 것을 확인한 뒤** 다음 단계로 간다
- 리팩터링(동작 안 바뀜)과 기능 수정(동작 바뀜)을 같은 커밋에 섞지 않는다
- 단계마다 커밋해서 되돌릴 지점을 남긴다
- 저장소 밖 경로를 건드리는 작업은 Manual 모드로 진행한다
