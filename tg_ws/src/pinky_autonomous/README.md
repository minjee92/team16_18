# pinky_autonomous

Pinky Pro 의 **차선 인식 기반 자율주행** ROS 2 패키지 (Jazzy, 로봇 RPi5 에서 실행).
카메라 영상을 YOLO(NCNN)로 분석해 좌·우 차선을 따라가고, 횡단보도·교차로를 처리하며, 초음파로 장애물 앞에서 멈춘다.
기존 `pinky_pro` 저장소는 수정하지 않는 별도 패키지다 (`pinky_interfaces`, `pinky_navigation`, `pinky_sensor_adc`, `pinky_lamp_control` 을 사용).

## 하는 일

| 상황 | 동작 | LED |
|---|---|---|
| 평소 | 좌·우 차선 중앙을 따라 주행 | 초록 깜빡 |
| 횡단보도 감지 | 감속하며 앞을 확인 → 일정 시간 물체가 없으면 정속 복귀, 있으면 정지 | 주황 깜빡 / 빨강 |
| 교차로(`cross_lane`) | 조금 더 전진해 정지 → SLAM 맵으로 출발점까지 최단 경로 쪽(좌/우/후진)을 골라 복귀 | 빨강 → 초록 |
| 초음파 ≤ 0.35 m | 감속 | 주황 깜빡 |
| 초음파 ≤ 0.10 m / ≤ 0.05 m | 정지 / 충돌 | 빨강 |
| 차선 소실 | 정지 | 빨강 |
| 출발점 도착(HOME) | 정지 | 빨강 |

상세 규칙은 `pinky_autonomous/autonomous_drive_node.py` 맨 위 설명과 `docs/overview_report.html` 참고.

## 구성

| 파일 | 설명 |
|---|---|
| `autonomous_drive_node.py` | 메인 노드 (`autonomous_drive`): 카메라 → YOLO → 차선 추종/상태 머신 → `cmd_vel`, LED, LCD |
| `route_planner.py` | 교차로에서 SLAM 맵(`/map`)으로 출발점까지 최단 경로 쪽을 고르는 순수 계산 모듈 |
| `ultrasonic_node.py` | GPIO(TRIG/ECHO, US-016) 초음파 퍼블리셔 (`ultrasonic_sensor`). ADC 보드를 쓰면 필요 없음 |
| `video_recorder_node.py` | 관제 PC 에서 `camera/compressed` 를 영상 파일로 저장 (`video_recorder`) |
| `launch/autonomous_drive.launch.xml` | **로봇**에서 실행: 자율주행 + LED 노드 + ADC 센서(+선택 SLAM) |
| `launch/control_pc_slam.launch.xml` | **관제 PC**에서 실행: 로봇의 `/scan`·`/tf` 로 SLAM 을 돌려 `/map`, `map→odom` 제공 |
| `launch/video_recorder.launch.xml` | **관제 PC**에서 실행: 주행 영상 저장 |

## 준비물

- 로봇: `pinky_bringup` 실행 중, `lamp_bringup.service`(ws2811) 실행 중, `pinky_emotion` 종료(LCD SPI 충돌 방지), Nav2 종료(`cmd_vel` 겹침 방지)
- 파이썬: `ultralytics`(YOLO), `picamera2`, `opencv`, `numpy`, `Pillow`. GPIO 초음파를 쓸 때만 `gpiozero lgpio`
- 관제 PC(복귀 기능 사용 시): `slam_toolbox`, 로봇과 같은 `ROS_DOMAIN_ID`, **시계 동기화**(chrony/NTP)
- **YOLO 모델(NCNN 폴더)** 은 저장소에 없다(용량·학습 산출물). 로봇에 직접 두고 `model_path` 로 지정한다.
  기본값 `/home/pinky/yolo_drive/best_ncnn_model`. 클래스: `left_lane`, `right_lane`, `crosswalk`, `cross_lane`.

## 빌드

```bash
cd ~/pinky_pro && source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_autonomous
source install/setup.bash
```
저장소를 `src/` 아래 어디에 두어도 된다 (예: `~/pinky_pro/src/pinky_autonomous`). `pinky_pro` 와 같은 `src/` 아래에 있으면 안 되는 것은 없지만,
**같은 이름의 패키지가 두 군데 있으면 colcon 이 빌드를 거부**하므로 `pinky_pro/pinky_autonomous` 는 지운다.

## 실행

```bash
# 관제 PC (교차로 복귀를 쓸 때. 로봇을 출발 위치에 세운 채 먼저)
ros2 launch pinky_autonomous control_pc_slam.launch.xml
# 로봇 (/map 이 올라온 뒤)
ros2 launch pinky_autonomous autonomous_drive.launch.xml model_path:=/home/pinky/yolo_drive/best_ncnn_model
# 관제 PC (선택) 주행 영상 저장 → ~/pinky_recordings/
ros2 launch pinky_autonomous video_recorder.launch.xml
```
복귀 기능을 끄고 차선 추종만 하려면 `enable_return_home:=false` (SLAM 불필요). 주요 인자:

| 인자 | 기본 | 설명 |
|---|---|---|
| `drive_speed` | 0.08 | 기본 직진 속도 (m/s) |
| `yolo_conf`, `yolo_imgsz` | 0.50, 320 | 검출 신뢰도, 입력 크기 |
| `crosswalk_min_area`, `crosswalk_check_time`, `crosswalk_stop_dist` | 0.05, 1.5, 0.30 | 횡단보도 감지 면적, 확인 시간(s), 물체로 볼 거리(m) |
| `junction_conf`, `junction_advance_dist` | 0.35, 0.25 | 교차로 신뢰도, 정지 전 추가 전진(m) |
| `ultrasonic_stop_dist`, `obstacle_dist`, `collision_dist` | 0.10, 0.35, 0.05 | 정지/감속/충돌 거리(m) |
| `ultrasonic_scale` | 1.43 | 센서 보정(실측 0.7배 → 1/0.7) |
| `legacy_tracking` | true | true 면 예전 추종 방식(heading/라벨 보정 끔). 비교 시험용으로 false |
| `run_slam_on_robot` | false | 로봇에서 SLAM 을 직접 돌릴 때만 (보통 관제 PC 에서) |

전체 파라미터는 노드 `declare_parameter` 목록(약 50개)과 launch 파일 인자를 참고.

## FMS(다중 로봇 관제)와의 관계

`pinky_fms` 저장소의 `robot/pinky_fms_bringup/launch/robot_autonomous.launch.xml` 이 이 패키지의 launch 를 robot namespace 안에서 실행한다
(GUI 의 향후 `AUTO_DRIVE` 미션용). 이 패키지 자체는 FMS 에 의존하지 않는다.

## 알려진 한계

- 모델 경로 기본값이 로봇 계정(`/home/pinky/...`)에 맞춰져 있다. 다른 계정이면 `model_path` 를 지정한다.
- 교차로 복귀는 SLAM 위치 품질에 의존한다(시계 동기화 필수, 로봇 SLAM 은 부하가 커서 관제 PC 권장).
- 자동 시험이 없다(ament lint 만 선언). 주행 로직은 실물 코스에서 시험했다.
