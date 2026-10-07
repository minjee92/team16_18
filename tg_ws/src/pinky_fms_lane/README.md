# pinky_fms_lane — 차선 주행 관제

Nav2 대신 카메라 차선 인식으로 Pinky 2대를 GUI 에서 클릭한 목표까지 보낸다.
기존 `pinky_fms`·`pinky_autonomous` 는 수정하지 않고 상속·래퍼·launch 리맵만 쓴다.

| 폴더 | 어디서 | 내용 |
|---|---|---|
| `control_pc/pinky_fms_lane` | 관제 PC | 코스 모델(`course.py`), 코스 파일(`course/`), 도구: `draw_course`, `course_tape`, `make_lane_map`, `send_initial_pose`, `record_course_points` |
| `robot/pinky_fms_lane_robot` | 로봇 | 위치 추정만 띄우는 launch(`robot_localization`), 녹화용 카메라 화면(`robot_camera_view`), 통신 설정 스크립트(`fms_robot_env.sh`) |

코스 좌표의 출처와 우선순위: **로봇 기록(robot) > 줄자(tape) > 추정(estimate)**.
`course/*.course.yaml`(추정·설계값) → `*.tape.yaml`(줄자 원본 측정, 사람이 씀) → `*.robot.yaml`(기록 도구가 씀) 순으로 덮어쓴다.

---

## 실물 첫날 실행 가이드: 위치 추정 확인 + 코스 좌표 기록 + 카메라 녹화

목표: 로봇 1대(amr_01)를 키보드로 몰아 코스 점 좌표를 기록하고, 그동안 차선 인식 화면을 녹화한다.
**이날은 차선 주행 노드를 띄우지 않는다.** 로봇은 키보드로만 움직인다.

### 터미널 한눈에 보기 (실행 순서)

| 순서 | 터미널 | 위치 | 하는 일 |
|---|---|---|---|
| 1 | P1 관제 스택 | 관제 PC | rosbridge·백엔드·웹 → GUI 에서 로봇 추가, **ON** (bringup) |
| 2 | R1 위치 추정 | 로봇 SSH | map_server + AMCL (Nav2 주행 노드 없음) |
| 3 | P2 초기 위치 | 관제 PC | `send_initial_pose` 로 R1 위치를 AMCL 에 알리고 맞는지 확인 |
| 4 | R2 카메라 화면 | 로봇 SSH | `robot_camera_view` (차선 인식 화면만, 로봇을 움직이지 않음) |
| 5 | P3 영상 저장 | 관제 PC | `video_recorder` 로 mp4 저장 |
| 6 | R3 키보드 조종 | 로봇 SSH | `teleop_twist_keyboard` |
| 7 | P4 좌표 기록 | 관제 PC | `record_course_points` |

아래에서 `<…>` 는 자기 값으로 바꾼다. 비밀번호는 어디에도 적지 않는다 (SSH 는 직접 입력하거나 GUI 의 키 등록을 쓴다).

### 0. 한 번만: 빌드와 로봇에 저장소 받기

관제 PC:
```bash
export FMS_WS=<저장소>/tg_ws                      # 예: ~/dev_ws/team16_18/tg_ws
cd $FMS_WS && source /opt/ros/jazzy/setup.bash && source ~/pinky/install/setup.bash
colcon build
```
로봇 (SSH): 저장소를 `~/team16_18` 에 받아 그 안의 `tg_ws` 에서 **이 패키지만** 빌드한다.
`~/pinky_pro/src` 는 팀원 배포 위치라 아무것도 넣지 않는다 (bringup·pinky_autonomous 는 거기 빌드된 것을 underlay 로 쓴다).
```bash
git clone <저장소 주소> ~/team16_18                      # 이미 있으면: cd ~/team16_18 && git pull
cd ~/team16_18 && git checkout <작업 브랜치>              # 예: feat/tg-fms-lane
cd ~/team16_18/tg_ws && source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/setup.bash
colcon build --packages-select pinky_fms_lane_robot \
  --packages-ignore pinky_fms_interfaces pinky_fms_core pinky_fms_traffic pinky_fms_bringup pinky_fms_sim \
                    pinky_autonomous pinky_fms_lane
ros2 pkg prefix teleop_twist_keyboard || sudo apt install ros-jazzy-teleop-twist-keyboard
```
- `--packages-ignore` 로 나머지 패키지(관제 PC·시뮬용, 그리고 `~/pinky_pro` 에 이미 있는 것과 이름이 같은 것)를 이 작업공간에서 아예 빼서, `~/pinky_pro` 에 빌드된 팀원 패키지를 그대로 쓴다. 저장소에 패키지가 늘면 `colcon list --names-only` 로 확인해 목록에 더한다.
- 정상: `Summary: 1 package finished`. `~/pinky_pro` 에 `pinky_fms_bringup`, `pinky_autonomous`, `pinky_navigation` 이 이미 빌드돼 있어야 한다 (GUI ON 과 기존 자율주행이 되던 로봇이면 있음).
- 지도는 저장소 안의 `~/team16_18/tg_ws/src/pinky_fms/maps/mission4_3_clean_1cm.yaml` 을 쓴다 (따로 복사하지 않음).

### 매 터미널 환경 설정

**관제 PC 터미널 (P1~P4) 마다:**
```bash
export FMS_WS=<저장소>/tg_ws
source /opt/ros/jazzy/setup.bash && source ~/pinky/install/setup.bash && source $FMS_WS/install/setup.bash
source $FMS_WS/src/pinky_fms/control_pc/fms_env.sh
export MAP=$FMS_WS/src/pinky_fms/maps/mission4_3_clean_1cm.yaml
```
정상: `[fms_env] ROS_DOMAIN_ID=16, DDS 탐색=unicast (RMW=rmw_cyclonedds_cpp)`

**로봇 SSH 터미널 (R1~R3) 마다 — GUI 에서 ON 한 뒤에:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/setup.bash && source ~/team16_18/tg_ws/install/setup.bash
source $(ros2 pkg prefix pinky_fms_lane_robot)/share/pinky_fms_lane_robot/scripts/fms_robot_env.sh amr_01
```
정상: `[fms_robot_env] amr_01: ROS_DOMAIN_ID=16 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/fms_cyclonedds_amr_01.xml`

- 이 스크립트는 **백엔드 ON 이 띄운 bringup 프로세스의 환경을 그대로 복사**한다 (`/tmp/fms_amr_01.pid` 의 프로세스에서 `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI` 를 읽음). 그래서 robot_localization·camera_view·teleop 이 bringup 과 똑같은 도메인·DDS(유니캐스트 피어 설정)로 통신한다.
- `bringup 이 없습니다` 가 나오면 → GUI 에서 ON 을 먼저 하거나, 로봇 ID 를 확인한다. ON 을 다시 하면 이 줄을 다시 source 한다.
- 손으로 `export ROS_DOMAIN_ID=…` 만 하면 유니캐스트 설정(CYCLONEDDS_URI)이 빠져서 bringup·관제 PC 와 서로 안 보일 수 있다.

### 1. P1 관제 스택 → GUI 에서 ON

로봇을 R1(꼬리 끝, 출구 = 서쪽을 봄)에 놓고 전원을 켠 뒤:
```bash
$FMS_WS/src/pinky_fms/scripts/run_real.sh
```
정상: `[real] 실행 중: http://localhost:8080`. 브라우저에서 로봇 추가(ID `amr_01`, IP, 사용자) → **ON**.
GUI 의 목표 지정·전체 시작은 누르지 않는다 (이날은 Nav2 를 쓰지 않는다).

확인 (P2 터미널):
```bash
ros2 topic list | grep /amr_01/
```
정상: `/amr_01/scan`, `/amr_01/odom`, `/amr_01/tf` 등이 보인다.
문제: 아무것도 안 보이면 → P2 에서 `fms_env.sh` 를 source 했는지, GUI 로그(`/tmp/fms_real_backend.log`)와 로봇의 `/tmp/fms_amr_01.log` 확인.

### 2. R1 위치 추정 (로봇)

```bash
ros2 launch pinky_fms_lane_robot robot_localization.launch.xml namespace:=amr_01 \
  map:=$HOME/team16_18/tg_ws/src/pinky_fms/maps/mission4_3_clean_1cm.yaml
```
정상 출력 (마지막 줄이 중요):
```
[map_io]: Read map /home/pinky/team16_18/tg_ws/src/pinky_fms/maps/mission4_3_clean_1cm.pgm: 280 X 230 map @ 0.01 m/cell
[amr_01.lifecycle_manager_localization]: Activating amcl
[amr_01.lifecycle_manager_localization]: Managed nodes are active
```
AMCL 이 초기 위치를 기다린다는 경고는 다음 단계에서 사라진다.
문제:
- `Managed nodes are active` 가 안 나옴 → 지도 경로(`~/team16_18/tg_ws/src/pinky_fms/maps/` 의 .yaml 과 같은 폴더의 .pgm) 확인.
- `package 'pinky_navigation' not found` 또는 `pinky_fms_lane_robot` 을 못 찾음 → `~/pinky_pro/install` 과 `~/team16_18/tg_ws/install` 을 둘 다 source 했는지 확인.
- 로봇에 Nav2(`robot_nav.launch.xml`)가 떠 있으면 끈다 (AMCL 이 두 개가 된다).

### 3. P2 초기 위치 (관제 PC)

```bash
ros2 run pinky_fms_lane send_initial_pose amr_01:R1 --map $MAP
```
정상 출력 (예):
```
[amr_01] 출발점 R1 (출처 추정 estimate): x 0.783  y 0.253  yaw 180°
[amr_01] 스캔 일치율 100% (빔 360개 중 끝이 벽 5 cm 안)
[amr_01]   세로: 지도 크기 차이 +0.0 cm, 로봇 위치 어긋남 -0.5 cm / 가로: 지도 크기 차이 +0.0 cm, 로봇 위치 어긋남 +0.5 cm
[amr_01] → OK
```
문제 (`→ 확인 필요` 일 때):
- `스캔 일치율 17%` 처럼 낮고 "원인을 나눌 수 없음" → 로봇이 R1 에 **서쪽을 보고** 놓였는지 확인하고 다시 실행.
- `세로: 지도 크기 차이 +8 cm 근처` → 로봇 배치가 아니라 지도 크기 문제 (계획서 "맵 크기 차이"). 일치율 기준(`--min-match`)을 낮춰 진행하고 결과를 기록해 둔다.
- `initialpose 를 받는 노드(AMCL)가 없음` → 2단계와 P2 의 `fms_env.sh` 확인.
- 겹침 그림이 `$FMS_WS/outputs/result_initpose_amr_01_*.png` 에 저장된다.

### 4. R2 카메라 화면 (로봇) — 키보드 조종 중 녹화용

pinky_autonomous 가 카메라를 독점하므로, 녹화하려면 **autonomous_drive 대신** 이 launch 를 띄운다.
`camera_view` 는 pinky_autonomous 를 상속해서 같은 YOLO·HUD 를 그리지만 **cmd_vel 을 내지 않는다**.
```bash
ros2 launch pinky_fms_lane_robot robot_camera_view.launch.xml namespace:=amr_01 enable_lcd:=false
```
정상 출력:
```
[camera_view]: ✅ YOLO 모델 로드 완료.
[camera_view]: 📸 카메라 시작됨 (video, 안정화 1.0s).
[camera_view]: 🎥 녹화용 차선 인식 화면만 냅니다 (cmd_vel 을 내지 않음: 로봇은 키보드로 조종)
```
확인 (P2):
```bash
ros2 topic info /amr_01/cmd_vel      # 이 시점: Publisher count: 0  (키보드를 켠 뒤: 1)
```
문제:
- 카메라 열기 실패 → 다른 카메라 노드(autonomous_drive, 이전에 띄운 camera_view)가 남아 있는지 `ps aux | grep -E '[a]utonomous_drive|[c]amera_view'` 로 확인하고 끈다.
- 모델 경로 오류 → `model_path:=<NCNN 모델 폴더>` 를 준다.
- LCD 를 쓰려면 pinky_emotion 을 끈 뒤 `enable_lcd:=true` (pinky_autonomous 와 같은 전제).
- HUD 의 `DRIVE`/`v`/`w` 는 "차선 노드였다면 냈을 명령"이다. 실제 움직임은 키보드다.

### 5. P3 영상 저장 (관제 PC)

```bash
ros2 run pinky_autonomous video_recorder --ros-args -r __ns:=/amr_01 \
  -p save_dir:=$FMS_WS/outputs/teleop_amr_01_$(date +%Y%m%d_%H%M%S)
```
정상: `🎥 Recording Video → …/outputs/teleop_amr_01_<시각>/<시각>.mp4`. Ctrl+C 로 끝내면 파일이 닫힌다.
문제: 파일 크기가 안 늘어남 → `ros2 topic hz /amr_01/camera/compressed` 로 영상이 오는지 확인 (구독자가 있어야 로봇이 영상을 보낸다).
참고: 640×480 JPEG 이라 Wi-Fi 로 약 0.3~0.6 MB/s 를 쓴다 (추정). 녹화가 끝나면 P3 을 끈다.
영상에서 볼 것: J 를 세 방향(꼬리→고리 직진, 고리→꼬리 좌회전, 고리→고리 우회전)으로 지날 때 `cross_lane` 이 잡히는지, C1 이 J 근처에서 횡단보도로 잡히는지, 꼬리 끝에서 U턴한 뒤 left/right 라벨.

### 6. R3 키보드 조종 (로봇)

키보드 조종은 **로봇 SSH 터미널**에서 띄운다 (로봇 안 통신이라 Wi-Fi 에서 정지 명령이 유실되지 않는다. SSH 는 늦게라도 키를 전달한다).
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amr_01/cmd_vel -p speed:=0.08 -p turn:=0.5
```
- `i` 전진, `,` 후진, `j`/`l` 제자리 회전, **`k` 또는 스페이스 = 정지**. `z` 로 속도 10% 줄이기.
- 기본값(0.5 m/s)은 이 코스에 너무 빠르다. 위처럼 0.08 m/s 로 시작한다.
- **로봇 base 에는 cmd_vel 시간 초과가 없다**: 마지막 명령을 계속 따른다. 점 위에 오면 반드시 `k` 로 세운다. 터미널이 멈추면 로봇을 들어 올리거나 전원을 끈다.

확인: `ros2 topic info /amr_01/cmd_vel` → `Publisher count: 1` (teleop 하나뿐).

### 7. P4 좌표 기록 (관제 PC)

```bash
ros2 run pinky_fms_lane record_course_points amr_01 --map $MAP
```
- 기본 코스는 `$FMS_WS/src/.../course/mission4_3_clean_1cm.course.yaml` (소스 폴더)이고, 기록은 같은 폴더의 `mission4_3_clean_1cm.robot.yaml` 에 저장된다. 덮어쓰기 전에 `*.bak.<시각>` 백업을 만든다.
- 이 도구는 로봇을 움직이지 않는다. R3 에서 몰고, 세운 뒤 여기서 Enter.

**기록 순서** (코스의 `record_order`, R1 에서 출발해 꼬리를 나와 고리를 시계방향으로 한 바퀴 → 꼬리로 돌아옴):

| 순서 | 점 | 어디 |
|---|---|---|
| 1 | `T_D` | 꼬리 아랫길 왼쪽 굽이 끝 |
| 2 | `T_C` | 꼬리 왼쪽 세로 길 아래 |
| 3 | `T_B` | 꼬리 왼쪽 세로 길 위 |
| 4 | `T_A` | 꼬리 윗길 왼쪽 굽이 시작 |
| 5 | `W_TAIL` | 꼬리 윗길 대기 지점 (C1 서쪽) |
| 6 | `C1` | 횡단보도 C1 가운데 |
| 7 | `J` | T자 갈림길 가운데 |
| 8 | `NE` | 고리 오른쪽 위 모서리 |
| 9 | `C2` | 횡단보도 C2 가운데 |
| 10 | `SE` | 고리 오른쪽 아래 모서리 (빨간 테이프) |
| 11 | `R2` | SE 와 같은 자리, **서쪽을 보게 세우고** 기록 (방향도 저장) |
| 12 | `SW` | 고리 왼쪽 아래 모서리 |
| 13 | `W_LOOP` | 고리 왼변 대기 지점 (J 아래) |
| 14 | `R1` | 꼬리로 돌아와 꼬리 끝에서 **서쪽을 보게** 세우고 기록 |

- R1 이 마지막인 이유: 처음에는 AMCL 이 R1 값으로 시작하므로 그 자리를 기록하면 같은 값이 다시 나온다. 벽을 보며 한 바퀴 돈 뒤에 기록한다.
- `T_END`(꼬리 끝 벽 면)는 로봇이 설 수 없어 기록하지 않는다 (줄자 값 사용).
- 길 폭은 이 도구로 기록하지 않는다. 다시 재면 `course/mission4_3_clean_1cm.tape.yaml` 의 원본 값과 `params.lane_width` 를 손으로 고친다.
- 입력: Enter = 안내한 점, 점 이름 = 그 점, `l` = 목록, `s` = 건너뛰기, `q` = 끝.

점마다 정상 출력 (예):
```
다음: T_D >
  T_D 측정 중: 멈춤 확인 → 제자리 갱신 3번 ...
  T_D: x 0.198  y 0.255  yaw 180°  (갱신 3번, 흔들림 0.4 cm / 0.3°)
  표준편차 위치 2.1 cm, 방향 3.0°, 스캔 일치율 92%, 길 가운데 선까지 0.5 cm
  지금 값 (추정 estimate): x 0.197  y 0.253 → 차이 0.2 cm
  저장하려면 Enter (취소 n) >
  저장: …/course/mission4_3_clean_1cm.robot.yaml (이전 파일 백업 mission4_3_clean_1cm.robot.yaml.bak.<시각>)
```
경고(⚠)가 나오면 `y` 를 입력해야만 저장된다. 경고별로 할 일:

| 경고 | 할 일 |
|---|---|
| `로봇이 … 멈추지 않음` | 키보드 `k` 로 세운 뒤 다시 Enter |
| `위치 표준편차 … > 5.0 cm` | 차선을 따라 조금 더 몰고 와서(AMCL 이 벽을 보며 수렴) 다시 |
| `갱신 사이 위치가 … 움직임` | 트인 구간에서 흐르는 중. 벽이 가까운 쪽으로 조금 옮겨 다시, 또는 `n` |
| `스캔 일치율 …% < 70%` | 위치가 틀렸을 수 있음. 3단계 초기 위치부터 다시 확인 |
| `길 가운데 선에서 … 떨어짐` | 로봇을 두 테이프 가운데에 세운다 |
| `지금 값(tape)과 … cm 차이` | 점 이름이 맞는지, 로봇이 그 점에 있는지 확인. 맞는데도 크면 줄자·지도 차이 → 저장하고 기록해 둔다 |
| `방향이 지금 값과 …° 다름` | R1·R2 는 서쪽을 보게 세운다 |

끝나면 (P4):
```bash
cd $FMS_WS && colcon build --packages-select pinky_fms_lane && source install/setup.bash    # 기록 파일을 설치 폴더에 반영
ros2 run pinky_fms_lane draw_course --map $MAP        # 점마다 출처(로봇 기록 = 검정)를 그림으로 확인
ros2 run pinky_fms_lane make_lane_map --map $MAP      # GUI 표시용 차선 지도 다시 만들기 (AMCL·Nav2·시뮬레이션에는 쓰지 않음)
```
정상: `[draw_course] 점 출처: 로봇 기록 14 · 줄자 … · 추정 …`. 기록 파일(`*.robot.yaml`)을 커밋할지는 결과를 보고 정한다.

### 끝내는 순서

1. R3 키보드: `k` 로 세운 뒤 Ctrl+C
2. P4 기록 도구: `q`
3. P3 영상 저장: Ctrl+C (mp4 가 닫힘)
4. R2 카메라 화면: Ctrl+C (카메라 점유가 풀림)
5. R1 위치 추정: Ctrl+C
6. GUI 에서 OFF → P1: Ctrl+C

### 주의: 키보드 조종과 차선·주행 노드를 동시에 켜지 않는다

- 키보드 조종 중에는 cmd_vel 을 내는 노드가 **teleop 하나뿐**이어야 한다. 아래를 같이 띄우지 않는다:
  - `pinky_fms_bringup robot_autonomous.launch.xml` / `pinky_autonomous autonomous_drive.launch.xml` (차선 주행)
  - 앞으로 만들 `robot_lane.launch.xml` (차선 실행기 + 게이트)
  - `pinky_fms_bringup robot_nav.launch.xml` (Nav2)
- 카메라 녹화는 cmd_vel 을 내지 않는 `robot_camera_view` 만 쓴다. 카메라를 독점하므로 차선 노드와 둘 중 하나만 뜬다.
- 키보드를 켜기 전과 후에 확인: `ros2 topic info /amr_01/cmd_vel` → 켜기 전 `Publisher count: 0`, 켠 뒤 `1`. 2 이상이면 키보드를 `k` 로 세우고 다른 노드부터 끈다.
- 차선 주행을 시험하려면 키보드(R3)를 먼저 끄고, camera_view(R2)도 끈 뒤 띄운다.

---

## 시험 (관제 PC)

```bash
cd $FMS_WS/src/pinky_fms_lane/control_pc/pinky_fms_lane && python3 -m pytest test        # 코스·줄자·지도·기록 도구 계산
cd $FMS_WS/src/pinky_fms_lane/robot/pinky_fms_lane_robot && python3 -m pytest test        # camera_view (가짜 카메라·모델)
FMS_WS=$FMS_WS $FMS_WS/src/pinky_fms_lane/robot/pinky_fms_lane_robot/tests/run_localization_test.sh      # 도메인 92
FMS_WS=$FMS_WS $FMS_WS/src/pinky_fms_lane/control_pc/pinky_fms_lane/tests/run_initial_pose_test.sh       # 도메인 95
FMS_WS=$FMS_WS $FMS_WS/src/pinky_fms_lane/control_pc/pinky_fms_lane/tests/run_record_test.sh             # 도메인 96
```
ROS 격리 시험은 가짜 로봇(`fake_base.py`)을 쓰고 이 PC 안에서만 통신한다. 먼저 `source ~/pinky/install/setup.bash` (pinky_navigation).
