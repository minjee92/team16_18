# 회색 바닥 / 흰색 차선 코스

`worlds/mission4_3_textured.sdf`는 승인한 생성 PNG를 기존 경기장에 배치한 월드다.
원본 `mission4_3_clean_1cm.sdf`의 13개 벽, `block_0`, ground collision,
physics 설정은 유지한다. 추가된 것은 z=0.001m의 바닥 visual뿐이다.

## 좌표 정합

PGM 전체는 280×230px, resolution=0.01m/px, origin=(-0.567,-1.056,0)이다.
경기장 사각형은 이미지 좌표 x=44.5~275.5, y=9.5~130.5 부근이다.
따라서 PNG 전체를 2.8×2.3m에 늘려 놓으면 맞지 않는다.

승인 PNG는 1711×919px이며 `worlds/textures/course_gray_white_lanes.png`에 보존했다.
`config/course_texture.json`에는 PNG checksum과 실제 픽셀에서 측정한 외곽,
T자 벽, 장애물 위치가 있다. 생성 이미지가 구간별로 조금씩 변형되어 있어
하나의 affine 변환 대신 **단조 증가하는 x/y 구간별 mesh**로 대응시킨다.
이미지 bytes는 변경하지 않고 mesh position과 UV만 보정한다.

pixel 좌표는 좌상단 edge=(0,0), pixel center=(column+0.5,row+0.5)다.
map 좌표로 변환할 때 이미지 y를 뒤집고 resolution과 origin yaw를 적용한다.
기준점과 메시가 맞는 것은 실물 차선 폭·카메라 외부 파라미터까지 측량되었다는 뜻은 아니다.
테이프 폭과 촬영 조건은 실제 주행 데이터로 추가 비교해야 한다.

## 월드 재생성

다음 명령은 **새 출력 경로**를 요구한다. 기존 파일을 조용히 덮어쓰지 않는다.
SDF 옆의 `_assets` 폴더에 PNG 복사본, COLLADA mesh, 정합 정보가 생성된다.
배포할 때 SDF와 해당 폴더를 함께 옮긴다.

```bash
cd <workspace>/src/pinky_fms/simulation/pinky_fms_sim
python3 tools/add_ground_texture.py \
  worlds/mission4_3_clean_1cm.sdf ../../maps/mission4_3_clean_1cm.yaml \
  worlds/textures/course_gray_white_lanes.png config/course_texture.json \
  /tmp/course-generated/mission4_3_textured.sdf
python3 -m unittest discover -s tests -v
```

이 저장소에서는 `<workspace>`가 `team16_18/tg_ws`다.
필요 Python 의존성은 NumPy, PyYAML, Pillow다.

## 빌드와 실행

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 조합을 사용한다.
외부 소스는 `pinklab-art/pinky_pro`의
`014a09f289e6988894fbffde2624fdb1e257815b`를 기준으로 한다.
`pinky_description`, `pinky_gz_sim`을 workspace에서 찾을 수 있어야 한다.
다른 패키지의 동명 복사본을 중복 배치하지 않는다.

```bash
sudo apt-get install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  python3-numpy python3-yaml python3-pil
source /opt/ros/jazzy/setup.bash
cd <workspace>
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --packages-select pinky_description pinky_gz_sim pinky_fms_bringup pinky_fms_sim \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
export ROS_DOMAIN_ID=91 GZ_PARTITION=tg_gazebo_course
ros2 launch pinky_fms_sim sim_world.launch.xml
```

다른 터미널에서도 ROS와 workspace를 source한 뒤:

```bash
export ROS_DOMAIN_ID=91 GZ_PARTITION=tg_gazebo_course
ros2 launch pinky_fms_sim sim_robot.launch.xml \
  namespace:=amr_01 x:=1.80 y:=0.10 yaw:=3.14159 nav:=false
```

`nav:=false`는 Nav2가 cmd_vel을 발행하지 않게 한다. 기본값 true는 기존 FMS와의
호환을 위한 것으로, 사용하려면 원래 Nav2 의존성과 지도를 준비해야 한다.
`camera:=true`가 기본이며 `/amr_01/camera/image_raw`를 bridge한다.
외부 모델의 카메라 설정은 1280×720, 30Hz, 기본 tilt 8도다.

화면 없는 머신에서는 월드 launch에 `gui:=false headless_rendering:=true`를 준다.
GPU가 없는 검증 머신에서는 Mesa 소프트웨어 렌더링을 사용한다.

## 기능 검증

아래 검증은 **cmd_vel을 실제 발행한다**. 고립된 시뮬레이션에서 다른 controller를
끄고 위의 시작 위치로 spawn한 뒤 실행한다. 실제 로봇 domain에서는 실행하지 않는다.

```bash
ros2 run pinky_fms_sim smoke_sim.py --robot amr_01 --output /tmp/fms-smoke-001
```

clock, LiDAR, odom, camera, wheel joint_states 수신을 기다린 뒤 0.06m/s로
1.5 simulation seconds 전진하고 정지한다. odom 이동량과 정지 후 drift를 확인하고
실제 camera 전후 이미지와 result.json을 새 디렉터리에 기록한다.
메시지 미수신, 빈 scan, 낮은 image contrast, 이동 실패 및 정지 실패는 오류로 종료한다.
이 검증은 YOLO 인식 정확도나 자율주행 완료를 검증하지 않는다.

## YOLO 차선 추종 실행

`sim_lane_follow.py`는 ROS 영상을 받아 기존 `pinky_autonomous`의 `LaneTracker`,
마스크 처리와 `_compute_command`를 직접 호출한다. 실물 패키지는 수정하지 않는다.
Picamera2와 sonar를 요구하는 실물 노드 전체를 실행하는 방식은 아니다.
Gazebo 영상은 이미 정방향이므로 실물 카메라의 180도 회전은 적용하지 않는다.

외부 `pinky_pro`의 `pinky_interfaces`와 저장소의 `pinky_autonomous`도 빌드한다.
아래 명령은 Ubuntu 24.04 ROS Jazzy 환경에서 실행한다.

```bash
sudo apt-get install python3-venv
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --executor sequential --packages-select pinky_interfaces pinky_autonomous
source install/setup.bash
python3 -m venv --system-site-packages /tmp/pinky-yolo-venv
source /tmp/pinky-yolo-venv/bin/activate
python -m pip install torch==2.14.1+cpu torchvision==0.29.1+cpu --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r src/pinky_fms/simulation/pinky_fms_sim/requirements-lane.txt
export ROS_DOMAIN_ID=91 GZ_PARTITION=tg_gazebo_course
ros2 run pinky_fms_sim sim_lane_follow.py \
  --model /absolute/path/260928_yolon_best.pt --output /tmp/lane-observe-001
```

기본값은 관찰만 수행한다. `--drive --duration 4`를 추가하면 4 simulation seconds 동안
cmd_vel을 발행한다. 새 출력 디렉터리에 검출 이미지와 조향·odom 기록을 남긴다.
NCNN은 `--model /absolute/path/best_ncnn_model`로 지정한다.
전방 LiDAR 0.12m 이내, 차선 유실, 센서 중단 시 정지 처리를 한다.
이는 실물 sonar 로직 검증을 대신하지 않는다.

GPU 없는 머신의 월드 실행 터미널에서는 다음을 먼저 설정한다.

```bash
export LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless
```

모델 바이너리는 저장소에 포함하지 않는다. 전달받은 PT SHA256:
`353e6c17d67fbb8652568a3ca4bc4e37b9f77f6618f286be445371c809905753`.
ZIP 내부 PT와 별도 PT는 동일하며 클래스는 cross_lane, crosswalk, left_lane, right_lane이다.

## 검증 결과와 한계

2026-10-08, Ubuntu 24.04 컨테이너 / ROS Jazzy / Gazebo 8.15.0 / Mesa CPU 렌더링에서 확인했다.

- 좌표 변환·벽 보존 등의 단위 테스트 9개 통과.
- Pinky spawn, clock, LiDAR 640개 유효 측정값, odom, joint_states, 1280×720 영상과 CameraInfo 수신.
- 0.06m/s 명령 시험: 8.112cm 이동, 정지 후 odom drift 0m.
- PT와 NCNN 모두 실제 Gazebo 전후 영상 2장에서 좌우 차선 마스크 검출.
- PT 주행 4초: 33/33 프레임 유효, BOTH 17 / SINGLE 16, 이동 30.797cm, guard stop 0회.

[검증 이미지와 원본 JSON](docs/validation/README.md)을 함께 보관했다.
NCNN은 저장된 영상 추론만 검증했다. 전체 코스 완주, 교차로·횡단보도 상태 전환,
복귀, Nav2 연동, 실제 로봇에서의 동일 성능은 아직 검증하지 않았다.

렌더링 중 흰색 바닥만 보였던 원인은 COLLADA의 material/UV binding 누락이었다.
정상 material binding과 normal을 추가해 실제 카메라로 확인했다.
초기 추론 중 ROS callback이 지연된 문제는 모델 warmup과 별도 수신 executor로 해결했다.
월드 재시작 시 launch 부모만 종료하면 Gazebo 자식이 남을 수 있다. 본 검증은 전용
컨테이너를 재시작해 중복 월드를 제거한 뒤 단일 월드에서 다시 수행한 결과다.

후속 90초 주행에서는 약 1.19m 진행 후 차선 유실로 정지해 완주하지 못했다.
횡단보도·복귀의 제어된 입력 검사 5개는 통과했으며, 자세한 제한과 기록은
[후속 검증](docs/validation/extended/README.md)을 참고한다.
