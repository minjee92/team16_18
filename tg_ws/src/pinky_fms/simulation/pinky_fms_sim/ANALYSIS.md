# Gazebo 코스 작업: 변경 전 구조 분석

분석 기준: `feat/tg-pinky-fms`의 `883a8dc5e70076f08bbe751d17ba5d9c1ab06501`에서 분리한
`feat/tg-gazebo-course`. Claude의 미커밋 작업 및 이후 커밋은 포함하지 않는다.
아래는 구현 전 조사 기록이다. 이후 구현·실행 결과와 현재 사용법은 [TEXTURE.md](TEXTURE.md)를 따른다.

## Workspace

저장소에는 `jh_ws`, `mj_ws`, `sj_ws`, `tg_ws`가 있다. 현재 브랜치에서 앞의 세
디렉터리는 자리표시자이고 ROS 패키지는 `tg_ws/src`에만 있다. build/install overlay는 없다.

| 패키지 | 역할 | 주요 외부 의존성 |
|---|---|---|
| pinky_fms_interfaces | 미션/로봇 상태 메시지 | rosidl, geometry_msgs |
| pinky_fms_core | 미션, 조정, mock robot | rclpy, Nav2 메시지 |
| pinky_fms_traffic | core 기반 충돌 방지 | core, NumPy, SciPy |
| pinky_fms_bringup | 실물 namespace 및 Nav2 래퍼 | pinky_bringup, pinky_navigation |
| pinky_fms_sim | 월드, 로봇 생성, bridge | pinky_description, pinky_gz_sim, ros_gz |
| pinky_autonomous | YOLO 차선 추종, 복귀 | pinky_interfaces, Picamera2, YOLO 모델 등 |

`pinky_fms_lane`은 이 커밋에는 없다. 외부 `pinky_pro` 모델/센서 패키지도 포함되지 않는다.
core/traffic/autonomous/bringup 파일은 분석만 하며 이 작업에서 수정하지 않는다.

## 다섯 파일과 실행 흐름

1. `tools/map_to_world.py`: YAML에서 상대 PGM 경로를 읽는다. P5 픽셀을 위아래로
   뒤집어 행 증가 방향을 world +y로 맞춘다. `occupied_thresh`를 넘는 점유 픽셀의
   가로 run 및 같은 폭의 연속 행을 합쳐 box collision/visual을 만든다.
   `--block`은 PGM 밖의 실물 장애물을 추가한다. 기본 벽 높이는 0.22m.
2. `worlds/mission4_3_clean_1cm.sdf`: `fms_course` 월드. physics, sensors(ogre2),
   user commands, scene broadcaster, IMU 시스템, 방향광, 단색 평면, 13개 벽,
   추가 `block_0`으로 구성된다. ground collision과 visual은 모두 20×20m다.
3. `launch/sim_world.launch.xml`: 외부 모델 resource path를 설정하고 Gazebo server를
   `-r -s`로 실행한다. GUI는 별도 `-g` 프로세스다. headless rendering 인자가 있고
   `/clock`을 Gazebo→ROS 방향으로 한 번 bridge한다.
4. `scripts/sim_urdf.sh`: 외부 `pinky_description/urdf/robot.urdf.xacro`를 생성한다.
   namespace가 붙은 Gazebo reference에서 일부 링크 접두어를 제거하고 LiDAR 노이즈를
   0.02→0.006m로 보정한다. 원본 외부 모델은 수정하지 않는다.
5. `launch/sim_robot.launch.xml`: 생성 URDF를 robot_state_publisher에 전달한다.
   기존 같은 이름의 모델 삭제 후 2초 timer로 z=0.05에 spawn한다.
   scan/odom은 Gazebo→ROS, cmd_vel은 ROS→Gazebo, `/tf`는 Gazebo→namespaced ROS로
   bridge한다. 가짜 배터리를 내보내고 4초 뒤 실물 Nav2/AMCL 래퍼를 실행한다.

상위 `scripts/run_sim.sh`는 월드, rosbridge, mission/traffic, backend, web을 묶는다.
백엔드가 `config/robots.sim.yaml`에 따라 sim_robot launch를 실행한다.
이번 코스 검증에는 FMS 전체를 띄우기보다 월드와 로봇 launch를 직접 사용하는 것이 적절하다.
Nav2와 lane controller가 동시에 cmd_vel을 발행하지 않도록 별도 실행 선택이 필요하다.

## 현재 수치 검증

현재 머신: Debian 13, ROS/Gazebo 없음. 따라서 아래 결과는 Python/XML/geometry 검증이며
Gazebo runtime, ROS launch 해석 및 센서 검증 결과가 아니다.

- PGM: 280×230px, resolution=0.01m/px → 2.8×2.3m.
- YAML origin: (-0.567, -1.056, yaw=0).
- map 이미지 범위: x=[-0.567, 2.233], y=[-1.056, 1.244].
- 점유 픽셀 1,680개를 기존 SDF의 벽 13개가 누락/중복 없이 정확히 표현한다.
- wall pose와 collision 크기를 역산한 cell boundary가 YAML 좌표와 일치한다.
- `block_0`은 PGM에서 생성되지 않은 추가 벽이다. 중심=(-0.36,-0.39),
  크기=(0.48,1.42,0.22)m. 월드 재생성 시 이 벽을 별도로 보존해야 한다.
- launch XML 2개는 XML parser로 정상 읽혔다. shell 파일은 구문 검사만 수행했다.
- protected 코드와 기존 월드 변경 없음.

origin yaw=0인 현재 지도는 일치하지만 generator는 일반적인 nonzero yaw를 무시한다.
현재 PGM reader는 P5/8-bit 가정을 검사하지 않으며 잘못된 입력에 명확한 오류가 없다.
셸 URDF pipeline은 pipefail이 없어 xacro 실패가 마지막 sed의 성공으로 가려질 수 있다.
timer는 spawn 완료나 Nav2 준비를 증명하지 않는다. 실제 서비스/토픽으로 확인해야 한다.

## 목표별 현재 상태

| 목표 | 구현 상태 | 실제 실행 검증 |
|---|---|---|
| PGM 벽 유지 | 기존 13개 벽 존재, geometry 정확 | Python 수치 검증 통과 |
| PNG ground texture | 없음, 사용자 PNG 필요 | 미실행 |
| map/world 좌표 일치 | 현재 yaw=0 지도는 일치 | 수치 검증 통과; 렌더링 미실행 |
| Pinky Pro spawn | 외부 xacro + create launch 존재 | 외부 소스/ROS/Gazebo 필요 |
| LiDAR/odom/cmd_vel | bridge 선언 존재 | 토픽과 이동 미검증 |
| 전방 카메라 | reference 보정만 있고 bridge 없음 | 외부 모델 sensor/topic 조사 필요 |
| autonomous 시험 | 직접 Picamera2 입력 사용 | remapping만으로 실행 불가 |

## 카메라 연결의 중요한 제약

`pinky_autonomous/autonomous_drive_node.py`는 YOLO 모델을 읽고 `Picamera2()`를 생성하며
`capture_array()`로 영상을 받는다. `camera/compressed`는 영상 **출력** 토픽이다.
Gazebo image topic을 기존 ROS 구독으로 연결할 수 있는 입력 인자는 없다.
따라서 bridge 추가만으로 동일 노드를 시험할 수 있다고 주장하면 안 된다.

실물 코드를 유지하려면 simulation 패키지 안의 명시적인 카메라 입력 adapter 등의
별도 설계가 필요하다. fake Picamera2나 무조건 성공하는 센서값으로 오류를 숨기지 않는다.
실제 ROS 영상으로 동일 주행 로직을 시험하는 방법 및 초음파/LED 서비스/TF 의존성을
먼저 설계하고 설명해야 한다. NCNN 모델은 현재 저장소에 없다.

## 다음 단계의 입력과 검증 기준

사용자 코스 PNG와 그것이 PGM 전체 범위와 대응하는지 확인할 자료가 필요하다.
크기가 다른 PNG를 임의로 늘려 정합됐다고 처리하지 않는다. 코스 기준점 대응을 확인하고
필요하면 crop/회전/scale을 명시한다. ground texture는 collision을 바꾸지 않는 visual로
배치하여 기존 벽 및 추가 block을 유지한다.

외부 Pinky Pro 소스 URL/커밋을 받아 센서 정의를 조사한다. 호스트에 Ubuntu 전용 ROS deb를
섞지 않고 Jazzy 호환 runtime을 준비한다. 월드 시작, spawn, scan 유효 ranges,
odom/TF 변화, cmd_vel 이동/정지, 카메라 영상의 방향·색·정합을 순서대로 실제 검증한다.
명령의 성공 exit 또는 XML parse만으로 이 단계들을 통과했다고 처리하지 않는다.

## 런타임 준비 결과와 차단 원인

공식 `ros:jazzy-ros-base` 이미지를 내려받았다. 이미지 digest:
`sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`.
컨테이너에서 `ros2 pkg prefix rclpy`와 `command -v colcon`은 성공했다.
컨테이너 이름은 `tg-gazebo-course-runtime`이며 기존 월드를 `/tmp`로 복사했다.
Gazebo와 ros_gz는 아직 설치되어 있지 않다.

설치 명령은 다음과 같다. 아직 통과하지 않았다.

```bash
docker exec tg-gazebo-course-runtime bash -lc '
  export DEBIAN_FRONTEND=noninteractive
  apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-robot-state-publisher ros-jazzy-xacro \
    python3-numpy python3-yaml
'
```

`apt-get update`는 Ubuntu index를 받았지만 ROS 저장소의 InRelease 요청에서
프록시 `403 Forbidden`으로 실패했다. HTTPS 직접 요청도 CONNECT 403으로 실패했다.
현재 허용 도메인에 `packages.ros.org`가 없음을 확인했다. 서명 검증을 끄거나
비공식 패키지로 우회하지 않았다. 해당 도메인 추가를 환경 설정 초안에 저장했다.
초안 저장은 현재 runtime에 네트워크 정책을 적용하지 않는다. 허용 적용 후 공식
HTTPS URI로 설정하고 서명 검증을 유지한 채 설치를 재시도해야 한다.

Docker daemon의 이미지/컨테이너 보존이 새 cloud task snapshot에서 검증되지는 않았다.
ROS 컨테이너 준비를 완료된 Gazebo 환경으로 간주하면 안 된다.

## 제공된 외부 Pinky Pro 소스 조사

사용자가 제공한 `https://github.com/pinklab-art/pinky_pro`의 기본 브랜치 `main`,
커밋 `014a09f289e6988894fbffde2624fdb1e257815b`를
`/workspace/pinky-pro-reference`에 조사용으로 받았다. 외부 원본은 수정하지 않았다.
이 커밋이 실제 로봇에 설치된 커밋과 동일하다는 확인은 아직 없다.

- `pinky_description/urdf/robot.urdf.xacro`: namespace, is_sim,
  cam_tilt_deg(기본 8도), screen_tilt_deg 인자.
- `pinky_gz.urdf.xacro`: DiffDrive 바퀴 간격 0.0961m, 반지름 0.028m,
  `${namespace}cmd_vel`, `${namespace}odom`(30Hz), odom/base_footprint frame,
  전역 `/tf`. JointStatePublisher는 `${namespace}joint_states`를 발행한다.
- LiDAR: GPU sensor, 640 samples, ±π, 10Hz, range 0.05~12m.
- 전방 카메라: 이미 sensor 정의가 있다. 1280×720, 30Hz,
  horizontal_fov=1.1519rad, clip=0.1~100m,
  `${namespace}camera/image_raw`, frame `${namespace}front_camera_link`.
  그러므로 새 camera를 중복 추가할 필요는 없다. namespace `amr_01/`에 대한
  `/amr_01/camera/image_raw` bridge가 다음 구현 대상이다.
- upstream `pinky_gz_sim/launch/launch_sim.launch.xml`은 `ros_gz_image image_bridge`를
  사용한다. camera_info bridge 설정도 있으나 실제 namespace별 Gazebo CameraInfo
  토픽 이름은 runtime discovery로 확인해야 한다.
- 현재 FMS launch에는 joint_states bridge가 없다. wheel joint의 robot_state_publisher
  TF를 검증할 때 이 토픽도 확인한다. 전역 Gazebo `/tf`를 로봇별 ROS `/tf`로 옮기는
  현재 구조는 다중 로봇에서는 전체 프레임이 각 namespace로 복제될 수 있다.
- 외부 모델은 custom `gz-sim-lamp-control-system`도 참조한다.
  실제 `pinky_gz_sim` plugin 빌드 및 resource/plugin path를 확인해야 하며,
  plugin load 실패를 조용히 무시하지 않는다.

현재 확인한 것은 소스 정의다. 실제 센서 발행, SDF 변환 후 sensor 유지,
카메라 렌더링 및 ROS 수신은 Gazebo 설치가 해결된 다음 검증한다.
