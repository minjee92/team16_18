# Pinky FMS 상세 설명 (등록·mock·네트워크)

※ 경로는 저장소 개편 전 기준입니다. 현재 위치는 최상위 README 의 표를 따르세요.

## 로봇 등록

- GUI 의 **로봇 추가**에서 로봇 ID(= ROS namespace, 예: `amr_01`), IP, SSH 사용자, 비밀번호를 입력한다.
  백엔드가 접속과 로봇의 launch 패키지를 확인한 뒤 등록하고, 원하면 바로 ON 한다. OFF 상태에서 카드의 휴지통으로 지운다.
- 로봇에서 실행할 명령과 워크스페이스 경로는 `robots.yaml` 의 `defaults`(`ws_setup`, `launch_cmd`)를 쓴다.
  `robots:` 에 적은 로봇은 항상 표시되는 고정 로봇이다 (GUI 에서 지울 수 없음).
- 비밀번호는 파일에 저장하지 않는다. 로봇이 켜져 있는 동안 재접속용으로만 백엔드 메모리에 두고, OFF/삭제 때 지운다.
  "다음부터 비밀번호 없이 접속"을 체크하면 이 PC 의 SSH 키(`~/.ssh/fms_ed25519`)를 로봇에 등록한다.
- 추가한 로봇·마지막 접속 정보·켠 로봇은 `pinky_fms_backend/state/<설정 파일 이름>/` 에 남는다 (git 에서 제외).
- 조정 노드는 `/<namespace>/battery/percent` 또는 `/<namespace>/odom` 토픽을 보고 로봇을 자동 발견하므로, 로봇을 추가해도 재시작할 필요가 없다.

## 로봇 없이 테스트 (mock)

`bash run_demo.sh` 는 `robots.mock.yaml`(`defaults.mode: local`)로 관제 스택을 띄운다. ON 이나 로봇 추가를 하면 이 PC 에서
가짜 로봇(`mock_robot`)이 뜨고, IP·비밀번호는 쓰이지 않는다. 다른 설정으로 띄우려면 `bash run_demo.sh <robots.yaml 경로>`.

## 실행 (관제PC, domain 16)

```bash
source /opt/ros/jazzy/setup.bash && source ~/pinky/install/setup.bash
source ~/pinky/src/pinky_fms/fms_env.sh                              # domain, DDS 설정 (모든 터미널)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml          # 터미널 1
ros2 launch pinky_fms_core fms_core.launch.xml robots_file:=$HOME/pinky/src/pinky_fms/pinky_fms_core/config/robots.yaml   # 터미널 2
cd src/pinky_fms/pinky_fms_backend && .venv/bin/uvicorn app:app --host 127.0.0.1 --port 8000   # 터미널 3
cd fms_web_platform/fms_web_platform && python3 -m http.server 8080  # 터미널 4 -> http://localhost:8080
```

## Wi-Fi 에서 통신이 끊길 때: DDS 유니캐스트 전환

Wi-Fi 는 멀티캐스트(DDS 자동 탐색)에 약해서 다중 로봇이면 끊김이 잦다. `robots.yaml` 에서 한 줄로 바꾼다 (RMW: rmw_cyclonedds_cpp).

```yaml
network:
  discovery: unicast        # multicast(기본) | unicast
  interface: wlp0s20f3      # 관제PC 의 Wi-Fi 인터페이스 (ip -br addr 로 확인). 비우면 자동
```

- 로봇: ON 할 때 백엔드가 설정 파일을 만들어 적용한다. 관제PC IP 는 접속 경로에서 자동으로 알아낸다(네트워크가 바뀌어도 OK). 로봇은 관제PC 만 피어로 지정하므로 로봇끼리는 서로 탐색하지 않는다.
- 관제PC: 터미널/프로세스에 같은 설정을 적용해야 한다 (적용 안 한 터미널에서는 `ros2 topic list` 에 로봇이 안 보인다).
  ```bash
  source ~/pinky/src/pinky_fms/fms_env.sh        # ROS_DOMAIN_ID 와 DDS 설정을 robots.yaml 대로 적용
  ```
  `run_demo.sh` 는 자동 적용한다. 멀티캐스트를 끄면 같은 PC 안의 프로세스끼리도 못 찾으므로 관제PC 설정에는 localhost 가 피어로 들어간다.
- `ros2 topic` CLI 는 데몬이 옛 설정을 들고 있을 수 있어 `ros2 daemon stop` 하거나 `--no-daemon` 을 쓴다 (fms_env.sh 가 데몬을 다시 띄운다).
- 로봇도 `rmw_cyclonedds_cpp`(ros-jazzy-rmw-cyclonedds-cpp)여야 한다.
- 시험 범위: 이 PC 에서 설정 적용, 멀티캐스트 끈 상태의 탐색(ON 후 약 4초 만에 IDLE), 인터페이스 고정까지 확인. 실제 Wi-Fi 에서의 안정성은 로봇으로 확인해야 한다.

## 구현 상태

- 완료(가짜 로봇으로 검증): GUI 로봇 추가·삭제, ON/OFF, 상태·배터리·위치 표시, 로그 뷰어, 맵 업로드/표시, 맵 클릭 주행(드래그: 방향), 초기 위치 지정, 로봇별 ping 추적(ON 이후), 유니캐스트 DDS 설정
- 아직 없음:
  - 업로드한 맵을 ON 된 로봇의 Nav2 에 자동 공유 (NAV 모드 ON). 지금 맵 업로드는 관제PC 화면용이라, 실제 로봇은 같은 맵으로 Nav2/AMCL 이 떠 있어야 위치가 맞는다
  - 자율주행(autonomous_drive)·자율 맵핑 미션 (모드 전환), 강제 주행(키보드), 도크 위치 자동 초기화
  - 카메라 영상, RETURN DOCK, 통신 상태 이력 로그
- 실제 로봇 SSH ON/OFF, 실제 AMCL/Nav2, 실제 Wi-Fi 안정성은 로봇이 없어 검증하지 못함 (local/mock 으로만 검증)
