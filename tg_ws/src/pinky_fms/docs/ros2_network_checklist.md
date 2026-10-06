# ROS 2 네트워크 연결 점검표

이 문서는 다음 구조에서 관제 PC와 두 로봇 사이의 ROS 2 통신을 확인하기 위한 절차입니다.

```text
관제 PC: ROS_DOMAIN_ID=0
  |
  +-- 공유기(robot1)
       +-- Robot 1: ROS_DOMAIN_ID=16
       +-- Robot 2: ROS_DOMAIN_ID=18
```

## 1. 통신 구조

각 로봇에는 namespace가 없고, 서로 다른 ROS domain으로 분리되어 있다고 가정합니다.

```text
Robot 1, Domain 16:
  /map
  /amcl_pose
  /scan
  /odom
  /tf
  /tf_static
  /navigate_to_pose

Robot 2, Domain 18:
  /map
  /amcl_pose
  /scan
  /odom
  /tf
  /tf_static
  /navigate_to_pose
```

관제 PC에서는 `domain_bridge`가 로봇별 토픽 이름을 구분해서 전달합니다.

```text
Domain 16 -> Domain 0: /robot1/map, /robot1/amcl_pose, ...
Domain 18 -> Domain 0: /robot2/map, /robot2/amcl_pose, ...
```

주행 액션은 현재 테스트 Python 파일의 worker가 각 로봇 domain에 직접 연결합니다.

```text
Robot 1 worker -> Domain 16 -> /navigate_to_pose
Robot 2 worker -> Domain 18 -> /navigate_to_pose
```

## 2. SSH 접속 확인

관제 PC에서 각 로봇으로 SSH 접속합니다.

```bash
ssh pinky@<ROBOT1_IP>
ssh pinky@<ROBOT2_IP>
```

SSH 접속 후 다음 정보를 확인합니다.

```bash
hostname
hostname -I
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
echo $RMW_IMPLEMENTATION
```

예상 결과는 다음과 같습니다.

```text
Robot 1: ROS_DOMAIN_ID=16
Robot 2: ROS_DOMAIN_ID=18
ROS_LOCALHOST_ONLY=0 또는 빈 값
```

`ROS_LOCALHOST_ONLY=1`이면 해당 컴퓨터 내부에서만 ROS 2 통신이 허용되므로 관제 PC와 통신할 수 없습니다.

## 3. IP 네트워크 확인

관제 PC에서 로봇 IP로 ping을 실행합니다.

```bash
ping -c 4 <ROBOT1_IP>
ping -c 4 <ROBOT2_IP>
```

정상 기준:

```text
패킷 손실 0%
응답 시간이 지속적으로 출력됨
```

ping이 실패하면 ROS 2 점검 전에 다음을 확인합니다.

- 관제 PC와 로봇이 같은 공유기의 일반 LAN에 연결되어 있는지
- 로봇 IP가 변경되지 않았는지
- 게스트 Wi-Fi를 사용하고 있지 않은지
- 공유기의 AP isolation/client isolation이 꺼져 있는지
- 로봇 SSH 접속이 가능한지
- PC 또는 로봇의 방화벽이 ICMP 또는 UDP 통신을 막고 있지 않은지

`ping` 성공은 IP 연결만 의미합니다. ping이 성공해도 DDS discovery가 차단될 수 있습니다.

## 4. 공유기 설정 확인

관제 PC와 두 로봇은 서로 직접 통신 가능한 같은 내부 네트워크에 있어야 합니다.

다음 기능은 꺼져 있어야 합니다.

```text
AP isolation
Client isolation
Wireless isolation
Guest network isolation
```

가능하면 다음 조건을 사용합니다.

```text
관제 PC, Robot 1, Robot 2 모두 같은 일반 Wi-Fi/LAN
각 장치가 서로 다른 사설 IP를 가짐
멀티캐스트 차단 기능이 꺼져 있음
```

## 5. ROS 2 환경 설정 확인

각 로봇의 `.bashrc`에 domain이 올바르게 설정되어 있는지 확인합니다.

Robot 1:

```bash
export ROS_DOMAIN_ID=16
export ROS_LOCALHOST_ONLY=0
```

Robot 2:

```bash
export ROS_DOMAIN_ID=18
export ROS_LOCALHOST_ONLY=0
```

설정 변경 후에는 새 SSH 세션을 열거나 다음을 실행합니다.

```bash
source ~/.bashrc
```

launch 직전에 반드시 다시 확인합니다.

```bash
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
```

관제 PC에서는 domain 0을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/tkoo/pinky/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

환경변수는 터미널마다 독립적입니다. 한 터미널에서 `export`해도 다른 터미널에는 자동 적용되지 않습니다.

## 6. 로봇 내부 ROS 2 확인

Robot 1에서 launch와 Nav2 bringup을 실행한 뒤 다음을 확인합니다.

```bash
export ROS_DOMAIN_ID=16
ros2 node list
ros2 topic list
ros2 action list
ros2 service list
```

Robot 2에서는 domain만 18로 바꿉니다.

```bash
export ROS_DOMAIN_ID=18
ros2 node list
ros2 topic list
ros2 action list
ros2 service list
```

각 로봇에서 최소한 다음 항목이 필요합니다.

```text
/map
/amcl_pose
/scan
/odom
/tf
/tf_static
/navigate_to_pose
/reinitialize_global_localization
```

실제 이름이 다르면 `domain_bridge_test_robot1.yaml`, `domain_bridge_test_robot2.yaml`, `pinky_navigation_test.py`의 이름을 실제 이름에 맞춰야 합니다.

## 7. 메시지 발행 확인

토픽 이름이 존재하는 것만으로는 충분하지 않습니다. 실제 메시지가 발행되는지 확인합니다.

Robot 1:

```bash
ROS_DOMAIN_ID=16 ros2 topic echo /map --once
ROS_DOMAIN_ID=16 ros2 topic echo /amcl_pose --once
ROS_DOMAIN_ID=16 ros2 topic echo /scan --once
ROS_DOMAIN_ID=16 ros2 topic echo /odom --once
```

Robot 2:

```bash
ROS_DOMAIN_ID=18 ros2 topic echo /map --once
ROS_DOMAIN_ID=18 ros2 topic echo /amcl_pose --once
ROS_DOMAIN_ID=18 ros2 topic echo /scan --once
ROS_DOMAIN_ID=18 ros2 topic echo /odom --once
```

다음 타입도 확인합니다.

```bash
ROS_DOMAIN_ID=16 ros2 topic type /amcl_pose
ROS_DOMAIN_ID=16 ros2 topic type /scan
ROS_DOMAIN_ID=16 ros2 topic type /odom
```

예상 타입:

```text
/amcl_pose: geometry_msgs/msg/PoseWithCovarianceStamped
/scan: sensor_msgs/msg/LaserScan
/odom: nav_msgs/msg/Odometry
```

## 8. TF 연결 확인

AMCL과 Nav2가 정상 동작하려면 일반적으로 다음 TF 연결이 필요합니다.

```text
map -> odom -> base_link -> laser
```

Robot 1:

```bash
ROS_DOMAIN_ID=16 ros2 run tf2_ros tf2_echo map base_link
ROS_DOMAIN_ID=16 ros2 run tf2_ros tf2_echo base_link laser
```

Robot 2:

```bash
ROS_DOMAIN_ID=18 ros2 run tf2_ros tf2_echo map base_link
ROS_DOMAIN_ID=18 ros2 run tf2_ros tf2_echo base_link laser
```

실제 센서 frame이 `laser`가 아니라 `laser_frame`, `base_scan` 등이라면 실제 frame 이름을 사용해야 합니다.

TF가 끊겨 있으면 다음 문제가 발생할 수 있습니다.

- AMCL pose가 발행되지 않음
- AMCL pose가 잘못된 위치로 계산됨
- Nav2 action server는 존재하지만 목표가 실패함
- GUI의 로봇 위치가 갱신되지 않음

## 9. NavigateToPose 액션 확인

각 로봇 domain에서 액션 서버가 발견되는지 확인합니다.

```bash
ROS_DOMAIN_ID=16 ros2 action list -t
ROS_DOMAIN_ID=18 ros2 action list -t
```

예상 결과:

```text
/navigate_to_pose [nav2_msgs/action/NavigateToPose]
```

액션 서버가 보이지 않으면 다음을 확인합니다.

- `bringup_launch.xml`이 정상 종료되지 않았는지
- Nav2 lifecycle node가 active 상태인지
- `bt_navigator`가 실행 중인지
- 로봇의 domain이 16/18로 올바른지
- 실제 액션 이름에 namespace가 붙어 있지 않은지

## 10. 관제 PC에서 domain bridge 확인

관제 PC의 bridge 터미널은 domain 0으로 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Robot 1 bridge:

```bash
ROS_LOG_DIR=/tmp ros2 run domain_bridge domain_bridge \
  /home/tkoo/pinky/src/pinky_pro/pinky_mission/pinky_mission/domain_bridge_test_robot1.yaml
```

Robot 2 bridge:

```bash
ROS_LOG_DIR=/tmp ros2 run domain_bridge domain_bridge \
  /home/tkoo/pinky/src/pinky_pro/pinky_mission/pinky_mission/domain_bridge_test_robot2.yaml
```

bridge 실행 중인 터미널에는 YAML parsing error가 없어야 합니다. 두 bridge는 각각 별도 터미널에서 실행합니다.

## 11. Domain 0에서 브리지 토픽 확인

새 관제 PC 터미널에서 다음을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

토픽 목록:

```bash
ros2 topic list | grep -E 'robot1|robot2'
```

예상 결과:

```text
/robot1/map
/robot1/amcl_pose
/robot1/tf
/robot1/tf_static
/robot2/map
/robot2/amcl_pose
/robot2/tf
/robot2/tf_static
```

실제 메시지 확인:

```bash
ros2 topic echo /robot1/amcl_pose
ros2 topic echo /robot2/amcl_pose
ros2 topic echo /robot1/map --once
```

해석 기준:

| 결과 | 의미 |
|---|---|
| 로봇 내부 토픽도 없고 Domain 0 토픽도 없음 | 로봇 launch/Nav2 또는 로봇 domain 문제 |
| 로봇 내부 토픽은 있지만 Domain 0 토픽이 없음 | DDS discovery, bridge 설정, 네트워크 문제 |
| Domain 0 토픽은 있지만 메시지가 없음 | 원본 토픽 발행, QoS, remap 문제 |
| AMCL은 있지만 값이 불안정함 | 초기화, 센서, odometry, TF 또는 map 문제 |
| map/AMCL은 정상이고 action만 실패 | Nav2 action/lifecycle 또는 worker 문제 |

## 12. Python GUI 실행 확인

관제 PC에서 `ROS_DOMAIN_ID=0`으로 GUI를 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/tkoo/pinky/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

python3 /home/tkoo/pinky/src/pinky_pro/pinky_mission/pinky_mission/pinky_navigation_test.py
```

정상 기준:

- GUI 창이 열림
- 선택한 맵이 표시됨
- AMCL 위치 패널의 Robot 1/2 값이 갱신됨
- 지도 위 Robot 1/2 위치 점이 표시됨
- 각 worker가 Domain 16/18에서 액션 서버를 발견함
- 목표를 추가하고 `Start navigation`을 눌렀을 때 action goal이 수락됨

GUI의 맵 파일 선택은 실제 로봇이 사용하는 map YAML과 일치해야 합니다. 관제 PC에서 선택한 맵 파일은 GUI 표시와 좌표 변환에 사용되며, 로봇의 map_server가 읽는 파일은 각 로봇 내부에 존재해야 합니다.

## 13. AMCL 초기 위치 점검

현재 Python 파일은 `/reinitialize_global_localization` 서비스를 호출할 수 있지만, 이 호출이 항상 정확한 위치를 보장하지는 않습니다.

서비스 존재 확인:

```bash
ROS_DOMAIN_ID=16 ros2 service list | grep reinitialize_global_localization
ROS_DOMAIN_ID=18 ros2 service list | grep reinitialize_global_localization
```

전역 위치추정은 다음 조건을 필요로 합니다.

```text
/map
/scan
/odom
map -> odom -> base_link -> laser TF
실제 환경과 일치하는 map
```

맵이 대칭적이거나 실제 환경과 크게 다르면 AMCL이 잘못된 위치로 수렴할 수 있습니다. 이 경우 GUI의 숫자만 믿지 말고 RViz에서 LaserScan과 map이 겹치는지 확인합니다.

## 14. 방화벽과 DDS discovery

IP ping은 되지만 `ros2 topic list`에 상대 장치의 ROS 2 노드가 나타나지 않는 경우 DDS discovery가 차단되었을 수 있습니다.

각 장치에서 다음을 확인합니다.

```bash
sudo ufw status
echo $ROS_LOCALHOST_ONLY
echo $RMW_IMPLEMENTATION
```

일반적으로 다음 조건이 필요합니다.

```text
ROS_LOCALHOST_ONLY=0
PC/로봇 간 UDP 통신 허용
공유기의 멀티캐스트 또는 DDS discovery 차단 없음
동일한 ROS 2 middleware 사용 권장
```

서로 다른 PC에서 통신할 때 방화벽을 무작정 끄기보다는 네트워크 정책에 맞게 DDS discovery/RTPS UDP 포트를 허용하는 것이 좋습니다. 사용하는 RMW와 DDS 설정에 따라 필요한 포트 범위가 달라질 수 있습니다.

## 15. 전체 점검 순서 요약

```text
1. 관제 PC에서 Robot 1/2로 SSH 접속
2. 각 로봇에서 ROS_DOMAIN_ID가 16/18인지 확인
3. 각 로봇에서 ROS_LOCALHOST_ONLY가 0인지 확인
4. 관제 PC에서 두 로봇 IP로 ping
5. Robot 1 센서/베이스 launch 실행
6. Robot 1 bringup_launch.xml 실행
7. Robot 2 센서/베이스 launch 실행
8. Robot 2 bringup_launch.xml 실행
9. 각 로봇에서 /map, /amcl_pose, /scan, /odom, /tf 확인
10. 각 로봇에서 /navigate_to_pose 확인
11. 관제 PC Domain 0에서 bridge 2개 실행
12. Domain 0에서 /robot1/*, /robot2/* 토픽 확인
13. Python GUI 실행
14. GUI에서 AMCL 위치와 지도 표시 확인
15. 목표를 추가하고 주행 시작
```

## 16. 빠른 진단 명령 모음

Robot 1:

```bash
ssh pinky@<ROBOT1_IP>
echo $ROS_DOMAIN_ID
ROS_DOMAIN_ID=16 ros2 topic list
ROS_DOMAIN_ID=16 ros2 action list -t
ROS_DOMAIN_ID=16 ros2 topic echo /amcl_pose --once
```

Robot 2:

```bash
ssh pinky@<ROBOT2_IP>
echo $ROS_DOMAIN_ID
ROS_DOMAIN_ID=18 ros2 topic list
ROS_DOMAIN_ID=18 ros2 action list -t
ROS_DOMAIN_ID=18 ros2 topic echo /amcl_pose --once
```

관제 PC:

```bash
ping -c 4 <ROBOT1_IP>
ping -c 4 <ROBOT2_IP>
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic list | grep -E 'robot1|robot2'
ros2 topic echo /robot1/amcl_pose --once
ros2 topic echo /robot2/amcl_pose --once
```

## 정상 상태 기준

다음 조건이 모두 만족되면 네트워크와 ROS 2 기본 연결은 정상입니다.

```text
[ ] SSH 접속 가능
[ ] Robot 1 IP ping 성공
[ ] Robot 2 IP ping 성공
[ ] Robot 1 ROS_DOMAIN_ID=16
[ ] Robot 2 ROS_DOMAIN_ID=18
[ ] 관제 PC ROS_DOMAIN_ID=0
[ ] 모든 장치 ROS_LOCALHOST_ONLY=0
[ ] Robot 1에서 /navigate_to_pose 확인
[ ] Robot 2에서 /navigate_to_pose 확인
[ ] Robot 1에서 /amcl_pose 메시지 확인
[ ] Robot 2에서 /amcl_pose 메시지 확인
[ ] Domain 0에서 /robot1/amcl_pose 확인
[ ] Domain 0에서 /robot2/amcl_pose 확인
[ ] GUI에 맵과 AMCL 위치 표시
```
