# fleet_lane -> 로봇(lane_executor) : 차선 주행 명령 (/<namespace>/fms/lane_cmd, 5 Hz)
#   5 Hz 로 계속 보내며, 이것이 곧 관제 신호(heartbeat)다. 로봇은 1.5 s 넘게 못 받으면 멈춘다 (받은 시각 기준이라 시계 동기화 불필요).
#   관제는 로봇을 직접 움직이지 않는다. 갈 길(path)과 "어디까지 가도 되는지"(s_limit)만 정한다.
#   s 로 끝나는 거리는 모두 path 첫 점부터 path 를 따라 잰 길이(m)다.
string GO="GO"
string STOP="STOP"

std_msgs/Header header       # stamp: 보낸 시각(기록용), frame_id: "map"
string session_id            # 관제 노드가 켜질 때마다 새로 정하는 ID. 서로 다른 값이 섞여 오면 관제가 2개 떠 있다는 뜻 → 로봇 정지
uint32 seq                   # 같은 session_id 안에서 보낼 때마다 1 증가. 로봇은 지난 값 이하면 버린다 (session 이 바뀌면 다시 센다)
string command               # GO | STOP. GO: path 를 따라 s_limit 까지 가도 된다. STOP(그 밖의 값 포함): 바로 정지
string mission_id            # GUI 가 정한 미션 ID. path 는 미션마다 고정 (STOP 이면 비어 있을 수 있음)
geometry_msgs/Point[] path   # 갈 길의 가운데 선 꼭짓점 (map, m). 첫 점 = 미션 시작 위치, 마지막 점 = 목표. 꺾이는 점만 보낸다
float32 s_goal               # 목표 위치 (m)
float32 s_limit              # 이동 허가선 (m). 로봇은 이 위치를 넘지 않는다. s_goal 이상이면 목표까지 가도 된다
#   s_limit 은 줄어들 수도 있다 (관제가 미션을 취소하지 않고 로봇을 잠깐 세울 때). 줄어도 로봇이 더 움직이는 일은 없다:
#     - 새 허가선이 아직 앞에 있으면: 그 위치에서 멈춘다 (평소와 같음).
#     - 이미 지났거나 멈출 수 있는 거리 안이면: 되도록 빨리 멈춘다. 후진하지 않고 그 자리에서 HOLD.
#   지금 자리에 바로 세우려면 s_limit 을 0 으로 보내고, 다시 늘리면 이어서 간다.
float32 arrive_tol           # s_goal 에서 이 거리(m) 안에 멈추면 도착
bool uturn_at_start          # 로봇이 path 반대 방향을 보고 있으면 출발 전에 제자리 180° 회전해도 된다 (관제가 회전 공간을 확인함)
LaneManeuver[] maneuvers     # 갈림길 동작 (path 순서대로)
