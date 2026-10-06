# fleet_coordinator -> GUI : 로봇 한 대의 종합 상태 (/fleet/robot_status, 로봇마다 1개씩 주기 발행)
string robot_id
string state                 # OFFLINE | IDLE | BUSY
float32 battery_percent      # 수신 전이면 -1
bool localized               # 맵 기준 위치(amcl_pose)를 받은 적이 있는가. false 이면 x, y, yaw 는 의미 없음
float32 x
float32 y
float32 yaw
string current_task_id
float32 last_seen_sec_ago    # 마지막 데이터 수신 후 경과 시간 (heartbeat)
