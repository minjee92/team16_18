# GUI -> fleet_mission : 운영자의 요청 (미션 층 입력)
string mission_id            # GUI 가 정한 고유 ID
string type                  # "NAV_GOTO" | "CANCEL"  (이후 "AUTO_DRIVE", "AUTO_MAPPING" 등 추가)
string robot_id              # 비우면 조정 층이 로봇을 고른다
geometry_msgs/PoseStamped goal   # NAV_GOTO 일 때 목표 (frame_id: map)
