# 로봇(lane_executor) -> fleet_lane : 차선 주행 상태 (/<namespace>/fms/lane_status, 5 Hz)
#   s 로 끝나는 거리는 지금 따르는 LaneCommand.path 기준이다.

# state 값
string STATE_WAIT_POSE="WAIT_POSE"   # 지도 위 위치를 모름 (초기 위치 지정 전, 또는 위치를 잃음) → 움직이지 않음
string STATE_IDLE="IDLE"             # 미션 없음 (STOP 명령) → 정지
string STATE_UTURN="UTURN"           # 출발 전 제자리 180° 회전 중
string STATE_DRIVING="DRIVING"       # path 를 따라 주행 중 (갈림길 통과, 횡단보도 감속 포함)
string STATE_HOLD="HOLD"             # 이동 허가선(s_limit)에 닿았거나 이미 지나서 멈춰 관제를 기다림
string STATE_ARRIVED="ARRIVED"       # 목표 도착 후 정지
string STATE_STOPPED="STOPPED"       # 안전 때문에 잠시 멈춤 (reason). 원인이 사라지면 다시 간다
string STATE_FAILED="FAILED"         # 이 미션을 계속할 수 없음 (reason). 새 명령이 올 때까지 정지

# reason 값 (state 가 HOLD / STOPPED / FAILED 일 때. 그 밖의 state 에서는 빈 문자열)
string REASON_AUTHORITY="AUTHORITY"                  # HOLD    : 이동 허가선(s_limit)에 닿았거나 이미 지남
string REASON_SONAR="SONAR"                          # STOPPED : 초음파가 앞 장애물을 감지
string REASON_SONAR_FAULT="SONAR_FAULT"              # STOPPED : 초음파 데이터가 없거나 이상함 (게이트가 정지시킴)
string REASON_CROSSWALK="CROSSWALK"                  # STOPPED : 횡단보도 위 물체를 확인하는 중
string REASON_LANE_LOST="LANE_LOST"                  # STOPPED : 차선을 잃음
string REASON_CMD_STALE="CMD_STALE"                  # STOPPED : 관제 신호(LaneCommand)가 1.5 s 넘게 끊김
string REASON_TWO_FMS="TWO_FMS"                      # STOPPED : 서로 다른 관제 2개의 명령을 받음
string REASON_CMD_VEL_CONFLICT="CMD_VEL_CONFLICT"    # STOPPED : 다른 노드(예: Nav2)도 cmd_vel 을 내고 있음 (게이트가 정지시킴)
string REASON_OFF_ROUTE="OFF_ROUTE"                  # FAILED  : path 에서 너무 벗어남 (길을 잘못 듦)
string REASON_HEADING="HEADING"                      # FAILED  : path 반대 방향을 보고 있는데 U턴 허가(uturn_at_start)가 없음
string REASON_UTURN_TIMEOUT="UTURN_TIMEOUT"          # FAILED  : 제자리 회전이 제한 시간 안에 끝나지 않음
string REASON_BAD_COMMAND="BAD_COMMAND"              # FAILED  : 명령 값이 잘못됨 (path 점 2개 미만 등)

std_msgs/Header header       # stamp: 보낸 시각(로봇 시계, 기록용), frame_id: "map"
string session_id            # 따르고 있는 관제의 session_id (관제가 2개라 멈춘 동안은 빈 문자열)
uint32 cmd_seq               # 마지막으로 적용한 명령의 seq. 관제는 새 허가가 반영됐는지 이것으로 확인한다
string mission_id            # 수행 중인 미션 ID (없으면 빈 문자열)
string state                 # 위 STATE_ 값 중 하나
string reason                # 위 REASON_ 값 중 하나 (HOLD / STOPPED / FAILED 가 아니면 빈 문자열)
float32 s                    # 진행도: path 첫 점부터 path 를 따라 잰 현재 위치 (m). path 가 없으면 0
float32 cross_track          # path 가운데 선에서 벗어난 거리 (m, 진행 방향 왼쪽이 +)
bool localized               # x, y, yaw 가 유효한가 (map→base_footprint 변환을 받았는가)
float32 x                    # 로봇 위치 (map, m)
float32 y
float32 yaw                  # 로봇 방향 (map, rad)
string drive_state           # pinky_autonomous 상태 이름 (예: DRIVING, CAUTION, OBSTACLE_STOP, JUNCTION/advance)
bool lane_valid              # 지금 차선을 보고 있는가
float32 sonar_range          # 앞 초음파 거리 (m). 장애물 없음(범위 밖) = +inf, 데이터 없음 = NaN (REP 117 과 같음)
float32 fps                  # 차선 인식 루프 속도 (프레임/초)
