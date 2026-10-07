# 갈림길에서 할 동작 하나 (LaneCommand.maneuvers 의 원소, path 순서대로 들어간다)
string STRAIGHT="STRAIGHT"
string LEFT="LEFT"
string RIGHT="RIGHT"

string node_id               # 코스 노드 이름 (예: "J")
float32 s_at                 # 노드 중심 위치: path 첫 점부터 path 를 따라 잰 거리 (m)
string turn                  # STRAIGHT | LEFT | RIGHT (로봇 진행 방향 기준)
string follow                # 갈림길을 지나는 동안 따라갈 테이프 쪽: LEFT | RIGHT (반대쪽 테이프는 잠시 무시)
float32 follow_dist          # follow 쪽 테이프만 따라가는 거리 (m). 지나면 양쪽 테이프 추종으로 돌아간다
