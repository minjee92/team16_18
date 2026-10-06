# 관제PC 터미널에서 `source` 해서 쓴다 (실행하는 게 아니라 환경변수를 현재 셸에 적용).
#   source <저장소>/control_pc/fms_env.sh [robots.yaml 경로]
#
# robots.yaml 의 domain_id 와 network.discovery 를 읽어서
#   - ROS_DOMAIN_ID 를 맞추고
#   - discovery 가 unicast 이면 RMW/CYCLONEDDS_URI(멀티캐스트 끔 + 인터페이스 고정)를 적용한다.
# 이 설정을 적용하지 않은 터미널에서는 유니캐스트로 동작 중인 로봇이 보이지 않는다.

_FMS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_FMS_CFG="${1:-$_FMS_DIR/ros/pinky_fms_core/config/robots.yaml}"

if [ ! -f "$_FMS_CFG" ]; then
  echo "[fms_env] 설정 파일이 없습니다: $_FMS_CFG" >&2
else
  _DOM="$(python3 -c "import sys,yaml;print(yaml.safe_load(open(sys.argv[1])).get('domain_id',0))" "$_FMS_CFG")"
  export ROS_DOMAIN_ID="$_DOM"
  unset ROS_LOCALHOST_ONLY
  _ENV="$(python3 "$_FMS_DIR/backend/netconf.py" pc-env "$_FMS_CFG")"
  if [ -n "$_ENV" ]; then
    eval "$_ENV"
    # ros2 CLI 데몬이 이전(멀티캐스트) 설정으로 떠 있으면 로봇이 안 보이므로 새 설정으로 다시 뜨게 한다
    ros2 daemon stop >/dev/null 2>&1 || true
    echo "[fms_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, DDS 탐색=unicast (RMW=$RMW_IMPLEMENTATION)"
  else
    echo "[fms_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, DDS 탐색=multicast (기본)"
  fi
fi
unset _FMS_DIR _FMS_CFG _DOM _ENV
