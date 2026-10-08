#!/bin/bash
# 바탕화면 아이콘(Pinky FMS)에서 실행: 실물 관제 스택(run_real.sh)을 띄우고, 준비되면 브라우저로 GUI 를 연다.
# 이 창에서 Ctrl+C 를 누르거나 창을 닫으면 관제 스택이 모두 종료된다 (로봇은 끄지 않는다. 끄려면 GUI 에서 OFF).
DIR=$(cd "$(dirname "$0")" && pwd)
URL=http://localhost:8080

up() { curl -s -o /dev/null -m 1 "$1"; }

if up $URL && up http://localhost:8000/config; then
  echo "[Pinky FMS] 관제 스택이 이미 실행 중입니다. GUI 만 엽니다."
  xdg-open $URL >/dev/null 2>&1
  sleep 2
  exit 0
fi

echo "[Pinky FMS] 관제 스택을 시작합니다 (rosbridge · traffic_core · 백엔드 · 웹)"
echo "             종료: 이 창에서 Ctrl+C 또는 창 닫기"
(
  for _ in $(seq 1 60); do
    if up $URL && up http://localhost:8000/config; then
      echo "[Pinky FMS] 준비 완료 → $URL 을 엽니다"
      xdg-open $URL >/dev/null 2>&1
      exit 0
    fi
    sleep 1
  done
  echo "[Pinky FMS] 60초 안에 GUI 가 준비되지 않았습니다. 로그: /tmp/fms_real_*.log"
) &

"$DIR/run_real.sh" "$@"
echo
read -rp "[Pinky FMS] 종료되었습니다. 엔터를 누르면 창이 닫힙니다."
