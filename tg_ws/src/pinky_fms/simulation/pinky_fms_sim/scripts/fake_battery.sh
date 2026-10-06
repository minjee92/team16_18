#!/bin/bash
# 시뮬레이션 로봇의 배터리 토픽 흉내 (관제가 /<ns>/battery/percent 로 로봇을 발견한다).  fake_battery.sh <namespace>
exec ros2 topic pub -r 1 /$1/battery/percent std_msgs/msg/Float32 "{data: 95.0}"
