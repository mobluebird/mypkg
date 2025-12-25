#!/bin/bash
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc

# talker 
timeout 20 ros2 run mypkg talker > /tmp/talker.log 2> /tmp/talker.err &
sleep 3
ros2 topic echo --once /utc_time > /tmp/utc.log
grep -q 'data:' /tmp/utc.log
pkill -f 'ros2 run mypkg talker' || true
sleep 1

# launch
#timeout 20 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log

timeout 20 ros2 launch mypkg talk_listen.launch.py \
  > /tmp/mypkg.log 2> /tmp/mypkg.err &

sleep 2

for i in {1..5}; do
  ros2 topic list | grep -q '^/utc_time$' && break
  sleep 1
done
ros2 topic list | grep -q '^/utc_time$'

# topic 存在確認
ros2 topic list | grep -q '^/utc_time$'
ros2 topic list | grep -q '^/julian_day$'
ros2 topic list | grep -q '^/gmst$'
ros2 topic list | grep -q '^/lst$'

# topic 型確認
ros2 topic info /utc_time    | grep -q 'std_msgs/msg/String'
ros2 topic info /julian_day  | grep -q 'std_msgs/msg/String'
ros2 topic info /gmst        | grep -q 'std_msgs/msg/String'
ros2 topic info /lst         | grep -q 'std_msgs/msg/String'

# 時間更新確認
ros2 topic echo /utc_time -n 2 > /tmp/utc2.log
[ "$(grep -c '^data:' /tmp/utc2.log)" -ge 2 ]

# listener 出力確認
grep -q '\[listener' /tmp/mypkg.log
grep -q 'UTC :'  /tmp/mypkg.log
grep -q 'JD  :'  /tmp/mypkg.log
grep -q 'GMST:'  /tmp/mypkg.log
grep -q 'LST (Tokyo):' /tmp/mypkg.log

# 異常終了なし
! grep -i 'error' /tmp/mypkg.err
! grep -i 'warn'  /tmp/mypkg.err
