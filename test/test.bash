#!/bin/bash
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc

# talker 
timeout 20 ros2 run mypkg talker > /tmp/talker.log &
sleep 3
ros2 topic echo --once /utc_time > /tmp/utc.log
grep -q 'data:' /tmp/utc.log
pkill -f 'ros2 run mypkg talker' || true

# launch
timeout 20 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log

# topic 存在確認
ros2 topic list | grep -q '^/utc_time$'
ros2 topic list | grep -q '^/julian_day$'
ros2 topic list | grep -q '^/gmst$'
ros2 topic list | grep -q '^/lst$'

# listener 出力確認
grep -q '\[listener' /tmp/mypkg.log
grep -q 'UTC :'  /tmp/mypkg.log
grep -q 'JD  :'  /tmp/mypkg.log
grep -q 'GMST:'  /tmp/mypkg.log
grep -q 'LST (Tokyo):' /tmp/mypkg.log

