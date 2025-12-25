#!/bin/bash
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc

timeout 20 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log

#grep -q '\[listener'  /tmp/mypkg.log
#grep -q 'UTC'  /tmp/mypkg.log
#grep -q 'JD'   /tmp/mypkg.log
#grep -q 'GMST' /tmp/mypkg.log
#grep -q 'LST'  /tmp/mypkg.log

ros2 topic list | grep -q '^/utc_time$'
ros2 topic list | grep -q '^/julian_day$'
ros2 topic list | grep -q '^/gmst$'
ros2 topic list | grep -q '^/lst$'

grep -q '\[listener' /tmp/mypkg.log
grep -q 'UTC :'  /tmp/mypkg.log
grep -q 'JD  :'  /tmp/mypkg.log
grep -q 'GMST:'  /tmp/mypkg.log
grep -q 'LST (Tokyo):' /tmp/mypkg.log

