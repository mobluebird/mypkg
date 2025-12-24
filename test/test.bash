#!/bin/bash
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause
#
# mypkg 4トピック統合テスト（CI安定・QoS対応版）

set -eo pipefail   # ROS2は set -u 非対応

#######################################
# 設定
#######################################
DIR=~
[ "${1-}" != "" ] && DIR="$1"

ROS_DISTRO=humble
LOG_FILE="/tmp/mypkg.log"
TOKYO_LONGITUDE=139.6917
FAIL=0

TOPICS=("utc_time" "julian_day" "gmst" "lst")

#######################################
# QoS指定 + retry 付き topic 取得
#######################################
get_topic_once () {
    local topic="$1"
    local out=""
    local data=""

    for i in {1..15}; do
        out=$(ros2 topic echo \
            --once \
            --qos-reliability best_effort \
            --qos-durability volatile \
            "/$topic" 2>/dev/null || true)

        # std_msgs/String の data: を剥がす
        data=$(echo "$out" | sed -n 's/^data:[[:space:]]*//p')

        if [ -n "$data" ]; then
            echo "$data"
            return 0
        fi
        sleep 0.5
    done

    echo ""
    return 1
}

#######################################
# 環境準備
#######################################
cd "$DIR/ros2_ws" || {
    echo "ERROR: ros2_ws ディレクトリが見つかりません"
    exit 1
}

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash

#######################################
# launch 起動
#######################################
timeout 20 ros2 launch mypkg talk_listen.launch.py > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
sleep 5   # CIは遅い

#######################################
# トピック存在確認（retry）
#######################################
echo "=== トピック存在確認 ==="
for t in "${TOPICS[@]}"; do
    found=0
    for i in {1..10}; do
        if ros2 topic list | grep -q "^/$t$"; then
            echo "$t: OK"
            found=1
            break
        fi
        sleep 0.5
    done
    if [ $found -eq 0 ]; then
        echo "$t: ERROR (not found)"
        FAIL=1
    fi
done

#######################################
# メッセージ形式チェック
#######################################
echo "=== メッセージ形式チェック ==="
utc1=$(get_topic_once utc_time)
jd1=$(get_topic_once julian_day)
gmst1=$(get_topic_once gmst)
lst1=$(get_topic_once lst)

if [ -z "$utc1" ] || [ -z "$jd1" ] || [ -z "$gmst1" ] || [ -z "$lst1" ]; then
    echo "ERROR: 初回メッセージ取得失敗"
    FAIL=1
fi

# UTC ISO8601
if ! [[ $utc1 =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T ]]; then
    echo "UTC format ERROR: $utc1"
    FAIL=1
fi

# JD（小数点5桁以上）
if ! [[ $jd1 =~ ^[0-9]+\.[0-9]{5,} ]]; then
    echo "JD format ERROR: $jd1"
    FAIL=1
fi

# GMST / LST hh:mm:ss.xx
for name in gmst lst; do
    val=$(eval echo \$$name"1")
    if ! [[ $val =~ ^[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{2}$ ]]; then
        echo "$name format ERROR: $val"
        FAIL=1
    fi
done

#######################################
# 時間変化チェック
#######################################
echo "=== 時間変化チェック ==="
sleep 1.5

utc2=$(get_topic_once utc_time)
jd2=$(get_topic_once julian_day)
gmst2=$(get_topic_once gmst)
lst2=$(get_topic_once lst)

if [ "$utc1" != "$utc2" ]; then
    echo "UTC changes over time: OK"
else
    echo "UTC did not change"
    FAIL=1
fi

if (( $(echo "$jd2 > $jd1" | bc -l) )); then
    echo "JD increases: OK"
else
    echo "JD did not increase"
    FAIL=1
fi

#######################################
# GMST / LST 論理整合性（bc 不使用）
#######################################

gmst_hours=$(echo "$gmst2" | awk -F: '{print $1 + $2/60 + $3/3600}')
lst_hours=$(echo "$lst2" | awk -F: '{print $1 + $2/60 + $3/3600}')

expected_lst=$(awk -v g="$gmst_hours" -v lon="$TOKYO_LONGITUDE" '
BEGIN {
    lst = g + lon / 15.0
    while (lst < 0)  lst += 24
    while (lst >= 24) lst -= 24
    print lst
}')

diff_abs=$(awk -v a="$lst_hours" -v b="$expected_lst" '
BEGIN {
    d = a - b
    if (d < 0) d = -d
    print d
}')

# 許容誤差：0.02 時間（約72秒）
if awk -v d="$diff_abs" 'BEGIN { exit !(d < 0.02) }'; then
    echo "LST vs GMST logical check: OK"
else
    echo "LST vs GMST logical check: ERROR"
    FAIL=1
fi

#######################################
# 終了処理
#######################################
kill $LAUNCH_PID || true

echo "----------------------------------"
if [ $FAIL -eq 0 ]; then
    echo "=== ALL TESTS PASSED ==="
    exit 0
else
    echo "=== TESTS FAILED ==="
    exit 1
fi

