#!/bin/bash
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause
#
# mypkg 4トピック統合テスト（CI安定版）

set -eo pipefail   # set -u は使わない（ROS非対応）

#######################################
# 設定
#######################################
DIR=~
[ "${1-}" != "" ] && DIR="$1"

ROS_DISTRO=humble
LOG_FILE="/tmp/mypkg.log"
TOKYO_LONGITUDE=139.6917

TOPICS=("utc_time" "julian_day" "gmst" "lst")
FAIL=0

#######################################
# 安全な topic 取得（retry 付き）
#######################################
get_topic_once () {
    local topic="$1"
    local out=""

    for i in {1..10}; do
        out=$(ros2 topic echo -n 1 "/$topic" 2>/dev/null || true)
        if [ -n "$out" ]; then
            echo "$out"
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

# ROS 2 環境（CIで安全）
source /opt/ros/$ROS_DISTRO/setup.bash

# ビルド
colcon build --symlink-install
source install/setup.bash

#######################################
# Launch 起動
#######################################
timeout 15 ros2 launch mypkg talk_listen.launch.py > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

# ノード起動待ち（CIは遅い）
sleep 4

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
# 初回メッセージ取得
#######################################
echo "=== メッセージ形式チェック ==="
utc1=$(get_topic_once utc_time)
jd1=$(get_topic_once julian_day)
gmst1=$(get_topic_once gmst)
lst1=$(get_topic_once lst)

if [ -z "$utc1" ] || [ -z "$jd1" ] || [ -z "$gmst1" ] || [ -z "$lst1" ]; then
    echo "ERROR: 初回メッセージを取得できません"
    FAIL=1
fi

# UTC ISO8601
if ! [[ $utc1 =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T ]]; then
    echo "UTC format ERROR: $utc1"
    FAIL=1
fi

# JD 小数点5桁以上
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
# 時間変化確認
#######################################
echo "=== 時間変化確認 ==="
sleep 1.5

utc2=$(get_topic_once utc_time)
jd2=$(get_topic_once julian_day)
gmst2=$(get_topic_once gmst)
lst2=$(get_topic_once lst)

# UTC 変化
if [ "$utc1" != "$utc2" ]; then
    echo "UTC changes over time: OK"
else
    echo "UTC did not change"
    FAIL=1
fi

# JD 増加
if (( $(echo "$jd2 > $jd1" | bc -l) )); then
    echo "JD increases: OK"
else
    echo "JD did not increase"
    FAIL=1
fi

#######################################
# GMST / LST 論理整合性
#######################################
gmst_hours=$(echo "$gmst2" | awk -F: '{print $1 + $2/60 + $3/3600}')
lst_hours=$(echo "$lst2" | awk -F: '{print $1 + $2/60 + $3/3600}')
expected_lst=$(echo "($gmst_hours + $TOKYO_LONGITUDE/15.0) % 24" | bc -l)

diff=$(echo "$lst_hours - $expected_lst" | bc -l)
diff_abs=$(echo "$diff" | awk '{if($1<0){print -$1}else{print $1}}')

# 許容誤差：0.02時間 ≒ 72秒
if (( $(echo "$diff_abs < 0.02" | bc -l) )); then
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

