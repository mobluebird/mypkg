#!/bin/bash
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

set -euo pipefail

# ディレクトリ指定
DIR=~
[ "${1-}" != "" ] && DIR="$1"

cd "$DIR/ros2_ws" || { echo "ros2_ws ディレクトリが見つかりません"; exit 1; }

# ROS2環境の読み込み
source install/setup.bash

# colcon build（必要に応じて）
colcon build --symlink-install

# ローンチをバックグラウンドで起動（10秒タイムアウト）
LOG_FILE="/tmp/mypkg.log"
timeout 10 ros2 launch mypkg talk_listen.launch.py > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
sleep 3  # ノード起動待ち

# トピックリスト
TOPICS=("utc_time" "julian_day" "gmst" "lst")
FAIL=0

echo "=== トピック存在確認 ==="
for t in "${TOPICS[@]}"; do
    if ros2 topic list | grep -q "^/${t}$"; then
        echo "$t: OK"
    else
        echo "$t: ERROR (topic not found)"
        FAIL=1
    fi
done

echo "=== メッセージ形式チェック ==="
# 各トピックの1回目取得
utc1=$(ros2 topic echo -n 1 utc_time 2>/dev/null)
jd1=$(ros2 topic echo -n 1 julian_day 2>/dev/null)
gmst1=$(ros2 topic echo -n 1 gmst 2>/dev/null)
lst1=$(ros2 topic echo -n 1 lst 2>/dev/null)

# UTC ISO8601チェック
if ! [[ $utc1 =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T ]]; then
    echo "UTC format ERROR: $utc1"
    FAIL=1
fi

# JD 小数点5桁以上チェック
if ! [[ $jd1 =~ ^[0-9]+\.[0-9]{5,} ]]; then
    echo "JD format ERROR: $jd1"
    FAIL=1
fi

# GMST/LST hh:mm:ss.xx チェック
for t in gmst lst; do
    val=$(eval echo \$$t"1")
    if ! [[ $val =~ ^[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{2}$ ]]; then
        echo "$t format ERROR: $val"
        FAIL=1
    fi
done

echo "=== 時間変化確認 ==="
sleep 1.5  # 少し待って2回目取得
utc2=$(ros2 topic echo -n 1 utc_time 2>/dev/null)
jd2=$(ros2 topic echo -n 1 julian_day 2>/dev/null)
gmst2=$(ros2 topic echo -n 1 gmst 2>/dev/null)
lst2=$(ros2 topic echo -n 1 lst 2>/dev/null)

# UTC変化確認
if [ "$utc1" != "$utc2" ]; then
    echo "UTC changes over time: OK"
else
    echo "UTC did not change: ERROR"
    FAIL=1
fi

# JD増加確認
if (( $(echo "$jd2 > $jd1" | bc -l) )); then
    echo "JD increases: OK"
else
    echo "JD did not increase: ERROR"
    FAIL=1
fi

# GMST/LST論理整合性チェック
TOKYO_LONGITUDE=139.6917
gmst_hours=$(echo "$gmst2" | awk -F: '{print $1 + $2/60 + $3/3600}')
lst_hours=$(echo "$lst2" | awk -F: '{print $1 + $2/60 + $3/3600}')
expected_lst=$(echo "($gmst_hours + $TOKYO_LONGITUDE/15.0)%24" | bc -l)
diff=$(echo "$lst_hours - $expected_lst" | bc -l)
diff_abs=$(echo "$diff" | awk '{if($1<0){print -1*$1}else{print $1}}')
if (( $(echo "$diff_abs < 0.02" | bc -l) )); then
    echo "LST vs GMST logical check: OK"
else
    echo "LST vs GMST logical check: ERROR"
    FAIL=1
fi

# ノード終了
kill $LAUNCH_PID || true

if [ $FAIL -eq 0 ]; then
    echo "=== ALL TESTS PASSED ==="
    exit 0
else
    echo "=== TESTS FAILED ==="
    exit 1
fi

