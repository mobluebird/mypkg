#!/bin/bash
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause


echo "===== TEST START ====="

#######################################
# ワークスペース設定
#######################################
DIR="${1:-$HOME}"
WS="$DIR/ros2_ws"

if [ ! -d "$WS" ]; then
    echo "ERROR: workspace not found: $WS"
    exit 1
fi

cd "$WS"

#######################################
# ビルド
#######################################
echo "[TEST] colcon build"
colcon build

#######################################
# 環境読み込み
#######################################
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
else
    echo "ERROR: install/setup.bash not found"
    exit 1
fi

#######################################
# launch 実行（一定時間だけ）
#######################################
LOG="/tmp/mypkg_test.log"
rm -f "$LOG"

echo "[TEST] ros2 launch mypkg talk_listen.launch.py"
timeout 10 ros2 launch mypkg talk_listen.launch.py > "$LOG" 2>&1 || true

#######################################
# ログ検証
#######################################
echo "[TEST] log check"

grep -q "UTC"  "$LOG" || { echo "ERROR: UTC not found";  exit 1; }
grep -q "JD"   "$LOG" || { echo "ERROR: JD not found";   exit 1; }
grep -q "GMST" "$LOG" || { echo "ERROR: GMST not found"; exit 1; }
grep -q "LST"  "$LOG" || { echo "ERROR: LST not found";  exit 1; }

#######################################
# 結果表示
#######################################
echo "===== TEST PASSED ====="

exit 0

