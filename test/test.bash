#!/bin/bash
set -euo pipefail

FAIL=0
LOG_FILE=launch.log
TOKYO_LONGITUDE=139.6917

TOPICS=(
  utc_time
  julian_day
  gmst
  lst
)

#######################################
# utility
#######################################
strip_quotes() {
  echo "$1" | sed "s/^'//; s/'$//"
}

get_topic_once() {
  local topic="$1"
  local out data

  for _ in {1..15}; do
    out=$(ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile "/$topic" 2>/dev/null || true)
    data=$(echo "$out" | sed -n 's/^data:[[:space:]]*//p')
    if [ -n "$data" ]; then
      strip_quotes "$data"
      return 0
    fi
    sleep 0.3
  done
  return 1
}

#######################################
# launch
#######################################
timeout 20 ros2 launch mypkg talk_listen.launch.py >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
sleep 5   # CI対策

#######################################
# topic existence
#######################################
echo "=== トピック存在確認 ==="
for t in "${TOPICS[@]}"; do
  found=0
  for _ in {1..10}; do
    if ros2 topic list | grep -qx "/$t"; then
      echo "$t: OK"
      found=1
      break
    fi
    sleep 0.5
  done
  [ $found -eq 1 ] || { echo "$t: ERROR"; FAIL=1; }
done

#######################################
# message format
#######################################
echo "=== メッセージ形式チェック ==="

utc1=$(get_topic_once utc_time)
jd1=$(get_topic_once julian_day)
gmst1=$(get_topic_once gmst)
lst1=$(get_topic_once lst)

[ -n "$utc1" ] && [ -n "$jd1" ] && [ -n "$gmst1" ] && [ -n "$lst1" ] || {
  echo "ERROR: 初回取得失敗"
  FAIL=1
}

# UTC ISO8601
[[ "$utc1" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T ]] || {
  echo "UTC format ERROR: $utc1"
  FAIL=1
}

# JD
[[ "$jd1" =~ ^[0-9]+\.[0-9]{5,}$ ]] || {
  echo "JD format ERROR: $jd1"
  FAIL=1
}

# GMST / LST
for name in gmst lst; do
  val=$(eval echo "\$${name}1")
  [[ "$val" =~ ^[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{2}$ ]] || {
    echo "$name format ERROR: $val"
    FAIL=1
  }
done

#######################################
# time progression
#######################################
echo "=== 時間変化チェック ==="
sleep 1.5

utc2=$(get_topic_once utc_time)
jd2=$(get_topic_once julian_day)
gmst2=$(get_topic_once gmst)
lst2=$(get_topic_once lst)

[ "$utc1" != "$utc2" ] && echo "UTC changes: OK" || { FAIL=1; }

awk -v a="$jd1" -v b="$jd2" 'BEGIN{exit !(b>a)}' \
  && echo "JD increases: OK" || { echo "JD did not increase"; FAIL=1; }

#######################################
# GMST / LST logical check
#######################################
gmst_hours=$(echo "$gmst2" | awk -F: '{print $1 + $2/60 + $3/3600}')
lst_hours=$(echo "$lst2"  | awk -F: '{print $1 + $2/60 + $3/3600}')

expected_lst=$(awk -v g="$gmst_hours" -v lon="$TOKYO_LONGITUDE" '
BEGIN{
  lst=g+lon/15
  while(lst<0)lst+=24
  while(lst>=24)lst-=24
  print lst
}')

diff=$(awk -v a="$lst_hours" -v b="$expected_lst" '
BEGIN{
  d=a-b
  if(d<0)d=-d
  if(d>12)d=24-d
  print d
}')

awk -v d="$diff" 'BEGIN{exit !(d<0.02)}' \
  && echo "LST vs GMST logical check: OK" \
  || { echo "LST vs GMST logical check: ERROR"; FAIL=1; }

#######################################
# cleanup
#######################################
kill "$LAUNCH_PID" 2>/dev/null || true
echo "----------------------------------"

if [ $FAIL -eq 0 ]; then
  echo "=== ALL TESTS PASSED ==="
  exit 0
else
  echo "=== TESTS FAILED ==="
  exit 1
fi

