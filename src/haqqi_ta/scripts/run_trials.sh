#!/usr/bin/env bash
# [FIX-VARQUANT] Template batch-run N trial skenario split + fault timed_pulse.
# Jalankan di PC robot (ROS 2 ter-source). SESUAIKAN bagian bertanda <<< EDIT.
set -u
N="${1:-10}"                       # jumlah trial
SCENARIO="${2:-split}"
TRIAL_DUR="${3:-90}"               # detik per trial (>= durasi misi + margin)
RESULTS_ROOT="${RESULTS_ROOT:-$HOME/haqqi_results}"   # <<< EDIT bila perlu
mkdir -p "$RESULTS_ROOT"

for i in $(seq 1 "$N"); do
  echo "==================== TRIAL $i / $N ===================="
  # <<< EDIT: launch sesuai setup Anda (bringup multi-robot + fault).
  ros2 launch haqqi_ta multi_robot_bringup.launch.py \
      scenario:="$SCENARIO" fault_mode:=timed_pulse &
  LP=$!
  # <<< EDIT: jika start via sinyal /experiment/start, publish di sini:
  # ros2 topic pub --once /experiment/start std_msgs/Bool '{data: true}'
  sleep "$TRIAL_DUR"
  # <<< EDIT: stop trial (sinyal stop / kill node logger menulis goal_result.csv)
  kill -INT "$LP" 2>/dev/null; wait "$LP" 2>/dev/null
  sleep 5
done

echo '==================== AGREGASI ===================='
python3 "$(dirname "$0")/aggregate_trials.py" "$RESULTS_ROOT" \
    --out "$RESULTS_ROOT/ringkasan_variabilitas.csv"
