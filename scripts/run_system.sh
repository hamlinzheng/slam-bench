#!/usr/bin/env bash
# Run one baseline over one bag and capture the normalized artifacts (plan §5):
#   ① trajectory.tum   — odom topic recorded to TUM
#   ③ resource.csv     — external /proc CPU% + RSS sampling of the system process
#      run.log         — system stdout/stderr
#      metrics.json    — completion + bag-play exit code (accuracy metrics computed later)
#
# Usage: run_system.sh <fast_lio|faster_lio> <bag_path> [out_dir]
set -uo pipefail
SYS=${1:?usage: run_system.sh <fast_lio|faster_lio> <bag_path> [out_dir]}
BAG=${2:?bag path required}
REPO=/slam-bench
source /opt/ros/noetic/setup.bash

# Per-system glue — mirrors configs/systems.yaml (the plan's Output Adapter Table).
case "$SYS" in
  fast_lio)
    WS=/ws/fast_lio;   LAUNCH=$REPO/configs/launch/fast_lio.launch
    ODOM=/Odometry;    PROC=fastlio_mapping ;;
  faster_lio)
    WS=/ws/faster_lio; LAUNCH=$REPO/configs/launch/faster_lio.launch
    ODOM=/Odometry;    PROC=run_mapping_online ;;
  *) echo "unknown system: $SYS" >&2; exit 2 ;;
esac

if [ ! -f "$WS/devel/setup.bash" ]; then
  echo "workspace $WS not built — run 'docker compose run --rm build' first" >&2
  exit 3
fi
source "$WS/devel/setup.bash"

# BAG may be a single .bag, or a directory (all *.bag played in timestamp order as
# one merged stream — our recordings are split into consecutive 1-minute chunks).
if [ -d "$BAG" ]; then
  mapfile -t BAGS < <(find "$BAG" -maxdepth 1 -name '*.bag' | sort)
  [ "${#BAGS[@]}" -gt 0 ] || { echo "no *.bag in $BAG" >&2; exit 4; }
  BAGNAME=$(basename "$BAG")
else
  BAGS=("$BAG"); BAGNAME=$(basename "$BAG"); BAGNAME=${BAGNAME%.bag}
fi
# NAME overrides the dataset label (the bag mount basename is often just "bags").
BAGNAME=${NAME:-$BAGNAME}

# Playback rate (real-time multiplier). Default 5x; the rig test tolerates >=3x.
RATE=${RATE:-5.0}

OUT=${3:-$REPO/results/$BAGNAME/$SYS}
mkdir -p "$OUT"
echo "system=$SYS bags=${#BAGS[@]} rate=${RATE}x out=$OUT"

cleanup() {
  kill -INT "${REC_PID:-}" "${SAMP_PID:-}" 2>/dev/null || true
  sleep 1
  kill -INT "${LAUNCH_PID:-}" 2>/dev/null || true
  sleep 2
  kill "${ROSCORE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

roscore & ROSCORE_PID=$!
sleep 3
rosparam set use_sim_time true

# System under test (headless).
roslaunch "$LAUNCH" rviz:=false > "$OUT/run.log" 2>&1 & LAUNCH_PID=$!
sleep 5

# Recorders: ① trajectory, ③ resource trace.
python3 "$REPO/eval/record_tum.py"      --topic "$ODOM" --out "$OUT/trajectory.tum" & REC_PID=$!
python3 "$REPO/eval/sample_resource.py" --proc "$PROC"  --out "$OUT/resource.csv"   & SAMP_PID=$!

# Playback with /clock for completion + real-time-factor assessment.
echo "playing ${#BAGS[@]} bag(s) at rate ${RATE}x ..."
rosbag play --clock -r "$RATE" "${BAGS[@]}"
PLAY_EXIT=$?
sleep 2

cleanup
trap - EXIT

TRAJ_LINES=$(wc -l < "$OUT/trajectory.tum" 2>/dev/null || echo 0)
printf '{"system":"%s","bag":"%s","rate":%s,"bag_play_exit":%s,"traj_poses":%s}\n' \
  "$SYS" "$BAGNAME" "$RATE" "$PLAY_EXIT" "$TRAJ_LINES" > "$OUT/metrics.json"
echo "done -> $OUT  (poses=$TRAJ_LINES, play_exit=$PLAY_EXIT)"
