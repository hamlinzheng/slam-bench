#!/usr/bin/env bash
# Run one baseline once, over one bag, and capture the normalized artifacts:
#   ① trajectory.tum   — odom topic recorded to TUM
#   ③ resource.csv     — external /proc CPU% + RSS sampling of the system process
#   ③ frame_events.csv — arrival wall clock of every input scan and every output odom,
#                        from which eval/aggregate.py derives per-frame latency
#      run.log         — system stdout/stderr
#      metrics.json    — run record: completion + full provenance (accuracy metrics later)
#
# Output goes to results/<dataset>/<system>/<preset>/run<NN>/ — repeated runs never
# overwrite each other. Aggregate them with eval/aggregate.py.
#
# INTERNAL: this runs INSIDE the container, invoked by the compose `run` service. The
# entry point meant for humans is ./run.sh at the repository root, which drives this
# one container per run and adds the repeat/sweep loop.
#
# Usage: run_system.sh <fast_lio|faster_lio> <bag_path> [out_dir]
#   PRESET=<name>  configuration variant (default: `default`)
#   RUN=<n>        explicit run index (default: next free)
#   FORCE=true     allow RUN to overwrite an existing run directory
#   NAME=<label>   dataset label   RATE=<x> playback speed   RVIZ=true open rviz
set -uo pipefail
SYS=${1:?usage: run_system.sh <fast_lio|faster_lio> <bag_path> [out_dir]}
BAG=${2:?bag path required}
REPO=/slam-bench
source "$REPO/scripts/lib.sh"
source /opt/ros/noetic/setup.bash

# Per-system glue — mirrors configs/systems.yaml (the plan's Output Adapter Table).
# BIN and SRC exist for provenance: the binary hash catches a patched build whose
# source has since been reverted, which git state cannot see.
case "$SYS" in
  fast_lio)
    WS=/ws/fast_lio;   ODOM=/Odometry;  PROC=fastlio_mapping
    BIN=$WS/devel/lib/fast_lio/fastlio_mapping
    SRC=$REPO/systems/FAST_LIO ;;
  faster_lio)
    WS=/ws/faster_lio; ODOM=/Odometry;  PROC=run_mapping_online
    BIN=$WS/devel/lib/faster_lio/run_mapping_online
    SRC=$REPO/systems/faster-lio ;;
  point_lio)
    WS=/ws/point_lio;  ODOM=/aft_mapped_to_init; PROC=pointlio_mapping
    BIN=$WS/devel/lib/point_lio/pointlio_mapping
    SRC=$REPO/systems/Point-LIO ;;
  super_lio)
    WS=/ws/super_lio;  ODOM=/lio/odom; PROC=super_lio_node
    BIN=$WS/devel/lib/super_lio/super_lio_node
    SRC=$REPO/systems/Super-LIO ;;
  pv_lio)
    WS=/ws/pv_lio;     ODOM=/Odometry; PROC=pv_lio_node
    BIN=$WS/devel/lib/pv_lio/pv_lio_node
    SRC=$REPO/systems/PV-LIO ;;
  bievr_lio)
    WS=/ws/bievr_lio;  ODOM=/bievr_lio/odom; PROC=process_topics
    BIN=$WS/devel/lib/bievr_lio_ros/process_topics
    SRC=$REPO/systems/BIEVR-LIO ;;
  *) echo "unknown system: $SYS" >&2; exit 2 ;;
esac

if [ ! -f "$WS/devel/setup.bash" ]; then
  echo "workspace $WS not built — run 'docker compose run --rm build' first" >&2
  exit 3
fi
source "$WS/devel/setup.bash"

# Preset resolution lives in scripts/lib.sh so the host entry point applies the same
# rule during pre-flight — the rule exists once, not once per side.
PRESET=${PRESET:-$BENCH_DEFAULT_PRESET}
LAUNCH=$REPO/$(preset_launch "$SYS" "$PRESET")
[ -f "$LAUNCH" ] || { echo "no preset launch file: $LAUNCH" >&2; exit 5; }

# The parameters live in the launch plus every YAML it loads through an <arg name="config...">.
# A system may have more than one (BIEVR-LIO takes a sensor config and a params file); all are
# hashed, in launch order. A value that is not a readable file is skipped — it may be a name.
mapfile -t CFGS < <(sed -n 's/.*name="config[^"]*"[^>]*default="\([^"]*\)".*/\1/p' "$LAUNCH")
KEPT=(); for c in "${CFGS[@]}"; do [ -f "$c" ] && KEPT+=("$c"); done
CFGS=("${KEPT[@]}")

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
DATASET=${NAME:-$BAGNAME}

# Playback rate (real-time multiplier). Defaults to 1x, where latency means anything; the rig tolerates >=3x
# if you are trading real-time fidelity for wall-clock on a smoke run.
RATE=${RATE:-$BENCH_DEFAULT_RATE}

# Refuse to join a master this run did not start. Every service uses network_mode: host,
# so a surviving container keeps port 11311 bound; the roscore below would then fail to
# bind while every node silently registers with the stale master, merging two runs'
# /Odometry into both trajectories. Observed in practice.
if timeout 2 bash -c 'exec 3<>/dev/tcp/127.0.0.1/11311' 2>/dev/null; then
  echo "a ROS master is already listening on 127.0.0.1:11311 — another run is still alive." >&2
  echo "this run would join it and both trajectories would be contaminated. Stop it first:" >&2
  echo "  docker rm -f \$(docker ps -q --filter ancestor=slam-bench:noetic)" >&2
  exit 7
fi

# Bag time bounds, recorded so aggregate.py can check that the trajectory actually came
# from THIS playback and covered it.
#
# Probed in the BACKGROUND: reading a bag index costs ~0.3 s, so on a 17-bag dataset this
# is ~5 s — and the value is not needed until metrics.json is written, at the very end.
# On the critical path it delayed even the first line of output.
BAG_START=""; BAG_END=""
BOUNDS_FILE=$(mktemp)
{
  for b in "${BAGS[@]}"; do
    # One invocation per bag, both keys parsed from it: two `-k` calls cost twice as much
    # for the same information. `^` anchors keep the nested per-topic keys out.
    rosbag info -y "$b" 2>/dev/null | sed -n 's/^\(start\|end\): *//p'
  done | python3 -c 'import sys
v = [float(x) for x in sys.stdin.read().split() if x.strip()]
print("%.6f %.6f" % (min(v), max(v)) if v else " ")'
} > "$BOUNDS_FILE" 2>/dev/null &
BOUNDS_PID=$!

if [ -n "${3:-}" ]; then
  OUT=$3
else
  PRESET_DIR=$REPO/results/$DATASET/$SYS/$PRESET
  mkdir -p "$PRESET_DIR"
  OUT=$(python3 "$REPO/eval/next_run_dir.py" "$PRESET_DIR" \
          ${RUN:+--run "$RUN"} ${FORCE:+--force}) || exit 6
fi
mkdir -p "$OUT"
RUN_INDEX=$(basename "$OUT"); RUN_INDEX=${RUN_INDEX#run}; RUN_INDEX=${RUN_INDEX#0}
echo "system=$SYS preset=$PRESET bags=${#BAGS[@]} rate=${RATE}x out=$OUT"

STARTED_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)
START_S=$SECONDS
PLAY_EXIT=""   # stays empty (null) if we die before playback finishes
SYS_ALIVE=""   # ditto: whether the system under test outlived the playback

sha_of() { [ -n "$1" ] && [ -f "$1" ] && sha256sum "$1" | cut -d' ' -f1 || true; }
# git: the container runs as root against a host-owned mount, so entrypoint.sh must
# have marked it safe.directory — otherwise these silently yield "" (recorded null).
git_commit() { git -C "$1" rev-parse --short HEAD 2>/dev/null || true; }
git_dirty() {
  git -C "$1" rev-parse HEAD >/dev/null 2>&1 || return 0
  if [ -n "$(git -C "$1" status --porcelain 2>/dev/null)" ]; then echo true; else echo false; fi
}

write_metrics() {
  # Collect the background bag-bounds probe started above; by now it has long finished.
  wait "${BOUNDS_PID:-}" 2>/dev/null || true
  read -r BAG_START BAG_END < "$BOUNDS_FILE" 2>/dev/null || true
  rm -f "$BOUNDS_FILE"

  TRAJ_LINES=$(wc -l < "$OUT/trajectory.tum" 2>/dev/null || echo 0)
  if [ "$PLAY_EXIT" = "0" ] && [ "$SYS_ALIVE" = "true" ] && [ "$TRAJ_LINES" -ge 2 ]; then
    STATUS=ok
  else
    STATUS=failed
  fi

  REL=("${LAUNCH#$REPO/}"); for c in "${CFGS[@]}"; do REL+=("${c#$REPO/}"); done

  M_SYSTEM=$SYS M_PRESET=$PRESET M_RUN=$RUN_INDEX M_DATASET=$DATASET M_RATE=$RATE \
  M_PLATFORM=$(uname -m) M_STATUS=$STATUS M_STARTED_AT=$STARTED_AT \
  M_OMP_WAIT_POLICY=$OMP_WAIT_POLICY M_SYS_ALIVE=$SYS_ALIVE \
  M_CPU_GOVERNOR=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || true) \
  M_WALL_S=$((SECONDS - START_S)) M_PLAY_EXIT=$PLAY_EXIT M_POSES=$TRAJ_LINES \
  M_BAG_START=$BAG_START M_BAG_END=$BAG_END \
  M_PRESET_SHA=$(cat "$LAUNCH" "${CFGS[@]}" | sha256sum | cut -d' ' -f1) \
  M_PRESET_FILES=$(IFS=:; echo "${REL[*]}") \
  M_BINARY_SHA=$(sha_of "$BIN") \
  M_SYSTEM_COMMIT=$(git_commit "$SRC") M_SYSTEM_DIRTY=$(git_dirty "$SRC") \
  M_BENCH_COMMIT=$(git_commit "$REPO") M_BENCH_DIRTY=$(git_dirty "$REPO") \
  python3 - "$OUT/metrics.json" <<'PY'
import json, os, sys

def text(key):
    """Empty means "could not be measured" — recorded as null, never as "" or 0."""
    return os.environ.get(key) or None

def num(key, cast=float):
    v = os.environ.get(key)
    try:
        return cast(v)
    except (TypeError, ValueError):
        return None

def flag(key):
    v = os.environ.get(key)
    return {"true": True, "false": False}.get(v)

files = os.environ.get("M_PRESET_FILES", "")
record = {
    "system": text("M_SYSTEM"),
    "preset": text("M_PRESET"),
    "run": num("M_RUN", int),
    "dataset": text("M_DATASET"),
    "rate": num("M_RATE"),
    "platform": text("M_PLATFORM"),
    # CPU% only compares across runs sharing this. Absent = predates it, i.e. libgomp's default.
    "omp_wait_policy": text("M_OMP_WAIT_POLICY"),
    # The host's CPU frequency governor. Under `powersave` the clock tracks demand, so a
    # lightly-loaded run executes the same frame more slowly than a saturated one — every
    # per-frame timing here only compares across runs that agree on this.
    "cpu_governor": text("M_CPU_GOVERNOR"),
    "status": text("M_STATUS"),
    "started_at": text("M_STARTED_AT"),
    "wall_s": num("M_WALL_S", int),
    "bag_play_exit": num("M_PLAY_EXIT", int),
    "system_alive": flag("M_SYS_ALIVE"),   # false = died before playback ended
    "traj_poses": num("M_POSES", int),
    "bag_start": num("M_BAG_START"),
    "bag_end": num("M_BAG_END"),
    "preset_sha": text("M_PRESET_SHA"),
    "preset_files": [f for f in files.split(":") if f],
    "binary_sha": text("M_BINARY_SHA"),
    "system_commit": text("M_SYSTEM_COMMIT"),
    "system_dirty": flag("M_SYSTEM_DIRTY"),
    "bench_commit": text("M_BENCH_COMMIT"),
    "bench_dirty": flag("M_BENCH_DIRTY"),
    "derived": {},
}
with open(sys.argv[1], "w") as fh:
    json.dump(record, fh, indent=2, sort_keys=True)
    fh.write("\n")
PY
  echo "done -> $OUT  (status=$STATUS, poses=$TRAJ_LINES, play_exit=${PLAY_EXIT:-none})"
}

cleanup() {
  kill -INT "${PLAY_PID:-}" 2>/dev/null || true
  kill -INT "${REC_PID:-}" "${SAMP_PID:-}" "${FRAMES_PID:-}" 2>/dev/null || true
  rm -f "${FRAMES_READY:-}" 2>/dev/null || true
  sleep 1
  kill -INT "${LAUNCH_PID:-}" 2>/dev/null || true
  sleep 2
  kill "${ROSCORE_PID:-}" 2>/dev/null || true
}

# metrics.json is written on EVERY exit path, not just the happy one: a run that dies
# mid-way must still leave a record, or aggregate.py cannot see that it happened and
# n silently shrinks — losing failure samples exactly when the failure rate is the
# measurement.
finish() { cleanup; write_metrics; }
trap finish EXIT

# These handlers are not politeness — they are what makes the container stoppable at all.
# This script is PID 1 inside the container, and Linux delivers a signal to PID 1 only if
# that process installed an explicit handler; anything left at its default disposition is
# discarded by the kernel. Without these traps `docker stop`, `docker kill -s INT` and
# Ctrl-C all leave the run alive, holding the ROS master for the next one.
on_signal() {
  echo "run_system: signal received — shutting down" >&2
  exit 130   # the EXIT trap does the cleanup and still writes metrics.json
}
trap on_signal INT TERM

# Wait for a condition rather than a fixed duration. Measured, the fixed sleeps this
# replaces were both wrong in both directions: roscore is usable in 0.8 s and the node
# advertises in 0.7 s (so 3 s + 5 s wasted ~6 s per run), while on a loaded machine a
# fixed wait can be too SHORT — and starting playback before the node is up silently
# loses the opening scans, which no check would attribute to a race.
wait_for() {   # wait_for <timeout_s> <what> <command...>
  # $1 must be captured before the shift: afterwards it is the command's first word, and
  # using it in $(( )) evaluated that word as a variable — which under set -u killed the
  # timeout path with "unbound variable" instead of reporting the timeout.
  local timeout=$1 what=$2
  local deadline=$((SECONDS + timeout))
  shift 2
  until "$@" >/dev/null 2>&1; do
    if [ "$SECONDS" -ge "$deadline" ]; then
      echo "timed out after ${timeout}s waiting for $what" >&2
      return 1
    fi
    sleep 0.2
  done
}

roscore & ROSCORE_PID=$!
wait_for 60 "roscore" rostopic list || exit 8
rosparam set use_sim_time true

# Park idle OpenMP workers rather than let them spin between parallel regions: /proc counts
# spinning as CPU, so artifact ③ would measure each system's thread cap as much as its cost
# (PV-LIO 1069% -> 327%, throughput unchanged). `active` restores the as-shipped default.
export OMP_WAIT_POLICY=${OMP_WAIT_POLICY:-passive}

# System under test. RVIZ=true opens the system's rviz (needs X11: xhost +local:root on host).
roslaunch "$LAUNCH" rviz:="${RVIZ:-false}" > "$OUT/run.log" 2>&1 & LAUNCH_PID=$!
# Ready means "subscribed to the input", not "advertising its output". Those coincide only for
# systems that advertise eagerly; BIEVR-LIO advertises on first publish, so waiting for its
# odometry deadlocks against playback that has not started. Subscription is also the signal
# this gate actually wants — it is what makes the opening scans reach the system.
wait_for 120 "$SYS to subscribe to $BENCH_LIDAR_TOPIC (see $OUT/run.log)" \
  bash -c "rostopic list -s | grep -qx '$BENCH_LIDAR_TOPIC'" || exit 9

# Recorders: ① trajectory, ③ resource trace + frame event trace.
#
# All three start AFTER the subscription gate above, and record_frames.py must: that gate
# asks whether $BENCH_LIDAR_TOPIC has a subscriber, and record_frames.py subscribes to it
# too — started any earlier it would hold the gate open on the system under test's behalf
# and playback would begin before the system was listening.
python3 "$REPO/eval/record_tum.py"      --topic "$ODOM" --out "$OUT/trajectory.tum" & REC_PID=$!
python3 "$REPO/eval/sample_resource.py" --proc "$PROC"  --out "$OUT/resource.csv"   & SAMP_PID=$!
FRAMES_READY=$(mktemp -u)   # -u: a name, not a file — its appearance is the signal
python3 "$REPO/eval/record_frames.py" --in-topic "$BENCH_LIDAR_TOPIC" --odom "$ODOM" \
  --out "$OUT/frame_events.csv" --ready-file "$FRAMES_READY" & FRAMES_PID=$!

# Playback must not start until the recorder is actually subscribed, or the opening poses
# are dropped. (This is not what makes a healthy run's coverage 0.99 rather than 1.00 —
# measured, that is FAST-LIO's own ~0.37 s IMU initialisation before its first odometry,
# and closing this race did not change it. The race is real but was not the cause.)
wait_for 60 "the trajectory recorder to subscribe to $ODOM" \
  bash -c "rostopic info '$ODOM' | sed -n '/Subscribers:/,\$p' | grep -q '\*'" || exit 10

# Same race, same reason, for the frame recorder — an input scan that arrives before it is
# listening is one this run can never account for, and out_ratio would report the system
# as having dropped it. The ready file appears once both of its subscriptions exist.
wait_for 60 "the frame recorder to subscribe to $BENCH_LIDAR_TOPIC and $ODOM" \
  test -e "$FRAMES_READY" || exit 11
rm -f "$FRAMES_READY"

# Playback with /clock for completion + real-time-factor assessment.
#
# Backgrounded and waited on rather than run in the foreground: bash defers a trapped
# signal until the running foreground command finishes, so with `rosbag play` in the
# foreground the handlers above could not fire until playback ended — the whole point of
# being interruptible. `wait` instead returns immediately when a trapped signal arrives.
echo "playing ${#BAGS[@]} bag(s) at rate ${RATE}x ..."
rosbag play --clock -r "$RATE" "${BAGS[@]}" & PLAY_PID=$!
wait "$PLAY_PID"
PLAY_EXIT=$?
sleep 2

# A system that dies mid-bag does not stop playback, so exit code and pose count alone score a
# crash as a clean run — PV-LIO was OOM-killed at 75% of a bag and still reported status=ok.
# roslaunch exits with its last node, so its state is the signal; read /proc rather than use
# `kill -0`, which reports a not-yet-reaped zombie as alive. (RVIZ=true adds a second node and
# would defeat this — one more reason that mode is not for measurement runs.)
SYS_ALIVE=$(awk '{ sub(/^.*\) /, ""); print ($1 == "Z") ? "false" : "true" }' \
              "/proc/${LAUNCH_PID}/stat" 2>/dev/null || echo false)
[ -n "$SYS_ALIVE" ] || SYS_ALIVE=false
