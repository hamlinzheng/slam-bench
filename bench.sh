#!/usr/bin/env bash
# slam-bench entry point — the only script meant to be executed by hand. Everything
# under scripts/ and eval/ is invoked by docker compose inside a container.
#
#   ./bench.sh setup                 build the docker image
#   ./bench.sh build                 compile the systems into .ws/
#   ./bench.sh run                   N benchmark runs (see the env table below)
#   ./bench.sh aggregate <dataset>   N-run statistics -> stats.txt + stats.json
#   ./bench.sh compare   <dataset>   evo trajectory overlay -> compare.pdf
#   ./bench.sh shell                 interactive container
#
# Anything after `--` is passed straight to `docker compose`, so a flag this wrapper
# does not expose is never a dead end:
#   ./bench.sh compare mydataset -- --no-deps
set -uo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$REPO/scripts/lib.sh"
COMPOSE=(docker compose -f "$REPO/docker/docker-compose.yml")

# Every container runs as the invoking user (docker-compose.yml `user:`), so nothing it
# writes into results/ or .ws/ comes back root-owned.
export HOST_UID HOST_GID
HOST_UID=$(id -u)
HOST_GID=$(id -g)

# Create the bind-mount sources here rather than letting Docker do it. A missing source
# directory is created by the daemon, which runs as root, and the unprivileged container
# then cannot write into it — `mkdir /ws/<system>: Permission denied` on a clean rebuild
# after .ws/ has been deleted.
mkdir -p "$REPO/.ws" "$REPO/results"

die() { echo "bench: $*" >&2; exit 1; }

usage() {
  # The header comment IS the help text — printed up to the first line of code, so it
  # cannot drift out of sync with a hardcoded line range.
  awk 'NR>1 && /^#/ { sub(/^# ?/, ""); print; next } NR>1 { exit }' "${BASH_SOURCE[0]}"
  cat <<'EOF'

run — environment:
  BAGS_DIR   required   host bag folder, mounted read-only at /bags
  NAME       required   the results/<dataset> label
  SYS        required   fast_lio | faster_lio | point_lio | super_lio | pv_lio | bievr_lio
  N          1          runs per (system, preset)
  PRESET     default    one or more presets, space separated
  RATE       1.0        playback speed multiplier (1x is where latency means anything)
  BAG        /bags      bag path inside the container
  RUN/FORCE  -          re-run one index; only accepted when the batch is a single run
  RVIZ       false      open the system's rviz (needs xhost +local:root)

  ./bench.sh run --dry-run    print the plan without starting anything

aggregate — options (forwarded to eval/aggregate.py, which defines them):
  --split-at <metres>   also bucket runs either side of this end_pos_m
                        (no default: the boundary is dataset-dependent)
  --min-coverage <f>    fraction of the bag a trajectory must span to count as
                        completed (default 0.9)

compare — options:
  [xyz|xy|xz|yz]        plot mode, default xyz
  ALL=true              overlay every run instead of each group's median
  DISPLAY=              force headless (save the PDF, open no window)
EOF
}

# Split `<args> -- <compose args>`; both halves may be empty.
ARGS=(); PASS=()
split_args() {
  local seen=false a
  for a in "$@"; do
    if $seen; then PASS+=("$a")
    elif [ "$a" = "--" ]; then seen=true
    else ARGS+=("$a"); fi
  done
}

compose_run() {   # compose_run <extra-flags...> -- <service>
  "${COMPOSE[@]}" run --rm "$@"
}

RUN_CONTAINER=""
ABORT=false
on_interrupt() {
  echo >&2
  if $ABORT; then
    echo "bench: second interrupt — exiting now" >&2
    [ -n "$RUN_CONTAINER" ] && docker rm -f "$RUN_CONTAINER" >/dev/null 2>&1
    exit 130
  fi
  ABORT=true
  if [ -n "$RUN_CONTAINER" ]; then
    echo "bench: interrupted — stopping $RUN_CONTAINER" >&2
    # `docker stop`, not `docker kill`: kill defaults to SIGKILL, which cannot be
    # trapped, so run_system.sh's EXIT handler never runs and the run vanishes without
    # a metrics.json — invisible to aggregate.py, shrinking n silently. stop sends
    # SIGTERM first and only escalates after the grace period.
    docker stop --timeout 30 "$RUN_CONTAINER" >/dev/null 2>&1 || true
  else
    echo "bench: interrupted" >&2
  fi
}

dataset_arg() {   # first positional, validated against results/
  local ds=${ARGS[0]:-}
  [ -n "$ds" ] || die "$1 needs a dataset name (a directory under results/)"
  [ -d "$REPO/results/$ds" ] || die "no such dataset: results/$ds"
  echo "$ds"
}

cmd_setup() {
  split_args "$@"
  "${COMPOSE[@]}" build ${PASS[@]+"${PASS[@]}"} build
}

cmd_build() {
  split_args "$@"
  compose_run -T ${PASS[@]+"${PASS[@]}"} build
}

cmd_aggregate() {
  split_args "$@"
  local ds
  ds=$(dataset_arg aggregate) || exit 1
  # Everything after the dataset is handed to aggregate.py untouched, so its argparse
  # stays the only place an option is declared — and it, not this wrapper, reports a
  # bad flag.
  DS="$ds" AGG_ARGS="${ARGS[*]:1}" compose_run -T ${PASS[@]+"${PASS[@]}"} aggregate
}

cmd_compare() {
  split_args "$@"
  local ds mode
  ds=$(dataset_arg compare) || exit 1
  mode=${ARGS[1]:-xyz}
  case "$mode" in xyz|xy|xz|yz) ;; *) die "compare: plot mode must be xyz|xy|xz|yz, got '$mode'" ;; esac
  DS="$ds" MODE="$mode" compose_run -T ${PASS[@]+"${PASS[@]}"} compare
}

cmd_shell() {
  split_args "$@"
  # No -T here: this one is interactive by definition.
  compose_run ${PASS[@]+"${PASS[@]}"} dev
}

# ---------------------------------------------------------------- run ----------
# Runs each (system, preset) N times, one fresh container per run, so the
# repeated runs that establish a noise floor cannot perturb one another.
cmd_run() {
  split_args "$@"
  local DRY_RUN=false
  [ "${ARGS[0]:-}" = "--dry-run" ] && DRY_RUN=true

  local SYSTEMS=${SYS:-} DATASET=${NAME:-} BAGS=${BAGS_DIR:-}
  [ -n "$SYSTEMS" ] || die "SYS is required (e.g. SYS=\"fast_lio faster_lio\")"
  [ -n "$DATASET" ] || die "NAME is required (the results/<dataset> label)"
  [ -n "$BAGS" ]    || die "BAGS_DIR is required (host bag directory)"
  local PRESETS=${PRESET:-$BENCH_DEFAULT_PRESET}
  local RATE=${RATE:-$BENCH_DEFAULT_RATE}
  local BAG=${BAG:-$BENCH_DEFAULT_BAG}
  local N=${N:-1}

  # Pre-flight, all of it, before the first container: a batch of five is roughly
  # twenty minutes, so a mistyped preset name must fail now, not after run four.
  [ -d "$BAGS" ] || die "BAGS_DIR does not exist: $BAGS"
  case "$N" in ''|*[!0-9]*) die "N must be a positive integer, got '$N'" ;; esac
  [ "$N" -ge 1 ] || die "N must be at least 1"
  local sys preset f
  for sys in $SYSTEMS; do
    [ -f "$REPO/.ws/$sys/devel/setup.bash" ] \
      || die "workspace not built for '$sys' — run './bench.sh build' first"
    for preset in $PRESETS; do
      f=$REPO/$(preset_launch "$sys" "$preset")
      [ -f "$f" ] || die "no launch file for preset '$preset' of '$sys': $f"
    done
  done

  # Every service runs with network_mode: host, so a live ROS master on 11311 is shared.
  # A second run's roscore then fails to bind and its nodes silently register with the
  # existing master — both runs publish /Odometry into each other's recording and both
  # trajectories are ruined, while each still reports bag_play_exit 0. Refuse to start.
  if timeout 2 bash -c 'exec 3<>/dev/tcp/127.0.0.1/11311' 2>/dev/null; then
    die "a ROS master is already listening on 11311 — another run is still alive, and this
     one would join it (both trajectories would be contaminated). Stop it with:
       docker rm -f \$(docker ps -q --filter ancestor=slam-bench:noetic)"
  fi
  local stale
  stale=$(docker ps --filter ancestor=slam-bench:noetic --format '{{.Names}}' 2>/dev/null)
  [ -z "$stale" ] || echo "bench: warning — slam-bench containers already running: $(echo "$stale" | tr '\n' ' ')" >&2

  local TOTAL=$((N * $(echo "$SYSTEMS" | wc -w) * $(echo "$PRESETS" | wc -w)))
  # RUN names one specific index, so it cannot address more than one run. Without this
  # guard the first run takes the index and every later one fails on the occupied
  # directory — reported, but only after the whole batch has burned its time.
  [ -z "${RUN:-}" ] || [ "$TOTAL" -eq 1 ] \
    || die "RUN=$RUN addresses a single run, but this batch plans $TOTAL — drop RUN, or narrow SYS/PRESET/N"

  echo "dataset=$DATASET systems=[$SYSTEMS] presets=[$PRESETS] N=$N rate=${RATE}x -> $TOTAL run(s)"

  # Ctrl-C must actually stop the run in progress, not merely stop launching new ones.
  # `docker compose run` cannot be relied on to take the container down with it, so the
  # container is given a name and killed by name here. A second Ctrl-C exits immediately.
  # Both are global on purpose: on_interrupt runs from a trap and must see the same
  # variables, and a `local` here would shadow them under bash's dynamic scoping.
  ABORT=false
  RUN_CONTAINER=""
  trap on_interrupt INT

  local OK=0 FAILED=0 i label log status out seq=0
  local DIRS=() FAILS=()
  for preset in $PRESETS; do
    for sys in $SYSTEMS; do
      for i in $(seq 1 "$N"); do
        $ABORT && break 3
        label="$sys/$preset $i/$N"
        local env_prefix=(BAGS_DIR="$BAGS" SYS="$sys" PRESET="$preset" BAG="$BAG"
                          RATE="$RATE" NAME="$DATASET" RUN="${RUN:-}"
                          FORCE="${FORCE:-}" RVIZ="${RVIZ:-}")

        if $DRY_RUN; then
          echo "[dry-run] $label -> ${env_prefix[*]} ${COMPOSE[*]} run --rm -T run"
          DIRS+=("results/$DATASET/$sys/$preset/run??"); OK=$((OK + 1))
          continue
        fi

        echo; echo "=== $label ==="
        log=$(mktemp)
        seq=$((seq + 1))
        RUN_CONTAINER="slam-bench-run-$$-$seq"
        # -T: no pseudo-TTY. A benchmark run needs no terminal, and an allocated TTY
        # both mangles the piped output and fails when stdin is not a terminal.
        # --name: so the interrupt handler can take this exact container down.
        #
        # setsid -w: put compose in its own session so the terminal's Ctrl-C does NOT
        # reach it. Measured, `docker compose run` responds to SIGINT by destroying the
        # container outright — a container with an explicit INT/TERM trap never got to
        # run it — so the run would die without writing metrics.json. Shielded here,
        # the only thing that stops the container is on_interrupt's graceful
        # `docker stop`, and run_system.sh's EXIT handler gets to finish.
        #
        # Backgrounded and waited on, for the same reason run_system.sh backgrounds
        # playback: bash defers a trapped signal until the running FOREGROUND command
        # ends. Left in the foreground, on_interrupt would only fire once the run had
        # already finished — Ctrl-C would again appear to do nothing. `tail --pid`
        # streams the log meanwhile and exits with compose.
        setsid -w env "${env_prefix[@]}" "${COMPOSE[@]}" run --rm -T --name "$RUN_CONTAINER" \
          ${PASS[@]+"${PASS[@]}"} run > "$log" 2>&1 &
        local compose_pid=$!
        tail -n +1 -f --pid="$compose_pid" "$log" 2>/dev/null &
        local tail_pid=$!
        wait "$compose_pid"; status=$?
        wait "$tail_pid" 2>/dev/null || true
        docker rm -f "$RUN_CONTAINER" >/dev/null 2>&1 || true
        RUN_CONTAINER=""

        # run_system.sh announces its output directory as `out=<container path>`.
        out=$(grep -oE 'out=[^ ]+' "$log" | tail -1 | cut -d= -f2)
        out=${out/#\/slam-bench/$REPO}
        rm -f "$log"

        # A crashed run is a data point, not a reason to stop: the explosion mode
        # is exactly what these batches exist to measure.
        if [ "$status" -eq 0 ]; then
          OK=$((OK + 1)); DIRS+=("${out:-?}")
        else
          FAILED=$((FAILED + 1)); FAILS+=("$label (exit $status) ${out:-?}")
        fi
      done
    done
  done
  trap - INT

  echo
  echo "=== $OK ok, $FAILED failed, of $TOTAL planned ==="
  for out in ${DIRS[@]+"${DIRS[@]}"}; do echo "  ok      $out"; done
  for out in ${FAILS[@]+"${FAILS[@]}"}; do echo "  FAILED  $out"; done
  $ABORT && echo "  (interrupted before completing)"

  echo
  echo "next:  ./bench.sh aggregate $DATASET"
  [ "$FAILED" -eq 0 ]
}

cmd=${1:-help}
[ $# -gt 0 ] && shift
case "$cmd" in
  setup)            cmd_setup "$@" ;;
  build)            cmd_build "$@" ;;
  run)              cmd_run "$@" ;;
  aggregate|agg)    cmd_aggregate "$@" ;;
  compare)          cmd_compare "$@" ;;
  shell|dev)        cmd_shell "$@" ;;
  help|-h|--help)   usage ;;
  *) echo "bench: unknown command '$cmd'" >&2; echo >&2; usage >&2; exit 1 ;;
esac
