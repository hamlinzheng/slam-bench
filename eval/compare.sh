#!/usr/bin/env bash
# Overlay trajectories for one dataset with evo.
#
# Statistics are NOT computed here — eval/aggregate.py owns every derived quantity and
# writes results/<dataset>/stats.txt. This script only draws.
#
# Drift is intentionally not reported — the datasets are not strictly closed-loop, so a
# start-end gap is not a valid drift measure (M-DRIFT applies to closed loops only).
#
# Produces results/<dataset>/compare.pdf (also shown live if a display exists).
#
# Usage (inside the container): eval/compare.sh <dataset> [plot_mode]
#   plot_mode: xy (default, plan view) | xyz | xz | yz
#              These are ground-vehicle routes on near-flat ground — one outdoor route
#              rises 14 m over 7.3 km — so the plan view is where the trajectory is
#              read, and a 3D view spends most of its third axis on vertical error.
#   ALL=true   overlay every run instead of each group's median run
#   PLOT=true  also open a live window (needs X11 reachable from the container)
#   GNSS=true  additionally draw compare_gnss.pdf — the same runs against the GNSS
#              track. A separate file, not another page: it needs Umeyama alignment
#              where compare.pdf deliberately uses --align_origin, and evo takes one
#              per invocation. Requires the bags mounted (they are, at /bags) and a
#              usable GNSS fix stream; on a dataset without one it says so and the
#              main plot is unaffected.
set -uo pipefail
DS=${1:?usage: compare.sh <dataset> [xy|xyz|xz|yz]}
MODE=${2:-xy}
RES=/slam-bench/results/$DS
[ -d "$RES" ] || { echo "no results dir: $RES" >&2; exit 1; }

# Which runs to draw. Five runs x two systems is ten curves, so by default each
# (system, preset) contributes only its median run by end_pos_m — which requires
# aggregate.py to have filled in `derived`. Groups where it has not fall back to
# every run, with a warning, rather than failing.
mapfile -t PICKS < <(ALL=${ALL:-false} python3 - "$RES" <<'PY'
import json, os, statistics, sys
from pathlib import Path

# This script arrives on stdin, so eval/ is not on sys.path (see eval/registry.py for
# why the module is not called systems.py).
sys.path.insert(0, "/slam-bench/eval")
import aggregate, registry

res = Path(sys.argv[1])
show_all = os.environ.get("ALL", "false").lower() == "true"

# aggregate owns the results-tree semantics here as everywhere, so the two steps cannot
# end up reporting different counts for the same omission.
disabled = registry.disabled_systems_or_warn("compare")
for system, reason, runs in aggregate.disabled_inventory(res, disabled):
    print(
        "compare: skipped {} (disabled: {}) — {} run(s) not drawn".format(
            system, reason, runs
        ),
        file=sys.stderr,
    )

groups = {}
for tum in sorted(res.glob("*/*/run*/trajectory.tum")):
    run = tum.parent
    system = run.parent.parent.name
    if system in disabled:
        continue
    groups.setdefault((system, run.parent.name), []).append(run)

for (system, preset), runs in sorted(groups.items()):
    chosen = runs
    if not show_all and len(runs) > 1:
        scored = []
        for run in runs:
            try:
                d = json.loads((run / "metrics.json").read_text()).get("derived") or {}
                scored.append((d["end_pos_m"], run))
            except (OSError, ValueError, KeyError):
                scored = None
                break
        if scored:
            median = statistics.median_low([v for v, _ in scored])
            chosen = [next(r for v, r in sorted(scored, key=lambda s: s[0]) if v == median)]
        else:
            print(
                "compare: {}/{} has runs without derived metrics — run aggregate first "
                "for median selection; drawing all of them".format(system, preset),
                file=sys.stderr,
            )
    for run in chosen:
        print("{}-{}-{}\t{}".format(system, preset, run.name, run / "trajectory.tum"))
PY
)
[ "${#PICKS[@]}" -gt 0 ] || {
  echo "no */*/run*/trajectory.tum to draw under $RES" >&2
  echo "  (a 'skipped' line above means the runs exist but their system is disabled)" >&2
  exit 2
}

# Stage each trajectory as <label>.tum so evo's legend shows system-preset-run
# (evo labels a trajectory by its file stem — the raw files are all trajectory.tum).
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
TUMS=()
for pick in "${PICKS[@]}"; do
  label=${pick%%$'\t'*}
  path=${pick#*$'\t'}
  cp "$path" "$TMP/$label.tum"
  TUMS+=("$TMP/$label.tum")
done
echo "overlaying ${#TUMS[@]} trajectory(ies): ${PICKS[*]%%$'\t'*}"

# Each system defines its world frame differently — FAST-LIO and faster-lio use the first
# body frame, Point-LIO gravity-aligns it (and gravity leaves yaw free). Measured 169.5°
# apart, drawing 5.4 m of real disagreement as 321.6 m. Aligning the first pose removes
# the convention without fitting the shape; on fast_lio it moves the RMSE by 0.5%.
#
# evo needs a --ref for --align_origin and draws it too, so the first trajectory becomes
# the reference and the rest stay positional. One trajectory has nothing to align to.
ALIGN=()
if [ "${#TUMS[@]}" -gt 1 ]; then
  ALIGN=(--ref "${TUMS[0]}" --align_origin)
  TUMS=("${TUMS[@]:1}")
fi

# Rendered to a scratch path and moved in only on success: evo prompts before overwriting,
# which deadlocks a service with no stdin, so writing straight to the destination meant
# deleting the old PDF first and losing it whenever a re-render failed.
#
# Headless by default, window opt-in. The old test was `[ -n "$DISPLAY" ]`, but DISPLAY comes
# from the host through docker-compose and says nothing about whether this container can reach
# an X server — on a desktop host it is always set, so the window branch was the default, and
# there `--plot` blocks on a window nobody can close and the PDF is never written.
OUT=$TMP/compare.pdf
if [ "${PLOT:-false}" = "true" ]; then
  evo_traj tum "${TUMS[@]}" ${ALIGN[@]+"${ALIGN[@]}"} \
    --plot_mode "$MODE" --plot --save_plot "$OUT"
else
  MPLBACKEND=Agg evo_traj tum "${TUMS[@]}" ${ALIGN[@]+"${ALIGN[@]}"} \
    --plot_mode "$MODE" --save_plot "$OUT"
fi
STATUS=$?

# Both conditions: evo can exit 0 having drawn nothing, so the file must also exist.
if [ "$STATUS" -ne 0 ] || [ ! -s "$OUT" ]; then
  echo "compare: evo wrote no plot (exit $STATUS)." >&2
  [ -f "$RES/compare.pdf" ] && echo "  $RES/compare.pdf is untouched, still the previous one." >&2
  exit 4
fi
mv -f "$OUT" "$RES/compare.pdf"

echo "saved -> $RES/compare.pdf  |  statistics: eval/aggregate.py results/$DS"

# ------------------------------------------------------------------ GNSS ----
# Non-fatal throughout: a missing or unusable fix stream must not cost the main plot,
# which was already written above.
#
# GNSS=true is only needed the first time. Building the reference scans every bag (24 GB
# on one dataset here) and needs BAGS_DIR pointed at it, neither of which should be a
# silent precondition of `compare`. Once it is built it is cached beside the results and
# costs nothing, so from then on it is drawn by default.
REF=$RES/gnss_ref.tum
if [ ! -s "$REF" ]; then
  [ "${GNSS:-false}" = "true" ] || exit 0
  python3 /slam-bench/eval/gnss_ref.py /bags --out "$RES"
fi

# The verdict is read from the report rather than from that exit code, so a cached
# reference is checked too. gnss_ref.py writes the report even when the answer is no —
# a dataset whose receiver kept emitting fixes underground still deserves the record of
# why its track is not a reference.
VERDICT=$(python3 - "$RES/gnss_ref.json" <<'PY'
import json, sys
try:
    r = json.loads(open(sys.argv[1]).read())
except (OSError, ValueError) as e:
    print("no reference report ({})".format(e))
else:
    print("" if r.get("usable") else "; ".join(r.get("unusable_because") or ["unusable"]))
PY
)
if [ -n "$VERDICT" ]; then
  echo "compare: GNSS track is not a reference for $DS — $VERDICT" >&2
  echo "  compare.pdf is unaffected; see $RES/gnss_ref.json" >&2
  exit 0
fi

# Same label=path pairs the overlay used, so the two figures name the same curves.
GARGS=()
for pick in "${PICKS[@]}"; do
  GARGS+=("${pick%%$'\t'*}=${pick#*$'\t'}")
done

GOUT=$TMP/compare_gnss.pdf
if python3 /slam-bench/eval/plot_gnss.py \
     --ref "$REF" --report "$RES/gnss_ref.json" --out "$GOUT" "${GARGS[@]}" \
   && [ -s "$GOUT" ]; then
  mv -f "$GOUT" "$RES/compare_gnss.pdf"
  echo "saved -> $RES/compare_gnss.pdf  |  reference quality: $RES/gnss_ref.json"
else
  echo "compare: GNSS overlay not written; $RES/compare.pdf is unaffected." >&2
fi
