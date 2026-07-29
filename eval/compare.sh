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
#   plot_mode: xyz (default, 3D) | xy | xz | yz
#   ALL=true   overlay every run instead of each group's median run
set -uo pipefail
DS=${1:?usage: compare.sh <dataset> [xyz|xy|xz|yz]}
MODE=${2:-xyz}
RES=/slam-bench/results/$DS
[ -d "$RES" ] || { echo "no results dir: $RES" >&2; exit 1; }

# Which runs to draw. Five runs x two systems is ten curves, so by default each
# (system, preset) contributes only its median run by end_pos_m — which requires
# aggregate.py to have filled in `derived`. Groups where it has not fall back to
# every run, with a warning, rather than failing.
mapfile -t PICKS < <(ALL=${ALL:-false} python3 - "$RES" <<'PY'
import json, os, statistics, sys
from pathlib import Path

res = Path(sys.argv[1])
show_all = os.environ.get("ALL", "false").lower() == "true"

groups = {}
for tum in sorted(res.glob("*/*/run*/trajectory.tum")):
    run = tum.parent
    groups.setdefault((run.parent.parent.name, run.parent.name), []).append(run)

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
[ "${#PICKS[@]}" -gt 0 ] || { echo "no */*/run*/trajectory.tum under $RES" >&2; exit 2; }

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

# Always save the PDF; also open a live window when a display is available. Remove any
# prior PDF first — evo prompts interactively before overwriting, which deadlocks under
# a non-interactive service (no stdin).
rm -f "$RES/compare.pdf"
if [ -n "${DISPLAY:-}" ]; then
  evo_traj tum "${TUMS[@]}" --plot_mode "$MODE" --plot --save_plot "$RES/compare.pdf"
else
  MPLBACKEND=Agg evo_traj tum "${TUMS[@]}" --plot_mode "$MODE" --save_plot "$RES/compare.pdf"
fi

echo "saved -> $RES/compare.pdf  |  statistics: eval/aggregate.py results/$DS"
