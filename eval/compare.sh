#!/usr/bin/env bash
# Overlay every system's trajectory for one dataset with evo, and report end-pose |pos|.
# Drift is intentionally NOT computed — the datasets are not strictly closed-loop, so a
# start-end gap is not a valid drift measure (plan §6 M-DRIFT applies to closed loops only).
#
# Produces, under results/<dataset>/:
#   compare.pdf   — evo_traj overlay of all systems (also opened live if a display exists)
#   summary.txt   — per-system poses / path length / end-pose |pos|
#
# Usage (inside the container): eval/compare.sh <dataset> [plot_mode]
#   plot_mode: xyz (default, 3D) | xy | xz | yz
set -uo pipefail
DS=${1:?usage: compare.sh <dataset> [xyz|xy|xz|yz]}
MODE=${2:-xyz}
RES=/slam-bench/results/$DS
[ -d "$RES" ] || { echo "no results dir: $RES" >&2; exit 1; }

# Stage each system's trajectory as <system>.tum so evo's legend shows system names
# (evo labels a trajectory by its file stem — raw files are all named trajectory.tum).
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
TUMS=()
for d in "$RES"/*/; do
  s=$(basename "$d")
  [ -f "$d/trajectory.tum" ] || continue
  cp "$d/trajectory.tum" "$TMP/$s.tum"
  TUMS+=("$TMP/$s.tum")
done
[ "${#TUMS[@]}" -gt 0 ] || { echo "no */trajectory.tum under $RES" >&2; exit 2; }

# Combined summary: trajectory (end-pose |pos|, drift skipped — not closed-loop) +
# resource usage from resource.csv (cpu_mean/cpu_max %, rss peak MB).
{
  echo "# $DS — end-pose |pos| (drift skipped: not closed-loop) + resource usage"
  printf "%-24s %8s %12s %12s %10s %10s %11s\n" \
    system poses path_len_m end_pos_m cpu_mean% cpu_max% rss_max_MB
  for d in "$RES"/*/; do
    s=$(basename "$d")
    [ -f "$d/trajectory.tum" ] || continue
    traj=$(awk '{ n++; if (NR > 1) { dx=$2-px; dy=$3-py; dz=$4-pz; L += sqrt(dx*dx+dy*dy+dz*dz) }
                  px=$2; py=$3; pz=$4 }
                END { printf "%d %.1f %.1f", n, L, sqrt(px*px+py*py+pz*pz) }' "$d/trajectory.tum")
    if [ -f "$d/resource.csv" ]; then
      res=$(awk -F, 'NR>1 { n++; c+=$2; if ($2>cm) cm=$2; if ($3>rm) rm=$3 }
                     END { if (n) printf "%.0f %.0f %.0f", c/n, cm, rm; else printf "- - -" }' "$d/resource.csv")
    else
      res="- - -"
    fi
    # shellcheck disable=SC2086
    printf "%-24s %8s %12s %12s %10s %10s %11s\n" "$s" $traj $res
  done
} | tee "$RES/summary.txt"

# Overlay: always save the PDF; also open a live window when a display is available.
# Remove any prior PDF first — evo prompts interactively before overwriting, which
# deadlocks under a non-interactive service (no stdin).
rm -f "$RES/compare.pdf"
if [ -n "${DISPLAY:-}" ]; then
  evo_traj tum "${TUMS[@]}" --plot_mode "$MODE" --plot --save_plot "$RES/compare.pdf"
else
  MPLBACKEND=Agg evo_traj tum "${TUMS[@]}" --plot_mode "$MODE" --save_plot "$RES/compare.pdf"
fi

# Hand outputs back to the host user (container writes as root); the mounted repo is
# owned by the host user, so match that.
OWNER=$(stat -c '%u:%g' /slam-bench 2>/dev/null || echo "")
[ -n "$OWNER" ] && chown "$OWNER" "$RES/compare.pdf" "$RES/summary.txt" 2>/dev/null || true

echo "saved -> $RES/compare.pdf  |  summary -> $RES/summary.txt"
