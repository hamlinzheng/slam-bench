# slam-bench

Reference-free comparative benchmark for LIO/SLAM systems on self-collected Livox MID-360
rosbags (no RTK ground truth). Compares systems on accuracy (start-end drift + map quality),
real-time performance, and robustness.

- **Plan / spec**: [`docs/comparison_plan.md`](docs/comparison_plan.md)

## Layout

```
systems/     baselines as git submodules (FAST_LIO, faster-lio, livox_ros_driver)
docker/      unified Noetic image + compose (build / run / dev)
scripts/     build_systems.sh (per-system isolated ws), run_system.sh (one system × bags)
configs/     per-system MID-360 overrides + launch + systems.yaml (output-adapter table) + bags.yaml
eval/        record_tum.py (trajectory ①), sample_resource.py (resource ③), compare.sh (evo overlay); metrics: later
bridge/      CustomMsg→PointCloud2 uniform input (only needed once a PC2-only baseline lands)
results/     per-run artifacts (gitignored)
```

## Setup (once)

```bash
git submodule update --init --recursive     # fetch systems/
cd docker
docker compose build build                  # build the image (incl. Livox-SDK, evo)
docker compose run --rm build               # compile FAST-LIO + faster-lio into ../.ws
```

Re-run `build` only after a Dockerfile change (image) or a system source change (workspaces).

## Run a system over bags

```bash
cd docker

BAGS_DIR=/media/hamlin/T7/NORCAT/20251118_NorcatUG \
  SYS=fast_lio   BAG=/bags  RATE=5.0  NAME=20251118_NorcatUG \
  docker compose run --rm run

BAGS_DIR=/media/hamlin/T7/NORCAT/20251118_NorcatUG \
  SYS=faster_lio BAG=/bags  RATE=5.0  NAME=20251118_NorcatUG \
  docker compose run --rm run

# watch the run live in rviz (host: xhost +local:root once) — not for timing/resource numbers
BAGS_DIR=/media/hamlin/T7/NORCAT/20251118_NorcatUG \
  SYS=fast_lio   BAG=/bags  RATE=1.0  NAME=20251118_NorcatUG  RVIZ=true \
  docker compose run --rm run
```

| Var | Meaning |
|---|---|
| `SYS` | `fast_lio` \| `faster_lio` |
| `BAG` | `/bags` (a directory → all `*.bag` played in timestamp order as one stream) or `/bags/one.bag` |
| `RATE` | playback speed multiplier (default `5.0`; use `1.0` for a real-time run) |
| `NAME` | output dataset label (without it a mounted directory is labeled `bags`) |
| `BAGS_DIR` | host bag folder, mounted read-only at `/bags` in the container |
| `RVIZ` | `true` opens the system's rviz during the run (default `false`; needs `xhost +local:root` on the host — see [Interactive shell](#interactive-shell-debugging)) |

Each run writes to `results/<NAME>/<SYS>/`:

- `trajectory.tum` — odom in TUM format (artifact ①)
- `resource.csv` — external CPU%/RSS trace (artifact ③)
- `run.log` — system stdout/stderr
- `metrics.json` — completion + bag-play exit + pose count

```bash
cat ../results/20251118_NorcatUG/fast_lio/metrics.json
```

Accuracy metrics (drift, MME, plane-RMSE) and the summary aggregator are the next stage
(plan §5–§6).

## Compare results

Overlay every system's trajectory for a dataset with evo and report end-pose `|pos|`:

```bash
cd docker
xhost +local:root                                         # HOST, once: allow the live plot window
DS=20251118_NorcatUG docker compose run --rm compare       # 3D (xyz) overlay window + saved files
MODE=xy   DS=20251118_NorcatUG docker compose run --rm compare       # top-down view instead
DS=20251118_NorcatUG docker compose run --rm -e DISPLAY= compare     # headless (save only)
```

Writes `results/<DS>/compare.pdf` (evo overlay, `xyz` by default) and `results/<DS>/summary.txt`
(per-system poses / path length / end-pose `|pos|`). Drift is intentionally **not** reported —
these datasets are not strictly closed-loop, so a start-end gap is not a valid drift measure
(plan §6). Outputs are chown'd back to the host user automatically.

## Interactive shell (debugging)

```bash
xhost +local:root                                   # HOST, once per login: allow container X11
BAGS_DIR=/path/to/bags docker compose run --rm dev
# inside the container:
#   source /ws/fast_lio/devel/setup.bash
#   roslaunch /slam-bench/configs/launch/fast_lio.launch rviz:=true &
#   rosbag play --clock -r 5.0 /bags/*.bag
```

rviz is forwarded via X11 (`DISPLAY` + `/tmp/.X11-unix` mount) **plus** GPU/OpenGL
passthrough (`runtime: nvidia` + `NVIDIA_DRIVER_CAPABILITIES=graphics,display`).
Both are required on an NVIDIA host — without the GPU caps rviz finds the X display but aborts
initializing its GL context (`process has died ... exit code -6`). If it still reports
`could not connect to display`, the host `xhost` line was skipped or the host `$DISPLAY`
differs — pass it explicitly (`DISPLAY=$DISPLAY ...`). Benchmark runs default to headless
(`rviz:=false`); pass `RVIZ=true` to watch a run live, but keep it off for timing/resource
numbers since rendering competes with the system under test.

## Notes

- **Isolated workspaces** — each system builds into its own `../.ws/<system>` (plan §11):
  FAST-LIO needs the workspace-level `livox_ros_driver`, faster-lio bundles its own copy,
  so one shared workspace would collide on the package name.
- **Fairness** — both systems ingest Livox `CustomMsg` natively (including
  `livox_ros_driver2/CustomMsg`, whose ROS md5 is identical, so no conversion is needed);
  MID-360 extrinsics match our rig, so only topics changed vs upstream — all algorithm
  parameters stay at upstream defaults (plan §9).
- **File ownership** — containers run as root, so `.ws/` and `results/` are root-owned; run
  `docker run --rm -v "$PWD/results":/r slam-bench:noetic chown -R $(id -u):$(id -g) /r`
  to reclaim them.

## Status

Pipeline validated end-to-end on the NORCAT underground dataset (17 bags, ~17 min, 5x), with an
evo trajectory-compare wrapper. Runs on this degenerate underground scene are **non-deterministic**
(identical config, same input can complete cleanly one run and diverge the next) — so single-run
numbers are provisional and N-run repeatability is a required next step. Group-A baselines
FAST-LIO + faster-lio are wired; more baselines and the eval/metric stage (map quality, latency,
aggregator) follow.
