# slam-bench

Reference-free comparative benchmark for LIO/SLAM systems on self-collected Livox MID-360
rosbags (no RTK ground truth). Compares systems on accuracy (start-end drift + map quality),
real-time performance, and robustness.

## Layout

```
bench.sh     THE entry point — the only script run by hand (host side)
systems/     baselines as git submodules (FAST_LIO, faster-lio, livox_ros_driver)
docker/      unified Noetic image + compose (build / run / compare / aggregate / dev)
scripts/     container-side: build_systems.sh, run_system.sh (one run), lib.sh (shared rules)
configs/     per-system MID-360 overrides + launch + presets/ (variants) + systems.yaml + bags.yaml
eval/        record_tum.py (trajectory ①), sample_resource.py (resource ③),
             aggregate.py (N-run statistics), compare.sh (evo overlay), tests/ (pytest)
bridge/      CustomMsg→PointCloud2 uniform input (only needed once a PC2-only baseline lands)
results/     per-run artifacts (gitignored)
```

Everything except `bench.sh` is invoked by `docker compose` inside a container — that split is
the rule for where a file lives.

```
./bench.sh setup                 build the docker image
./bench.sh build                 compile the systems into .ws/
./bench.sh run                   N benchmark runs
./bench.sh aggregate <dataset>   N-run statistics -> stats.txt + stats.json
./bench.sh compare   <dataset>   evo trajectory overlay -> compare.pdf
./bench.sh shell                 interactive container
./bench.sh help                  the full env/option reference
```

Anything after `--` goes straight to `docker compose`, so a flag the wrapper does not
expose is never a dead end: `./bench.sh compare <dataset> -- --no-deps`.

## Setup (once)

```bash
git submodule update --init --recursive     # fetch systems/
./bench.sh setup                            # build the image (incl. Livox-SDK, evo)
./bench.sh build                            # compile FAST-LIO + faster-lio into .ws/
```

`./bench.sh setup` is only needed after a Dockerfile change; `./bench.sh build` after a
system source change.

## Run

`N` runs of each `(system, preset)`, one fresh container per run:

```bash
# one run
BAGS_DIR=/path/to/bags NAME=mydataset SYS=fast_lio ./bench.sh run

# the noise floor: 5 runs of each system
BAGS_DIR=/path/to/bags NAME=mydataset SYS="fast_lio faster_lio" N=5 ./bench.sh run

# sweep presets, no human editing configs in between
BAGS_DIR=/path/to/bags NAME=mydataset SYS=fast_lio PRESET="default cube400" N=5 ./bench.sh run

# trade real-time fidelity for wall clock on a smoke run (not for timing/resource numbers)
BAGS_DIR=/path/to/bags NAME=mydataset SYS=fast_lio RATE=5.0 ./bench.sh run

# watch one run live in rviz (host: xhost +local:root once)
BAGS_DIR=/path/to/bags NAME=mydataset SYS=fast_lio RVIZ=true ./bench.sh run

./bench.sh run --dry-run                 # print the plan without running anything
```

| Var | Meaning |
|---|---|
| `BAGS_DIR` | **required** — host bag folder, mounted read-only at `/bags` |
| `NAME` | **required** — the `results/<dataset>` label |
| `SYS` | **required** — `fast_lio` \| `faster_lio`, one or more, space separated |
| `N` | runs per (system, preset), default `1` |
| `PRESET` | one or more, default `default` = the as-shipped launch (see [`configs/presets/`](configs/presets/README.md)) |
| `RATE` | playback speed multiplier (default `1.0` — the rate latency and real-time factor are defined at; raise it only for smoke runs) |
| `BAG` | default `/bags` (a directory → all `*.bag` played in timestamp order as one stream); or `/bags/one.bag` |
| `RUN` | explicit run index, `FORCE=true` to overwrite it — only accepted when the batch is a single run |
| `RVIZ` | `true` opens the system's rviz (needs `xhost +local:root` — see [Interactive shell](#interactive-shell-debugging)) |

A crashed run does not abort the batch — a crash *is* a data point here.
Pre-flight checks (bag directory, preset file, built workspace, `RUN` vs batch size) all run
before the first container starts, so a typo fails in a second rather than after twenty
minutes.

**Ctrl-C** stops the container currently running and ends the batch — measured, about four
seconds. The interrupted run still leaves a `metrics.json`, marked failed, so it is counted
rather than silently missing. Press Ctrl-C a second time to force an immediate exit; that
path uses `SIGKILL`, which cannot be trapped, so the run leaves artefacts with no
`metrics.json` — `aggregate` reports such a directory as `no metrics.json (run was killed)`
instead of skipping it.

**Only one run at a time on a host.** Every container uses `network_mode: host`, so they
share one ROS master on port 11311. If a second run started while a first was alive, its
`roscore` would fail to bind, its nodes would silently join the existing master, and both
recorders would capture both systems' `/Odometry` — two ruined trajectories, each still
reporting `bag_play_exit 0`. Both `bench.sh` and `run_system.sh` now refuse to start when
11311 is already listening. If you ever need to clear a stuck run:

```bash
docker rm -f $(docker ps -q --filter ancestor=slam-bench:noetic)
```

Each run writes to `results/<NAME>/<SYS>/<PRESET>/run<NN>/` — repeated runs never overwrite
each other:

- `trajectory.tum` — odom in TUM format (artifact ①)
- `resource.csv` — external CPU%/RSS trace (artifact ③)
- `run.log` — system stdout/stderr
- `metrics.json` — completion + provenance (`preset_sha`, `binary_sha`, submodule commit
  and dirty state, the played bag's `bag_start`/`bag_end`), plus the derived quantities
  `aggregate.py` writes back

```bash
cat results/mydataset/fast_lio/default/run01/metrics.json
```

Accuracy metrics (drift, MME, plane-RMSE) are the next stage.

## Aggregate results

Statistics over the repeated runs of a dataset:

```bash
./bench.sh aggregate mydataset
./bench.sh aggregate mydataset --split-at 50        # bucket by end_pos_m
./bench.sh aggregate mydataset --min-coverage 0.5   # relax the completion threshold
```

Writes `results/<DS>/stats.txt` (human) and `stats.json` (machine, the input to the
cross-dataset summary matrix), and fills each run's `metrics.json` with its derived quantities.

Per `(system, preset)` it reports **median [min–max]** plus every run's raw value. Mean and
standard deviation are deliberately absent: the distribution is bimodal, so a mean lands in
the empty gap between the modes and describes an outcome no run produced. `--split-at` has no
default for the same honesty reason — the recovered/exploded boundary is a property of the
dataset, not of the metric.

Flags after the dataset name are passed straight to `eval/aggregate.py`, so `--help` on it
lists everything available and a new option needs declaring in one place only.

If runs inside one group were built from different configurations or binaries, the group is
split by fingerprint and flagged `⚠ MIXED` rather than silently averaged — the failure mode
that once invalidated a whole configuration sweep here.

A run is excluded from the statistics, with its reason shown in the table, when any of these
holds:

| Check | Catches |
|---|---|
| `bag_play_exit != 0`, or null | crashed or interrupted playback |
| trajectory timestamps outside the played bag | odometry that came from a *different* run (a shared ROS master) |
| trajectory spans < `--min-coverage` of the bag (default `0.9`) | a run that stopped early |
| a `VOID` file in the run directory | a human verdict; its first line is the reason |

## Compare trajectories

```bash
xhost +local:root                          # HOST, once: allow the live plot window
./bench.sh compare mydataset               # 3D (xyz) overlay window + saved file
./bench.sh compare mydataset xy            # top-down view instead
ALL=true  ./bench.sh compare mydataset     # every run, not just medians
DISPLAY=  ./bench.sh compare mydataset     # headless (save only)
```

Writes `results/<DS>/compare.pdf`. By default each `(system, preset)` contributes only its
**median run** by `end_pos_m` (ten curves are unreadable); `ALL=true` draws every run, which is
the view for inspecting the bimodal split itself. Median selection reads the `derived` block,
so run `aggregate` first — groups without it fall back to all runs with a warning.

Drift is intentionally **not** reported — these datasets are not strictly closed-loop, so a
start-end gap is not a valid drift measure.

## Interactive shell (debugging)

```bash
xhost +local:root                                   # HOST, once per login: allow container X11
BAGS_DIR=/path/to/bags ./bench.sh shell
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
  parameters stay at upstream defaults.
- **File ownership** — containers run as the invoking user (`bench.sh` passes `HOST_UID`
  / `HOST_GID`), so everything under `results/` and `.ws/` belongs to you and needs no
  `chown`. A bare `docker compose` call without those variables falls back to root and
  does leave root-owned files behind.

## Tests

`eval/aggregate.py` and `eval/next_run_dir.py` use the Python standard library only, so their
tests run on the host with no container, ROS, or bag:

```bash
python3 -m pytest eval/tests/ -q
```

The shell components are covered by the end-to-end run instead — their failure modes (roscore
not coming up, container exit codes, GPU contention) are not reachable from a unit test.

## Status

Pipeline validated end-to-end on the NORCAT underground dataset (17 bags, ~17 min, 5x), with an
evo trajectory-compare wrapper. Runs on this degenerate underground scene are **non-deterministic**
(identical config, same input can complete cleanly one run and diverge the next), which is why
results are recorded per run and aggregated over N. Group-A baselines FAST-LIO + faster-lio are
wired; more baselines and the eval/metric stage (map quality, latency, the cross-dataset matrix) follow.

The next measurement is the noise floor itself: faster-lio ×3 (is it
deterministic?) and FAST-LIO ×5 stock (explosion rate and distribution). No configuration
comparison is interpretable before it exists.
