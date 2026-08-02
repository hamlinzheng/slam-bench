# slam-bench

Reference-free comparative benchmark for LIO/SLAM systems on self-collected Livox MID-360
rosbags (no RTK ground truth). Compares systems on accuracy (start-end drift + map quality),
real-time performance, and robustness.

## Layout

```
bench.sh     THE entry point — the only script run by hand (host side)
systems/     baselines as submodules (FAST_LIO, faster-lio, Point-LIO, Super-LIO, PV-LIO,
             BIEVR-LIO) + livox_msgs/ (vendored Livox CustomMsg types, not a baseline)
docker/      unified Noetic image + compose (build / run / compare / aggregate / dev)
scripts/     container-side: build_systems.sh, run_system.sh (one run), lib.sh (shared rules)
configs/     per-system MID-360 overrides + launch + presets/ (variants) + systems.yaml + bags.yaml
eval/        record_tum.py (trajectory ①), sample_resource.py (resource ③),
             record_frames.py (frame timing ③), aggregate.py (N-run statistics),
             compare.sh (evo overlay), tests/ (pytest)
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
./bench.sh setup                            # build the image (ROS Noetic + PCL, evo)
./bench.sh build                            # compile every baseline into .ws/
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
| `SYS` | **required** — `fast_lio` \| `faster_lio` \| `point_lio` \| `super_lio` \| `pv_lio` \| `bievr_lio`, one or more, space separated |
| `N` | runs per (system, preset), default `1` |
| `PRESET` | one or more, default `default` = the as-shipped launch (see [`configs/presets/`](configs/presets/README.md)) |
| `RATE` | playback speed multiplier (default `1.0` — the rate every timing and CPU number is defined at. No per-frame cost here is rate-portable, so raise it only for runs whose numbers will not be compared against 1×; runs at different rates are never merged into one sample) |
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

**Each run is network-isolated.** The `run` service has `network_mode: none`: a run's
`roscore`, the system under test, `rosbag play` and the three recorders all live in that one
container's network namespace, and bags and results are mounts, so a run needs no network at
all. Two consequences:

- a ROS master on the host — any unrelated ROS application of yours — neither blocks a run
  nor can be joined by one;
- a surviving run can no longer capture a second one's `/Odometry`, the failure mode the old
  host-network setup allowed (two ruined trajectories, both reporting `bag_play_exit 0`).

The cost: **`rostopic echo` from the host can no longer watch a live run.** Go in through the
container instead — which is also more precise, being that run's master by construction
rather than whoever holds port 11311:

```bash
docker exec -it $(docker ps -q --filter ancestor=slam-bench:noetic) bash
#   source /opt/ros/noetic/setup.bash && rostopic hz /Odometry
```

rviz (`RVIZ=true`) is unaffected — X11 travels over the `/tmp/.X11-unix` mount, not TCP. The
`dev` shell keeps `network_mode: host` for poking at the host's ROS graph by hand; don't
start a measurement run from inside it. If you need to clear a stuck run:

```bash
docker rm -f $(docker ps -q --filter ancestor=slam-bench:noetic)
```

**Still one run at a time, but now for a measurement reason rather than a plumbing one.**
Concurrent runs no longer corrupt each other's topics; they compete for cores, memory
bandwidth and RAM, and every latency/CPU number here is only comparable across runs that had
the machine on the same terms — the same reason `metrics.json` records `cpu_governor`.
`bench.sh` warns rather than refuses, since accuracy-only work is unaffected.

Each run writes to `results/<NAME>/<SYS>/<PRESET>/run<NN>/` — repeated runs never overwrite
each other:

- `trajectory.tum` — odom in TUM format (artifact ①)
- `resource.csv` — external CPU%/RSS trace (artifact ③)
- `frame_events.csv` — arrival wall clock of every input scan (`in`) and every output odom
  (`out`), artifact ③'s timing half. Raw events only: `aggregate.py` pairs them, so a better
  pairing rule later costs seconds rather than another replay of every bag
- `run.log` — system stdout/stderr
- `metrics.json` — completion + provenance (`preset_sha`, `binary_sha`, submodule commit
  and dirty state, `omp_wait_policy`, the played bag's `bag_start`/`bag_end`), plus the derived
  quantities `aggregate.py` writes back. `status=ok` needs playback to exit 0, ≥2 poses **and**
  `system_alive` — a system killed mid-bag leaves all the other signals looking healthy

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

If runs inside one group were built from different configurations or binaries — **or played
at different rates** — the group is split by fingerprint and flagged `⚠ MIXED` rather than
silently averaged. That is the failure mode which once invalidated a whole configuration
sweep here; playback rate belongs in the same fingerprint because at 5× every wall-clock
second carries five seconds of work, so CPU% is not even the same quantity.

`stats.txt` carries two tables. **accuracy** is `path_len_m` and `end_pos_m`; **real-time**
puts resource cost and frame timing on one row, which is the only place the two can be read
against each other:

| Column | Meaning |
|---|---|
| `lat_p50ms` / `lat_p99ms` | end-to-end latency, queueing included. The tail is what misses a deadline |
| `cpu_ms/f` | processor time one frame cost: total CPU seconds ÷ poses produced |
| `parallel` | `cpu_ms/f` ÷ `lat_p50ms` — cores kept busy while a frame was handled |
| `sat` | fraction of frames that arrived while the previous was still being processed |
| `out_ratio` | poses out ÷ scans in. Below 1 = the system is skipping frames |

**A system can be cheap on CPU for three different reasons**, and it takes two columns to tell
them apart: it computes less, it is falling behind, or it uses one core where another uses four.
Measured at 1× on one NORCAT bag, Point-LIO / FAST-LIO / faster-lio all land at 21.1 / 21.0 /
18.1 ms of latency — near enough to call equivalent — while their CPU per frame is 21.0 / 42.9 /
79.1 ms. Both readings are true; `parallel` (1.00 / 2.05 / 4.37) is the difference.

**Read `sat` before any latency.** Above roughly zero it means frames are queueing, and their
latency is partly the backlog in front of them rather than the system's own work. `cpu_ms/f` is
immune — waiting costs no processor time.

**And read the warnings before that.** A high `sat` only means the system was behind if the input
arrived evenly. On one 5× run it did not: playback off an external drive stalled 349 times for
28.3 s — 12 % of the replay — and delivered the backlog in clumps, so frames queued while FAST-LIO
was in fact keeping up (11 729 poses for 11 732 scans). That is what `in_jitter` and `⚠ UNEVEN
INPUT` exist to catch; above 1.5 every timing in the row describes the replay, not the system.

**Nothing here survives a change of playback rate.** Both per-frame costs fall as the rate rises —
Point-LIO reads 21.1 / 18.8 / 14.8 ms of latency and 21.0 / 16.8 / 12.4 ms of CPU at 1× / 3× / 5×
on one binary and one bag. Part is definitional (a scan waits for the IMU covering it, which is a
bag-time wait and so shrinks by the rate) and the rest is unexplained. Only `parallel` holds still.
Judge real-time behaviour at the rate the sensor actually runs; rate is in the fingerprint, so such
runs never merge. `stats.json` additionally carries `lag_growth_ms` (how much further behind the
run ended than it started, which separates a queue that drains from one that diverges), `sensor_hz`,
`rate_actual` (what the replay actually achieved, which `rosbag play -r` does not guarantee),
`in_jitter`, `skipped_in` (scans consumed with no pose — the initialisation lead-in, measured at
3–5 frames, plus any genuine drop) and `unmatched_out`.

Runs recorded before `frame_events.csv` existed report every one of these as null, never as
zero — the same rule the resource trace already follows.

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
  FAST-LIO and Point-LIO need a workspace-level `livox_ros_driver`, while faster-lio and
  Super-LIO each bundle their own copy of the CustomMsg definitions, so one shared workspace
  would collide on the package name. (Super-LIO's upstream repo is itself a catkin workspace,
  so both of its packages — `src/basic` and `src/super_lio` — get linked into `.ws/super_lio`.)
- **Messages, not the driver** — [`systems/livox_msgs/`](systems/livox_msgs/README.md) supplies
  the Livox `CustomMsg` types under both `livox_ros_driver` and `livox_ros_driver2`. We replay
  bags and never run a Livox node, so the image needs no Livox-SDK.
- **Fairness** — every system ingests Livox `CustomMsg` natively (our bags record
  `livox_ros_driver2/CustomMsg`, whose ROS md5 is identical to gen1's, so no conversion is
  needed); MID-360 extrinsics match our rig, so only topics changed vs upstream — all
  algorithm parameters stay at upstream defaults. The one further plumbing edit is disabling
  each system's own PCD dump (`pcd_save_en` / `save_map`): map artifact ② is accumulated
  externally, and dumping during a run perturbs the resource trace ③.
- **CPU % measures work, not spinning** — runs set `OMP_WAIT_POLICY=passive` (plan §7). `/proc`
  cannot tell a thread doing maths from one busy-waiting, and libgomp spins by default, so the
  metric otherwise reads each system's hardcoded thread count as much as its cost: PV-LIO drops
  1069 % → 327 % with the pose rate unchanged. Only systems with their own `#pragma omp` move at
  all (FAST-LIO, PV-LIO); the rest sit within 1.03×. `OMP_WAIT_POLICY=active` restores the
  as-shipped default, and each run records which it used.
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
results are recorded per run and aggregated over N. Group-A baselines FAST-LIO, faster-lio,
Point-LIO, Super-LIO and PV-LIO are wired and produce usable trajectories. BIEVR-LIO builds and
runs but its estimate diverges on this dataset, so it stays `screen` in the plan rather than
`admit` — see the plan's §2 notes. The eval/metric stage (map quality, latency, the cross-dataset
matrix) follows.

The next measurement is the noise floor itself: faster-lio ×3 (is it
deterministic?) and FAST-LIO ×5 stock (explosion rate and distribution). No configuration
comparison is interpretable before it exists.
