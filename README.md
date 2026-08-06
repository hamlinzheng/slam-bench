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
             compare.py (evo overlay), init_dataset.py + gnss_ref.py (per-dataset
             preparation), gnss_ape.py + plot_gnss.py (error against that reference),
             tests/ (pytest)
bridge/      CustomMsg→PointCloud2 uniform input (only needed once a PC2-only baseline lands)
results/     per-run artifacts (gitignored)
```

Everything except `bench.sh` is invoked by `docker compose` inside a container — that split is
the rule for where a file lives.

```
./bench.sh setup                 build the docker image
./bench.sh build                 compile the systems into .ws/
./bench.sh init      <dataset>   prepare a dataset once (GNSS reference)
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

### Taking a system out of the comparison

Give it a `disabled:` line in `configs/systems.yaml`, whose value is the reason:

```yaml
bievr_lio:
  disabled: diverges past the first long open stretch; the numbers are not comparable
  group: A
  ...
```

`aggregate` and `compare` then leave its runs out, each naming the system, the reason and
how many runs it dropped — the omission is reported, never silent. `run` still runs it
when `SYS` names it, with a warning: the switch decides what the default comparison
contains, not what you are allowed to do. So a run made while a system is disabled lands
in `results/` as usual but stays out of both eval steps until the line is gone.

Nothing is rebuilt either way, so bringing a system back is deleting that one line. There
is deliberately no flag to override the switch per invocation: `configs/systems.yaml` is
meant to be the only place that says whether a system counts.

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

### Runs that fly away

A LIO system that loses tracking does not stop; it keeps publishing odometry, and the
trajectory grows to hundreds of kilometres. `eval/divergence.py` watches for that, live:
`record_tum.py` feeds every pose to the detector as it records it, and when the rule fires
it drops a `DIVERGED` file in the run directory, which `run_system.sh` sees and uses to stop
playback. The run is recorded as `status: diverged` with the reason, and **the remaining
repeats still run** — divergence is stochastic (BIEVR-LIO diverged on one of three runs of
one dataset), so cutting the batch short would erase the failure rate, which is itself the
measurement.

**The rule.** A run is diverged when, inside any 2-second window of at least 5 steps, half
the steps imply a speed over **40 m/s** between consecutive poses. Non-finite poses count
as over-limit steps.

The threshold is a constant in `eval/divergence.py`, not a flag or a config field. Across
the 112 trajectories in `results/` the healthy runs top out at 24.6 m/s and the diverged
ones start at 69.0 m/s, with nothing in between, on datasets from a 1.8 km underground
drift to a 7.3 km road — because the limit bounds the *vehicle*, not the route. (That is
the difference from `--split-at`, whose boundary is a dataset-dependent distance and so
ships with no default.) Benchmarking something that is not a ground vehicle means changing
that constant.

**What it does not detect: drift.** A run that stays physically plausible and simply ends
300 m off is not diverged — that is accuracy, and `gnss_ape_*` owns it. This catches the
blow-up, nothing subtler.

Detection also runs offline, over `trajectory.tum`, so `aggregate` reclassifies results
recorded before any of this existed without replaying a bag. A diverged run is excluded
from its group's statistics and listed under `## excluded` with its reason.

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

`./bench.sh init` writes two more, once per dataset rather than once per run:

- `gnss_ref.tum` — the GNSS track in local ENU, the reference `evo_ape` and the plots use.
  Its quaternion is a dummy: a single-antenna receiver has no attitude, so `evo_ape -r full`
  against it would return a plausible but meaningless number
- `gnss_ref.json` — what that reference is worth, all measured: fix mix, the gap
  *distribution* (a 95 % fix rate whose missing 5 % is one continuous hole leaves that
  stretch with no reference at all), and a per-axis noise floor taken while the vehicle was
  stopped. It also carries `usable`, and when false, why — a receiver keeps publishing
  underground, so "has a GNSS topic" and "has a reference" are different questions and only
  the second one gates anything

Map-quality metrics (MME, plane-RMSE) are the next stage.

## Aggregate results

Statistics over the repeated runs of a dataset:

```bash
./bench.sh aggregate mydataset
./bench.sh aggregate mydataset --split-at 50        # bucket by end_pos_m
./bench.sh aggregate mydataset --min-coverage 0.5   # relax the completion threshold
```

Writes `results/<DS>/stats.txt` (human) and `stats.json` (machine, the input to the
cross-dataset summary matrix), and fills each run's `metrics.json` with its derived quantities.

Start-end displacement is reported split as well as whole — `end_pos_m`, `end_horiz_m`,
`end_vert_m`. The 3D scalar alone ranked runs wrongly on one outdoor dataset: a 7.4 km route
with 14 m of relief, where the baselines end 22–48 m off vertically against 3–7 m
horizontally, so the single number was almost entirely its vertical component and the
vertical is free to wander out and back inside it. Where `./bench.sh init` found a usable
GNSS reference, four more columns appear (`ape_horiz_m`, `ape_vert_m`, `end_err_h_m`,
`end_err_v_m`); everywhere else they are blank, which is the honest reading rather than a
zero.

Status precedence per run is **VOID > diverged > incomplete > ok**. A run the online
detector aborted has low coverage *because playback was stopped*, so judging completion
first would report the consequence and bury the cause; where both are true the reason names
both (`… over 40 m/s at 1288.5s; also incomplete: trajectory coverage 65% of the bag`),
which is what keeps an out-of-memory death visible on a run that also blew up. See
[Runs that fly away](#runs-that-fly-away).

The tables report every one of those as **`failed`** — `2 (1 failed)` in the `n` column,
`FAILED` in `## excluded` — because a comparison table marks a cell that carries no result,
and which way it failed is a sentence rather than a column. That sentence is right beside
it: `FAILED  diverged: 52% of a 2s window over 40 m/s at 156.8s`. The distinction survives
in `stats.json`, where `status` is still `diverged`, `failed` or `void` — it is a real one,
since an OOM kill is fixed with more memory and a blow-up is not.

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

## Cross-dataset summary

```bash
./bench.sh summary          # every dataset's stats.json -> results/SUMMARY.md
```

One Markdown document over every dataset that has been aggregated: a completion matrix, the
accuracy and real-time metrics as system × dataset tables, then each dataset's full tables
with their `[min–max]` ranges. Needs no bags and no rebuild — it reads only what `aggregate`
already wrote, so no number in it can disagree with a `stats.txt`.

**Nothing is averaged across datasets.** The routes run from 1.8 km of underground drift to
7.3 km of open road; a figure spanning them describes no route that was driven. Every cell
stays attached to the dataset it came from.

Three things a cell can say instead of a number, because they are three different facts:
`–` the system was never run there, **fail** it ran and no run survived, `n/m` the quantity
was never measured. The document also carries a section naming what
[§10 of the plan](docs/comparison_plan.md) asks for that no collector produces yet — map
metrics, ARM latency — rather than leaving those columns blank.

## Compare trajectories

```bash
xhost +local:root                          # HOST, once: allow the live plot window
./bench.sh compare mydataset               # plan (xy) overlay window + saved file
./bench.sh compare mydataset xyz           # 3D view instead
ALL=true  ./bench.sh compare mydataset     # every run, not just medians
REF=super_lio ./bench.sh compare mydataset # draw in another system's frame instead
ALIGN=origin  ./bench.sh compare mydataset # pin to the reference's opening pose instead
DISPLAY=  ./bench.sh compare mydataset     # headless (save only)
```

Writes `results/<DS>/compare.pdf`. By default each `(system, preset)` contributes only its
**median run** by `end_pos_m` (ten curves are unreadable); `ALL=true` draws every run, which is
the view for inspecting the bimodal split itself. Median selection reads the `derived` block,
so run `aggregate` first — groups without it fall back to all runs with a warning.

A group with a healthy run is never represented by one that flew away, and a run that did is
drawn **truncated to the moment it diverged**, labelled `…-FAILED-28s` — the same word the
tables use, since the two are read side by side. Without that, evo's
autoscaling turns every other system into a dot at the origin — one ELDORADO trajectory spans
34,616 km vertically against fast_lio's 197 m. What is left is the part that was still a
trajectory, which is worth seeing: faster-lio's first 25 s on that dataset are real. In
`compare_gnss.pdf` these are drawn grey and dashed, and the truncation matters more there
still — that figure aligns with a whole-trajectory Umeyama fit, which a runaway curve would
drag, taking every other curve's error with it.

Every curve carries a filled circle where it starts and a `×` where it ends, on both
figures. Neither end is guessable otherwise: a truncated run stops in open ground, where
without the mark it reads as an unattached start rather than a run cut short. It is an evo
setting rather than a flag, so `compare` writes it into a throwaway config under a temporary
`HOME`; your own `~/.evo` is left alone.

Under the default alignment the starts do **not** coincide — a whole-trajectory fit is free
to place each curve where its shape fits best, so how far a start sits from the others is
itself a reading of how badly that run is shaped. Under `ALIGN=origin` they all coincide by
construction, and direction is the only thing separating the curves.

`compare_gnss.pdf` draws the plan view **twice**, under the two alignments it already
maintained for its other pages, because they answer different questions:

* **whole-run Umeyama** — the APE convention, so this is the page that corresponds to
  `gnss_ape_*` in `stats.txt`. The fit splits each run's error between its two ends, which
  is why the starts do not coincide here: measured on OPENROAD_20260325 it puts the six
  starts up to **300 m** from the reference's. How far each was pushed is itself a reading.
* **yaw-aligned on the opening 100 m** — every trajectory leaves the reference's own start
  (within 0.8 m on that dataset) and the drift fans out from there. This is the view for
  "how did it go wrong", and it exposes what a whole-run fit averages away.

  Only the **yaw** is fitted on the opening, and the vertical is offset rather than fitted.
  Up is defined by gravity, the whole-run fit already agrees with the reference about it,
  and an opening segment has nothing worth hearing to say about it: 100 m of *path* at the
  start is 43–99 m of net displacement on these routes, so fitting a full rotation to it
  came out 11–75° tilted, and that tilt levered along a 3–7 km route drew **59–1896 m** of
  vertical RMSE where the whole-run fit reads 14–113. Six independent systems agreed on
  that error to within 5% while their real vertical errors differed threefold, which is how
  it was caught — the number was the alignment's, not theirs. With yaw only, the
  opening-aligned vertical error lands at 1.0–2.1× the whole-run figure, which is the shape
  drift-from-a-pinned-start should have, and the result barely moves between a 20 m and a
  1000 m window where the full fit swung 27×.

A run truncated where it blew up can be too short to contain that opening travel — one
survives 32.6 s, before the vehicle has covered 100 m. It is left off the opening-aligned
pages only, and named on them, rather than withholding those pages from every other system.

**`REF` picks whose frame the figure is drawn in**, defaulting to `point_lio`. Every system
defines its world frame differently and evo aligns the whole overlay onto the first
trajectory's opening pose, so this one choice orients the entire plan view.

It has to be a system that **gravity-aligns**, or the plan view is not a plan view. Measured
against the GNSS reference — the angle between each system's own vertical and the true one,
on the two datasets where every system behaves:

| | `point_lio` | `bievr_lio` | `super_lio` | `fast_lio` | `faster_lio` | `pv_lio` |
|---|---|---|---|---|---|---|
| tilt from vertical | **1.9–3.3°** | 1.6–2.1° | 2.0–2.4° | **14.1–16.0°** | 13.2–24.2° | 14.9–16.3° |
| its vertical vs ENU up | +0.998 | +0.999 | +0.999 | **−0.961** | — | — |

The sign matters as much as the angle: `fast_lio`'s vertical points *down*, so aligning on
it draws the whole route upside down.

Reproducibility is the second axis and the softer one. Between two runs of one bag the
opening attitude moves by 0.37° (`fast_lio`), 0.79°, 1.24°, 2.12°, 10.43° (`point_lio`) and
25.99° (`bievr_lio`). For a gravity-aligned system nearly all of that is yaw — 10.20° of
`point_lio`'s 10.43° — which turns the figure without deforming it, unlike a tilt, which
projects vertical motion into the plan view. So `point_lio`'s wobble costs an angle and
`fast_lio`'s steadiness costs the shape.

This used to be whichever system sorted first by name — `bievr_lio`, gravity-aligned but the
loosest of the six in yaw, so anything that changed which run represented a group silently
turned the figure by up to 22.7°. Only the opening pose is read, so a system that later blew
up is still a valid frame; `REF=` (empty) restores the old first-drawn-run behaviour.

**`ALIGN` chooses how**, and the default `umeyama` fits each curve onto the reference by
position alone. `ALIGN=origin` instead pins every curve to the reference's opening pose,
attitude included, and lets the disagreement accumulate from there.

Position-only is the default because a benchmark has to be able to draw a system it did not
write, and attitude conventions are not universal. One system here publishes its orientation
rolled 180° about forward — its body y and z read 179.9° and 166.3° from every other
system's, i.e. FRD where ROS REP-103 says FLU. Its *positions* are the closest of any system
to the reference (0.26 m RMS over the opening kilometre); only the attitude disagrees. Under
`origin`, which aligns on attitude, that curve is drawn flipped and reads as **1598 m** of
error. Under `umeyama` the same run reads **5.0 m**.

The cost is that fitting the shape flatters everything. On one dataset, RMS distance to the
reference under `origin` → `umeyama`: `pv_lio` 7.6 → 1.5 m, `fast_lio` 16.6 → 9.3,
`bievr_lio` 42.7 → 27.1, `super_lio` 203.7 → 132.9. Every system reads 1.5–5× closer than it
is from a common start, because a least-squares fit spreads each run's drift across both
ends — and the thing being fitted onto here is another system under test, not a truth.

**So do not read drift off this figure.** Read it under `ALIGN=origin`, where it accumulates
from a common start, or against the GNSS track in `compare_gnss.pdf`, which is the only
alignment in this repo with a truth behind it — and which is unaffected by all of the above,
since `plot_gnss.py` never reads a quaternion.

The default view is the plan one: these are ground-vehicle routes on near-flat ground, so a
3D view spends most of its third axis on vertical error.

Where `./bench.sh init` found a usable GNSS reference, `compare_gnss.pdf` is written too —
the same runs against the GNSS track, in three pages: plan view, each axis against time, and
per-axis error against time with the reference's own noise floor drawn as a band, since a
separation smaller than that band is the receiver rather than the system. It is a separate
file rather than more pages of `compare.pdf` because the two need opposite alignments and evo
takes one per invocation: `compare.pdf` aligns origins, which removes each system's frame
convention without fitting the shape, while a GNSS reference lives in ENU and differs from
every system frame by a yaw only a Umeyama fit recovers. Attitude stays out of it — the
reference has none, and `compare.pdf` already compares RPY without needing one.

Drift is intentionally **not** reported here — `aggregate` owns every derived quantity.

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

`eval/aggregate.py` and `eval/next_run_dir.py` compute everything from plain arguments, so their
tests run on the host with no container, ROS, or bag — `pyyaml` (for `eval/registry.py`) is the
only dependency beyond the standard library:

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
