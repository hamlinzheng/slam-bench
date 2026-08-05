#!/usr/bin/env python3
"""Aggregate N repeated runs of a dataset into per-(system, preset) statistics.

Every derived quantity is computed from plain arguments — no container, no ROS, no bag —
so the tests run on the host under plain pytest. Only main() reads anything about the
world: which systems are disabled, via eval/registry.py. collect, disabled_inventory and
render_text take that answer as an argument and never consult the registry themselves.
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

import divergence
import registry

NO_RESOURCE = {
    "cpu_mean": None,
    "cpu_max": None,
    "rss_max_MB": None,
    "cpu_s_total": None,
}

# Every quantity frame_stats can produce, all absent. Same reason as NO_RESOURCE: a run
# that was never measured must read as null, not as zero latency.
NO_FRAMES = {
    "lat_p50_ms": None,
    "lat_p99_ms": None,
    "out_ratio": None,
    "lag_growth_ms": None,
    "sensor_hz": None,
    "rate_actual": None,
    "in_jitter": None,
    "saturated_frac": None,
    "skipped_in": None,
    "unmatched_out": None,
}

# lag_growth_ms compares the first and last tenth of the run. Below this many frames the
# two ends would overlap, so there is no growth to speak of.
MIN_FRAMES_FOR_LAG = 20

# A healthy run's trajectory covers essentially the whole bag (measured: 99%). The
# threshold is dimensionless and 1.0 is the ideal by definition — unlike the explosion
# split, whose scale is a property of the dataset — so a default is defensible.
DEFAULT_MIN_COVERAGE = 0.9
# Odometry for a scan can be stamped slightly outside the bag's own start/end.
BAG_RANGE_TOLERANCE_S = 5.0


class InsufficientTrajectory(Exception):
    """Fewer than two poses — nothing can be derived from this run."""


class MalformedRun(Exception):
    """The run record cannot be read at all."""


def load_run(run_dir, min_coverage=DEFAULT_MIN_COVERAGE):
    """Read one run directory into a record: its metrics.json plus status + derived.

    Status precedence — VOID > diverged > incomplete > ok:

      * a human VOID verdict outranks every automatic check, because a run can exit
        cleanly and still be worthless (a starved run once exited 0 after the map was
        starved);
      * divergence outranks the completion checks because a run aborted mid-bag by the
        online detector has low coverage *by construction* — that is what killing playback
        does. Judged the other way round, every aborted run would report "coverage 43% of
        the bag, below 90%", which states the consequence and hides the cause.

    The cost of that ordering, stated rather than hidden: a run killed by the OOM reaper
    that also produced a sustained burst of implausible poses reads as diverged rather
    than as a crash. The sustained-window rule keeps a single bad pose from doing this,
    but the ordering has this consequence and no check here can tell the two apart.
    """
    run_dir = Path(run_dir)
    try:
        rec = json.loads((run_dir / "metrics.json").read_text())
    except (OSError, ValueError) as e:
        raise MalformedRun("{}/metrics.json: {}".format(run_dir, e))

    rec["run_dir"] = str(run_dir)
    rec["derived"] = _derive(run_dir, rec.get("bag_start"), rec.get("bag_end"))

    void = run_dir / "VOID"
    if void.exists():
        rec["status"] = "void"
        rec["void_reason"] = _marker_reason(void)
        return rec

    reason = _divergence_failure(rec, run_dir)
    if reason:
        rec["status"] = "diverged"
        # Both facts, not just the one that won. PV-LIO on ELDORADO is the case that
        # forced this: its unbounded map was OOM-killed on all three runs (26-41 GB) and
        # the trajectory blew up as it died. "diverged" alone would have quietly retired
        # the record of an out-of-memory failure that is the more actionable of the two.
        also = _completion_failure(rec, min_coverage)
        if also:
            reason = "{}; also incomplete: {}".format(reason, also)
        rec["fail_reason"] = reason
        return rec

    reason = _completion_failure(rec, min_coverage)
    rec["status"] = "failed" if reason else "ok"
    if reason:
        rec["fail_reason"] = reason
    return rec


def _divergence_failure(rec, run_dir):
    """Why this run's trajectory blew up, or None if it did not.

    Two sources, and the DIVERGED marker is not redundant with the scan. The scan runs on
    every run anyway — v_max_mps belongs in `derived` whatever the verdict — so it alone
    would classify every diverged run correctly. What the marker adds is that the run was
    *aborted*, which is the only thing that explains its coverage: a run stopped at 30% of
    the bag and a run that played to the end and blew up at 90% both read "diverged", and
    only one of them is missing most of its route.

    Reading the scan as well as the marker is what lets results recorded before any of
    this existed be reclassified by re-running aggregate, without replaying a single bag.
    """
    marker = run_dir / "DIVERGED"
    scanned = (rec.get("derived") or {}).get("diverged_reason")
    if marker.exists():
        noted = _marker_reason(marker)
        return "aborted mid-run: {}".format(noted or scanned or "detector fired")
    return scanned


def _marker_reason(path):
    """The first line of a VOID / DIVERGED marker, or "" when it says nothing.

    Empty rather than an exception: the marker's existence is the signal and its content
    is the courtesy. `touch VOID` is a reasonable thing for a human to do in a hurry, and
    it must not crash the aggregation of every other run in the dataset.
    """
    try:
        lines = path.read_text().strip().splitlines()
    except OSError:
        return ""
    return lines[0].strip() if lines else ""


def _completion_failure(rec, min_coverage):
    """Why this run is not a completed run, or None if it is.

    Beyond the exit code, completion means the trajectory actually came from the bag
    this run played and covered it. Both checks exist because a run can report
    bag_play_exit 0 and still be worthless: back when every container shared the host
    network, a surviving one kept the ROS master and the recorder then captured a
    *different* run's /Odometry — observed, and invisible to every other check. Runs are
    network-isolated now, but the checks stay: they catch that shape of fault from any cause.
    """
    code = rec.get("bag_play_exit")
    if code is None:
        # run_system.sh records the exit code only once playback returns; null means it
        # never did. "exited None" would read as a bug in the tooling.
        return "playback never finished (interrupted or died)"
    if code != 0:
        return "bag playback exited {}".format(code)
    d = rec["derived"]
    if not d:
        return "no usable trajectory"
    if d.get("in_bag_range") is False:
        return "trajectory timestamps fall outside the played bag"
    coverage = d.get("coverage")
    if coverage is not None and coverage < min_coverage:
        return "trajectory coverage {:.0%} of the bag, below {:.0%}".format(
            coverage, min_coverage
        )
    return None


def _gnss_ape(run_dir):
    """Position error against the dataset's GNSS reference, when there is one.

    The reference is per-dataset, not per-run: results/<dataset>/gnss_ref.tum, built by
    eval/gnss_ref.py. Its report carries the verdict on whether the track is a reference
    at all — a receiver keeps publishing underground, where the track covers 780 m of a
    2849 m route, so the presence of the file is not the question and `usable` is.

    Imported here rather than at module scope: gnss_ape needs numpy, and this module is
    stdlib-only so its tests run on the host without a container. Absent numpy, or absent
    a reference, the keys read null — never 0, which would claim a perfect run.
    """
    ds = Path(run_dir).parents[2]
    ref = ds / "gnss_ref.tum"
    if not ref.exists():
        return {}
    try:
        report = json.loads((ds / "gnss_ref.json").read_text())
    except (OSError, ValueError):
        return {}
    if not report.get("usable"):
        return {}
    try:
        from gnss_ape import NO_APE, ape
    except ImportError:
        return {}
    try:
        return {**NO_APE, **ape(ref, Path(run_dir) / "trajectory.tum")}
    except (OSError, ValueError):
        return dict(NO_APE)


def _derive(run_dir, bag_start=None, bag_end=None):
    """Derived quantities, or {} when the trajectory cannot carry any."""
    try:
        derived = trajectory_stats(run_dir / "trajectory.tum")
    except (OSError, InsufficientTrajectory):
        return {}
    # A second read of trajectory.tum rather than a shared parse: divergence.py owns its
    # own tolerant reader (a run killed mid-write ends in a partial line, and that run's
    # verdict is exactly what it is for), and one pass over a 1 MB file is not worth
    # coupling the two modules' parsing.
    derived.update(divergence.scan(run_dir / "trajectory.tum"))
    derived.update(_gnss_ape(run_dir))
    csv = run_dir / "resource.csv"
    try:
        derived.update(resource_stats(csv))
    except OSError:
        derived.update(NO_RESOURCE)
    try:
        derived.update(frame_stats(run_dir / "frame_events.csv"))
    except OSError:
        derived.update(NO_FRAMES)
    derived["cpu_ms_per_frame"] = _cpu_per_frame(
        derived["cpu_s_total"], derived["n_poses"]
    )
    derived["parallelism"] = _parallelism(
        derived["cpu_ms_per_frame"], derived["lat_p50_ms"]
    )
    derived.update(_completion(derived, bag_start, bag_end))
    return derived


def _parallelism(cpu_ms_per_frame, lat_p50_ms):
    """Cores kept busy while a frame was handled: processor time over wall time.

    The one quantity here that barely moves with playback rate (faster-lio: 4.37 / 4.42 /
    4.47 at 1x / 3x / 5x while both its inputs shifted 16-18%), and the only one that
    tells "cheap because it computes less" from "cheap because it uses one core".

    Not the configured thread count: point_lio, fast_lio and faster-lio read 1.00, 2.05
    and 4.37 against compiled caps of 4, 3 and no OpenMP at all — faster-lio parallelises
    through std::execution::par_unseq onto TBB. What it measures is how much of a frame
    runs inside a parallel region. Biased low wherever saturated_frac is not ~0, queue
    wait having inflated the denominator.
    """
    if not cpu_ms_per_frame or not lat_p50_ms:
        return None
    return cpu_ms_per_frame / lat_p50_ms


def _cpu_per_frame(cpu_s_total, n_poses):
    """Processor-milliseconds one frame cost, whatever the run's length or idle time.

    Carries no information cpu_mean lacks — one quantity in two units, cpu_mean% =
    cpu_ms_per_frame x frame rate / 10 — but dividing by frames rather than by time is
    what stops a system that drops half the scans from reading as half the cost, and it
    is the numerator of `parallelism`.

    NOT comparable across playback rates: one FAST-LIO binary on one bag costs 41.4 /
    36.1 / 22.3 ms per frame at 1x / 2x / 5x. Ruled out as the cause — startup dilution
    (0.1-1.5%), differing provenance (identical preset/binary/commit, OMP_WAIT_POLICY
    passive throughout), an idle spin floor (4-6%), and frequency scaling, which on this
    host runs the wrong way (a busy core bills 2.5x MORE for the same instructions).
    Still unexplained; part is idle-time background CPU amortised over fewer frames at
    low rates. Compare within a rate only, as `rate` in FINGERPRINT already enforces.

    Also a property of the build: MP_PROC_NUM is baked in from the build host's core
    count, so the same source compiled on a smaller machine parallelises less. binary_sha
    is in FINGERPRINT, so those are never one sample.
    """
    if not cpu_s_total or not n_poses:
        return None
    return cpu_s_total / n_poses * 1000.0


def _completion(derived, bag_start, bag_end):
    """Coverage of the bag, and whether the stamps lie inside it.

    Both are None when the run recorded no bag bounds — runs written before this
    existed must not be retroactively condemned.
    """
    if bag_start is None or bag_end is None or bag_end <= bag_start:
        return {"coverage": None, "in_bag_range": None}
    t0, t1 = derived["traj_start"], derived["traj_end"]
    tol = BAG_RANGE_TOLERANCE_S
    return {
        "coverage": (t1 - t0) / (bag_end - bag_start),
        "in_bag_range": t0 >= bag_start - tol and t1 <= bag_end + tol,
    }


def trajectory_stats(tum_path):
    """Path length and displacement from the first pose (metres), plus the time span.

    The span is what lets the completion check compare this trajectory against the bag
    that was played — see `_completion`.

    end_pos_m is reported split as well as whole, because on one outdoor dataset the
    3D scalar turned out to be almost entirely its vertical component and ranked the
    runs wrongly. Against a GNSS reference on that 7.4 km route — 14 m of true relief —
    the four trajectories measured 22.5 / 48.4 / 2.0 / 40.0 m of vertical end-gap against
    horizontal APE that agreed within 25% (rmse 3.4-6.1 m). One point_lio arm read
    end_pos 10.6 m against another's 43.1 m purely because its vertical wander happened
    to come back, while being the *less* accurate of the two horizontally. A single 3D
    number mixes an axis every system is bad at, and can cancel inside it, with one they
    all handle; kept for continuity, but read the components.
    """
    stamps, poses = _read_poses(tum_path)
    if len(poses) < 2:
        raise InsufficientTrajectory(
            "{}: {} pose(s), need at least 2".format(tum_path, len(poses))
        )
    path_len = 0.0
    for a, b in zip(poses, poses[1:]):
        path_len += _dist(a, b)
    first, last = poses[0], poses[-1]
    return {
        "path_len_m": path_len,
        "end_pos_m": _dist(first, last),
        # Magnitudes, not signed offsets, so the two compose back into end_pos_m
        # regardless of which way the run drifted.
        "end_pos_horiz_m": _dist(first[:2], last[:2]),
        "end_pos_vert_m": abs(last[2] - first[2]),
        "traj_start": stamps[0],
        "traj_end": stamps[-1],
        # One pose per processed scan, so this is the frame count cpu_ms_per_frame
        # divides by. Taken from the trajectory rather than frame_events.csv on purpose:
        # every run under results/ has a trajectory, including those recorded before the
        # frame trace existed.
        "n_poses": len(poses),
    }


def _read_poses(tum_path):
    stamps, poses = [], []
    with open(tum_path) as fh:
        for line in fh:
            f = line.split()
            if len(f) >= 4:
                stamps.append(float(f[0]))
                poses.append((float(f[1]), float(f[2]), float(f[3])))
    return stamps, poses


def resource_stats(csv_path):
    """Mean/max CPU% and peak RSS from the sampler's `wall_s,cpu_pct,rss_mb` trace,
    plus the processor time the whole run consumed.

    A trace with no sample rows yields None rather than 0 — the sampler writes a
    header-only file when it never finds the process, and 0 would read as
    "consumed no CPU" instead of "not measured".
    """
    stamps, cpu, rss = [], [], []
    with open(csv_path) as fh:
        next(fh, None)  # header
        for line in fh:
            f = line.strip().split(",")
            if len(f) >= 3:
                stamps.append(float(f[0]))
                cpu.append(float(f[1]))
                rss.append(float(f[2]))
    if not cpu:
        return dict(NO_RESOURCE)
    return {
        "cpu_mean": sum(cpu) / len(cpu),
        "cpu_max": max(cpu),
        "rss_max_MB": max(rss),
        "cpu_s_total": _cpu_seconds(stamps, cpu),
    }


def _cpu_seconds(stamps, cpu):
    """Processor-seconds the sampled process consumed, reintegrated from the trace.

    Each row of sample_resource.py holds the mean CPU% over the interval *ending* at its
    own stamp, so each one is weighted by its own interval rather than averaged. On a
    metronomic trace (measured: 0.501 s, max 0.502) this is the same as cpu_mean x span;
    it differs exactly when the sampler stuttered, which is when the mean is wrong.

    The first row's interval began before the trace did, so it is charged the median of
    the intervals that are visible — the sampler's period is fixed, so that is what it
    was. A single row has no interval to infer and yields None rather than a guess.
    """
    # Not filtered: every row must keep its own interval, or the zip below pairs a
    # sample with someone else's duration.
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    if not gaps:
        return None
    first = statistics.median_low(gaps)
    return sum(c / 100.0 * max(dt, 0.0) for c, dt in zip(cpu, [first] + gaps))


def read_frame_events(csv_path):
    """Split record_frames.py's `kind,stamp,wall` trace into its two event streams.

    Both come back in sensor-stamp order, which for one-pose-per-scan is also the order
    they were produced in — the order `pair_frames` walks.
    """
    ins, outs = [], []
    with open(csv_path) as fh:
        next(fh, None)  # header
        for line in fh:
            f = line.strip().split(",")
            if len(f) != 3:
                continue
            try:
                event = (float(f[1]), float(f[2]))
            except ValueError:
                continue
            if f[0] == "in":
                ins.append(event)
            elif f[0] == "out":
                outs.append(event)
    ins.sort()
    outs.sort()
    return ins, outs


# How far an output's stamp may sit from its scan's before that scan cannot be its
# source. Scans of the same run are not evenly spaced — measured on MID-360, the interval
# between input stamps wanders over 0.086–0.115 s around a nominal 0.1 — so the window has
# to be wider than that jitter while staying well under the two periods a genuinely
# skipped frame would put between them.
SKIP_TOLERANCE_PERIODS = 1.5

# Above this many outputs per input the one-pose-per-scan correspondence that pairing
# rests on is not what the system is doing, and no timing is derived at all.
MAX_OUTPUTS_PER_INPUT = 1.5


def pair_frames(ins, outs, sensor_period):
    """Match each output to the scan it came from.

    Outputs consume inputs in order — one scan in, one pose out — so the k-th pose came
    from the k-th scan consumed, and pairing is a two-pointer walk rather than a timestamp
    lookup. An earlier version matched on stamps ("the most recent input not stamped later
    than this output") and was wrong on real data: a baseline stamps its odometry at the
    end of the sweep, roughly a period after the scan's own stamp, so whenever the next
    scan arrives sooner than that the output lands past it and is credited to a scan not
    yet processed. On one 11 729-frame run that misattributed 1 514 and claimed a 13 %
    frame drop against a true count of zero.

    Stamps still do the job order cannot: separating "the system skipped this scan" from
    "it has not got to it yet". An output more than SKIP_TOLERANCE_PERIODS ahead of the
    pending input walks the pointer past it — which is what the opening scans are, consumed
    during IMU initialisation with no pose published.

    Returns (pairs, skipped_in, unmatched_out); each pair is (wall_out, lat_ms, queued),
    where lat_ms = wall_out - wall_in and `queued` marks a frame that arrived before its
    predecessor was published, i.e. one whose lat_ms includes queue wait.
    """
    tol = SKIP_TOLERANCE_PERIODS * sensor_period
    pairs = []
    skipped = unmatched = 0
    prev_out_wall = None
    j = 0
    for out_stamp, out_wall in outs:
        # Scans consumed without a pose: the initialisation lead-in, or a genuine drop.
        while j < len(ins) and out_stamp - ins[j][0] > tol:
            j += 1
            skipped += 1
        # Nothing left to attribute this to, or it predates the pending scan.
        if j >= len(ins) or out_stamp < ins[j][0] - tol:
            unmatched += 1
            continue
        wall_in = ins[j][1]
        if out_wall < wall_in:
            # Published before its scan arrived: the correspondence has broken down. Drop
            # the frame rather than admit a negative latency.
            unmatched += 1
            j += 1
            continue
        queued = prev_out_wall is not None and wall_in < prev_out_wall
        pairs.append((out_wall, (out_wall - wall_in) * 1000.0, queued))
        prev_out_wall = out_wall
        j += 1
    return pairs, skipped, unmatched


def frame_stats(csv_path):
    """Per-frame wall-clock latency from one run's frame event trace.

    Three different kinds of nothing, kept distinct on purpose:
      no events at all      -> everything null (never measured)
      inputs but no matches -> out_ratio 0.0 (measured; the system emitted no pose)
      no inputs at all      -> out_ratio null (the input topic was wrong)
    """
    ins, outs = read_frame_events(csv_path)
    stats = dict(NO_FRAMES)
    if not ins and not outs:
        return stats

    # Read off the input stream alone, and before pairing, which needs the sensor period.
    # sensor_hz is on the sim-time axis, so it is the rig's true frame rate whatever the
    # playback rate; the wall-clock rate beside it is what playback actually achieved, and
    # their ratio is the only place that shows up — `rosbag play -r` does not guarantee
    # the rate it was asked for, and metrics.json records only what was asked.
    stats["sensor_hz"] = _median_rate([stamp for stamp, _ in ins])
    wall_hz = _median_rate([wall for _, wall in ins])
    if stats["sensor_hz"] and wall_hz:
        stats["rate_actual"] = wall_hz / stats["sensor_hz"]
    stats["in_jitter"] = _in_jitter([wall for _, wall in ins])

    if not ins or stats["sensor_hz"] is None:
        # Outputs with no scan stream to attribute them to: the input topic was wrong.
        stats["unmatched_out"] = len(outs)
        return stats
    if len(outs) > MAX_OUTPUTS_PER_INPUT * len(ins):
        # More poses than scans — the system is publishing per IMU sample rather than per
        # scan (Point-LIO with publish_odometry_without_downsample), and one-pose-per-scan
        # pairing would put confident numbers on frames it had matched wrongly.
        stats["unmatched_out"] = len(outs)
        return stats

    pairs, skipped, unmatched = pair_frames(ins, outs, 1.0 / stats["sensor_hz"])
    stats["skipped_in"] = skipped
    stats["unmatched_out"] = unmatched
    stats["out_ratio"] = len(pairs) / len(ins)

    if not pairs:
        return stats
    lat = [p[1] for p in pairs]
    # How much of lat_* is queue wait rather than the system's own work. Near 0 the
    # latencies are the system; above that they are partly the backlog in front of it.
    stats["saturated_frac"] = sum(1 for p in pairs if p[2]) / len(pairs)
    stats["lat_p50_ms"] = _nearest_rank(lat, 0.50)
    stats["lat_p99_ms"] = _nearest_rank(lat, 0.99)
    stats["lag_growth_ms"] = _lag_growth(lat)  # pairs are already in output order
    return stats


def _in_jitter(walls):
    """p99 / p50 of the gaps between input arrivals — how evenly the bag was delivered.

    1.0 is a metronome, and it is the precondition every latency here rests on: scans
    arriving in clumps queue behind each other without the system being behind at all.
    Measured 1.07-1.08 on smooth replays against 4.31 on one played off an external drive,
    which stalled 349 times for 28.3 s total (12 % of the replay) and then delivered the
    backlog in bursts.
    """
    gaps = sorted(b - a for a, b in zip(walls, walls[1:]) if b > a)
    if len(gaps) < 2:
        return None
    p50 = gaps[_nearest_index(gaps, 0.50)]
    return (gaps[_nearest_index(gaps, 0.99)] / p50) if p50 > 0 else None


def _nearest_index(ordered, q):
    return max(1, math.ceil(q * len(ordered))) - 1


def _median_rate(values):
    """Hz from the median gap between consecutive values, or None below two of them."""
    gaps = [b - a for a, b in zip(values, values[1:]) if b > a]
    if not gaps:
        return None
    return 1.0 / statistics.median_low(gaps)


def _nearest_rank(values, q):
    """The q-quantile as a real observation — no interpolation.

    Same reason as `summarize`'s median_low: an interpolated percentile can land on a
    value no frame ever took.
    """
    ordered = sorted(values)
    return ordered[_nearest_index(ordered, q)]


def _lag_growth(values):
    """How much further behind the run ended than it started, in ms.

    First and last tenth compared by low median rather than a regression slope over the
    whole run: robust to the odd outlier, and "800 ms further behind by the end" is a
    sentence, where "slope 2.3 ms/s" needs one built around it.
    """
    if len(values) < MIN_FRAMES_FOR_LAG:
        return None
    k = max(1, len(values) // 10)
    return statistics.median_low(values[-k:]) - statistics.median_low(values[:k])


def collect(dataset_dir, min_coverage=DEFAULT_MIN_COVERAGE, skip_systems=()):
    """Load every run under results/<dataset>/<system>/<preset>/run<NN>/.

    Derived quantities are written back into each run's metrics.json so that no other
    consumer has to recompute them — this module is their only implementation.

    skip_systems names systems to leave out entirely (see disabled_inventory, which
    counts what this drops so the omission is reported rather than merely happening).
    """
    skip_systems = set(skip_systems)
    records = []
    for run_dir in sorted(Path(dataset_dir).glob("*/*/run*")):
        if not run_dir.is_dir():
            continue
        if run_dir.parent.parent.name in skip_systems:
            continue
        metrics = run_dir / "metrics.json"
        if not metrics.exists():
            # SIGKILL cannot be trapped, so a force-killed run (`docker rm -f`) leaves
            # artefacts and no record. Globbing for metrics.json would skip it silently
            # and n would shrink unnoticed — the very thing the run record exists to prevent.
            print(
                "aggregate: {} has no metrics.json — run was killed".format(run_dir),
                file=sys.stderr,
            )
            records.append(_killed_record(run_dir))
            continue
        try:
            rec = load_run(run_dir, min_coverage=min_coverage)
        except MalformedRun as e:
            print("aggregate: skipping {} ({})".format(run_dir, e), file=sys.stderr)
            continue
        _write_derived(metrics, rec["derived"])
        records.append(rec)
    return records


def disabled_inventory(dataset_dir, disabled):
    """What collect() will drop for `disabled`, as [(system, reason, runs)].

    A separate pass so collect() never opens a disabled run at all — a system disabled
    for being broken would otherwise still warn about it once per run. Only systems with
    runs present appear; a dataset that never ran one has nothing to report.
    """
    found = {}
    for run_dir in sorted(Path(dataset_dir).glob("*/*/run*")):
        if run_dir.is_dir() and run_dir.parent.parent.name in disabled:
            system = run_dir.parent.parent.name
            found[system] = found.get(system, 0) + 1
    return [(system, disabled[system], runs) for system, runs in sorted(found.items())]


def _killed_record(run_dir):
    """A stand-in record for a run directory whose metrics.json never got written.

    system/preset come from the path, which is the only surviving evidence, so the run
    still appears under the right group instead of vanishing from the count.
    """
    rec = {
        "system": run_dir.parent.parent.name,
        "preset": run_dir.parent.name,
        "run_dir": str(run_dir),
        "status": "failed",
        "fail_reason": "no metrics.json (run was killed)",
        "derived": {},
    }
    rec.update(dict.fromkeys(FINGERPRINT))   # nothing is known about how it was built
    return rec


def _write_derived(metrics_path, derived):
    """Merge `derived` into metrics.json, leaving every field run_system.sh wrote.

    An empty derivation is never written. Aggregating while a batch is still running finds
    trajectories with fewer than two poses, and persisting that emptiness replaced a
    complete record with nothing — the run then read as failed until someone re-aggregated.
    Since an empty result says only "could not derive", skipping the write can lose nothing
    and overwriting can lose everything.
    """
    if not derived:
        return
    on_disk = json.loads(metrics_path.read_text())
    on_disk["derived"] = derived
    metrics_path.write_text(json.dumps(on_disk, indent=2, sort_keys=True) + "\n")


# `rate` belongs here with the build fingerprints because playback speed changes what the
# numbers mean, not just their spread: at 5x every wall-clock second carries five seconds
# of work, so CPU% is a different quantity, and end-to-end latency stops being a property
# of the system at all (§ the rate note in render_text). Runs at different rates are
# therefore never one sample, exactly as runs from different binaries are not.
FINGERPRINT = ("preset_sha", "binary_sha", "system_commit", "rate")


def group_runs(records):
    """Group runs by (system, preset), splitting further when fingerprints disagree.

    A group whose runs were built from different configurations or different binaries
    is not one sample. It is split by fingerprint and every resulting subgroup is
    flagged `consistent: False`, so its statistics can still be read but never as a
    single population — the mistake that once invalidated a whole sweep.
    """
    by_pair = {}
    for r in records:
        by_pair.setdefault((r["system"], r["preset"]), []).append(r)

    groups = []
    for (system, preset), runs in sorted(by_pair.items()):
        # A run with no fingerprint at all (killed before metrics.json was written) is
        # not evidence of a configuration change. Splitting on it would raise ⚠ MIXED,
        # claiming these runs were built differently, which is not what happened — so
        # such runs join the first subgroup instead of forming one of their own.
        by_fp, unknown = {}, []
        for r in runs:
            fp = tuple(r.get(k) for k in FINGERPRINT)
            if any(fp):
                by_fp.setdefault(fp, []).append(r)
            else:
                unknown.append(r)

        ordered = sorted(by_fp.items(), key=lambda kv: str(kv[0])) or [
            ((None,) * len(FINGERPRINT), [])
        ]
        consistent = len(ordered) == 1
        for i, (fp, fp_runs) in enumerate(ordered):
            members = fp_runs + (unknown if i == 0 else [])
            groups.append(
                {
                    "system": system,
                    "preset": preset,
                    "fingerprint": dict(zip(FINGERPRINT, fp)),
                    "consistent": consistent,
                    "runs": [r for r in members if r["status"] == "ok"],
                    "excluded": [r for r in members if r["status"] != "ok"],
                }
            )
    return groups


# The table columns, as (derived key, heading, decimal places). Split in two because one
# table wide enough for all of them does not fit a terminal, along the plan's own boundary
# of accuracy (§6) against real-time performance (§7).
ACCURACY_COLUMNS = (
    ("path_len_m", "path_len_m", 1),
    ("end_pos_m", "end_pos_m", 1),
    ("end_pos_horiz_m", "end_horiz_m", 1),
    ("end_pos_vert_m", "end_vert_m", 1),
    # Blank on every dataset without a usable GNSS reference, which is most of them —
    # and that blank is the honest reading: an open route has no closure, so without a
    # reference there is no accuracy number to be had at all.
    ("gnss_ape_horiz_rmse_m", "ape_horiz_m", 2),
    ("gnss_ape_vert_rmse_m", "ape_vert_m", 2),
    ("gnss_end_err_horiz_m", "end_err_h_m", 2),
    ("gnss_end_err_vert_m", "end_err_v_m", 2),
)
PERF_COLUMNS = (
    ("lat_p50_ms", "lat_p50ms", 1),
    ("lat_p99_ms", "lat_p99ms", 1),
    ("cpu_ms_per_frame", "cpu_ms/f", 1),
    ("parallelism", "parallel", 2),
    ("cpu_mean", "cpu_mean%", 1),
    ("cpu_max", "cpu_max%", 1),
    ("rss_max_MB", "rss_max_MB", 1),
    ("saturated_frac", "sat", 2),
    ("out_ratio", "out_ratio", 3),
)
COLUMNS = ACCURACY_COLUMNS + PERF_COLUMNS

# Printed under the real-time table. Here rather than in render_text because it documents
# the columns above it, and the two drift apart if they live in different files' worth of
# scrolling.
PERF_LEGEND = (
    "lat_p50ms = wall clock from scan arriving to its pose being published, and",
    "lat_p99ms the tail of the same — the tail is what misses a deadline, not the",
    "median, so a system is only comfortably real-time when p99 fits the frame period.",
    "cpu_ms/f  = processor time one frame cost (total CPU seconds / poses produced).",
    "parallel  = cpu_ms/f over lat_p50ms, i.e. cores busy while a frame was handled.",
    "sat       = share of frames that arrived before the previous one was published,",
    "            so their lat_* includes queue wait. Near 0 the latencies are the",
    "            system; well above it they are partly the backlog in front of it.",
    "out_ratio = 1.0 when every input scan produced a pose.",
    "",
    "Neither latency column compares across playback rates: part of lat_* is waiting for",
    "the other topics a scan needs, which is a bag-time quantity and so shrinks by the",
    "rate. Judge real-time behaviour at the rate the sensor actually runs.",
)

# Summarized into stats.json but kept out of the tables, which are already as wide as a
# terminal allows.
JSON_ONLY_METRICS = (
    # Summarized across a group's ok runs, where it reads as the fastest the vehicle was
    # ever measured to move — a sanity check on the limit that defines divergence at all.
    "v_max_mps",
    "lag_growth_ms",
    "sensor_hz",
    "rate_actual",
    "in_jitter",
    "skipped_in",
    "unmatched_out",
)

METRICS = tuple(key for key, _, _ in COLUMNS) + JSON_ONLY_METRICS
SPLIT_METRIC = "end_pos_m"


def group_stats(group, split_at=None):
    """Statistics for one group: per-metric summaries plus the raw per-run values.

    The raw values are kept alongside the summaries on purpose — on a bimodal sample
    no summary statistic shows the two modes, but the sorted list does.
    """
    runs = group["runs"]
    metrics, per_run = {}, {}
    for name in METRICS:
        values = [
            r["derived"][name]
            for r in runs
            if r["derived"].get(name) is not None
        ]
        metrics[name] = summarize(values) if values else None
        per_run[name] = sorted(values)

    return {
        "system": group["system"],
        "preset": group["preset"],
        "fingerprint": group["fingerprint"],
        "consistent": group["consistent"],
        "n": len(runs),
        "no_spread": len(runs) == 1,
        "metrics": metrics,
        "per_run": per_run,
        "split": _split(per_run[SPLIT_METRIC], split_at),
        "excluded": [
            {
                "run_dir": r.get("run_dir"),
                "status": r["status"],
                "reason": r.get("void_reason") or r.get("fail_reason"),
            }
            for r in group["excluded"]
        ],
    }


def _split(values, threshold):
    """Two-bucket count either side of an explicitly supplied threshold.

    There is no default: the boundary between "recovered" and "exploded" is a
    dataset-dependent distance, so shipping one would encode a number we do not know.
    """
    if threshold is None:
        return None
    return {
        "threshold": threshold,
        "below": sum(1 for v in values if v < threshold),
        "above": sum(1 for v in values if v >= threshold),
    }


def summarize(values):
    """Count, median and range of one quantity across a group's runs.

    `median_low` rather than `median`: on an even sample the arithmetic median
    averages the two central observations, which on a bimodal distribution
    lands in the empty gap between the modes and reports an outcome
    no run produced. The low median is always a real observation. Mean and standard
    deviation are omitted for the same reason.
    """
    return {
        "n": len(values),
        "median": statistics.median_low(values),
        "min": min(values),
        "max": max(values),
    }


def _dist(a, b):
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


def render_text(dataset, stats, disabled=()):
    """The human-readable tables. stats.json carries the same content verbatim."""
    labels = _labels(stats)
    width = max([len(l) for l in labels] + [10])

    out = ["# {} — N-run repeatability: median [min–max]".format(dataset), ""]
    # Under the title rather than in a footnote: a system missing from the tables without
    # a word here reads as one that was never run, not one that was set aside.
    if disabled:
        out += ["## disabled — present in results/, left out of everything below", ""]
        out += ["{}  ({} run(s)) — {}".format(s, n, r) for s, r, n in disabled]
        out += ["", "Delete its `disabled:` line in configs/systems.yaml to bring it back.", ""]
    out += ["## accuracy", ""] + _metric_table(labels, stats, ACCURACY_COLUMNS)
    out += ["", "## real-time", ""] + _metric_table(labels, stats, PERF_COLUMNS)
    out += [""] + list(PERF_LEGEND)
    out += _jitter_note(labels, stats)
    out += _rate_note(labels, stats)

    mixed = [l for l, s in zip(labels, stats) if not s["consistent"]]
    if mixed:
        out += [
            "",
            "⚠ MIXED: {} — runs inside one (system, preset) were built from different"
            .format(", ".join(mixed)),
            "  configurations or binaries, so each fingerprint is summarized separately",
            "  and never merged into one sample. See ## provenance below.",
        ]

    out += ["", "## per-run (sorted by {})".format(SPLIT_METRIC)]
    for label, s in zip(labels, stats):
        values = s["per_run"][SPLIT_METRIC]
        out.append(
            "{:<{w}}  {}".format(
                label, "  ".join(_num(v) for v in values) or "-", w=width
            )
        )
        if s["split"] and values:
            sp = s["split"]
            out.append(
                "{:<{w}}  split at {}: {} below / {} above".format(
                    "", _num(sp["threshold"]), sp["below"], sp["above"], w=width
                )
            )

    excluded = [(l, e) for l, s in zip(labels, stats) for e in s["excluded"]]
    if excluded:
        out += ["", "## excluded"]
        for label, e in excluded:
            out.append(
                "{:<{w}}  {:<7} {}".format(
                    label, _displayed(e["status"]).upper(), _excluded_reason(e),
                    w=width,
                )
            )

    out += ["", "## provenance"]
    for label, s in zip(labels, stats):
        fp = s["fingerprint"]
        out.append(
            "{:<{w}}  rate {}  preset_sha {}  binary_sha {}  commit {}".format(
                label,
                "?" if fp.get("rate") is None else _num(fp["rate"]),
                _short(fp.get("preset_sha")),
                _short(fp.get("binary_sha")),
                _short(fp.get("system_commit")),
                w=width,
            )
        )
    return "\n".join(out) + "\n"


def _labels(stats):
    """`system/preset`, suffixed `#1`, `#2` … when a group was split by fingerprint.

    Without the suffix two subgroups of the same (system, preset) would be
    indistinguishable everywhere below the table.
    """
    total = {}
    for s in stats:
        key = (s["system"], s["preset"])
        total[key] = total.get(key, 0) + 1

    seen, labels = {}, []
    for s in stats:
        key = (s["system"], s["preset"])
        label = "{}/{}".format(*key)
        if total[key] > 1:
            seen[key] = seen.get(key, 0) + 1
            label += "#{}".format(seen[key])
        labels.append(label)
    return labels


def _metric_table(labels, stats, columns):
    header = ["system", "preset", "n"] + [label for _, label, _ in columns]
    rows = [
        # The preset cell carries the label's `#N` suffix so a split group's rows are
        # distinguishable in the table itself, not only in the sections below.
        [s["system"], label.split("/", 1)[1], _n_cell(s)]
        + [_cell(s["metrics"][key], places) for key, _, places in columns]
        for label, s in zip(labels, stats)
    ]
    return _table([header] + rows)


# Above this ratio of p99 to median input gap the bag was not delivered on the schedule
# the sensor recorded it on. Measured: 1.07–1.08 on smooth replays, 4.31 on one that
# stalled for 12 % of its duration, so the line sits far from both.
MAX_IN_JITTER = 1.5


def _jitter_note(labels, stats):
    """Name the runs whose input never arrived smoothly, and say what that costs.

    This comes first because it is upstream of everything else in the table: when scans
    arrive in clumps, a frame lands while the previous is still being processed without
    the system being behind at all, so `sat` fills up and `lat_*` inherits a queue the bag
    reader created rather than the system.
    """
    rough = []
    for label, s in zip(labels, stats):
        summary = s["metrics"].get("in_jitter")
        if summary and summary["max"] > MAX_IN_JITTER:
            rough.append("{} ({:.1f}x)".format(label, summary["max"]))
    if not rough:
        return []
    return [
        "",
        "⚠ UNEVEN INPUT: {}".format(", ".join(rough)),
        "  The p99 gap between input scans is that many times the median, so playback did",
        "  not feed the system at the rate the sensor recorded. Frames arriving in clumps",
        "  queue behind each other on their own, which inflates sat and puts the reader's",
        "  backlog into lat_*. Treat every timing in this row as describing the replay, not",
        "  the system, and re-run from storage that sustains the rate.",
    ]


def _rate_note(labels, stats):
    """Name the groups played at anything but 1.0x, and say what that costs.

    They are already a separate sample — `rate` is part of FINGERPRINT — but the split
    alone does not tell a reader that half the columns in front of them stopped
    describing the system.
    """
    off = [
        "{} ({}x)".format(label, _num(s["fingerprint"]["rate"]))
        for label, s in zip(labels, stats)
        if s["fingerprint"].get("rate") not in (None, 1.0)
    ]
    if not off:
        return []
    return [
        "",
        "⚠ NOT 1.0x: {}".format(", ".join(off)),
        "  Every column above describes the playback speed as much as the system, and none",
        "  of them convert back to 1.0x. Measured on one bag, point_lio reads lat_p50 21.1 /",
        "  18.8 / 14.8 ms and cpu_ms/f 21.0 / 16.8 / 12.4 at 1x / 3x / 5x, from one binary on",
        "  one dataset. Only `parallel` held still (faster-lio: 4.37 / 4.42 / 4.47).",
    ]


def _table(rows):
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return [
        "  ".join(c.ljust(w) for c, w in zip(row, widths)).rstrip() for row in rows
    ]


# How a status is *reported* in the tables, where it is not how it is *recorded*. A
# comparison table marks a cell that carries no result, and one word does that job — the
# way this field's tables have always marked one. Which way a run failed is a sentence,
# and it goes in the reason beside it rather than into the count.
#
# `status` in the records and in stats.json keeps the distinction, because the distinction
# is real and measured: PV-LIO was OOM-killed at 23% of one bag with a perfectly plausible
# trajectory, and blew up to a 3851 km path on another. One is fixed with more memory and
# the other is not, so anything reading stats.json can still tell them apart.
DISPLAY_STATUS = {"diverged": "failed"}
# void first: a human verdict outranks the automatic ones, here as in load_run.
DISPLAY_ORDER = ("void", "failed")


def _displayed(status):
    return DISPLAY_STATUS.get(status, status)


def _excluded_reason(entry):
    """The reason as the tables show it, naming the failure mode the count dropped."""
    reason = entry["reason"] or entry["run_dir"] or ""
    if _displayed(entry["status"]) != entry["status"]:
        return "{}: {}".format(entry["status"], reason)
    return reason


def _n_cell(s):
    notes = []
    for status in DISPLAY_ORDER:
        count = sum(1 for e in s["excluded"] if _displayed(e["status"]) == status)
        if count:
            notes.append("{} {}".format(count, status))
    if s["no_spread"]:
        notes.append("no spread")
    return "{}{}".format(s["n"], " ({})".format(", ".join(notes)) if notes else "")


def _cell(summary, places=1):
    """`median [min–max]`, collapsed to the bare value when the runs agree."""
    if summary is None:
        return "-"
    if summary["min"] == summary["max"]:
        return _num(summary["median"], places)
    return "{} [{}–{}]".format(
        _num(summary["median"], places),
        _num(summary["min"], places),
        _num(summary["max"], places),
    )


def _num(value, places=1):
    """One decimal by default; ratios need more before rounding erases the signal —
    out_ratio 0.998 is a system dropping two frames in a thousand, and 1.0 is not."""
    return "{:.{p}f}".format(value, p=places).rstrip("0").rstrip(".")


def _short(sha):
    return (sha or "?")[:7]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("dataset_dir", help="results/<dataset>")
    p.add_argument(
        "--split-at",
        type=float,
        default=None,
        metavar="M",
        help="count runs either side of this {} threshold (no default: the "
        "boundary is dataset-dependent)".format(SPLIT_METRIC),
    )
    p.add_argument(
        "--min-coverage",
        type=float,
        default=DEFAULT_MIN_COVERAGE,
        metavar="F",
        help="fraction of the bag a trajectory must span to count as completed "
        "(default %(default)s)",
    )
    a = p.parse_args(argv)

    dataset_dir = Path(a.dataset_dir)

    disabled = registry.disabled_systems_or_warn("aggregate")
    inventory = disabled_inventory(dataset_dir, disabled)
    for system, reason, runs in inventory:
        print(
            "aggregate: skipped {} (disabled: {}) — {} run(s)".format(
                system, reason, runs
            ),
            file=sys.stderr,
        )

    records = collect(dataset_dir, min_coverage=a.min_coverage, skip_systems=disabled)
    if not records:
        # Distinct from an empty dataset: the fix is one line in configs/systems.yaml.
        what = (
            "every run under {} belongs to a disabled system"
            if inventory
            else "no runs under {}"
        )
        print("aggregate: " + what.format(dataset_dir), file=sys.stderr)
        return 1

    stats = [group_stats(g, split_at=a.split_at) for g in group_runs(records)]
    text = render_text(dataset_dir.name, stats, disabled=inventory)

    (dataset_dir / "stats.txt").write_text(text)
    (dataset_dir / "stats.json").write_text(
        json.dumps(
            {
                "dataset": dataset_dir.name,
                "groups": stats,
                # So a reader of stats.json alone can tell "set aside" from "never run".
                "disabled": [
                    {"system": s, "reason": r, "runs": n} for s, r, n in inventory
                ],
            },
            indent=2,
        )
        + "\n"
    )
    print(text, end="")
    print(
        "saved -> {0}/stats.txt  |  {0}/stats.json".format(dataset_dir), file=sys.stderr
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
