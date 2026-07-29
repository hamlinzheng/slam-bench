#!/usr/bin/env python3
"""Aggregate N repeated runs of a dataset into per-(system, preset) statistics.

Standard library only, by design: the tests then run on the host with plain
pytest — no container, no ROS, no bag.
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

NO_RESOURCE = {"cpu_mean": None, "cpu_max": None, "rss_max_MB": None}

# A healthy run's trajectory covers essentially the whole bag (measured: 99%). The
# threshold is dimensionless and 1.0 is the ideal by definition — unlike the explosion
# split of §6.3, whose scale is a property of the dataset — so a default is defensible.
DEFAULT_MIN_COVERAGE = 0.9
# Odometry for a scan can be stamped slightly outside the bag's own start/end.
BAG_RANGE_TOLERANCE_S = 5.0


class InsufficientTrajectory(Exception):
    """Fewer than two poses — nothing can be derived from this run."""


class MalformedRun(Exception):
    """The run record cannot be read at all."""


def load_run(run_dir, min_coverage=DEFAULT_MIN_COVERAGE):
    """Read one run directory into a record: its metrics.json plus status + derived.

    Status precedence: a human VOID verdict outranks the automatic checks, because a
    run can exit cleanly and still be worthless (findings §4.1 diagB exited 0 after
    the map was starved).
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
        rec["void_reason"] = void.read_text().strip().splitlines()[0].strip()
        return rec

    reason = _completion_failure(rec, min_coverage)
    rec["status"] = "failed" if reason else "ok"
    if reason:
        rec["fail_reason"] = reason
    return rec


def _completion_failure(rec, min_coverage):
    """Why this run is not a completed run, or None if it is.

    Beyond the exit code, completion means the trajectory actually came from the bag
    this run played and covered it (plan §5.3). Both checks exist because a run can
    report bag_play_exit 0 and still be worthless: with network_mode host a surviving
    container keeps the ROS master, and the recorder then captures a *different* run's
    /Odometry — observed, and invisible to every other check.
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


def _derive(run_dir, bag_start=None, bag_end=None):
    """Derived quantities, or {} when the trajectory cannot carry any."""
    try:
        derived = trajectory_stats(run_dir / "trajectory.tum")
    except (OSError, InsufficientTrajectory):
        return {}
    csv = run_dir / "resource.csv"
    try:
        derived.update(resource_stats(csv))
    except OSError:
        derived.update(NO_RESOURCE)
    derived.update(_completion(derived, bag_start, bag_end))
    return derived


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
    """
    stamps, poses = _read_poses(tum_path)
    if len(poses) < 2:
        raise InsufficientTrajectory(
            "{}: {} pose(s), need at least 2".format(tum_path, len(poses))
        )
    path_len = 0.0
    for a, b in zip(poses, poses[1:]):
        path_len += _dist(a, b)
    return {
        "path_len_m": path_len,
        "end_pos_m": _dist(poses[0], poses[-1]),
        "traj_start": stamps[0],
        "traj_end": stamps[-1],
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
    """Mean/max CPU% and peak RSS from the sampler's `wall_s,cpu_pct,rss_mb` trace.

    A trace with no sample rows yields None rather than 0 — the sampler writes a
    header-only file when it never finds the process, and 0 would read as
    "consumed no CPU" instead of "not measured".
    """
    cpu, rss = [], []
    with open(csv_path) as fh:
        next(fh, None)  # header
        for line in fh:
            f = line.strip().split(",")
            if len(f) >= 3:
                cpu.append(float(f[1]))
                rss.append(float(f[2]))
    if not cpu:
        return dict(NO_RESOURCE)
    return {
        "cpu_mean": sum(cpu) / len(cpu),
        "cpu_max": max(cpu),
        "rss_max_MB": max(rss),
    }


def collect(dataset_dir, min_coverage=DEFAULT_MIN_COVERAGE):
    """Load every run under results/<dataset>/<system>/<preset>/run<NN>/.

    Derived quantities are written back into each run's metrics.json so that no other
    consumer has to recompute them — this module is their only implementation.
    """
    records = []
    for run_dir in sorted(Path(dataset_dir).glob("*/*/run*")):
        if not run_dir.is_dir():
            continue
        metrics = run_dir / "metrics.json"
        if not metrics.exists():
            # SIGKILL cannot be trapped, so a force-killed run (`docker rm -f`) leaves
            # artefacts and no record. Globbing for metrics.json would skip it silently
            # and n would shrink unnoticed — the very thing §8.2 exists to prevent.
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
    """Merge `derived` into metrics.json, leaving every field run_system.sh wrote."""
    on_disk = json.loads(metrics_path.read_text())
    on_disk["derived"] = derived
    metrics_path.write_text(json.dumps(on_disk, indent=2, sort_keys=True) + "\n")


FINGERPRINT = ("preset_sha", "binary_sha", "system_commit")


def group_runs(records):
    """Group runs by (system, preset), splitting further when fingerprints disagree.

    A group whose runs were built from different configurations or different binaries
    is not one sample. It is split by fingerprint and every resulting subgroup is
    flagged `consistent: False`, so its statistics can still be read but never as a
    single population — the mistake diagnosed in findings §4.3.
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


METRICS = ("path_len_m", "end_pos_m", "cpu_mean", "cpu_max", "rss_max_MB")
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
    averages the two central observations, which on the bimodal distribution of
    findings §2.3 lands in the empty gap between the modes and reports an outcome
    no run produced. The low median is always a real observation. Mean and standard
    deviation are omitted for the same reason (§6.2 of the design).
    """
    return {
        "n": len(values),
        "median": statistics.median_low(values),
        "min": min(values),
        "max": max(values),
    }


def _dist(a, b):
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


COLUMNS = (
    ("path_len_m", "path_len_m"),
    ("end_pos_m", "end_pos_m"),
    ("cpu_mean", "cpu_mean%"),
    ("cpu_max", "cpu_max%"),
    ("rss_max_MB", "rss_max_MB"),
)


def render_text(dataset, stats):
    """The human-readable table. stats.json carries the same content verbatim."""
    labels = _labels(stats)
    width = max([len(l) for l in labels] + [10])

    header = ["system", "preset", "n"] + [label for _, label in COLUMNS]
    rows = [
        # The preset cell carries the label's `#N` suffix so a split group's rows are
        # distinguishable in the table itself, not only in the sections below.
        [s["system"], label.split("/", 1)[1], _n_cell(s)]
        + [_cell(s["metrics"][key]) for key, _ in COLUMNS]
        for label, s in zip(labels, stats)
    ]
    out = [
        "# {} — N-run repeatability: median [min–max]".format(dataset),
        "",
    ] + _table([header] + rows)

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
                    label, e["status"].upper(), e["reason"] or e["run_dir"] or "",
                    w=width,
                )
            )

    out += ["", "## provenance"]
    for label, s in zip(labels, stats):
        fp = s["fingerprint"]
        out.append(
            "{:<{w}}  preset_sha {}  binary_sha {}  commit {}".format(
                label,
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


def _table(rows):
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return [
        "  ".join(c.ljust(w) for c, w in zip(row, widths)).rstrip() for row in rows
    ]


def _n_cell(s):
    notes = []
    for status in ("void", "failed"):
        count = sum(1 for e in s["excluded"] if e["status"] == status)
        if count:
            notes.append("{} {}".format(count, status))
    if s["no_spread"]:
        notes.append("no spread")
    return "{}{}".format(s["n"], " ({})".format(", ".join(notes)) if notes else "")


def _cell(summary):
    """`median [min–max]`, collapsed to the bare value when the runs agree."""
    if summary is None:
        return "-"
    if summary["min"] == summary["max"]:
        return _num(summary["median"])
    return "{} [{}–{}]".format(
        _num(summary["median"]), _num(summary["min"]), _num(summary["max"])
    )


def _num(value):
    return "{:.1f}".format(value).rstrip("0").rstrip(".")


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
    records = collect(dataset_dir, min_coverage=a.min_coverage)
    if not records:
        print("aggregate: no runs under {}".format(dataset_dir), file=sys.stderr)
        return 1

    stats = [group_stats(g, split_at=a.split_at) for g in group_runs(records)]
    text = render_text(dataset_dir.name, stats)

    (dataset_dir / "stats.txt").write_text(text)
    (dataset_dir / "stats.json").write_text(
        json.dumps({"dataset": dataset_dir.name, "groups": stats}, indent=2) + "\n"
    )
    print(text, end="")
    print(
        "saved -> {0}/stats.txt  |  {0}/stats.json".format(dataset_dir), file=sys.stderr
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
