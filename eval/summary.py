#!/usr/bin/env python3
"""Collect every dataset's stats.json into one Markdown summary.

aggregate.py answers "how did these systems do on this dataset". This answers "how did
they do across the datasets we have", which is a different question and mostly a different
shape: a matrix of system against dataset, one per metric, rather than one table per run
group.

Nothing is measured here. Every number comes from a stats.json that aggregate.py wrote,
so the two can never disagree — and a metric that is absent there is absent here, named
rather than blanked.

**Nothing is averaged across datasets.** The routes are 1.8 km of underground drift and
7.3 km of open road; a median APE over both describes no route that was driven. Each cell
stays attached to the dataset it was measured on, and the reader does the comparing.

Usage (inside the container): python3 eval/summary.py [results_dir] [--out FILE]

Stdlib only, like aggregate.py and for the same reason: its tests run on the host under
plain pytest, with no container.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import aggregate  # noqa: E402

# Cross-dataset matrices, as (section, [(metric key, heading, decimals)]). Headline
# quantities only — the per-dataset detail below carries the rest, with its ranges.
ACCURACY = (
    ("gnss_ape_horiz_rmse_m", "APE horizontal RMSE (m)", 2),
    ("gnss_ape_vert_rmse_m", "APE vertical RMSE (m)", 2),
    ("gnss_end_err_horiz_m", "End-point horizontal error (m)", 2),
)
REALTIME = (
    ("lat_p99_ms", "Latency p99 (ms)", 1),
    ("cpu_ms_per_frame", "CPU per frame (ms)", 1),
    ("parallelism", "Cores busy per frame", 2),
)
RESOURCE = (
    ("cpu_mean", "CPU mean (%)", 1),
    ("rss_max_MB", "RSS peak (MB)", 1),
)

# The full per-dataset tables, matching what stats.txt prints.
DETAIL_ACCURACY = (
    ("path_len_m", "path_len_m", 1),
    ("end_pos_m", "end_pos_m", 1),
    ("end_pos_horiz_m", "end_horiz_m", 1),
    ("end_pos_vert_m", "end_vert_m", 1),
    ("gnss_ape_horiz_rmse_m", "ape_horiz_m", 2),
    ("gnss_ape_vert_rmse_m", "ape_vert_m", 2),
    ("gnss_end_err_horiz_m", "end_err_h_m", 2),
    ("gnss_end_err_vert_m", "end_err_v_m", 2),
)
DETAIL_REALTIME = (
    ("lat_p50_ms", "lat_p50ms", 1),
    ("lat_p99_ms", "lat_p99ms", 1),
    ("cpu_ms_per_frame", "cpu_ms/f", 1),
    ("parallelism", "parallel", 2),
    ("cpu_mean", "cpu_mean%", 1),
    ("cpu_max", "cpu_max%", 1),
    ("rss_max_MB", "rss_max_MB", 1),
    ("v_max_mps", "v_max_m/s", 1),
)

# What §10 of docs/comparison_plan.md asks for that no measurement here can supply. Listed
# on the page rather than left as empty columns: a blank cell reads as "measured, nothing
# there", and these were never measured at all.
NOT_MEASURED = (
    ("MME", "map metric M-MAP (§6.2) has no collector in eval/ yet"),
    ("Plane RMSE", "same — `roi_boxes` is carried in the bag registry but read by nothing"),
    ("Thickness", "same"),
    ("Lat p99 @ORIN", "every run here is x86_64; no ARM runs exist"),
)


def load(results_dir):
    """[(dataset, stats)] for every dataset carrying a stats.json, dataset order sorted."""
    out = []
    for path in sorted(Path(results_dir).glob("*/stats.json")):
        try:
            out.append((path.parent.name, json.loads(path.read_text())))
        except (OSError, ValueError) as e:
            warn("summary: skipping {} ({})".format(path, e))
    return out


def label_of(group):
    """`system` for the shipped preset, `system/preset` otherwise.

    The bare name for `default` because most rows are that, and a column of `/default`
    suffixes costs width without carrying anything.
    """
    return (
        group["system"]
        if group["preset"] == "default"
        else "{}/{}".format(group["system"], group["preset"])
    )


def rows_of(datasets):
    """Every (system, preset) label seen anywhere, sorted, for the matrix rows."""
    return sorted({label_of(g) for _, s in datasets for g in s["groups"]})


def cell(group, key, places):
    """One matrix cell: the group's median, or why there is no number."""
    if group is None:
        return "–"           # the system was not run on this dataset at all
    if not group["n"]:
        return "**fail**"    # it ran and no run survived; accuracy would be meaningless
    summary = (group["metrics"] or {}).get(key)
    if not summary:
        return "n/m"         # not measured — no reference, no trace, no collector
    return num(summary["median"], places)


def matrix(datasets, key, places):
    """A Markdown table of one metric, systems down and datasets across."""
    heads = [d for d, _ in datasets]
    by = {(d, label_of(g)): g for d, s in datasets for g in s["groups"]}
    lines = [
        "| system | " + " | ".join(short(h) for h in heads) + " |",
        "|---|" + "---|" * len(heads),
    ]
    for label in rows_of(datasets):
        cells = [cell(by.get((d, label)), key, places) for d in heads]
        lines.append("| `{}` | ".format(label) + " | ".join(cells) + " |")
    return lines


def completion(datasets):
    """Runs that survived, over runs attempted, per system and dataset."""
    heads = [d for d, _ in datasets]
    by = {(d, label_of(g)): g for d, s in datasets for g in s["groups"]}
    lines = [
        "| system | " + " | ".join(short(h) for h in heads) + " | total |",
        "|---|" + "---|" * (len(heads) + 1),
    ]
    for label in rows_of(datasets):
        cells, ok, total = [], 0, 0
        for d in heads:
            g = by.get((d, label))
            if g is None:
                cells.append("–")
                continue
            n, attempted = g["n"], g["n"] + len(g["excluded"])
            ok, total = ok + n, total + attempted
            cells.append(
                "{}/{}".format(n, attempted) if n else "**0/{}**".format(attempted)
            )
        lines.append(
            "| `{}` | ".format(label) + " | ".join(cells)
            + " | {}/{} |".format(ok, total)
        )
    return lines


def failures(datasets):
    """Every run that was set aside, with the reason aggregate.py recorded for it."""
    out = []
    for dataset, stats in datasets:
        for group in stats["groups"]:
            for e in group["excluded"]:
                out.append((dataset, label_of(group), e))
    return out


def provenance(datasets):
    """The fingerprints every number here shares, and any place they disagree.

    Playback rate leads because it is the one that silently invalidates a comparison:
    part of the measured latency is waiting for the other topics a scan needs, which is a
    bag-time quantity and shrinks by the rate, so runs at different rates are not one
    population and stats.txt already refuses to merge them.
    """
    rates, presets = {}, {}
    for dataset, stats in datasets:
        for group in stats["groups"]:
            rate = (group["fingerprint"] or {}).get("rate")
            rates.setdefault(rate, set()).add(dataset)
            presets.setdefault(label_of(group), set()).add(dataset)
    return rates, presets


def render(datasets, results_dir, verdicts=None):
    """The whole document."""
    if not datasets:
        return "No stats.json under {} — run `bench.sh aggregate <dataset>` first.\n".format(
            results_dir
        )

    verdicts = verdicts or {}
    rates, _ = provenance(datasets)
    runs = sum(g["n"] + len(g["excluded"]) for _, s in datasets for g in s["groups"])
    out = [
        "# slam-bench — cross-dataset summary",
        "",
        "> {} datasets, {} runs. Generated from each dataset's `stats.json`; every number "
        "here was computed by `eval/aggregate.py` and is not recomputed.".format(
            len(datasets), runs
        ),
        "",
        "**Nothing is averaged across datasets.** The routes differ by kilometres and by "
        "kind, so a figure spanning them would describe no route that was driven. Each "
        "cell stays attached to the dataset it came from.",
        "",
        "Cell values are the **median** of a group's surviving runs. The `[min–max]` "
        "range is what carries a bimodal failure rate, so read the per-dataset tables "
        "below before quoting a median.",
        "",
        "| | meaning |",
        "|---|---|",
        "| `–` | the system was not run on that dataset |",
        "| **fail** | it ran and no run survived; accuracy would be meaningless |",
        "| `n/m` | not measured — no reference, no trace, or no collector for it |",
        "",
    ]

    out += _rate_note(rates)
    out += ["## Completion", "",
            "Runs that survived, over runs attempted. A run is set aside when it did not "
            "finish the bag or when its trajectory blew up — see "
            "[Runs that fly away](../README.md#runs-that-fly-away).", ""]
    out += completion(datasets) + [""]
    out += _failure_note(datasets)

    out += _accuracy_section(datasets, verdicts)

    out += ["## Real-time", ""]
    for key, heading, places in REALTIME:
        out += ["### {}".format(heading), ""] + matrix(datasets, key, places) + [""]
    out += ["## Resource", ""]
    for key, heading, places in RESOURCE:
        out += ["### {}".format(heading), ""] + matrix(datasets, key, places) + [""]

    out += _detail(datasets)
    out += _gaps(datasets)
    return "\n".join(out) + "\n"


def _rate_note(rates):
    """A heading warning when the runs are not all at 1x, or not all at one rate."""
    if set(rates) == {1.0}:
        return []
    listed = ", ".join(
        "{}x on {}".format(num(r, 1) if r is not None else "?", len(d))
        for r, d in sorted(rates.items(), key=lambda kv: str(kv[0]))
    )
    lines = ["> **Playback rate: {}.** Every latency and CPU figure below describes the "
             "playback speed as much as the system, and none of them convert back to 1x: "
             "part of the measured latency is waiting for the other topics a scan needs, "
             "which is a bag-time quantity and shrinks by the rate. Judge real-time "
             "behaviour at the rate the sensor actually runs.".format(listed)]
    if len(rates) > 1:
        lines.append(">")
        lines.append("> The runs collected here do **not** share one rate, so the "
                     "real-time tables below compare quantities that are not the same "
                     "quantity. Treat them as per-dataset readings only.")
    return lines + [""]


def _failure_note(datasets):
    rows = failures(datasets)
    if not rows:
        return ["Every run survived on every dataset.", ""]
    out = ["<details><summary>{} run(s) set aside — reasons</summary>".format(len(rows)),
           "", "| dataset | system | | reason |", "|---|---|---|---|"]
    for dataset, label, entry in rows:
        # aggregate owns how a status is worded, so that its tables and this one cannot
        # drift apart: `failed` whichever way it failed, with the kind named in the reason.
        out.append("| {} | `{}` | {} | {} |".format(
            short(dataset), label,
            aggregate.displayed_status(entry["status"]).upper(),
            aggregate.excluded_reason(entry)))
    return out + ["", "</details>", ""]


def _accuracy_section(datasets, verdicts):
    """Accuracy, and an explicit account of the datasets that cannot carry it.

    A dataset with no usable GNSS reference is not a gap in the measurement — it is a
    measured fact about the receiver, and gnss_ref.json says why. Reproduced here so the
    blank column is explained on the page rather than three files away.
    """
    out = ["## Accuracy against the GNSS reference", ""]
    withref = [(d, s) for d, s in datasets if _has_reference(s)]
    if not withref:
        out += ["No dataset here has a usable GNSS reference, so there is no accuracy "
                "measurement to report. `path_len_m` and `end_pos_m` in the per-dataset "
                "tables are shape and closure, not accuracy.", ""]
    else:
        out += ["Only the {} of {} datasets with a usable reference appear here. The "
                "others are not missing a measurement — they were measured and found "
                "not to have one; see below.".format(len(withref), len(datasets)), ""]
        for key, heading, places in ACCURACY:
            out += ["### {}".format(heading), ""] + matrix(withref, key, places) + [""]
    noref = [d for d, s in datasets if not _has_reference(s)]
    if noref:
        out += ["### Datasets with no usable reference", "",
                "Measured, not missing: `eval/gnss_ref.py` scanned each fix stream and "
                "recorded what it found. A receiver that keeps reporting good fixes "
                "underground still has no reference there, which is the fault this check "
                "exists to catch.", "",
                "| dataset | why the track is not a reference |", "|---|---|"]
        for d in noref:
            out.append("| {} | {} |".format(short(d), verdicts.get(d, "no report")))
        out += ["", "On these routes `end_pos_m` is the gap between where a run started "
                "and where it ended. That is drift only where the vehicle was driven back "
                "to its start; on an open route it is mostly the route.", ""]
    return out


def _has_reference(stats):
    return any((g["metrics"] or {}).get("gnss_ape_horiz_rmse_m") for g in stats["groups"])


def reference_verdicts(results_dir):
    """{dataset: why its GNSS track is not a reference}, for the ones that are not.

    Read from the report gnss_ref.py wrote rather than restated here, for the same reason
    every other number in this file comes from stats.json: one measurement, one place.
    """
    out = {}
    for path in sorted(Path(results_dir).glob("*/gnss_ref.json")):
        try:
            report = json.loads(path.read_text())
        except (OSError, ValueError) as e:
            out[path.parent.name] = "report unreadable ({})".format(e)
            continue
        if not report.get("usable"):
            out[path.parent.name] = "; ".join(
                report.get("unusable_because") or ["unusable, no reason recorded"])
    return out


def _detail(datasets):
    """Per-dataset tables, `median [min–max]`, matching stats.txt column for column."""
    out = ["## Per-dataset detail", "",
           "The same content as each `results/<dataset>/stats.txt`, collected. The range "
           "is the part a median cannot show.", ""]
    for dataset, stats in datasets:
        out += ["### {}".format(dataset), ""]
        for title, columns in (("Accuracy", DETAIL_ACCURACY),
                               ("Real-time", DETAIL_REALTIME)):
            out += ["**{}**".format(title), ""]
            out += _detail_table(stats, columns) + [""]
        if stats.get("disabled"):
            out += ["Disabled, left out of everything above: " + ", ".join(
                "`{}` ({})".format(d["system"], d["reason"])
                for d in stats["disabled"]), ""]
    return out


def _detail_table(stats, columns):
    heads = ["system", "n"] + [h for _, h, _ in columns]
    lines = ["| " + " | ".join(heads) + " |", "|---|" + "---|" * (len(heads) - 1)]
    for group in stats["groups"]:
        cells = [
            "`{}`".format(label_of(group)),
            "{}{}".format(group["n"], _excluded_note(group)),
        ]
        for key, _, places in columns:
            cells.append(spread((group["metrics"] or {}).get(key), places)
                         if group["n"] else "**fail**")
        lines.append("| " + " | ".join(cells) + " |")
    return lines


def _excluded_note(group):
    if not group["excluded"]:
        return ""
    # `failed` whichever way it failed, as aggregate's own tables report it; the reason
    # beside each run is where the two kinds are told apart.
    return " ({} failed)".format(len(group["excluded"]))


def _gaps(datasets):
    """What the plan asks for that this cannot answer, named rather than silently absent."""
    out = ["## What this summary cannot say", "",
           "`docs/comparison_plan.md` §10 specifies a master table wider than the "
           "measurements that exist. The columns with no pipeline behind them:", "",
           "| column | why it is absent |", "|---|---|"]
    for name, why in NOT_MEASURED:
        out.append("| {} | {} |".format(name, why))
    noref = [d for d, s in datasets if not _has_reference(s)]
    if noref:
        # Named, not restated: the reasons are above, measured, one per dataset.
        out += ["", "Accuracy on {} — see [Datasets with no usable reference]"
                "(#datasets-with-no-usable-reference).".format(
                    ", ".join("`{}`".format(short(d)) for d in noref))]
    return out + [""]


def short(dataset):
    """A dataset name narrow enough for a matrix header, without losing which one it is."""
    parts = dataset.split("_")
    if len(parts) < 2:
        return dataset
    site, stamp = parts[0], parts[1]
    return "{}·{}".format(site[:8], stamp[4:8] if len(stamp) >= 8 else stamp)


def spread(summary, places=1):
    """`median [min–max]`, collapsed to the bare value when the runs agree."""
    if not summary:
        return "n/m"
    if summary["min"] == summary["max"]:
        return num(summary["median"], places)
    return "{} [{}–{}]".format(num(summary["median"], places),
                               num(summary["min"], places),
                               num(summary["max"], places))


def num(value, places=1):
    return "{:.{p}f}".format(value, p=places).rstrip("0").rstrip(".")


def warn(*msg):
    print(*msg, file=sys.stderr)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("results_dir", nargs="?", default="/slam-bench/results")
    p.add_argument("--out", default=None,
                   help="output path (default <results_dir>/SUMMARY.md; - for stdout)")
    a = p.parse_args(argv)

    datasets = load(a.results_dir)
    if not datasets:
        warn("summary: no stats.json under {} — run aggregate first".format(a.results_dir))
        return 1

    text = render(datasets, a.results_dir, reference_verdicts(a.results_dir))
    if a.out == "-":
        print(text, end="")
        return 0
    out = Path(a.out or Path(a.results_dir) / "SUMMARY.md")
    out.write_text(text)
    warn("saved -> {}  ({} datasets)".format(out, len(datasets)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
