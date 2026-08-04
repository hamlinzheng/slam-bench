#!/usr/bin/env python3
"""Overlay trajectories for one dataset with evo.

Statistics are NOT computed here — eval/aggregate.py owns every derived quantity and
writes results/<dataset>/stats.txt. This script only draws.

Drift is intentionally not reported — `aggregate` owns it, and a start-end gap is only a
drift measure on a closed loop anyway.

Produces results/<dataset>/compare.pdf, plus compare_gnss.pdf where the dataset has a
usable GNSS reference.

Usage (inside the container): python3 eval/compare.py <dataset> [plot_mode]
  plot_mode: xy (default, plan view) | xyz | xz | yz
             These are ground-vehicle routes on near-flat ground — one outdoor route
             rises 14 m over 7.3 km — so the plan view is where the trajectory is read,
             and a 3D view spends most of its third axis on vertical error.
  ALL=true   overlay every run instead of each group's median run
  PLOT=true  also open a live window (needs X11 reachable from the container)
  GNSS=true  build the GNSS reference here if `./bench.sh init` has not been run yet;
             needs the bags. Not required once it exists — the reference is cached
             beside the results and drawn from then on.

Was a shell script until it stopped being one in substance: run selection and the
reference verdict had both become Python delivered on stdin, the first of them importing
aggregate and registry through a sys.path insert that only existed because of how the
heredoc was invoked. Shell still suits the part that builds an argv and execs evo; it did
not suit the rest, and none of it could be tested.

evo, gnss_ref and plot_gnss stay subprocesses rather than imports. That boundary is what
keeps a failure in the GNSS half from touching the compare.pdf already written, and it
keeps matplotlib and rosbag out of this process.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import aggregate  # noqa: E402
import registry  # noqa: E402

REPO = Path("/slam-bench")
EVAL = REPO / "eval"

# Exit codes, unchanged from the shell version — bench.sh and whoever ran it both read
# them. 0 also covers "the main plot was written and the GNSS half had nothing to add",
# which is a success: a dataset without a usable reference is not a failed comparison.
EXIT_NO_DATASET = 1
EXIT_NOTHING_TO_DRAW = 2
EXIT_EVO_WROTE_NOTHING = 4


def pick_runs(res, show_all, disabled):
    """[(label, trajectory path)] for the runs to draw, most-representative first.

    Ten curves are unreadable, so by default each (system, preset) contributes only its
    median run by end_pos_m — which needs the `derived` block aggregate.py writes back.
    A group without it falls back to all of its runs, with a warning, rather than failing.
    """
    groups = {}
    for tum in sorted(Path(res).glob("*/*/run*/trajectory.tum")):
        run = tum.parent
        system = run.parent.parent.name
        if system in disabled:
            continue
        groups.setdefault((system, run.parent.name), []).append(run)

    picks = []
    for (system, preset), runs in sorted(groups.items()):
        chosen = runs
        if not show_all and len(runs) > 1:
            scored = _scored(runs)
            if scored:
                median = _median_low([v for v, _ in scored])
                chosen = [
                    next(r for v, r in sorted(scored, key=lambda s: s[0]) if v == median)
                ]
            else:
                warn(
                    "compare: {}/{} has runs without derived metrics — run aggregate "
                    "first for median selection; drawing all of them".format(
                        system, preset
                    )
                )
        for run in chosen:
            picks.append(
                ("{}-{}-{}".format(system, preset, run.name), run / "trajectory.tum")
            )
    return picks


def _scored(runs):
    """[(end_pos_m, run)] or None if any run in the group lacks it."""
    scored = []
    for run in runs:
        try:
            derived = json.loads((run / "metrics.json").read_text()).get("derived") or {}
            scored.append((derived["end_pos_m"], run))
        except (OSError, ValueError, KeyError):
            return None
    return scored


def _median_low(values):
    import statistics

    return statistics.median_low(values)


def reference_verdict(report_path):
    """Why this dataset's GNSS track is not a reference, or None when it is.

    Read from the report rather than from gnss_ref.py's exit code, so a reference built
    on an earlier run is checked too. The report exists even when the answer is no — a
    receiver that kept emitting fixes underground still deserves the record of why its
    track is not a reference.
    """
    try:
        report = json.loads(Path(report_path).read_text())
    except (OSError, ValueError) as e:
        return "no reference report ({})".format(e)
    if report.get("usable"):
        return None
    return "; ".join(report.get("unusable_because") or ["unusable"])


def evo_argv(tums, mode, out, plot):
    """The evo_traj command line for these trajectories.

    Each system defines its world frame differently — FAST-LIO and faster-lio use the
    first body frame, Point-LIO gravity-aligns it and gravity leaves yaw free. Measured
    169.5 deg apart, which drew 5.4 m of real disagreement as 321.6 m. `--align_origin`
    removes the convention without fitting the shape; on fast_lio it moves the RMSE by
    0.5%. evo needs a `--ref` to align to and draws it too, so the first trajectory
    becomes the reference and the rest stay positional — and a lone trajectory has
    nothing to align to, so it gets neither flag.
    """
    argv = ["evo_traj", "tum"]
    align = []
    if len(tums) > 1:
        align = ["--ref", str(tums[0])]
        tums = tums[1:]
    argv += [str(t) for t in tums] + align
    if align:
        argv.append("--align_origin")
    argv += ["--plot_mode", mode, "--save_plot", str(out)]
    if plot:
        argv.append("--plot")
    return argv


def warn(*msg):
    print(*msg, file=sys.stderr)


def _run(argv, **kw):
    return subprocess.run(argv, **kw).returncode


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dataset")
    ap.add_argument("mode", nargs="?", default="xy", choices=["xy", "xyz", "xz", "yz"])
    args = ap.parse_args(argv)

    res = REPO / "results" / args.dataset
    if not res.is_dir():
        warn("no results dir: {}".format(res))
        return EXIT_NO_DATASET

    disabled = registry.disabled_systems_or_warn("compare")
    for system, reason, runs in aggregate.disabled_inventory(res, disabled):
        warn(
            "compare: skipped {} (disabled: {}) — {} run(s) not drawn".format(
                system, reason, runs
            )
        )

    picks = pick_runs(res, os.environ.get("ALL", "false").lower() == "true", disabled)
    if not picks:
        warn("no */*/run*/trajectory.tum to draw under {}".format(res))
        warn("  (a 'skipped' line above means the runs exist but their system is disabled)")
        return EXIT_NOTHING_TO_DRAW

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        # Staged as <label>.tum because evo labels a trajectory by its file stem, and the
        # raw files are all named trajectory.tum.
        tums = []
        for label, path in picks:
            staged = tmp / "{}.tum".format(label)
            shutil.copy(path, staged)
            tums.append(staged)
        print("overlaying {} trajectory(ies): {}".format(
            len(tums), " ".join(label for label, _ in picks)))

        # Rendered to a scratch path and moved in only on success: evo prompts before
        # overwriting, which deadlocks a service with no stdin, so writing straight to the
        # destination meant deleting the old PDF first and losing it whenever a re-render
        # failed.
        #
        # Headless unless PLOT asks otherwise. Testing DISPLAY instead does not work:
        # docker-compose passes the host's through, so on a desktop host it is always set
        # and says nothing about whether this container can reach an X server — where
        # `--plot` then blocks on a window nobody can close and the PDF is never written.
        out = tmp / "compare.pdf"
        plot = os.environ.get("PLOT", "false").lower() == "true"
        env = dict(os.environ)
        if not plot:
            env["MPLBACKEND"] = "Agg"
        status = _run(evo_argv(tums, args.mode, out, plot), env=env)

        # Both conditions: evo can exit 0 having drawn nothing.
        if status != 0 or not (out.exists() and out.stat().st_size):
            warn("compare: evo wrote no plot (exit {}).".format(status))
            if (res / "compare.pdf").exists():
                warn("  {} is untouched, still the previous one.".format(
                    res / "compare.pdf"))
            return EXIT_EVO_WROTE_NOTHING
        # shutil.move, not os.replace: /tmp is a tmpfs and the results tree is a bind
        # mount, so the rename is cross-device. This is what `mv -f` did before, and the
        # property that matters survives either way — a failed render never destroys the
        # PDF already there, because nothing is moved until one exists.
        shutil.move(str(out), str(res / "compare.pdf"))
        print("saved -> {}  |  statistics: eval/aggregate.py results/{}".format(
            res / "compare.pdf", args.dataset))

        return _gnss(res, args.dataset, picks, tmp)


def _gnss(res, dataset, picks, tmp):
    """The GNSS-referenced overlay. Non-fatal throughout: the main plot is already
    written, and nothing here may cost it.

    GNSS=true is only needed the first time. Building the reference scans every bag and
    needs BAGS_DIR pointed at this dataset, neither of which should be a silent
    precondition of `compare`; once built it is cached beside the results and costs
    nothing, so from then on it is drawn by default.
    """
    ref = res / "gnss_ref.tum"
    if not (ref.exists() and ref.stat().st_size):
        if os.environ.get("GNSS", "false").lower() != "true":
            return 0
        _run([sys.executable, str(EVAL / "gnss_ref.py"), "/bags", "--out", str(res)])

    verdict = reference_verdict(res / "gnss_ref.json")
    if verdict:
        warn("compare: GNSS track is not a reference for {} — {}".format(dataset, verdict))
        warn("  compare.pdf is unaffected; see {}".format(res / "gnss_ref.json"))
        return 0

    out = tmp / "compare_gnss.pdf"
    # The same label=path pairs the overlay used, so the two figures name the same curves.
    status = _run(
        [sys.executable, str(EVAL / "plot_gnss.py"),
         "--ref", str(ref), "--report", str(res / "gnss_ref.json"), "--out", str(out)]
        + ["{}={}".format(label, path) for label, path in picks]
    )
    if status == 0 and out.exists() and out.stat().st_size:
        shutil.move(str(out), str(res / "compare_gnss.pdf"))
        print("saved -> {}  |  reference quality: {}".format(
            res / "compare_gnss.pdf", res / "gnss_ref.json"))
    else:
        warn("compare: GNSS overlay not written; {} is unaffected.".format(
            res / "compare.pdf"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
