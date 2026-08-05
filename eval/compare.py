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
  REF=<system>  whose frame to draw in (default point_lio, which gravity-aligns, so its
             xy plane is the horizontal one). Empty keeps the first run drawn.
  ALIGN=umeyama (default) | origin
             umeyama fits each curve onto the reference by position alone, which is the
             only alignment that works for a system publishing its attitude in another
             convention. origin instead pins every curve to the reference's opening pose,
             attitude included, and lets the disagreement accumulate from there — the
             more honest reading of drift, and 35-80% larger for it.
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
import collections
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


# A trajectory to draw. `diverged_at_s` is None for a run that stayed plausible, and
# otherwise the second at which divergence.py's rule fired — which is where the curve is
# truncated before evo ever sees it.
Pick = collections.namedtuple("Pick", "label path diverged_at_s")

# Whose frame the whole figure is drawn in. evo aligns every trajectory onto the first
# one's opening pose, so this choice sets the orientation of the entire plan view.
#
# It has to be a *gravity-aligned* system, which is the property that decides whether the
# figure is a plan view at all. Measured against the GNSS reference — the angle between a
# system's own z and true vertical, on the two datasets where every system behaves:
#
#     point_lio 1.9-3.3    bievr_lio 1.6-2.1    super_lio 2.0-2.4
#     fast_lio  14.1-16.0  faster_lio 13.2-24.2  pv_lio    14.9-16.3
#
# and the sign matters as much as the angle: fast_lio's vertical reads -0.961 against ENU
# up, so aligning on it draws the whole route upside down. The three on the top row are
# +0.998 or better.
#
# Reproducibility is the second axis, and a softer one. Between two runs of one bag the
# opening attitude moves by: fast_lio 0.37 deg, faster_lio 0.79, pv_lio 1.24, super_lio
# 2.12, point_lio 10.43, bievr_lio 25.99. For a gravity-aligned system almost all of that
# is yaw (point_lio: 10.20 of its 10.43), which rotates the figure without deforming it —
# unlike a tilt, which projects vertical motion into the plan view. So point_lio's wobble
# costs an angle and fast_lio's steadiness costs the shape.
#
# bievr_lio is what this used to pick, by nothing better than sorting the system names and
# taking the first: gravity-aligned, but the loosest of the six in yaw, and two runs of one
# ELDORADO bag put the whole overlay 22.7 deg apart.
#
# Only the opening pose is read, so a system that later blew up is still a valid frame.
DEFAULT_REF_SYSTEM = "point_lio"


# Position-only, by default. `origin` pins each curve to the reference's opening pose,
# attitude included — which is the more honest reading of drift, and which any system
# publishing its orientation in another convention breaks outright: one here is FRD where
# ROS REP-103 says FLU, so `origin` rotates its (best-fitting) trajectory by 180 deg about
# forward and draws 1598 m of error where there is 5 m. A benchmark has to be able to draw
# a system it did not write, and only the position fit works for every one of them.
#
# The cost, measured on OPENROAD_20260323 as RMS distance to the reference under
# origin -> umeyama: pv_lio 7.6 -> 1.5 m, fast_lio 16.6 -> 9.3, bievr_lio 42.7 -> 27.1,
# super_lio 203.7 -> 132.9. A least-squares fit spreads each run's drift across both ends,
# so every system reads 1.5-5x closer to the reference than it does from a common start —
# and the reference here is another system under test, not a truth. Read drift under
# ALIGN=origin, or against the GNSS track in compare_gnss.pdf, which has one.
ALIGN_MODES = ("umeyama", "origin")
DEFAULT_ALIGN = ALIGN_MODES[0]


def pick_runs(res, show_all, disabled):
    """[Pick] for the runs to draw, most-representative first.

    Ten curves are unreadable, so by default each (system, preset) contributes only its
    median run by end_pos_m — which needs the `derived` block aggregate.py writes back.
    A group without it falls back to all of its runs, with a warning, rather than failing.

    A diverged run is never the representative of a group that has a healthy one. Until
    now that was luck: bievr_lio on NORCAT_20251118 has end_pos_m 108.4 / 5625734.9 / 71.0
    and median_low happened to land on 108.4. Reorder those three and the group would have
    been represented by a trajectory 5600 km long.

    Divergence is read from `derived`, not recomputed: this script draws and aggregate.py
    measures, and the warning above already tells a reader who ran them out of order.
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
        derived = {run: _derived(run) for run in runs}
        at = {run: derived[run].get("diverged_at_s") for run in runs}

        # Only a group with nothing else to offer is represented by a diverged run. This
        # narrows the choice of representative, and nothing else: ALL=true means every run
        # that exists, and the failures are part of the spread it is asked to show.
        pool = [r for r in runs if at[r] is None] or runs
        chosen = runs if show_all else pool
        if not show_all and len(pool) > 1:
            scored = _scored(pool, derived)
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
            label = "{}-{}-{}".format(system, preset, run.name)
            if at[run] is not None:
                # FAILED, not DIVERGED, to match the word aggregate's tables use: a figure
                # and a table read side by side, and two words for one outcome make the
                # pair look like two outcomes. The second it failed still tells a reader
                # which kind this was — an incomplete run has no such moment.
                #
                # Hyphens, not brackets or spaces: evo labels a trajectory by its file
                # stem, so this label is also a filename and a word of argv.
                label += "-FAILED-{:.0f}s".format(at[run])
            picks.append(Pick(label, run / "trajectory.tum", at[run]))
    return picks


def reference_first(picks, system):
    """`picks` reordered so `system` leads it, which makes it evo's `--ref`.

    Falls back to the order it was given, with a warning, when that system is not among
    the trajectories being drawn — disabled, never run, or filtered out. Silently drawing
    in somebody else's frame would be the worse outcome: the figure would still look
    right, and only a rotation of the whole plan view would say otherwise.
    """
    if not system:
        return picks
    lead = [p for p in picks if p.label.startswith(system + "-")]
    if not lead:
        warn("compare: no {} among the drawn runs — aligning on {} instead".format(
            system, picks[0].label if picks else "nothing"))
        return picks
    if lead[0].diverged_at_s is not None:
        # Its opening pose is still what the alignment reads, so the frame is sound. But
        # evo draws the reference in black, and a black curve that stops early is worth a
        # word before someone reads it as the whole route.
        warn("compare: aligning on {}, which is drawn truncated — the reference curve "
             "ends where it blew up".format(lead[0].label))
    return lead[:1] + [p for p in picks if p is not lead[0]]


def _derived(run):
    """One run's `derived` block, or {} when it has none yet."""
    try:
        return json.loads((run / "metrics.json").read_text()).get("derived") or {}
    except (OSError, ValueError):
        return {}


def _scored(runs, derived):
    """[(end_pos_m, run)] or None if any run in the group lacks it."""
    scored = []
    for run in runs:
        try:
            scored.append((derived[run]["end_pos_m"], run))
        except KeyError:
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


def evo_argv(tums, mode, out, plot, align=DEFAULT_ALIGN):
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
    ref = []
    if len(tums) > 1:
        ref = ["--ref", str(tums[0])]
        tums = tums[1:]
    argv += [str(t) for t in tums] + ref
    if ref:
        # `-a` is a whole-trajectory Umeyama fit of each curve onto the reference. It
        # reads positions only, so it is immune to a system publishing its attitude in
        # another convention — but it also fits the shape, and on a figure with no ground
        # truth "fit each system onto another system" is least-squares agreement, which
        # hides exactly the disagreement the overlay exists to show. Never with `-s`:
        # LiDAR range and IMU acceleration are metric, so scale is observable and
        # correcting it would hide a real error.
        argv.append("--align_origin" if align == "origin" else "-a")
    argv += ["--plot_mode", mode, "--save_plot", str(out)]
    if plot:
        argv.append("--plot")
    return argv



def _align_mode(value=None):
    """ALIGN, checked. An unrecognised value is named, not quietly taken as the default.

    The two modes differ by 35-80% in how much disagreement they show, and neither writes
    its name on the figure — so `ALIGN=umeyema` falling through to the default would be a
    typo that changes the answer and leaves no trace of having done so.
    """
    if value is None:
        value = os.environ.get("ALIGN", DEFAULT_ALIGN)
    value = value.strip().lower() or DEFAULT_ALIGN
    if value not in ALIGN_MODES:
        warn("compare: ALIGN={} is not one of {} — using {}".format(
            value, "/".join(ALIGN_MODES), DEFAULT_ALIGN))
        return DEFAULT_ALIGN
    return value


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
    # REF= (empty) keeps whatever order pick_runs produced, i.e. the pre-REF behaviour.
    picks = reference_first(picks, os.environ.get("REF", DEFAULT_REF_SYSTEM).strip())
    if not picks:
        warn("no */*/run*/trajectory.tum to draw under {}".format(res))
        warn("  (a 'skipped' line above means the runs exist but their system is disabled)")
        return EXIT_NOTHING_TO_DRAW

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        # Staged as <label>.tum because evo labels a trajectory by its file stem, and the
        # raw files are all named trajectory.tum. A diverged run is staged truncated to
        # the moment its rule fired: evo autoscales, and one curve 4.7e6 m across turns
        # every other system into a dot at the origin. What is left is the part that was
        # still a trajectory — faster_lio's first 25 s on ELDORADO are real, and worth
        # seeing against the systems that kept going.
        staged_picks = []
        for pick in picks:
            staged = tmp / "{}.tum".format(pick.label)
            if pick.diverged_at_s is None:
                shutil.copy(pick.path, staged)
            else:
                _stage_truncated(pick.path, staged, pick.diverged_at_s)
            staged_picks.append(pick._replace(path=staged))
        tums = [p.path for p in staged_picks]

        print("overlaying {} trajectory(ies): {}".format(
            len(tums), " ".join(p.label for p in picks)))
        for pick in picks:
            if pick.diverged_at_s is not None:
                print("  {} diverged at {:.1f}s — drawn truncated there, and excluded "
                      "from the statistics by aggregate".format(
                          pick.label, pick.diverged_at_s))

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
        _start_end_markers(env, tmp)
        status = _run(
            evo_argv(tums, args.mode, out, plot, align=_align_mode()), env=env)

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

        return _gnss(res, args.dataset, staged_picks, tmp)


def _start_end_markers(env, tmp):
    """Ask evo to mark where each curve begins and ends.

    Without them a plan view is a bowl of spaghetti with no direction: every trajectory
    here starts at the same origin (`--align_origin` puts it there), so the only thing
    telling two curves apart is which way they went, and a truncated one simply stops in
    open ground where it looks like an unattached start rather than a run cut short.

    It is a setting rather than a flag — evo_traj reads SETTINGS.plot_start_end_markers —
    so it has to be written to a config file first. HOME is redirected into this run's
    temporary directory so that file is ours: writing to the invoking user's ~/.evo would
    change how every other evo command on the machine behaves, which is not something a
    plot script should do behind their back.

    Non-fatal. A figure without markers is the figure we had before.
    """
    home = tmp / "evo-home"
    home.mkdir(exist_ok=True)
    env["HOME"] = str(home)
    if _run(["evo_config", "set", "plot_start_end_markers", "true"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL):
        warn("compare: could not enable evo's start/end markers; drawing without them")


def _stage_truncated(src, dst, at_s):
    """Copy `src` up to `at_s` seconds after its first pose.

    Line-wise rather than through divergence.read_tum: the quaternion columns must survive
    into the staged file, and a run killed mid-write can end in a partial line that is
    simply dropped here.
    """
    t0 = None
    with open(src) as fh, open(dst, "w") as out:
        for line in fh:
            fields = line.split()
            if len(fields) < 4:
                continue
            try:
                t = float(fields[0])
            except ValueError:
                continue
            if t0 is None:
                t0 = t
            if t - t0 > at_s:
                break
            out.write(line)


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
    # The same labels and the same staged files the overlay used, so the two figures name
    # and draw the same curves — including the truncation, which matters more here:
    # plot_gnss aligns with a whole-trajectory Umeyama fit, and a curve that ran away
    # would drag the fit and with it every other curve's error.
    argv = [sys.executable, str(EVAL / "plot_gnss.py"),
            "--ref", str(ref), "--report", str(res / "gnss_ref.json"), "--out", str(out)]
    diverged = [p.label for p in picks if p.diverged_at_s is not None]
    if diverged:
        argv += ["--diverged", ",".join(diverged)]
    status = _run(argv + ["{}={}".format(p.label, p.path) for p in picks])
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
