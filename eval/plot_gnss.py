#!/usr/bin/env python3
"""One-page ground-referenced XY overlay: every trajectory against the GNSS track.

Separate from compare.pdf on purpose, and not simply `evo_traj --ref gnss_ref.tum`,
for two reasons:

  * **The alignment conventions are opposite and evo takes one per invocation.**
    compare.pdf uses `--align_origin`: it removes each system's frame convention
    (Point-LIO gravity-aligns and leaves yaw free — measured 169.5 deg from FAST-LIO)
    without fitting the shape, which is what keeps relative drift visible. A GNSS
    reference lives in ENU and differs from every system frame by an unknown yaw that
    only a Umeyama fit recovers. `-a` and `--align_origin` are mutually exclusive.
  * **evo_traj draws the reference on every page, including RPY.** The GNSS quaternion
    is a dummy, so it would contribute three flat zero lines to the one page carrying
    attitude information, next to six real curves.

So: this draws position only, where GNSS has something to say, and leaves attitude to
compare.pdf, where it needs no reference at all.

Scale is never corrected. LiDAR measures metric range and IMU metric acceleration, so a
LIO trajectory's scale is observable; fitting it would hide a real error.
"""
import argparse
import json
from pathlib import Path

# Alignment and pairing live in gnss_ape so the figure and the numbers in stats.txt are
# produced by the same code — a plot that aligned differently from the metric would be
# the more misleading of the two.
import numpy as np

from gnss_ape import associate, load_tum, umeyama


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ref", required=True, help="gnss_ref.tum")
    ap.add_argument("--out", required=True)
    ap.add_argument("--report", help="gnss_ref.json, for the noise-floor annotation")
    ap.add_argument("--t-max-diff", type=float, default=0.05)
    ap.add_argument(
        "--align-metres", type=float, default=100.0,
        # By distance, not by pose count. On one outdoor route the opening 200 poses
        # cover 1.75 m — the vehicle stands still for the first 20 s — which leaves yaw
        # unconstrained and rotates the whole trajectory.
        #
        # Only the yaw is fitted here (see _opening_correction), and yaw needs far less
        # than a full rotation does: measured across the three referenced datasets, the
        # vertical error moves by under 2x between a 20 m and a 1000 m window, where
        # fitting the full rotation swung it 27x over the same range.
        help="opening travel the yaw alignment is fitted on; 0 disables those pages",
    )
    ap.add_argument(
        "--diverged", default="",
        # compare.py hands these in already truncated to the moment they blew up, so what
        # is drawn is a real trajectory — but it ends early and for a reason, and a curve
        # that simply stops would otherwise read as a short run.
        help="comma-separated labels to draw as diverged: grey, dashed, behind the rest",
    )
    ap.add_argument(
        "trajectories", nargs="+", help="label=path/to/trajectory.tum, repeated"
    )
    args = ap.parse_args()
    diverged = {lab for lab in args.diverged.split(",") if lab}

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    _apply_evo_style(plt)

    ref_t, ref_p = load_tum(args.ref)
    rep = {}
    if args.report and Path(args.report).exists():
        rep = json.loads(Path(args.report).read_text())

    # Aligned once, and every page draws from that one fit — a figure that aligned
    # differently from stats.txt would be the more misleading of the two.
    series = [s for s in (_prepare(spec, ref_t, ref_p, args.t_max_diff, args.align_metres)
                          for spec in args.trajectories) if s]
    if not series:
        raise SystemExit("plot_gnss: nothing overlapped the reference; no plot written")

    colours = _colours([s["label"] for s in series], plt)
    floors = _floors(rep)
    sub = _subtitle(rep)
    AXES = ("east", "north", "up")
    t0 = ref_t[0]

    with PdfPages(args.out) as pdf:
        pages = 0

        # 1. plan view, whole-run aligned. This is the APE convention, so it is the page
        # that corresponds to gnss_ape_* in stats.txt.
        _plan_page(pdf, plt, series, ref_p, "xyz", diverged, colours,
                   "GNSS-referenced overlay (whole-run Umeyama aligned, scale not "
                   "corrected)\nthe alignment behind gnss_ape_* in stats.txt" + sub)
        pages += 1

        # 2. the same plan view aligned on the opening travel instead, where every
        # trajectory leaves the reference's start rather than its own best-fit offset.
        # Measured on OPENROAD_20260325, the whole-run fit puts the six starts up to 300 m
        # from the reference's; aligned on the opening 100 m they all sit within 0.8 m of
        # it. That is the difference between reading "where did each run end up" and
        # reading "how did each run drift away from a common origin", and both are worth a
        # page — which is why this is an extra one rather than a replacement.
        #
        # Per series, not all-or-nothing. A run truncated where it blew up can be too
        # short to contain the opening travel at all — faster_lio survives 32.6 s on
        # OPENROAD_20260803 and the vehicle has not covered 100 m by then — and under an
        # `all()` test that one run withheld every opening-aligned page from the other
        # five. Whoever is missing is named on the page instead.
        opening = [s for s in series if s["xyz0"] is not None] if args.align_metres else []
        missing = _omitted(series, opening)
        if opening:
            _plan_page(pdf, plt, opening, ref_p, "xyz0", diverged, colours,
                       "GNSS-referenced overlay — yaw-aligned on the opening {:g} m\n"
                       "the starts coincide and the drift fans out from them".format(
                           args.align_metres) + missing + sub)
            pages += 1

        # 3. each axis against time, opening-aligned. A whole-run fit offsets each start
        # by 19-35 m vertically on this route — more than the 14 m the ground rises — so
        # the up panel would show the fit rather than the terrain.
        #
        # Drawn from the same `opening` set as the page above. It used to fall back to the
        # whole-run fit whenever any series lacked the opening one, while keeping the title
        # that named the opening alignment \u2014 so the figure misdescribed itself, which is
        # the one failure this module's own docstring is most insistent about.
        if opening:
            fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
            for i, (ax, name) in enumerate(zip(axs, AXES)):
                ax.plot(ref_t - t0, ref_p[:, i], "k--", lw=1.4, label="GNSS reference")
                for s in opening:
                    ax.plot(s["t"] - t0, s["xyz0"][:, i], label=s["label"],
                            **_style(s["label"], diverged, colours, lw=1.2))
                ax.set_ylabel("{} (m)".format(name))
            axs[0].legend(loc="upper right", fontsize=8, ncol=2)
            axs[-1].set_xlabel("time (s)")
            axs[0].set_title(
                "Position against time \u2014 yaw-aligned on the opening {:g} m".format(
                    args.align_metres) + missing + sub, fontsize=10)
            fig.tight_layout(); pdf.savefig(fig); plt.close(fig)
            pages += 1

        # 4. error against time — the page compare.pdf cannot draw: with no reference it
        # can show that systems disagree but never which is wrong.
        #
        # Origin-aligned. A whole-run fit drives the mean error to zero, which reads as
        # every trajectory being wrong at both ends and right in the middle — the
        # centring, not the run. This differs from `gnss_ape_*` in stats.txt, which is
        # whole-run aligned per the APE convention; the two rank runs differently, so each
        # title names the one it drew.
        if opening:
            fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
            for i, (ax, name) in enumerate(zip(axs, AXES)):
                ax.axhline(0, color="k", lw=1.0, ls="--")
                f = floors.get(name)
                if f is not None:
                    ax.axhspan(-f, f, color="k", alpha=0.10, lw=0,
                               label="reference noise floor \u00b1{:.2f} m".format(f))
                for s in opening:
                    ax.plot(s["t_err"] - t0, s["err0"][:, i], label=s["label"],
                            **_style(s["label"], diverged, colours, lw=1.2))
                ax.set_ylabel("{} error (m)".format(name))
                ax.grid(alpha=0.3)
            axs[0].legend(loc="upper right", fontsize=8, ncol=2)
            axs[-1].set_xlabel("time (s)")
            axs[0].set_title(
                "Error against the reference, per axis \u2014 yaw-aligned on the opening "
                "{:g} m\ndrift from the start: the starts coincide and the ends fan out"
                .format(args.align_metres) + missing + sub, fontsize=10)
            fig.tight_layout(); pdf.savefig(fig); plt.close(fig)
            pages += 1

    print("plot_gnss: {} trajectory(ies), {} pages -> {}".format(
        len(series), pages, args.out))


def _apply_evo_style(plt):
    """Match the styling evo_traj gives compare.pdf, so the two documents read as one set.

    Read off evo's own defaults, not guessed: seaborn `darkgrid`, the `deep6` palette,
    linewidth 1.5, sans-serif. The palette is the part that matters — a system drawn blue
    in one figure and orange in the other makes the pair unreadable side by side.

    One difference is not fixable here: compare.py must hand evo a trajectory as `--ref`
    for `--align_origin` and evo draws references in black, so compare.pdf's first system
    is black while every system is coloured here, and the palette runs one position apart.
    """
    try:
        import seaborn as sns

        sns.set_theme(style="darkgrid", palette="deep6", font="sans-serif",
                      font_scale=1.0)
    except ImportError:
        # evo pulls seaborn in, so this is the bare-repo path rather than a real one.
        plt.rcParams["axes.grid"] = True
    plt.rcParams["lines.linewidth"] = 1.5


def _omitted(series, opening):
    """A line for the title naming who the opening-aligned pages had to leave out.

    On the page rather than only on stderr: these figures are read long after the command
    that made them, and a curve silently absent from one page and present on another is
    the kind of difference a reader attributes to the system rather than to the plot.
    """
    left_out = [s["label"] for s in series if s not in opening]
    if not left_out:
        return ""
    print("plot_gnss: too short for the opening-travel fit, left off the "
          "opening-aligned pages: {}".format(", ".join(left_out)))
    return "\nleft out, too short for the opening fit: {}".format(", ".join(left_out))


def _plan_page(pdf, plt, series, ref_p, key, diverged, colours, title):
    """One east-north overlay of every trajectory under the alignment named by `key`.

    Two pages differ only in that key, so they share this: a reader comparing them is
    comparing alignments, and any other difference between the figures would be noise in
    that comparison.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(ref_p[:, 0], ref_p[:, 1], "k--", lw=1.6, label="GNSS reference", zorder=1)
    ax.plot(*ref_p[0, :2], "ko", ms=9, mfc="none", mew=2, zorder=5)
    ax.annotate(" start", ref_p[0, :2], fontsize=9, va="center")
    # The reference ends too, and on an open route that end is a different place from its
    # start — without the mark, which end of the dashed track is which is a guess.
    ax.plot(*ref_p[-1, :2], "kx", ms=9, mew=2, zorder=5)
    for s in series:
        style = _style(s["label"], diverged, colours)
        ax.plot(s[key][:, 0], s[key][:, 1], label=s["label"], **style)
        _mark_ends(ax, s[key], style)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.legend(loc="best", fontsize=9)
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)


def _mark_ends(ax, xyz, style):
    """A circle where the curve starts and a cross where it stops, as evo marks them.

    Both ends, and for every series, because on this page neither is guessable. The
    trajectories are Umeyama-aligned over their whole length rather than pinned at the
    origin, so unlike compare.pdf the starts do not coincide — where the fit put each one
    is itself worth seeing. And a truncated run stops in open ground, which without a mark
    reads as a curve that begins there.

    No label: these carry the colour of the curve they belong to, and six more legend
    entries would cost more than they explain.
    """
    marker = {"color": style["color"], "zorder": style["zorder"] + 1,
              "alpha": style["alpha"], "ls": "none"}
    ax.plot(*xyz[0, :2], marker="o", ms=5, **marker)
    ax.plot(*xyz[-1, :2], marker="x", ms=6, mew=1.6, **marker)


def _style(label, diverged, colours, lw=1.5):
    """Line kwargs for one series — grey and dashed once the run is known to have blown up.

    Grey rather than a palette colour on purpose: the colours carry identity across
    compare.pdf and this document, and a diverged run should not spend one. Drawn behind
    everything else (zorder) so it never hides a run that survived.
    """
    if label in diverged:
        return {"lw": 1.2, "alpha": 0.7, "color": "0.55", "ls": "--", "zorder": 2}
    return {"lw": lw, "alpha": 0.9, "color": colours.get(label), "zorder": 3}


def _colours(labels, plt):
    """Colour per label, fixed by sorted order so a system keeps its colour across pages
    and across re-runs — the point of matching the palette at all."""
    cycle = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    order = sorted(labels)
    return {lab: cycle[i % len(cycle)] for i, lab in enumerate(order)} if cycle else {}


def _prepare(spec, ref_t, ref_p, t_max_diff, align_metres=0.0):
    """One trajectory under both alignment conventions, with its per-pair error."""
    label, _, path = spec.partition("=")
    if not path:
        label, path = Path(spec).stem, spec
    ts, ps = load_tum(path)
    src, dst = associate(ts, ps, ref_t, ref_p, t_max_diff)
    if len(src) < 3:
        print("plot_gnss: {} has no overlap with the reference \u2014 skipped".format(label))
        return None
    r, t = umeyama(src, dst)
    idx = np.clip(np.searchsorted(ref_t, ts), 1, len(ref_t) - 1)
    pick = np.where(np.abs(ref_t[idx] - ts) < np.abs(ref_t[idx - 1] - ts), idx, idx - 1)
    keep = np.abs(ref_t[pick] - ts) <= t_max_diff

    # Everything below starts from the whole-run fit, which is well conditioned: it has
    # kilometres of trajectory to determine the rotation with. The opening alignment then
    # only *turns* that result, and never tips it — see _opening_correction.
    aligned_src = (r @ src.T).T + t
    aligned_ps = (r @ ps.T).T + t

    err0 = xyz0 = None
    if align_metres > 0:
        travelled = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(dst[:, :2], axis=0), axis=1))]
        )
        n = int(np.searchsorted(travelled, align_metres)) + 1
        if n < len(src):
            r0, t0 = _opening_correction(aligned_src, dst, n)
            err0 = (r0 @ aligned_src.T).T + t0 - dst
            xyz0 = (r0 @ aligned_ps.T).T + t0
    return {
        "label": label,
        "t": ts,
        "xyz": aligned_ps,
        "t_err": ts[keep],
        "err": aligned_src - dst,
        "err0": err0,
        "xyz0": xyz0,
    }


def _opening_correction(aligned, dst, n):
    """Yaw and translation that put the opening `n` poses onto the reference's.

    Yaw only, deliberately. Fitting a full rotation to the opening instead — which is what
    this did — leaves pitch and roll almost unconstrained, because an opening segment is
    short and nearly planar, and the vertical error that follows is enormous. Measured on
    the three referenced datasets here: 100 m of *path* at the start is 43-99 m of net
    displacement, the fitted rotation comes out 11-75 deg tilted, and levered along a
    3-7 km route that drew 59-1896 m of vertical RMSE where the whole-run fit reads 14-113.
    Six independent systems agreed on that error to within 5% while their real vertical
    errors differed threefold, which is how it was caught: the number was the alignment's,
    not theirs.

    Vertical is not fitted at all, only offset. Up is defined by gravity, both the ENU
    reference and the whole-run fit already agree on it, and a few tens of metres of
    opening travel has nothing to say about it that is worth hearing.
    """
    a, b = aligned[:n, :2], dst[:n, :2]
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    h = (a - ca).T @ (b - cb)
    u, _, vt = np.linalg.svd(h)
    # The reflection guard Kabsch needs: without it a poorly-spread opening can "fit" by
    # mirroring the route, which is not a pose.
    flip = np.sign(np.linalg.det(vt.T @ u.T))
    r2 = vt.T @ np.diag([1.0, flip]) @ u.T

    rot = np.eye(3)
    rot[:2, :2] = r2
    shift = np.zeros(3)
    shift[:2] = cb - r2 @ ca
    shift[2] = dst[:n, 2].mean() - aligned[:n, 2].mean()
    return rot, shift


def _floors(rep):
    """Per-axis noise floor at 30 s, keyed by the plot's axis names."""
    w30 = rep.get("noise_floor", {}).get("w30s", {})
    return {
        k: (w30[v]["rmse_m"] if v in w30 else None)
        for k, v in {"east": "east", "north": "north", "up": "vertical"}.items()
    }


def _subtitle(rep):
    bits = []
    w30 = rep.get("noise_floor", {}).get("w30s", {})
    floors = ["{} {:.2f}".format(a[0].upper(), w30[a]["rmse_m"])
              for a in ("east", "north", "vertical") if a in w30]
    if floors:
        # Repeated on every page because a separation smaller than the floor is not a
        # difference between systems, it is the reference's own noise. Measured while the
        # vehicle was stopped — a residual against the moving track would be the driving.
        bits.append("reference noise floor @30s: {} m".format(", ".join(floors)))
    gaps = rep.get("gaps", {})
    if gaps.get("gaps"):
        bits.append("{} gap(s) > 2s, {:.0f}s total \u2014 no reference there".format(
            len(gaps["gaps"]), gaps["gap_total_s"]))
    return ("\n" + "   |   ".join(bits)) if bits else ""


if __name__ == "__main__":
    main()
