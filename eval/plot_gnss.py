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
        # unconstrained, rotates the whole trajectory and made every error on the
        # origin-aligned page enormous. Once there is real motion the fit is stable:
        # 20 m and 1000 m of opening travel put the same run within 3.5 m of each other.
        help="opening travel used for the origin-aligned view; 0 disables that page",
    )
    ap.add_argument(
        "trajectories", nargs="+", help="label=path/to/trajectory.tum, repeated"
    )
    args = ap.parse_args()

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
        # 1. plan view
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(ref_p[:, 0], ref_p[:, 1], "k--", lw=1.6, label="GNSS reference", zorder=1)
        ax.plot(*ref_p[0, :2], "ko", ms=9, mfc="none", mew=2, zorder=5)
        ax.annotate(" start", ref_p[0, :2], fontsize=9, va="center")
        for s in series:
            ax.plot(s["xyz"][:, 0], s["xyz"][:, 1], lw=1.5, alpha=0.9,
                    color=colours.get(s["label"]), label=s["label"])
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)")
        ax.legend(loc="best", fontsize=9)
        ax.set_title(
            "GNSS-referenced overlay (Umeyama aligned, scale not corrected)" + sub,
            fontsize=10,
        )
        fig.tight_layout(); pdf.savefig(fig); plt.close(fig)

        # 2. each axis against time, origin-aligned. A whole-run fit offsets each start
        # by 19-35 m vertically on this route — more than the 14 m the ground rises — so
        # the up panel would show the fit rather than the terrain.
        fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        key = "xyz0" if all(s["xyz0"] is not None for s in series) else "xyz"
        for i, (ax, name) in enumerate(zip(axs, AXES)):
            ax.plot(ref_t - t0, ref_p[:, i], "k--", lw=1.4, label="GNSS reference")
            for s in series:
                ax.plot(s["t"] - t0, s[key][:, i], lw=1.2, alpha=0.9,
                        color=colours.get(s["label"]), label=s["label"])
            ax.set_ylabel("{} (m)".format(name))
        axs[0].legend(loc="upper right", fontsize=8, ncol=2)
        axs[-1].set_xlabel("time (s)")
        axs[0].set_title(
            "Position against time \u2014 aligned on the opening {:g} m".format(
                args.align_metres) + sub, fontsize=10)
        fig.tight_layout(); pdf.savefig(fig); plt.close(fig)

        # 3. error against time — the page compare.pdf cannot draw: with no reference it
        # can show that systems disagree but never which is wrong.
        #
        # Origin-aligned. A whole-run fit drives the mean error to zero, which reads as
        # every trajectory being wrong at both ends and right in the middle — the
        # centring, not the run. This differs from `gnss_ape_*` in stats.txt, which is
        # whole-run aligned per the APE convention; the two rank runs differently, so each
        # title names the one it drew.
        pages = 2
        if args.align_metres and all(s["err0"] is not None for s in series):
            fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
            for i, (ax, name) in enumerate(zip(axs, AXES)):
                ax.axhline(0, color="k", lw=1.0, ls="--")
                f = floors.get(name)
                if f is not None:
                    ax.axhspan(-f, f, color="k", alpha=0.10, lw=0,
                               label="reference noise floor \u00b1{:.2f} m".format(f))
                for s in series:
                    ax.plot(s["t_err"] - t0, s["err0"][:, i], lw=1.2, alpha=0.9,
                            color=colours.get(s["label"]), label=s["label"])
                ax.set_ylabel("{} error (m)".format(name))
                ax.grid(alpha=0.3)
            axs[0].legend(loc="upper right", fontsize=8, ncol=2)
            axs[-1].set_xlabel("time (s)")
            axs[0].set_title(
                "Error against the reference, per axis \u2014 aligned on the opening "
                "{:g} m\ndrift from the start: the starts coincide and the ends fan out"
                .format(args.align_metres) + sub, fontsize=10)
            fig.tight_layout(); pdf.savefig(fig); plt.close(fig)
            pages = 3

    print("plot_gnss: {} trajectory(ies), {} pages -> {}".format(
        len(series), pages, args.out))


def _apply_evo_style(plt):
    """Match the styling evo_traj gives compare.pdf, so the two documents read as one set.

    Read off evo's own defaults, not guessed: seaborn `darkgrid`, the `deep6` palette,
    linewidth 1.5, sans-serif. The palette is the part that matters — a system drawn blue
    in one figure and orange in the other makes the pair unreadable side by side.

    One difference is not fixable here: compare.sh must hand evo a trajectory as `--ref`
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
    err0 = xyz0 = None
    if align_metres > 0:
        travelled = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(dst[:, :2], axis=0), axis=1))]
        )
        n = int(np.searchsorted(travelled, align_metres)) + 1
        if n < len(src):
            r0, t0 = umeyama(src[:n], dst[:n])
            err0 = (r0 @ src.T).T + t0 - dst
            xyz0 = (r0 @ ps.T).T + t0
    return {
        "label": label,
        "t": ts,
        "xyz": (r @ ps.T).T + t,
        "t_err": ts[keep],
        "err": (r @ src.T).T + t - dst,
        "err0": err0,
        "xyz0": xyz0,
    }


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
