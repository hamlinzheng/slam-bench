#!/usr/bin/env python3
"""GNSS reference track for a dataset, plus an honest statement of what it is worth.

Writes two files:
  gnss_ref.tum   — the reference trajectory `evo_ape` / `evo_rpe` consume
  gnss_ref.json  — the measured quality of that reference: fix mix, gap structure,
                   and a noise floor per axis, so a metric computed against it can be
                   read against the floor rather than taken at face value

Reads `sensor_msgs/NavSatFix`, not NMEA. That is the stable contract: today
`nmea_navsat_driver` publishes it, tomorrow `ublox_gps` publishes the same type on the
same topic, and this file does not change. NavPVT would add real per-epoch accuracy
(hAcc/vAcc) and Doppler velocity, but deserialising it needs `ublox_msgs` in the image,
so it stays out of the required path.

**The quaternion is identity and carries no information.** A single-antenna GNSS track
has no attitude: heading is recoverable from motion (measured 1.5 deg median above
5 m/s, useless below 2 m/s) but roll and pitch are not obtainable at all. Filling the
quaternion with a motion-derived yaw would make the file look like a pose reference and
`evo_ape -r full` would then return a plausible, meaningless number. So:

    evo_ape tum gnss_ref.tum <traj>.tum -a -r trans_part     # correct
    evo_ape tum gnss_ref.tum <traj>.tum -a -r full           # WRONG, reads the dummy

Run inside the container (needs rosbag); the analysis half is import-free of ROS so its
tests run on the host with plain pytest, the same split aggregate.py uses.
"""
import argparse
import json
import math
import statistics
from pathlib import Path

# WGS84.
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2 - _F)

# sensor_msgs/NavSatStatus. -1 is NO_FIX; everything >= 0 carries a position.
STATUS_NO_FIX = -1
STATUS_NAMES = {-1: "no_fix", 0: "fix", 1: "sbas", 2: "gbas"}

# sensor_msgs/NavSatFix position_covariance_type.
COVARIANCE_APPROXIMATED = 1

# Measured while the vehicle is stopped, where all position variation is the receiver.
# A residual against a rolling median of the moving track instead measures the driving
# on any axis the vehicle actually covers ground along — measured 8.2 m east on a route
# where the truth was 0.28 m.
STATIONARY_SPEED_MPS = 0.3
STATIONARY_MIN_SPAN_S = 10.0

# Timescales the stationary residual is summarised at. A floor is only meaningful next
# to one: the same receiver answers "how noisy is it" differently at 10 s and at 120 s,
# and a metric compared against the wrong answer is misread.
NOISE_WINDOWS_S = (10.0, 30.0, 120.0)

# A gap longer than this is a hole in the reference rather than a dropped sample. Below
# it, interpolation is defensible; above it there is simply no reference for that stretch.
GAP_THRESHOLD_S = 2.0

# Above this fraction of the run without a fix, the track is not a reference for it.
# The receiver keeps publishing underground: one mine bag holds a single 871 s hole over
# 74% of the run and yields a 780 m "reference" for a 2849 m route, still flagged `sbas`. So
# neither presence nor the quality flag decides this. Not a delicate threshold — the two
# datasets measured sit at 0.7% and 74.2%.
MAX_GAP_FRACTION = 0.10
MIN_FIXES = 100


def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    """WGS84 geodetic -> ECEF. `alt` is height above the ellipsoid, which is what
    NavSatFix.altitude carries — note NMEA GGA field 9 is height above the *geoid*
    instead, the two differing by the geoid separation (-35.9 m on this rig). Mixing
    them silently offsets every altitude by that much."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    s = math.sin(lat)
    n = _A / math.sqrt(1 - _E2 * s * s)
    return (
        (n + alt_m) * math.cos(lat) * math.cos(lon),
        (n + alt_m) * math.cos(lat) * math.sin(lon),
        (n * (1 - _E2) + alt_m) * s,
    )


def to_enu(fixes):
    """[(t, lat, lon, alt), ...] -> [(t, e, n, u), ...] about the first fix.

    Proper ECEF -> ENU rather than scaling degrees by a single earth radius: at 43 deg
    latitude the meridional and normal radii differ by ~0.17%, which is 12 m of scale
    error over a 7 km route — the size of the effects being measured.
    """
    if not fixes:
        return []
    t0, lat0, lon0, alt0 = fixes[0]
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, alt0)
    lat, lon = math.radians(lat0), math.radians(lon0)
    sla, cla, slo, clo = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    out = []
    for t, la, lo, al in fixes:
        x, y, z = geodetic_to_ecef(la, lo, al)
        dx, dy, dz = x - x0, y - y0, z - z0
        out.append(
            (
                t,
                -slo * dx + clo * dy,
                -sla * clo * dx - sla * slo * dy + cla * dz,
                cla * clo * dx + cla * slo * dy + sla * dz,
            )
        )
    return out


def gap_stats(times, threshold_s=GAP_THRESHOLD_S):
    """Where the reference is missing, not merely how often a sample was dropped.

    Reported as the gap distribution rather than a fix rate: 95% coverage whose missing
    5% is one continuous hole leaves that stretch of road with no reference at all, and
    a rate cannot tell that from 5% scattered evenly.
    """
    if len(times) < 2:
        return {"n": len(times), "span_s": 0.0, "median_dt_s": None, "gaps": []}
    deltas = [b - a for a, b in zip(times, times[1:]) if b > a]
    gaps = sorted((d for d in deltas if d > threshold_s), reverse=True)
    span = times[-1] - times[0]
    return {
        "n": len(times),
        "span_s": span,
        "median_dt_s": statistics.median(deltas) if deltas else None,
        "max_dt_s": max(deltas) if deltas else None,
        "gaps": gaps[:10],
        "gap_total_s": sum(gaps),
        "gap_frac": (sum(gaps) / span) if span > 0 else None,
    }


def stationary_spans(enu, speed_mps=STATIONARY_SPEED_MPS, min_span_s=STATIONARY_MIN_SPAN_S):
    """Index ranges over which the vehicle was not moving.

    Speed is taken horizontally: a stopped vehicle's altitude still wanders with the
    receiver, which is the point — that wander is the measurement.
    """
    spans, start = [], None
    for i in range(len(enu) - 1):
        dt = enu[i + 1][0] - enu[i][0]
        if dt <= 0:
            continue
        v = math.dist(enu[i][1:3], enu[i + 1][1:3]) / dt
        if v <= speed_mps:
            start = i if start is None else start
        elif start is not None:
            if enu[i][0] - enu[start][0] >= min_span_s:
                spans.append((start, i))
            start = None
    if start is not None and enu[-1][0] - enu[start][0] >= min_span_s:
        spans.append((start, len(enu) - 1))
    return spans


def stationary_noise(enu, windows=NOISE_WINDOWS_S):
    """Per-axis noise floor from the stopped stretches, pooled.

    Within a stationary span the receiver's own variation is the whole signal, so the
    residual against the span's median needs no assumption about the route. Reported per
    window because the floor grows with the timescale: a span is only counted at windows
    it is long enough to cover, so the 120 s figure is absent unless the vehicle actually
    stood still that long.
    """
    spans = stationary_spans(enu)
    out = {"n_spans": len(spans), "total_s": 0.0}
    if not spans:
        out["reason"] = "no stationary span of at least {:g}s".format(STATIONARY_MIN_SPAN_S)
        return out
    out["total_s"] = sum(enu[b][0] - enu[a][0] for a, b in spans)
    axes = {"east": 1, "north": 2, "vertical": 3}
    for w in windows:
        pooled = {k: [] for k in axes}
        for a, b in spans:
            if enu[b][0] - enu[a][0] < w:
                continue
            for name, col in axes.items():
                vals = [enu[i][col] for i in range(a, b + 1)]
                med = statistics.median(vals)
                pooled[name].extend(v - med for v in vals)
        if not any(pooled.values()):
            continue
        out["w{:g}s".format(w)] = {
            name: {
                "rmse_m": math.sqrt(sum(r * r for r in res) / len(res)),
                "p99_m": sorted(abs(r) for r in res)[max(0, int(len(res) * 0.99) - 1)],
                "n": len(res),
            }
            for name, res in pooled.items()
            if res
        }
    return out


def assess(enu, statuses, cov_types):
    """The reference's own quality report — every field measured, none declared."""
    times = [p[0] for p in enu]
    vert = [p[3] for p in enu]
    path = sum(
        math.dist(a[1:3], b[1:3]) for a, b in zip(enu, enu[1:])
    )
    gaps = gap_stats(times)
    counts = {}
    for s in statuses:
        counts[STATUS_NAMES.get(s, str(s))] = counts.get(STATUS_NAMES.get(s, str(s)), 0) + 1
    return {
        "n_fixes": len(enu),
        "status_counts": counts,
        "gaps": gaps,
        # Along-track, so it is comparable with a trajectory's path_len_m.
        "path_len_m": path,
        "closure_horiz_m": math.dist(enu[0][1:3], enu[-1][1:3]) if enu else None,
        "closure_vert_m": (enu[-1][3] - enu[0][3]) if enu else None,
        "extent_m": {
            "e": max(p[1] for p in enu) - min(p[1] for p in enu),
            "n": max(p[2] for p in enu) - min(p[2] for p in enu),
            "u": max(vert) - min(vert),
        },
        # Measured while stopped — see stationary_noise.
        "noise_floor": stationary_noise(enu),
        "covariance": {
            "types": {str(k): cov_types.count(k) for k in sorted(set(cov_types))},
            # Measured on this rig: an NMEA-derived fix advertised 0.085 m horizontal and
            # 0.34 m vertical sigma where the truth was metres. APPROXIMATED means the
            # driver back-computed it from HDOP, which is geometry, not accuracy.
            "trustworthy": not any(c == COVARIANCE_APPROXIMATED for c in cov_types),
        },
        "attitude": "none — quaternion is identity; -r full/rot_part/angle_deg are invalid",
        **_verdict(enu, gaps),
    }


def _verdict(enu, gaps):
    """Whether this track may be used as a reference at all, and why not if not.

    Decided from the measurement rather than declared per dataset: a hand-written
    `gnss_reliable: true` cannot see that a receiver kept emitting SBAS-quality fixes
    after the vehicle drove into a mine.
    """
    reasons = []
    if len(enu) < MIN_FIXES:
        reasons.append("only {} fixes (need {})".format(len(enu), MIN_FIXES))
    frac = gaps.get("gap_frac")
    if frac is not None and frac > MAX_GAP_FRACTION:
        reasons.append(
            "{:.0%} of the run has no fix (largest hole {:.0f}s); the track covers only "
            "part of it".format(frac, gaps.get("max_dt_s") or 0)
        )
    return {"usable": not reasons, "unusable_because": reasons}


def write_tum(path, enu):
    """TUM with an identity quaternion. See the module docstring for why it stays dummy."""
    with open(path, "w") as fh:
        for t, e, n, u in enu:
            fh.write("%.9f %.6f %.6f %.6f 0 0 0 1\n" % (t, e, n, u))


def read_navsatfix(bag_paths, topic):
    """[(t, lat, lon, alt)], [status], [covariance_type] — ROS-only, imported lazily so
    the analysis above stays testable off-container."""
    import rosbag  # noqa: PLC0415 — deliberate: keeps the module importable on the host

    fixes, statuses, cov_types = [], [], []
    for p in bag_paths:
        with rosbag.Bag(str(p)) as bag:
            for _, msg, _ in bag.read_messages(topics=[topic]):
                if msg.status.status == STATUS_NO_FIX:
                    statuses.append(msg.status.status)
                    continue
                if math.isnan(msg.latitude) or math.isnan(msg.longitude):
                    continue
                fixes.append(
                    (
                        msg.header.stamp.to_sec(),
                        msg.latitude,
                        msg.longitude,
                        msg.altitude,
                    )
                )
                statuses.append(msg.status.status)
                cov_types.append(msg.position_covariance_type)
    fixes.sort()
    return fixes, statuses, cov_types


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags", help="a .bag, or a directory of them played as one stream")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--topic", default="/sensing/gnss/fix")
    args = ap.parse_args()

    src = Path(args.bags)
    paths = sorted(src.glob("*.bag")) if src.is_dir() else [src]
    if not paths:
        raise SystemExit("no *.bag under {}".format(src))

    fixes, statuses, cov_types = read_navsatfix(paths, args.topic)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    if len(fixes) < 2:
        # Not an error: a dataset with no GNSS is a perfectly healthy dataset. The report
        # is still written, because "this dataset has no reference, and here is why" is
        # the answer the downstream steps need — the alternative is each of them
        # rediscovering it from an absent file.
        report = {
            "n_fixes": len(fixes),
            "usable": False,
            "unusable_because": [
                "{} fix(es) on {}".format(len(fixes), args.topic)
            ],
        }
        (out / "gnss_ref.json").write_text(json.dumps(report, indent=2, sort_keys=True))
        print("no GNSS reference: {} fix(es) on {}".format(len(fixes), args.topic))
        return 3

    enu = to_enu(fixes)
    write_tum(out / "gnss_ref.tum", enu)
    report = assess(enu, statuses, cov_types)
    (out / "gnss_ref.json").write_text(json.dumps(report, indent=2, sort_keys=True))

    print("{} fixes over {:.0f} s, {:.0f} m along track".format(
        report["n_fixes"], report["gaps"]["span_s"], report["path_len_m"]))
    print("status: {}".format(report["status_counts"]))
    print("gaps > {:g}s: {} totalling {:.1f}s".format(
        GAP_THRESHOLD_S, len(report["gaps"]["gaps"]), report["gaps"]["gap_total_s"]))
    nf = report["noise_floor"]
    print("noise floor from {} stationary span(s), {:.0f}s total{}".format(
        nf["n_spans"], nf["total_s"], "" if nf["n_spans"] else " — " + nf.get("reason", "")))
    for w in sorted(k for k in nf if k.startswith("w")):
        print("  @{}: ".format(w) + "  ".join(
            "{} {:.2f} m".format(a, nf[w][a]["rmse_m"]) for a in sorted(nf[w])))
    if not report["covariance"]["trustworthy"]:
        print("NOTE: position_covariance is APPROXIMATED — do not use it for weighting")
    print("-> {} , {}".format(out / "gnss_ref.tum", out / "gnss_ref.json"))
    if not report["usable"]:
        # Both files are still written — the assessment is the record of *why* this
        # dataset has no reference, which is worth keeping.
        print("NOT USABLE as a reference: " + "; ".join(report["unusable_because"]))
        return 3
    return 0


def _entry():
    raise SystemExit(main())


if __name__ == "__main__":
    _entry()
