#!/usr/bin/env python3
"""Prepare a dataset once, before aggregate and compare run against it.

Everything here is per-dataset, derived from the bags, and consumed by more than one
later step — which is why it is its own command rather than a side effect of one of
them. This is also the only step that reads /bags: aggregate and compare use what it
writes and never touch the recordings.

**A component that cannot run is not a failure.** A dataset with no GNSS is a healthy
dataset; it simply has no reference, and saying so is the useful output. So each
component reports its own verdict into its own artifact, and the exit code is reserved
for something actually going wrong — the bags being unreadable, not the site being
underground. Under the old `gnss` name the two were the same signal, and a perfectly
good dataset came back as a failure.

Components:
  gnss_ref   results/<dataset>/gnss_ref.{tum,json} — the reference track and a measured
             statement of what it is worth (fix mix, gap structure, noise floor)
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def component_gnss_ref(bags, out, topic):
    """Build the GNSS reference. Returns (ok, summary) — ok is False only when the
    component could not be attempted, not when the dataset has no usable GNSS."""
    cmd = [
        sys.executable,
        str(HERE / "gnss_ref.py"),
        str(bags),
        "--out",
        str(out),
        "--topic",
        topic,
    ]
    proc = subprocess.run(cmd)
    report = out / "gnss_ref.json"
    if not report.exists():
        return False, "gnss_ref.py wrote no report (exit {})".format(proc.returncode)
    try:
        r = json.loads(report.read_text())
    except ValueError as e:
        return False, "unreadable report: {}".format(e)
    if r.get("usable"):
        return True, "reference over {} fixes".format(r.get("n_fixes"))
    return True, "no reference — " + "; ".join(r.get("unusable_because") or ["unusable"])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags", help="a .bag, or a directory of them played as one stream")
    ap.add_argument("--out", required=True, help="results/<dataset>")
    ap.add_argument("--gnss-topic", default="/sensing/gnss/fix")
    args = ap.parse_args()

    bags, out = Path(args.bags), Path(args.out)
    if not bags.exists():
        raise SystemExit("init: no such bag path: {}".format(bags))
    out.mkdir(parents=True, exist_ok=True)

    results = [("gnss_ref", *component_gnss_ref(bags, out, args.gnss_topic))]

    print()
    print("dataset prepared: {}".format(out.name))
    for name, ok, summary in results:
        print("  {:<10} {}  {}".format(name, "ok  " if ok else "FAIL", summary))
    # Non-zero only for a component that could not be attempted at all.
    return 0 if all(ok for _, ok, _ in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
