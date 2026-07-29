#!/usr/bin/env python3
"""Allocate the next run directory under a preset directory.

Used by scripts/run_system.sh, which needs the path before the run starts:

    eval/next_run_dir.py results/<ds>/<sys>/<preset> [--run N] [--force]

Prints the allocated path on stdout. Exits non-zero if an explicitly requested
index is already occupied and --force was not given.
"""
import argparse
import re
import sys
from pathlib import Path

RUN_RE = re.compile(r"^run(\d+)$")


class RunDirExists(Exception):
    """An explicitly requested run index already holds results."""


def allocate(preset_dir, run=None, force=False):
    """Return the run directory to write into.

    Without `run`, picks the next free index (highest existing + 1). With `run`,
    uses that index, refusing to overwrite an existing one unless `force`.
    """
    preset_dir = Path(preset_dir)
    if run is None:
        return preset_dir / _fmt(_highest(preset_dir) + 1)

    target = preset_dir / _fmt(run)
    if target.exists() and not force:
        raise RunDirExists(
            "{} already exists — pass FORCE=true to overwrite it".format(target)
        )
    return target


def _fmt(index):
    return "run{:02d}".format(index)


def _highest(preset_dir):
    highest = 0
    if preset_dir.is_dir():
        for child in preset_dir.iterdir():
            m = RUN_RE.match(child.name)
            if m and child.is_dir():
                highest = max(highest, int(m.group(1)))
    return highest


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("preset_dir")
    p.add_argument("--run", type=int, default=None, help="explicit run index")
    p.add_argument("--force", action="store_true", help="allow reusing an index")
    a = p.parse_args(argv)
    try:
        print(allocate(a.preset_dir, run=a.run, force=a.force))
    except RunDirExists as e:
        print(e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
