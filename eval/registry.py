"""The system registry — the only reader of configs/systems.yaml.

Named registry.py, not systems.py: the repo root holds a vendored `systems/` directory,
which Python 3 sees as a namespace package, and eval/compare.sh runs its python with the
repo root at sys.path[0].

A system is stopped out of the benchmark by a `disabled:` key whose value is the reason:

    bievr_lio:
      disabled: diverges past the first long open stretch; not comparable

Presence is the switch and the value is the reason, rather than `enabled: false` with the
reason in a comment: every consumer prints the reason straight back to whoever ran it,
and a YAML comment cannot be printed. See README, "Taking a system out of the
comparison", for what each consumer then does.
"""

import sys
from pathlib import Path

import yaml

REGISTRY = Path(__file__).resolve().parent.parent / "configs" / "systems.yaml"


class RegistryError(Exception):
    """configs/systems.yaml is missing, unreadable, or not the mapping expected here."""


def load(path=REGISTRY):
    """The registry as {system: fields}."""
    try:
        doc = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as e:
        raise RegistryError("cannot read {}: {}".format(path, e))
    if not isinstance(doc, dict):
        raise RegistryError("{} is not a mapping of system name -> fields".format(path))
    return doc


def disabled_systems(path=REGISTRY):
    """{system: reason} for every system stopped out of the benchmark.

    `disabled: false` leaves the system enabled — read as "disabled, reason: False" it
    would be a trap. Every other form disables; one with nothing to say reports "no
    reason given" rather than being ignored.
    """
    out = {}
    for name, fields in load(path).items():
        if not isinstance(fields, dict) or "disabled" not in fields:
            continue
        reason = fields["disabled"]
        if reason is False:
            continue
        if reason is True or reason is None or not str(reason).strip():
            reason = "no reason given"
        out[name] = str(reason).strip()
    return out


def disabled_systems_or_warn(prog, path=REGISTRY):
    """Same, but an unreadable registry warns and reports nothing disabled.

    The fail-open direction is the load-bearing part, so it lives here rather than at
    each consumer: subtracting systems from a comparison on the strength of a file
    nobody could parse is the one outcome worse than showing too much.
    """
    try:
        return disabled_systems(path)
    except RegistryError as e:
        print(
            "{}: warning — {}; treating every system as enabled".format(prog, e),
            file=sys.stderr,
        )
        return {}


def main(argv=None):
    """`registry.py disabled-reason <system>` — the shell's way in.

    Exit 0 with the reason on stdout when disabled, 1 when not, 2 when the registry
    could not be read; run_system.sh tells 2 from 1 so an unreadable registry cannot
    pass for a clean bill of health.
    """
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2 or argv[0] != "disabled-reason":
        print("usage: registry.py disabled-reason <system>", file=sys.stderr)
        return 2
    try:
        disabled = disabled_systems()
    except RegistryError as e:
        print("registry: {}".format(e), file=sys.stderr)
        return 2
    reason = disabled.get(argv[1])
    if reason is None:
        return 1
    print(reason)
    return 0


if __name__ == "__main__":
    sys.exit(main())
