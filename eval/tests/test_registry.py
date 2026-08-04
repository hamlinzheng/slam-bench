import re
from pathlib import Path

import pytest

import registry

REPO = Path(__file__).resolve().parent.parent.parent


def write_registry(tmp_path, text):
    path = tmp_path / "systems.yaml"
    path.write_text(text)
    return path


def test_a_system_without_the_key_is_enabled(tmp_path):
    path = write_registry(tmp_path, "fast_lio:\n  ws: /ws/fast_lio\n")
    assert registry.disabled_systems(path) == {}


def test_the_value_of_disabled_is_the_reason(tmp_path):
    path = write_registry(
        tmp_path,
        "fast_lio:\n  ws: /ws/fast_lio\n"
        "bievr_lio:\n  disabled: diverges past the first open stretch\n"
        "pv_lio:\n  ws: /ws/pv_lio\n",
    )
    assert registry.disabled_systems(path) == {
        "bievr_lio": "diverges past the first open stretch"
    }


def test_disabled_false_leaves_the_system_enabled(tmp_path):
    path = write_registry(tmp_path, "bievr_lio:\n  disabled: false\n")
    assert registry.disabled_systems(path) == {}


@pytest.mark.parametrize("value", ["", " true", ' ""'])
def test_a_disabled_key_with_no_reason_still_disables(tmp_path, value):
    # Ignoring it would keep a system someone meant to set aside in the comparison.
    path = write_registry(tmp_path, "bievr_lio:\n  disabled:{}\n".format(value))
    assert registry.disabled_systems(path) == {"bievr_lio": "no reason given"}


@pytest.mark.parametrize("text", [None, "fast_lio:\n  ws: [unclosed\n"])
def test_an_unreadable_registry_raises(tmp_path, text):
    path = tmp_path / "systems.yaml"
    if text is not None:
        path.write_text(text)
    with pytest.raises(registry.RegistryError):
        registry.disabled_systems(path)


def test_a_registry_that_is_not_a_mapping_raises(tmp_path):
    path = write_registry(tmp_path, "- fast_lio\n- faster_lio\n")
    with pytest.raises(registry.RegistryError):
        registry.disabled_systems(path)


def test_or_warn_reports_nothing_disabled_and_says_why(tmp_path, capsys):
    assert registry.disabled_systems_or_warn("agg", tmp_path / "absent.yaml") == {}
    assert "treating every system as enabled" in capsys.readouterr().err


def test_the_repositorys_own_registry_parses():
    # A syntax error here would otherwise first surface hours into a batch.
    assert isinstance(registry.disabled_systems(), dict)


def test_every_name_in_the_registry_is_a_system_that_can_actually_run():
    # The failure mode this feature introduces: `bievr-lio:` for `bievr_lio` disables
    # nothing and reports nothing, because a system with no runs on disk is exactly what
    # disabled_inventory stays silent about. This is what makes the registry the
    # authority rather than a document that hopes to be one.
    case = (
        (REPO / "scripts" / "run_system.sh")
        .read_text()
        .split('case "$SYS" in', 1)[1]
        .split("esac", 1)[0]
    )
    assert set(registry.load()) == set(re.findall(r"^  ([a-z0-9_]+)\)", case, re.M))


def test_cli_prints_the_reason_and_exits_zero_when_disabled(monkeypatch, capsys):
    monkeypatch.setattr(
        registry, "disabled_systems", lambda _p=None: {"bievr_lio": "unusable"}
    )
    assert registry.main(["disabled-reason", "bievr_lio"]) == 0
    assert capsys.readouterr().out.strip() == "unusable"


def test_cli_exits_one_and_says_nothing_for_an_enabled_system(monkeypatch, capsys):
    monkeypatch.setattr(registry, "disabled_systems", lambda _p=None: {})
    assert registry.main(["disabled-reason", "fast_lio"]) == 1
    assert capsys.readouterr().out == ""


def test_cli_exits_two_when_the_registry_cannot_be_read(monkeypatch):
    def boom():
        raise registry.RegistryError("cannot read it")

    monkeypatch.setattr(registry, "disabled_systems", boom)
    assert registry.main(["disabled-reason", "fast_lio"]) == 2
