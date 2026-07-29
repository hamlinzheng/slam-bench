import pytest

import next_run_dir


def mkruns(root, *names):
    for n in names:
        (root / n).mkdir()
    return root


def test_first_run_in_empty_preset_dir(tmp_path):
    assert next_run_dir.allocate(tmp_path) == tmp_path / "run01"


def test_continues_after_the_existing_run(tmp_path):
    mkruns(tmp_path, "run01")
    assert next_run_dir.allocate(tmp_path) == tmp_path / "run02"


def test_continues_from_the_highest_index_not_the_count(tmp_path):
    mkruns(tmp_path, "run01", "run03")
    assert next_run_dir.allocate(tmp_path) == tmp_path / "run04"


def test_widens_padding_past_run09(tmp_path):
    mkruns(tmp_path, "run09")
    assert next_run_dir.allocate(tmp_path) == tmp_path / "run10"


def test_ignores_directories_that_are_not_runs(tmp_path):
    mkruns(tmp_path, "run01", "runX", "notes")
    assert next_run_dir.allocate(tmp_path) == tmp_path / "run02"


def test_explicit_index_overrides_the_scan(tmp_path):
    mkruns(tmp_path, "run01")
    assert next_run_dir.allocate(tmp_path, run=3) == tmp_path / "run03"


def test_explicit_index_refuses_to_overwrite_an_existing_run(tmp_path):
    mkruns(tmp_path, "run03")
    with pytest.raises(next_run_dir.RunDirExists):
        next_run_dir.allocate(tmp_path, run=3)


def test_force_allows_reusing_an_existing_run_index(tmp_path):
    mkruns(tmp_path, "run03")
    assert next_run_dir.allocate(tmp_path, run=3, force=True) == tmp_path / "run03"
