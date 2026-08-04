import json

import compare


def make_run(root, system, preset, name, end_pos=None):
    run = root / system / preset / name
    run.mkdir(parents=True)
    (run / "trajectory.tum").write_text("0 0 0 0 0 0 0 1\n")
    metrics = {"system": system, "preset": preset}
    if end_pos is not None:
        metrics["derived"] = {"end_pos_m": end_pos}
    (run / "metrics.json").write_text(json.dumps(metrics))
    return run


def labels(picks):
    return [label for label, _ in picks]


def test_each_group_contributes_only_its_median_run(tmp_path):
    # Ten curves are unreadable, which is what the median selection is for. end_pos
    # 5 / 30 / 9 puts run03 in the middle — not the first run, and not the best one.
    for name, end in [("run01", 5.0), ("run02", 30.0), ("run03", 9.0)]:
        make_run(tmp_path, "fast_lio", "default", name, end)
    assert labels(compare.pick_runs(tmp_path, False, set())) == ["fast_lio-default-run03"]


def test_the_median_is_an_observed_run_not_an_interpolated_value(tmp_path):
    # Four runs: median_low picks the lower middle, so a real run is always drawn.
    for name, end in [("run01", 5.0), ("run02", 9.0), ("run03", 30.0), ("run04", 99.0)]:
        make_run(tmp_path, "fast_lio", "default", name, end)
    assert labels(compare.pick_runs(tmp_path, False, set())) == ["fast_lio-default-run02"]


def test_all_draws_every_run(tmp_path):
    for name in ("run01", "run02", "run03"):
        make_run(tmp_path, "fast_lio", "default", name, 5.0)
    assert len(compare.pick_runs(tmp_path, True, set())) == 3


def test_a_group_without_derived_metrics_falls_back_to_all_of_its_runs(tmp_path, capsys):
    # Rather than failing: aggregate may simply not have run yet, and drawing everything
    # is a worse figure but still a figure.
    make_run(tmp_path, "fast_lio", "default", "run01", 5.0)
    make_run(tmp_path, "fast_lio", "default", "run02", None)
    picks = compare.pick_runs(tmp_path, False, set())
    assert len(picks) == 2
    assert "run aggregate first" in capsys.readouterr().err


def test_a_single_run_group_needs_no_median(tmp_path):
    make_run(tmp_path, "fast_lio", "default", "run01", None)
    assert labels(compare.pick_runs(tmp_path, False, set())) == ["fast_lio-default-run01"]


def test_disabled_systems_are_left_out(tmp_path):
    make_run(tmp_path, "fast_lio", "default", "run01", 5.0)
    make_run(tmp_path, "bievr_lio", "default", "run01", 5.0)
    assert labels(compare.pick_runs(tmp_path, False, {"bievr_lio"})) == [
        "fast_lio-default-run01"
    ]


def test_presets_of_one_system_are_separate_groups(tmp_path):
    make_run(tmp_path, "point_lio", "default", "run01", 5.0)
    make_run(tmp_path, "point_lio", "ivox1", "run01", 9.0)
    assert len(compare.pick_runs(tmp_path, False, set())) == 2


# --- the evo command line ----------------------------------------------------------


def test_the_first_trajectory_becomes_the_reference_to_align_the_rest_to(tmp_path):
    argv = compare.evo_argv([tmp_path / "a.tum", tmp_path / "b.tum"], "xy",
                            tmp_path / "o.pdf", plot=False)
    assert "--align_origin" in argv
    assert argv[argv.index("--ref") + 1].endswith("a.tum")
    # The reference is not also passed positionally, or evo draws it twice.
    assert sum(a.endswith("a.tum") for a in argv) == 1


def test_a_lone_trajectory_gets_no_alignment_flags(tmp_path):
    # Nothing to align to; passing --ref with no positional trajectory left is an error.
    argv = compare.evo_argv([tmp_path / "a.tum"], "xy", tmp_path / "o.pdf", plot=False)
    assert "--align_origin" not in argv and "--ref" not in argv
    assert any(a.endswith("a.tum") for a in argv)


def test_plot_mode_and_output_are_passed_through(tmp_path):
    argv = compare.evo_argv([tmp_path / "a.tum"], "xyz", tmp_path / "o.pdf", plot=False)
    assert argv[argv.index("--plot_mode") + 1] == "xyz"
    assert argv[argv.index("--save_plot") + 1].endswith("o.pdf")


def test_the_live_window_is_opt_in(tmp_path):
    args = [tmp_path / "a.tum"], "xy", tmp_path / "o.pdf"
    assert "--plot" not in compare.evo_argv(*args, plot=False)
    assert "--plot" in compare.evo_argv(*args, plot=True)


# --- the reference verdict ---------------------------------------------------------


def test_a_usable_reference_has_no_verdict(tmp_path):
    p = tmp_path / "gnss_ref.json"
    p.write_text(json.dumps({"usable": True}))
    assert compare.reference_verdict(p) is None


def test_an_unusable_reference_reports_why(tmp_path):
    p = tmp_path / "gnss_ref.json"
    p.write_text(json.dumps({"usable": False, "unusable_because": ["74% has no fix"]}))
    assert "74% has no fix" in compare.reference_verdict(p)


def test_a_missing_report_is_a_verdict_not_a_crash(tmp_path):
    assert "no reference report" in compare.reference_verdict(tmp_path / "absent.json")


def test_an_unparseable_report_is_a_verdict_not_a_crash(tmp_path):
    p = tmp_path / "gnss_ref.json"
    p.write_text("{ truncated")
    assert "no reference report" in compare.reference_verdict(p)


def test_a_report_with_no_verdict_field_is_treated_as_unusable(tmp_path):
    # Fail closed here, unlike the registry: drawing against a reference whose report
    # cannot vouch for it is the outcome to avoid.
    p = tmp_path / "gnss_ref.json"
    p.write_text(json.dumps({"n_fixes": 10}))
    assert compare.reference_verdict(p) == "unusable"
