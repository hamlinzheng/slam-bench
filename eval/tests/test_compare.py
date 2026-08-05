import json

import compare


def make_run(root, system, preset, name, end_pos=None, diverged_at=None, poses=None):
    run = root / system / preset / name
    run.mkdir(parents=True)
    (run / "trajectory.tum").write_text(
        "".join("{:.6f} {} {} {} 0 0 0 1\n".format(t, x, y, z)
                for t, x, y, z in poses)
        if poses is not None
        else "0 0 0 0 0 0 0 1\n"
    )
    metrics = {"system": system, "preset": preset}
    derived = {}
    if end_pos is not None:
        derived["end_pos_m"] = end_pos
    if diverged_at is not None:
        derived["diverged_at_s"] = diverged_at
    if derived:
        metrics["derived"] = derived
    (run / "metrics.json").write_text(json.dumps(metrics))
    return run


def labels(picks):
    return [p.label for p in picks]


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


# --- diverged runs -----------------------------------------------------------------


def test_a_diverged_run_never_represents_a_group_that_has_a_healthy_one(tmp_path):
    # By end_pos_m alone run02 is the median of 5 / 30 / 900000 and would be drawn. It is
    # the run that blew up, so the group is represented by a survivor instead.
    make_run(tmp_path, "fast_lio", "default", "run01", 5.0)
    make_run(tmp_path, "fast_lio", "default", "run02", 30.0, diverged_at=12.0)
    make_run(tmp_path, "fast_lio", "default", "run03", 900000.0, diverged_at=8.0)
    assert labels(compare.pick_runs(tmp_path, False, set())) == ["fast_lio-default-run01"]


def test_a_group_of_only_diverged_runs_is_still_drawn_and_marked(tmp_path):
    # Dropping it entirely would make a system that ran and failed look like one that was
    # never run — the same reason disabled systems get a line of their own.
    make_run(tmp_path, "faster_lio", "default", "run01", 5.0, diverged_at=27.6)
    make_run(tmp_path, "faster_lio", "default", "run02", 9.0, diverged_at=26.2)
    picks = compare.pick_runs(tmp_path, False, set())
    assert labels(picks) == ["faster_lio-default-run01-FAILED-28s"]
    assert picks[0].diverged_at_s == 27.6


def test_the_label_carries_the_moment_it_blew_up(tmp_path):
    # FAILED to match aggregate's tables — a figure and a table are read side by side, and
    # two words for one outcome make the pair look like two outcomes.
    make_run(tmp_path, "faster_lio", "default", "run01", 5.0, diverged_at=188.1)
    assert labels(compare.pick_runs(tmp_path, False, set())) == [
        "faster_lio-default-run01-FAILED-188s"
    ]


def test_a_healthy_run_carries_no_failure_marker(tmp_path):
    make_run(tmp_path, "fast_lio", "default", "run01", 5.0)
    pick = compare.pick_runs(tmp_path, False, set())[0]
    assert pick.diverged_at_s is None
    assert "FAILED" not in pick.label


def test_all_draws_diverged_runs_too(tmp_path):
    # ALL=true means every run, including the ones that failed — that mode exists to look
    # at the spread, and the failures are part of it.
    make_run(tmp_path, "fast_lio", "default", "run01", 5.0)
    make_run(tmp_path, "fast_lio", "default", "run02", 30.0, diverged_at=12.0)
    assert len(compare.pick_runs(tmp_path, True, set())) == 2


# --- truncation --------------------------------------------------------------------


def test_truncation_keeps_the_poses_up_to_the_moment_it_blew_up(tmp_path):
    src = tmp_path / "trajectory.tum"
    src.write_text("".join("{:.6f} {} 0 0 0 0 0 1\n".format(i * 0.1, i)
                           for i in range(100)))
    dst = tmp_path / "staged.tum"
    compare._stage_truncated(src, dst, 2.0)
    lines = dst.read_text().splitlines()
    assert len(lines) == 21          # t = 0.0 .. 2.0 inclusive, at 10 Hz
    assert lines[-1].startswith("2.000000")


def test_truncation_is_measured_from_the_first_pose_not_from_zero(tmp_path):
    # Trajectory timestamps are bag time, which starts wherever the recording did.
    src = tmp_path / "trajectory.tum"
    src.write_text("".join("{:.6f} {} 0 0 0 0 0 1\n".format(1774441874.0 + i * 0.1, i)
                           for i in range(100)))
    dst = tmp_path / "staged.tum"
    compare._stage_truncated(src, dst, 1.0)
    assert len(dst.read_text().splitlines()) == 11


def test_truncation_preserves_the_quaternion_columns(tmp_path):
    # compare.pdf has an attitude page; dropping the rotation would empty it.
    src = tmp_path / "trajectory.tum"
    src.write_text("0.0 1 2 3 0.1 0.2 0.3 0.4\n1.0 4 5 6 0.5 0.6 0.7 0.8\n")
    dst = tmp_path / "staged.tum"
    compare._stage_truncated(src, dst, 10.0)
    assert dst.read_text().splitlines()[0].split()[4:] == ["0.1", "0.2", "0.3", "0.4"]


def test_truncation_drops_a_partial_final_line(tmp_path):
    # A run killed mid-write leaves one, and it must not reach evo.
    src = tmp_path / "trajectory.tum"
    src.write_text("0.0 1 2 3 0 0 0 1\n0.1 4 5 6 0 0 0 1\n0.2 7 8")
    dst = tmp_path / "staged.tum"
    compare._stage_truncated(src, dst, 10.0)
    assert len(dst.read_text().splitlines()) == 2


# --- start / end markers -----------------------------------------------------------


def test_markers_are_configured_under_a_throwaway_home(tmp_path, monkeypatch):
    # Never the invoking user's ~/.evo: this is a plot script, and changing how every
    # other evo command on the machine behaves is not its business.
    calls = []
    monkeypatch.setattr(compare, "_run", lambda argv, **kw: calls.append((argv, kw)) or 0)
    env = {"HOME": "/home/someone"}
    compare._start_end_markers(env, tmp_path)
    argv, _ = calls[0]
    assert argv[:2] == ["evo_config", "set"]
    assert "plot_start_end_markers" in argv
    assert env["HOME"] == str(tmp_path / "evo-home")
    assert (tmp_path / "evo-home").is_dir()


def test_a_config_failure_costs_the_markers_and_nothing_else(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(compare, "_run", lambda argv, **kw: 1)
    compare._start_end_markers({}, tmp_path)
    assert "without them" in capsys.readouterr().err
