import json

import pytest

import aggregate
import registry


def write_tum(path, points):
    """points: [(x, y, z), ...] — timestamps and quaternions are irrelevant here."""
    path.write_text(
        "".join(
            "{:.6f} {} {} {} 0 0 0 1\n".format(i * 0.1, x, y, z)
            for i, (x, y, z) in enumerate(points)
        )
    )
    return path


# A right triangle walked open: (0,0,0) -> (3,0,0) -> (3,4,0).
# Path length 3 + 4 = 7; displacement from start = 5 (the hypotenuse).
TRIANGLE = [(0, 0, 0), (3, 0, 0), (3, 4, 0)]


def test_path_length_sums_consecutive_translation_deltas(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", TRIANGLE)
    assert aggregate.trajectory_stats(tum)["path_len_m"] == 7.0


def test_end_pos_is_displacement_from_the_first_pose(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", TRIANGLE)
    assert aggregate.trajectory_stats(tum)["end_pos_m"] == 5.0


def test_end_pos_is_measured_from_the_first_pose_not_the_origin(tmp_path):
    # Same triangle, translated away from the origin: the answer must not change.
    shifted = [(x + 100, y + 200, z + 300) for x, y, z in TRIANGLE]
    tum = write_tum(tmp_path / "trajectory.tum", shifted)
    assert aggregate.trajectory_stats(tum)["end_pos_m"] == 5.0


# (0,0,0) -> (3,4,12): horizontal leg 5, vertical leg 12, so the 3D displacement is 13.
# Chosen so all three numbers are exact and no two of them coincide.
SLOPED = [(0, 0, 0), (3, 4, 12)]


def test_end_pos_splits_into_horizontal_and_vertical(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", SLOPED)
    d = aggregate.trajectory_stats(tum)
    assert d["end_pos_m"] == 13.0
    assert d["end_pos_horiz_m"] == 5.0
    assert d["end_pos_vert_m"] == 12.0


def test_vertical_displacement_is_a_magnitude_not_a_signed_height(tmp_path):
    # Descending must read the same as climbing: end_pos_m is a magnitude and its
    # components have to compose back into it whichever way the run went.
    tum = write_tum(tmp_path / "trajectory.tum", [(0, 0, 0), (3, 4, -12)])
    d = aggregate.trajectory_stats(tum)
    assert d["end_pos_vert_m"] == 12.0
    assert d["end_pos_m"] == 13.0


def test_flat_trajectory_puts_all_of_end_pos_in_the_horizontal_component(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", TRIANGLE)
    d = aggregate.trajectory_stats(tum)
    assert d["end_pos_horiz_m"] == 5.0
    assert d["end_pos_vert_m"] == 0.0


def test_empty_trajectory_is_rejected_rather_than_crashing(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", [])
    with pytest.raises(aggregate.InsufficientTrajectory):
        aggregate.trajectory_stats(tum)


def test_single_pose_trajectory_is_rejected(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", [(0, 0, 0)])
    with pytest.raises(aggregate.InsufficientTrajectory):
        aggregate.trajectory_stats(tum)


def write_resource(path, rows):
    """rows: [(cpu_pct, rss_mb), ...] — one sample per second."""
    path.write_text(
        "wall_s,cpu_pct,rss_mb\n"
        + "".join("{:.3f},{},{}\n".format(i, c, r) for i, (c, r) in enumerate(rows))
    )
    return path


def test_resource_stats_report_mean_and_max_cpu_and_peak_rss(tmp_path):
    csv = write_resource(tmp_path / "resource.csv", [(100, 10), (200, 30), (300, 20)])
    assert aggregate.resource_stats(csv) == {
        "cpu_mean": 200.0,
        "cpu_max": 300.0,
        "rss_max_MB": 30.0,
        "cpu_s_total": 6.0,  # (100 + 200 + 300)% x 1 s each
    }


def test_cpu_seconds_integrate_each_sample_over_its_own_interval(tmp_path):
    """Not cpu_mean x span: the sampler's rows are a rate, and unevenly spaced rows would
    weight a long interval the same as a short one."""
    path = tmp_path / "resource.csv"
    # Two 1 s samples at 100%, then one covering a 4 s gap at 50%:
    # 1.0 + 1.0 + 2.0 = 4.0 CPU-seconds. Weighting the three rows equally instead would
    # give cpu_mean 83.3% over a 5 s span = 4.17.
    path.write_text("wall_s,cpu_pct,rss_mb\n1.000,100,10\n2.000,100,10\n6.000,50,10\n")
    assert aggregate.resource_stats(path)["cpu_s_total"] == pytest.approx(4.0)


def test_cpu_seconds_are_absent_when_one_sample_cannot_show_its_interval(tmp_path):
    # A lone row carries a rate but no duration to apply it over. None, not zero.
    csv = write_resource(tmp_path / "resource.csv", [(100, 10)])
    assert aggregate.resource_stats(csv)["cpu_s_total"] is None


def test_cpu_per_frame_divides_processor_time_by_the_poses_produced(tmp_path):
    """The quantity this exists for: CPU cost of one frame, independent of how long the
    run took or how much of it the process spent idle."""
    write_tum(tmp_path / "trajectory.tum", TRIANGLE)  # 3 poses
    # 3 samples of 200% at 1 s each = 6 CPU-seconds over 3 frames = 2000 ms/frame.
    write_resource(tmp_path / "resource.csv", [(200, 10)] * 3)
    assert aggregate._derive(tmp_path)["cpu_ms_per_frame"] == pytest.approx(2000.0)


def test_cpu_per_frame_is_absent_when_the_run_has_no_resource_trace(tmp_path):
    write_tum(tmp_path / "trajectory.tum", TRIANGLE)
    assert aggregate._derive(tmp_path)["cpu_ms_per_frame"] is None


def test_summary_reports_count_median_min_and_max():
    assert aggregate.summarize([30.2, 99.4, 484.7]) == {
        "n": 3,
        "median": 99.4,
        "min": 30.2,
        "max": 484.7,
    }


def test_median_of_an_even_sample_is_an_observed_value_not_an_average():
    # The distribution is bimodal (recover ~22-30 m, explode ~99-603 m).
    # Averaging the two middle values would report 64.8 m — the empty gap between the
    # modes, an outcome no run ever produced. That is the same objection raised
    # against the mean, so the low median is used: it is always a real observation.
    bimodal = [22.3, 30.2, 99.4, 484.7]
    assert aggregate.summarize(bimodal)["median"] == 30.2


def test_resource_stats_of_a_header_only_file_are_absent_not_zero(tmp_path):
    # A sampler that never found the process writes nothing but the header. Zero would
    # read as "used no CPU"; absent reads as "not measured".
    csv = write_resource(tmp_path / "resource.csv", [])
    assert aggregate.resource_stats(csv) == {
        "cpu_mean": None,
        "cpu_max": None,
        "rss_max_MB": None,
        "cpu_s_total": None,
    }


def make_run(root, name="run01", metrics=None, poses=TRIANGLE, resource=((100, 10),)):
    run = root / name
    run.mkdir(parents=True)
    base = {
        "system": "fast_lio",
        "preset": "default",
        "run": 1,
        "dataset": "ds",
        "bag_play_exit": 0,
        "preset_sha": "aaa",
        "binary_sha": "bbb",
        "system_commit": "ccc",
    }
    base.update(metrics or {})
    (run / "metrics.json").write_text(json.dumps(base))
    if poses is not None:
        write_tum(run / "trajectory.tum", poses)
    if resource is not None:
        write_resource(run / "resource.csv", resource)
    return run


def test_healthy_run_loads_with_its_derived_quantities(tmp_path):
    rec = aggregate.load_run(make_run(tmp_path))
    assert rec["status"] == "ok"
    assert rec["derived"]["path_len_m"] == 7.0
    assert rec["derived"]["end_pos_m"] == 5.0
    assert rec["derived"]["cpu_max"] == 100.0


def test_nonzero_bag_play_exit_marks_the_run_failed(tmp_path):
    rec = aggregate.load_run(make_run(tmp_path, metrics={"bag_play_exit": 2}))
    assert rec["status"] == "failed"


def test_unusable_trajectory_marks_the_run_failed_rather_than_raising(tmp_path):
    rec = aggregate.load_run(make_run(tmp_path, poses=[]))
    assert rec["status"] == "failed"
    assert rec["derived"] == {}


def test_void_file_marks_the_run_void_and_carries_its_reason(tmp_path):
    run = make_run(tmp_path)
    (run / "VOID").write_text("starved — det_range 30 cannot serve a 70 m sensor\n")
    rec = aggregate.load_run(run)
    assert rec["status"] == "void"
    assert rec["void_reason"] == "starved — det_range 30 cannot serve a 70 m sensor"


def test_void_wins_over_an_automatic_verdict(tmp_path):
    # A run can exit 0 and still be worthless (a starved run). The human
    # judgement must not be overridden by the clean exit code.
    run = make_run(tmp_path)
    (run / "VOID").write_text("starved\n")
    assert aggregate.load_run(run)["status"] == "void"


def test_missing_resource_csv_leaves_resource_fields_absent(tmp_path):
    rec = aggregate.load_run(make_run(tmp_path, resource=None))
    assert rec["status"] == "ok"
    assert rec["derived"]["path_len_m"] == 7.0
    assert rec["derived"]["cpu_mean"] is None


def test_unparseable_metrics_json_is_rejected(tmp_path):
    run = make_run(tmp_path)
    (run / "metrics.json").write_text("{ truncated")
    with pytest.raises(aggregate.MalformedRun):
        aggregate.load_run(run)


BAG_START, BAG_END = 1000.0, 1060.0   # a 60 s bag


def stamped_tum(path, t0, t1, n=100):
    """A trajectory spanning [t0, t1] with n evenly spaced poses on a straight line."""
    step = (t1 - t0) / (n - 1)
    path.write_text(
        "".join(
            "{:.6f} {} 0 0 0 0 0 1\n".format(t0 + i * step, i) for i in range(n)
        )
    )
    return path


def make_timed_run(root, traj_span, name="run01"):
    run = root / name
    run.mkdir(parents=True)
    (run / "metrics.json").write_text(
        json.dumps(
            {
                "system": "fast_lio",
                "preset": "default",
                "bag_play_exit": 0,
                "bag_start": BAG_START,
                "bag_end": BAG_END,
            }
        )
    )
    stamped_tum(run / "trajectory.tum", *traj_span)
    return run


def test_a_trajectory_covering_the_bag_is_healthy(tmp_path):
    rec_ = aggregate.load_run(make_timed_run(tmp_path, (1000.5, 1059.5)))
    assert rec_["status"] == "ok"
    assert rec_["derived"]["coverage"] == pytest.approx(0.983, abs=0.01)


def test_a_trajectory_stamped_outside_the_bag_did_not_come_from_this_run(tmp_path):
    # The exact failure seen in practice: a surviving container kept the ROS master, so
    # the recorder captured /Odometry from a different run replaying a different bag.
    # Its timestamps cannot lie inside the bag this run played.
    rec_ = aggregate.load_run(make_timed_run(tmp_path, (900000.0, 900050.0)))
    assert rec_["status"] == "failed"
    assert rec_["fail_reason"] == "trajectory timestamps fall outside the played bag"


def test_an_interrupted_run_says_so_rather_than_reporting_a_null_exit_code(tmp_path):
    # run_system.sh records bag_play_exit only once playback returns, so an interrupted
    # run leaves it null. "exited None" reads like a bug in the tooling.
    run = make_timed_run(tmp_path, (1000.0, 1005.0))
    m = json.loads((run / "metrics.json").read_text())
    m["bag_play_exit"] = None
    (run / "metrics.json").write_text(json.dumps(m))
    rec_ = aggregate.load_run(run)
    assert rec_["status"] == "failed"
    assert rec_["fail_reason"] == "playback never finished (interrupted or died)"


def test_a_trajectory_covering_a_third_of_the_bag_did_not_run_to_the_end(tmp_path):
    rec_ = aggregate.load_run(make_timed_run(tmp_path, (1000.0, 1020.0)))
    assert rec_["status"] == "failed"
    assert "coverage" in rec_["fail_reason"]


def test_the_coverage_threshold_is_adjustable(tmp_path):
    run = make_timed_run(tmp_path, (1000.0, 1020.0))
    assert aggregate.load_run(run, min_coverage=0.3)["status"] == "ok"


def test_completion_is_unchecked_when_the_run_recorded_no_bag_bounds(tmp_path):
    # Runs written before bag bounds existed must not all become "failed".
    run = make_timed_run(tmp_path, (1000.0, 1020.0))
    m = json.loads((run / "metrics.json").read_text())
    del m["bag_start"], m["bag_end"]
    (run / "metrics.json").write_text(json.dumps(m))
    rec_ = aggregate.load_run(run)
    assert rec_["status"] == "ok"
    assert rec_["derived"]["coverage"] is None


def rec(system="fast_lio", preset="default", status="ok", end_pos_m=10.0, **kw):
    r = {
        "system": system,
        "preset": preset,
        "status": status,
        "preset_sha": "aaa",
        "binary_sha": "bbb",
        "system_commit": "ccc",
        "derived": {"end_pos_m": end_pos_m},
    }
    r.update(kw)
    return r


def groups_of(records):
    return {(g["system"], g["preset"]): g for g in aggregate.group_runs(records)}


def test_runs_are_grouped_by_system_and_preset():
    groups = groups_of([rec(), rec(), rec(preset="cube400"), rec(system="faster_lio")])
    assert len(groups) == 3
    assert len(groups[("fast_lio", "default")]["runs"]) == 2


def test_excluded_runs_are_listed_but_kept_out_of_the_statistics():
    groups = groups_of(
        [rec(), rec(status="failed"), rec(status="void", void_reason="starved")]
    )
    g = groups[("fast_lio", "default")]
    assert len(g["runs"]) == 1
    assert [e["status"] for e in g["excluded"]] == ["failed", "void"]


def test_a_uniform_group_is_marked_consistent():
    assert groups_of([rec(), rec()])[("fast_lio", "default")]["consistent"] is True


def test_a_differing_binary_sha_splits_the_group():
    # findings §4.2: the source patch was reverted and the tree is clean, but .ws/
    # still held the patched binary. Only the binary hash separates those runs, and
    # merging them into one median is exactly the error that invalidated a past sweep.
    subgroups = [
        g
        for g in aggregate.group_runs([rec(), rec(), rec(binary_sha="patched")])
        if g["system"] == "fast_lio" and g["preset"] == "default"
    ]
    assert len(subgroups) == 2
    assert all(g["consistent"] is False for g in subgroups)
    assert sorted(len(g["runs"]) for g in subgroups) == [1, 2]


def test_a_run_with_no_fingerprint_does_not_look_like_a_configuration_change():
    # A killed run carries no fingerprint at all. Splitting the group on it would raise
    # ⚠ MIXED — "these runs were built differently" — which is not what happened.
    killed = rec(status="failed", preset_sha=None, binary_sha=None, system_commit=None)
    groups = aggregate.group_runs([rec(), rec(), killed])
    assert len(groups) == 1
    assert groups[0]["consistent"] is True
    assert len(groups[0]["runs"]) == 2
    assert len(groups[0]["excluded"]) == 1


def test_a_differing_preset_sha_splits_the_group():
    subgroups = aggregate.group_runs([rec(), rec(preset_sha="edited")])
    assert len(subgroups) == 2
    assert all(g["consistent"] is False for g in subgroups)


def make_dataset(root, layout):
    """layout: {(system, preset): [run_name, ...]} under results/<ds>/."""
    for (system, preset), runs in layout.items():
        for name in runs:
            make_run(
                root / system / preset,
                name=name,
                metrics={"system": system, "preset": preset},
            )
    return root


def test_collect_walks_system_preset_run_and_finds_every_run(tmp_path):
    make_dataset(
        tmp_path,
        {
            ("fast_lio", "default"): ["run01", "run02"],
            ("fast_lio", "cube400"): ["run01"],
            ("faster_lio", "default"): ["run01"],
        },
    )
    records = aggregate.collect(tmp_path)
    assert len(records) == 4
    assert {r["system"] for r in records} == {"fast_lio", "faster_lio"}


def test_collect_skips_a_malformed_run_with_a_warning(tmp_path, capsys):
    make_dataset(tmp_path, {("fast_lio", "default"): ["run01", "run02"]})
    (tmp_path / "fast_lio" / "default" / "run02" / "metrics.json").write_text("{ nope")

    records = aggregate.collect(tmp_path)

    assert len(records) == 1
    assert "run02" in capsys.readouterr().err


def test_collect_reports_a_run_directory_that_has_no_metrics_json(tmp_path, capsys):
    # SIGKILL cannot be trapped, so `docker rm -f` (the second Ctrl-C, or a manual
    # cleanup) leaves artefacts with no run record. Globbing for metrics.json would skip
    # it in silence and n would shrink without anyone noticing.
    make_dataset(tmp_path, {("fast_lio", "default"): ["run01"]})
    orphan = tmp_path / "fast_lio" / "default" / "run02"
    orphan.mkdir()
    write_tum(orphan / "trajectory.tum", TRIANGLE)

    records = aggregate.collect(tmp_path)

    assert len(records) == 2
    killed = [r for r in records if r["status"] == "failed"]
    assert killed and killed[0]["fail_reason"] == "no metrics.json (run was killed)"
    assert "run02" in capsys.readouterr().err


def test_collect_writes_derived_quantities_back_into_metrics_json(tmp_path):
    make_dataset(tmp_path, {("fast_lio", "default"): ["run01"]})
    run = tmp_path / "fast_lio" / "default" / "run01"

    aggregate.collect(tmp_path)

    assert json.loads((run / "metrics.json").read_text())["derived"]["path_len_m"] == 7.0


def test_collect_does_not_overwrite_derived_when_nothing_can_be_derived(tmp_path):
    """Aggregating while a batch is still running must not destroy an earlier result.

    A run whose trajectory has not reached two poses yet derives nothing, and writing that
    back replaced a complete record with an empty one — the run then read as failed until
    someone re-aggregated. Observed: it marked a healthy 1818 m BIEVR-LIO run as having no
    usable trajectory. An empty derivation carries no information, so it is never written.
    """
    make_dataset(tmp_path, {("fast_lio", "default"): ["run01"]})
    run = tmp_path / "fast_lio" / "default" / "run01"
    aggregate.collect(tmp_path)

    write_tum(run / "trajectory.tum", [])  # as if the recorder had only just started
    aggregate.collect(tmp_path)

    assert json.loads((run / "metrics.json").read_text())["derived"]["path_len_m"] == 7.0


def group_with(*derived_dicts, **kw):
    records = [rec(**kw) for _ in derived_dicts]
    for r, d in zip(records, derived_dicts):
        r["derived"] = d
    return aggregate.group_runs(records)[0]


def test_group_statistics_summarize_every_derived_metric():
    g = group_with(
        {"end_pos_m": 30.2, "cpu_mean": 100.0},
        {"end_pos_m": 99.4, "cpu_mean": 200.0},
        {"end_pos_m": 484.7, "cpu_mean": 300.0},
    )
    stats = aggregate.group_stats(g)
    assert stats["metrics"]["end_pos_m"] == {
        "n": 3,
        "median": 99.4,
        "min": 30.2,
        "max": 484.7,
    }
    assert stats["metrics"]["cpu_mean"]["median"] == 200.0


def test_per_run_values_are_sorted_so_bimodality_is_visible():
    g = group_with(
        {"end_pos_m": 484.7}, {"end_pos_m": 22.3}, {"end_pos_m": 99.4}
    )
    assert aggregate.group_stats(g)["per_run"]["end_pos_m"] == [22.3, 99.4, 484.7]


def test_a_single_run_group_is_flagged_as_having_no_spread():
    stats = aggregate.group_stats(group_with({"end_pos_m": 30.2}))
    assert stats["n"] == 1
    assert stats["no_spread"] is True


def test_a_metric_missing_from_some_runs_is_summarized_over_the_rest():
    g = group_with(
        {"end_pos_m": 10.0, "cpu_mean": 100.0},
        {"end_pos_m": 20.0, "cpu_mean": None},
    )
    stats = aggregate.group_stats(g)
    assert stats["metrics"]["end_pos_m"]["n"] == 2
    assert stats["metrics"]["cpu_mean"]["n"] == 1


def test_a_metric_absent_from_every_run_is_reported_as_absent():
    g = group_with({"end_pos_m": 10.0, "cpu_mean": None})
    assert aggregate.group_stats(g)["metrics"]["cpu_mean"] is None


def test_no_bucketing_is_reported_unless_a_threshold_is_given():
    g = group_with({"end_pos_m": 30.2}, {"end_pos_m": 484.7})
    assert aggregate.group_stats(g)["split"] is None


def test_split_at_buckets_runs_either_side_of_the_given_threshold():
    # The threshold is never defaulted: on this 2846 m trajectory the bimodal gap sits
    # between 30 and 99 m, but that is a property of this dataset, not of the metric.
    g = group_with(
        {"end_pos_m": 22.3},
        {"end_pos_m": 30.2},
        {"end_pos_m": 99.4},
        {"end_pos_m": 484.7},
    )
    assert aggregate.group_stats(g, split_at=50.0)["split"] == {
        "threshold": 50.0,
        "below": 2,
        "above": 2,
    }


def render(records, split_at=None):
    groups = aggregate.group_runs(records)
    return aggregate.render_text(
        "ds", [aggregate.group_stats(g, split_at=split_at) for g in groups]
    )


def test_rendered_table_carries_the_median_and_the_range():
    text = render([rec(end_pos_m=30.2), rec(end_pos_m=99.4), rec(end_pos_m=484.7)])
    assert "fast_lio" in text
    assert "99.4" in text and "30.2" in text and "484.7" in text


def test_rendered_table_flags_a_group_whose_fingerprints_disagree():
    text = render([rec(), rec(binary_sha="patched")])
    assert "MIXED" in text


def test_rendered_table_carries_the_void_reason():
    text = render(
        [rec(), rec(status="void", void_reason="starved — det_range 30 too small")]
    )
    assert "starved — det_range 30 too small" in text


def test_rendered_table_carries_the_automatic_failure_reason():
    text = render(
        [rec(), rec(status="failed", fail_reason="trajectory timestamps fall outside the played bag")]
    )
    assert "trajectory timestamps fall outside the played bag" in text


def test_collect_applies_the_coverage_threshold_it_is_given(tmp_path):
    make_timed_run(tmp_path / "fast_lio" / "default", (1000.0, 1020.0))
    assert aggregate.collect(tmp_path)[0]["status"] == "failed"
    assert aggregate.collect(tmp_path, min_coverage=0.3)[0]["status"] == "ok"


def test_rendered_table_counts_exclusions_beside_n():
    text = render([rec(), rec(status="void", void_reason="x"), rec(status="failed")])
    assert "1 void" in text and "1 failed" in text


def test_rendered_table_annotates_a_group_that_has_no_spread():
    assert "no spread" in render([rec()])


@pytest.fixture
def disabled(monkeypatch):
    """Set which systems main() sees as disabled, without touching configs/systems.yaml.

    Every test that reaches main() uses it, including the ones about something else:
    unpatched they would read the repository's real registry, and disabling a system
    there would break tests that have nothing to do with the switch.
    """

    def use(mapping):
        monkeypatch.setattr(registry, "disabled_systems", lambda _p=None: dict(mapping))

    use({})
    return use


def test_main_writes_both_stats_files(tmp_path, disabled):
    make_dataset(tmp_path, {("fast_lio", "default"): ["run01", "run02"]})

    assert aggregate.main([str(tmp_path)]) == 0

    assert "fast_lio" in (tmp_path / "stats.txt").read_text()
    payload = json.loads((tmp_path / "stats.json").read_text())
    assert payload["groups"][0]["metrics"]["end_pos_m"]["n"] == 2


def test_collect_leaves_out_the_systems_it_is_told_to_skip(tmp_path):
    make_dataset(
        tmp_path,
        {
            ("fast_lio", "default"): ["run01", "run02"],
            ("bievr_lio", "default"): ["run01"],
        },
    )
    records = aggregate.collect(tmp_path, skip_systems={"bievr_lio"})
    assert {r["system"] for r in records} == {"fast_lio"}


def test_inventory_counts_the_runs_a_skip_would_drop_across_every_preset(tmp_path):
    make_dataset(
        tmp_path,
        {
            ("fast_lio", "default"): ["run01"],
            ("bievr_lio", "default"): ["run01", "run02"],
            ("bievr_lio", "ivox1"): ["run01"],
        },
    )
    assert aggregate.disabled_inventory(tmp_path, {"bievr_lio": "unusable"}) == [
        ("bievr_lio", "unusable", 3)
    ]


def test_inventory_says_nothing_about_a_system_this_dataset_never_ran(tmp_path):
    make_dataset(tmp_path, {("fast_lio", "default"): ["run01"]})
    assert aggregate.disabled_inventory(tmp_path, {"bievr_lio": "unusable"}) == []


def test_main_reports_what_it_dropped_instead_of_dropping_it_quietly(
    tmp_path, capsys, disabled
):
    make_dataset(
        tmp_path,
        {
            ("fast_lio", "default"): ["run01"],
            ("bievr_lio", "default"): ["run01", "run02"],
        },
    )
    disabled({"bievr_lio": "diverges past the first open stretch"})

    assert aggregate.main([str(tmp_path)]) == 0

    err = capsys.readouterr().err
    assert "skipped bievr_lio" in err and "2 run(s)" in err
    text = (tmp_path / "stats.txt").read_text()
    assert "diverges past the first open stretch" in text
    assert "bievr_lio-default" not in text
    payload = json.loads((tmp_path / "stats.json").read_text())
    assert payload["disabled"] == [
        {
            "system": "bievr_lio",
            "reason": "diverges past the first open stretch",
            "runs": 2,
        }
    ]


def test_main_distinguishes_all_disabled_from_an_empty_dataset(
    tmp_path, capsys, disabled
):
    make_dataset(tmp_path, {("bievr_lio", "default"): ["run01"]})
    disabled({"bievr_lio": "unusable"})

    assert aggregate.main([str(tmp_path)]) == 1

    assert "belongs to a disabled system" in capsys.readouterr().err


def test_main_draws_everything_when_the_registry_cannot_be_read(
    tmp_path, capsys, monkeypatch
):
    # The safe direction is showing too much: a registry that fails to parse must not
    # subtract systems from the tables on the strength of a guess.
    def boom(_path=None):
        raise registry.RegistryError("cannot read it")

    monkeypatch.setattr(registry, "disabled_systems", boom)
    make_dataset(tmp_path, {("fast_lio", "default"): ["run01"]})

    assert aggregate.main([str(tmp_path)]) == 0

    assert "treating every system as enabled" in capsys.readouterr().err
    assert "fast_lio" in (tmp_path / "stats.txt").read_text()
