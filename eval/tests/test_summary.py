import json

import summary


def metric(median, lo=None, hi=None):
    lo = median if lo is None else lo
    hi = median if hi is None else hi
    return {"n": 3, "median": median, "min": lo, "max": hi}


def group(system, preset="default", n=3, metrics=None, excluded=()):
    return {
        "system": system,
        "preset": preset,
        "fingerprint": {"rate": 1.0},
        "consistent": True,
        "n": n,
        "no_spread": n == 1,
        "metrics": metrics or {},
        "per_run": {},
        "split": None,
        "excluded": list(excluded),
    }


def dataset(root, name, groups, ref=None):
    d = root / name
    d.mkdir(parents=True)
    (d / "stats.json").write_text(
        json.dumps({"dataset": name, "groups": groups, "disabled": []})
    )
    if ref is not None:
        (d / "gnss_ref.json").write_text(json.dumps(ref))
    return d


def render(root):
    data = summary.load(root)
    return summary.render(data, root, summary.reference_verdicts(root))


# --- what a cell means -------------------------------------------------------------


def test_a_system_never_run_on_a_dataset_is_not_a_failure(tmp_path):
    dataset(tmp_path, "DS_20260101", [group("fast_lio", metrics={"cpu_mean": metric(10)})])
    dataset(tmp_path, "DS_20260102", [group("fast_lio", metrics={"cpu_mean": metric(10)}),
                                      group("pv_lio", metrics={"cpu_mean": metric(20)})])
    text = render(tmp_path)
    # pv_lio has no row on the first dataset — an en dash, not `fail` and not a blank.
    row = next(l for l in text.splitlines() if l.startswith("| `pv_lio` |"))
    assert "–" in row


def test_a_group_that_lost_every_run_reads_fail_not_zero(tmp_path):
    # Zero would be a measurement. There is none: accuracy of a run that did not finish
    # is not a small number, it is not a number.
    dataset(tmp_path, "DS_20260101",
            [group("pv_lio", n=0, metrics={"cpu_mean": None},
                   excluded=[{"run_dir": "r1", "status": "diverged", "reason": "blew up"}])])
    text = render(tmp_path)
    assert "**fail**" in text
    assert "| 0 |" not in text


def test_a_surviving_run_whose_metric_was_never_collected_reads_not_measured(tmp_path):
    # The run finished, so `fail` would be wrong; the quantity was never captured, so a
    # number would be an invention. Neither, and it has to be visibly neither.
    dataset(tmp_path, "DS_20260101",
            [group("fast_lio", metrics={"cpu_mean": metric(10), "lat_p99_ms": None})])
    text = render(tmp_path)
    latency = text.split("### Latency p99")[1].split("###")[0]
    assert "n/m" in latency
    assert "**fail**" not in latency


def test_with_no_reference_anywhere_the_accuracy_section_says_so(tmp_path):
    dataset(tmp_path, "DS_20260101", [group("fast_lio", metrics={"cpu_mean": metric(10)})])
    assert "No dataset here has a usable GNSS reference" in render(tmp_path)


# --- completion --------------------------------------------------------------------


def test_completion_counts_attempts_not_just_survivors(tmp_path):
    dataset(tmp_path, "DS_20260101",
            [group("bievr_lio", n=2,
                   excluded=[{"run_dir": "r3", "status": "diverged", "reason": "blew up"}])])
    text = render(tmp_path)
    assert "2/3" in text


def test_a_wipeout_is_marked_so_it_cannot_be_skimmed_past(tmp_path):
    dataset(tmp_path, "DS_20260101",
            [group("pv_lio", n=0, excluded=[
                {"run_dir": "r{}".format(i), "status": "diverged", "reason": "blew up"}
                for i in range(3)])])
    assert "**0/3**" in render(tmp_path)


def test_every_set_aside_run_is_listed_with_its_reason(tmp_path):
    dataset(tmp_path, "DS_20260101",
            [group("pv_lio", n=2, excluded=[
                {"run_dir": "r3", "status": "failed",
                 "reason": "trajectory coverage 23% of the bag, below 90%"}])])
    text = render(tmp_path)
    assert "trajectory coverage 23%" in text


def test_a_diverged_run_is_reported_as_failed_with_the_kind_in_the_reason(tmp_path):
    # The same wording aggregate's tables use, and from the same code, so the two cannot
    # drift apart.
    dataset(tmp_path, "DS_20260101",
            [group("pv_lio", n=2, excluded=[
                {"run_dir": "r3", "status": "diverged", "reason": "60% over 40 m/s"}])])
    text = render(tmp_path)
    assert "| FAILED |" in text
    assert "diverged: 60% over 40 m/s" in text
    assert "| DIVERGED |" not in text


# --- accuracy ----------------------------------------------------------------------


def test_only_datasets_with_a_reference_appear_in_the_accuracy_tables(tmp_path):
    dataset(tmp_path, "WITH_20260101",
            [group("fast_lio", metrics={"gnss_ape_horiz_rmse_m": metric(3.7)})],
            ref={"usable": True})
    dataset(tmp_path, "WITHOUT_20260102",
            [group("fast_lio", metrics={"cpu_mean": metric(10)})],
            ref={"usable": False, "unusable_because": ["74% of the run has no fix"]})
    text = render(tmp_path)
    table = text.split("### APE horizontal RMSE")[1].split("###")[0]
    assert "WITH" in table and "WITHOUT" not in table


def test_a_dataset_without_a_reference_says_which_check_failed(tmp_path):
    # Measured, not missing: a receiver that keeps reporting fixes underground is exactly
    # the fault this check exists to catch, so the reason has to reach the page.
    dataset(tmp_path, "WITH_20260101",
            [group("fast_lio", metrics={"gnss_ape_horiz_rmse_m": metric(3.7)})],
            ref={"usable": True})
    dataset(tmp_path, "NORCAT_20260102",
            [group("fast_lio", metrics={"cpu_mean": metric(10)})],
            ref={"usable": False, "unusable_because": ["74% of the run has no fix"]})
    text = render(tmp_path)
    assert "74% of the run has no fix" in text


def test_an_unreadable_reference_report_is_reported_rather_than_assumed(tmp_path):
    d = dataset(tmp_path, "DS_20260101",
                [group("fast_lio", metrics={"cpu_mean": metric(10)})])
    (d / "gnss_ref.json").write_text("{ truncated")
    assert "report unreadable" in summary.reference_verdicts(tmp_path)["DS_20260101"]


# --- provenance --------------------------------------------------------------------


def test_a_non_realtime_playback_rate_is_declared(tmp_path):
    g = group("fast_lio", metrics={"lat_p99_ms": metric(30)})
    g["fingerprint"]["rate"] = 5.0
    dataset(tmp_path, "DS_20260101", [g])
    text = render(tmp_path)
    assert "Playback rate: 5x" in text


def test_runs_at_one_rate_need_no_warning(tmp_path):
    dataset(tmp_path, "DS_20260101",
            [group("fast_lio", metrics={"lat_p99_ms": metric(30)})])
    assert "Playback rate" not in render(tmp_path)


def test_mixed_playback_rates_are_called_out_as_not_one_population(tmp_path):
    a = group("fast_lio", metrics={"lat_p99_ms": metric(30)})
    b = group("fast_lio", metrics={"lat_p99_ms": metric(20)})
    b["fingerprint"]["rate"] = 5.0
    dataset(tmp_path, "DS_20260101", [a])
    dataset(tmp_path, "DS_20260102", [b])
    assert "do **not** share one rate" in render(tmp_path)


# --- shape -------------------------------------------------------------------------


def test_the_shipped_preset_is_not_spelled_out_but_a_variant_is(tmp_path):
    dataset(tmp_path, "DS_20260101", [group("fuselm", preset="default"),
                                      group("fuselm", preset="pointlio")])
    text = render(tmp_path)
    assert "`fuselm`" in text and "`fuselm/pointlio`" in text
    assert "fuselm/default" not in text


def test_the_detail_tables_keep_the_range_a_median_cannot_show(tmp_path):
    dataset(tmp_path, "DS_20260101",
            [group("bievr_lio", metrics={"path_len_m": metric(7130, 7129, 7132)})])
    assert "7130 [7129–7132]" in render(tmp_path)


def test_the_unmeasurable_columns_of_the_plan_are_named(tmp_path):
    dataset(tmp_path, "DS_20260101", [group("fast_lio")])
    text = render(tmp_path)
    for column in ("MME", "Plane RMSE", "Thickness", "Lat p99 @ORIN"):
        assert column in text


def test_an_empty_results_tree_says_what_to_run(tmp_path):
    assert "aggregate" in summary.render([], tmp_path)


def test_an_unreadable_stats_file_is_skipped_not_fatal(tmp_path, capsys):
    dataset(tmp_path, "GOOD_20260101", [group("fast_lio")])
    bad = tmp_path / "BAD_20260102"
    bad.mkdir()
    (bad / "stats.json").write_text("{ truncated")
    assert [d for d, _ in summary.load(tmp_path)] == ["GOOD_20260101"]
    assert "skipping" in capsys.readouterr().err
