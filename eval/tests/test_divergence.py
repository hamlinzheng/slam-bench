import math

import pytest

import divergence


HZ = 10.0
DT = 1.0 / HZ
# At 10 Hz a 1 m step is 10 m/s and a 10 m step is 100 m/s, either side of the 40 m/s
# limit with room to spare — so no test here depends on the exact threshold.
WALK = 1.0
LEAP = 10.0


def poses(steps, start=(0.0, 0.0, 0.0), t0=0.0):
    """[(t, x, y, z)] walking along +x, one entry per step distance in `steps`."""
    x, y, z = start
    out = [(t0, x, y, z)]
    for i, d in enumerate(steps):
        x += d
        out.append((t0 + (i + 1) * DT, x, y, z))
    return out


def feed(det, entries):
    """Push every pose; return the reason if the detector latched, else None."""
    reason = None
    for t, x, y, z in entries:
        got = det.push(t, x, y, z)
        if got is not None and reason is None:
            reason = got
    return reason


def write_tum(path, entries):
    path.write_text(
        "".join("{:.9f} {} {} {} 0 0 0 1\n".format(*e) for e in entries)
    )
    return path


def test_a_plausible_walk_is_not_diverged():
    det = divergence.Detector()
    assert feed(det, poses([WALK] * 100)) is None
    assert det.verdict is None
    assert det.diverged_at_s is None
    assert det.v_max_mps == pytest.approx(10.0)


def test_a_sustained_blow_up_is_diverged():
    det = divergence.Detector()
    assert feed(det, poses([WALK] * 20 + [LEAP] * 20)) is not None
    assert det.verdict is not None


def test_one_spurious_step_is_not_a_blow_up():
    # The whole reason the rule is sustained rather than per-step: a single bad pose must
    # not be able to kill a run online. One leap in a 2 s window is 5%, far under 50%.
    det = divergence.Detector()
    assert feed(det, poses([WALK] * 40 + [LEAP] + [WALK] * 40)) is None
    assert det.verdict is None
    # It is still seen — the run's v_max reports it even though the verdict does not.
    assert det.v_max_mps == pytest.approx(100.0)


def test_a_burst_shorter_than_half_the_window_is_not_a_blow_up():
    # Nine leaps inside a 20-step window is 45%, just under the fraction. The boundary is
    # exercised on purpose; measured runs are nowhere near it (100% against 0%).
    det = divergence.Detector()
    assert feed(det, poses([WALK] * 40 + [LEAP] * 9 + [WALK] * 40)) is None


def test_diverged_at_is_the_first_crossing_not_the_last():
    det = divergence.Detector()
    # 100 healthy steps = 10 s, then the blow-up. A 2 s window at 10 Hz spans 21 steps
    # (both ends inclusive), so the 11th leap is the one that reaches half — 1.1 s into
    # the burst, and nowhere near the end of it.
    feed(det, poses([WALK] * 100 + [LEAP] * 40))
    assert det.diverged_at_s == pytest.approx(11.1)


def test_a_late_blow_up_is_still_diverged():
    # fast_lio on OPENROAD_20260325 first goes over 967 s into a ~1036 s run.
    det = divergence.Detector()
    assert feed(det, poses([WALK] * 500 + [LEAP] * 30)) is not None
    assert det.diverged_at_s > 50.0


def test_an_onset_that_is_dense_then_intermittent_is_diverged():
    # The case that nearly cost this design a second rule: fast_lio on OPENROAD_20260325
    # has only 23% of its steps over the limit overall, so a whole-run fraction would miss
    # it. The onset is dense, and that is what the sliding window sees.
    intermittent = [LEAP if i % 4 == 0 else WALK for i in range(200)]
    det = divergence.Detector()
    assert feed(det, poses([WALK] * 50 + [LEAP] * 15 + intermittent)) is not None


def test_the_verdict_latches_and_is_reported_once():
    det = divergence.Detector()
    entries = poses([WALK] * 20 + [LEAP] * 40)
    reasons = [r for r in (det.push(*e) for e in entries) if r is not None]
    assert len(reasons) == 1


def test_a_non_finite_pose_counts_against_the_window():
    # NaN is treated as an over-limit step, not as an instant verdict — one rule, so a
    # single non-finite sample is no more fatal than a single fast one. It therefore takes
    # rather more than half a window of them: at 10 Hz, 10 NaNs against 11 surviving
    # healthy steps is 48% and does not trigger, which is the rule working as specified.
    det = divergence.Detector()
    entries = poses([WALK] * 20)
    t = entries[-1][0]
    for i in range(15):
        entries.append((t + (i + 1) * DT, float("nan"), 0.0, 0.0))
    assert feed(det, entries) is not None
    assert det.nonfinite_poses == 15
    assert "non-finite" in det.verdict


def test_a_brief_burst_of_non_finite_poses_is_not_a_verdict():
    # The other side of the same boundary, kept as a test so the choice above is not
    # silently reversed later.
    det = divergence.Detector()
    entries = poses([WALK] * 20)
    t = entries[-1][0]
    for i in range(5):
        entries.append((t + (i + 1) * DT, float("nan"), 0.0, 0.0))
    assert feed(det, entries) is None
    assert det.nonfinite_poses == 5


def test_a_non_finite_pose_does_not_poison_the_next_step():
    # The pose after an Inf must be measured against the last real one, not against Inf —
    # a difference of infinities is nan, and `nan > limit` is False, which would read as
    # a healthy step.
    det = divergence.Detector()
    entries = poses([WALK] * 5)
    t = entries[-1][0]
    entries.append((t + DT, float("inf"), 0.0, 0.0))
    entries += [(t + (i + 2) * DT, 5.0 + i, 0.0, 0.0) for i in range(5)]
    feed(det, entries)
    assert math.isfinite(det.v_max_mps)


def test_too_few_steps_to_fill_the_window_is_not_a_verdict():
    # Four steps, every one of them over the limit: below MIN_STEPS this is the opening of
    # a run, not a window, and a run must not be killed on its first frames.
    det = divergence.Detector()
    assert feed(det, poses([LEAP] * 4)) is None
    assert det.verdict is None


def test_a_repeated_timestamp_defines_no_speed_and_is_skipped():
    det = divergence.Detector()
    det.push(0.0, 0.0, 0.0, 0.0)
    assert det.push(0.0, 1000.0, 0.0, 0.0) is None
    assert det.v_max_mps is None


def test_a_backwards_timestamp_defines_no_speed_and_is_skipped():
    det = divergence.Detector()
    det.push(1.0, 0.0, 0.0, 0.0)
    assert det.push(0.5, 1000.0, 0.0, 0.0) is None
    assert det.v_max_mps is None


def test_a_trajectory_with_no_step_has_no_maximum_speed():
    # Null, never 0.0 — which would claim a stationary vehicle rather than no measurement.
    det = divergence.Detector()
    det.push(0.0, 0.0, 0.0, 0.0)
    assert det.v_max_mps is None


def test_scan_reads_a_file_and_reports_the_same_verdict(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", poses([WALK] * 20 + [LEAP] * 40))
    out = divergence.scan(tum)
    assert out["diverged_at_s"] is not None
    assert out["diverged_reason"] is not None
    assert out["v_max_mps"] == pytest.approx(100.0)


def test_scan_of_a_healthy_trajectory_reports_no_divergence(tmp_path):
    tum = write_tum(tmp_path / "trajectory.tum", poses([WALK] * 50))
    out = divergence.scan(tum)
    assert out["diverged_at_s"] is None
    assert out["diverged_reason"] is None
    assert out["nonfinite_poses"] == 0


def test_scan_tolerates_a_truncated_final_line(tmp_path):
    # A run killed mid-write leaves a partial line, and that run's verdict is the point.
    tum = tmp_path / "trajectory.tum"
    write_tum(tum, poses([WALK] * 20 + [LEAP] * 40))
    tum.write_text(tum.read_text() + "1234.5 6.7 8.9")
    assert divergence.scan(tum)["diverged_at_s"] is not None


def test_scan_of_an_empty_file_measures_nothing(tmp_path):
    tum = tmp_path / "trajectory.tum"
    tum.write_text("")
    out = divergence.scan(tum)
    assert out["v_max_mps"] is None
    assert out["diverged_at_s"] is None


def test_the_limit_is_configurable_without_touching_the_rule():
    # Same trajectory, two limits: 10 m/s steps are healthy by default and diverged under
    # a limit meant for a walking platform.
    entries = poses([WALK] * 60)
    assert feed(divergence.Detector(), entries) is None
    assert feed(divergence.Detector(max_speed_mps=2.0), entries) is not None
