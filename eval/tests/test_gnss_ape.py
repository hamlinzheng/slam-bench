import math

import numpy as np
import pytest

import gnss_ape


def _rot_z(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def test_umeyama_recovers_a_known_yaw_and_offset():
    # The case this exists for: system frames differ from ENU by an unknown yaw —
    # Point-LIO gravity-aligns and leaves yaw free, measured 169.5 deg from FAST-LIO.
    src = np.random.default_rng(0).normal(size=(200, 3)) * 50
    truth_r, truth_t = _rot_z(169.5), np.array([12.0, -3.0, 4.0])
    dst = (truth_r @ src.T).T + truth_t
    r, t = gnss_ape.umeyama(src, dst)
    assert np.allclose(r, truth_r, atol=1e-9)
    assert np.allclose(t, truth_t, atol=1e-9)


def test_umeyama_does_not_correct_scale():
    # A trajectory 10% short must stay 10% short: LiDAR range and IMU acceleration are
    # both metric, so scale is observable and fitting it would hide a real error.
    src = np.random.default_rng(1).normal(size=(200, 3)) * 50
    r, t = gnss_ape.umeyama(src, src * 1.10)
    aligned = (r @ src.T).T + t
    assert np.linalg.norm(aligned, axis=1).mean() == pytest.approx(
        np.linalg.norm(src, axis=1).mean(), rel=0.02
    )


def test_umeyama_does_not_mirror_a_degenerate_cloud():
    # Points on a plane leave the third singular vector free; without the det guard the
    # SVD can return a reflection, which would silently flip the trajectory.
    src = np.random.default_rng(2).normal(size=(200, 3)) * 50
    src[:, 2] = 0.0
    r, _ = gnss_ape.umeyama(src, (_rot_z(30) @ src.T).T)
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_association_drops_pairs_outside_the_time_tolerance():
    ref_t = np.arange(0.0, 10.0, 1.0)
    ref_p = np.column_stack([ref_t, ref_t, ref_t])
    # Estimates land 0.4 s after each reference sample: inside a 0.5 s tolerance, and
    # outside a 0.1 s one.
    ts = ref_t + 0.4
    ps = ref_p.copy()
    assert len(gnss_ape.associate(ts, ps, ref_t, ref_p, 0.5)[0]) == len(ts)
    assert len(gnss_ape.associate(ts, ps, ref_t, ref_p, 0.1)[0]) == 0


def test_association_picks_the_nearest_reference_sample_not_the_next_one():
    ref_t = np.array([0.0, 1.0, 2.0])
    ref_p = np.array([[0.0, 0, 0], [10.0, 0, 0], [20.0, 0, 0]])
    src, dst = gnss_ape.associate(np.array([0.9]), np.zeros((1, 3)), ref_t, ref_p, 0.5)
    assert dst[0][0] == pytest.approx(10.0)


def _write_tum(path, xyz, t0=0.0, dt=0.1):
    path.write_text(
        "".join(
            "{:.6f} {} {} {} 0 0 0 1\n".format(t0 + i * dt, *p) for i, p in enumerate(xyz)
        )
    )
    return path


def test_ape_is_zero_for_a_trajectory_that_only_differs_by_a_rigid_transform(tmp_path):
    # The frame convention must not count as error: that is what the alignment removes.
    pts = np.random.default_rng(3).normal(size=(300, 3)) * 20
    ref = _write_tum(tmp_path / "ref.tum", pts)
    moved = (_rot_z(75) @ pts.T).T + np.array([100.0, -40.0, 7.0])
    est = _write_tum(tmp_path / "est.tum", moved)
    r = gnss_ape.ape(ref, est)
    assert r["gnss_ape_rmse_m"] == pytest.approx(0.0, abs=1e-6)
    assert r["gnss_ape_pairs"] == 300


def test_ape_splits_horizontal_from_vertical(tmp_path):
    # A flat circuit in XY, so the alignment is well conditioned — a straight reference
    # leaves rotation about its own axis free — plus a vertical wander on the estimate.
    # A sinusoid on purpose: a constant offset or a linear ramp is a rigid transform (the
    # ramp is a rotation, and on a straight line it also rescales), so alignment removes
    # both and neither tests the split. This is the shape actually measured on
    # an outdoor route where every baseline oscillated tens of metres vertically over
    # ground with 14 m of true relief.
    n = 400
    a = np.linspace(0, 2 * np.pi, n)
    pts = np.column_stack([200 * np.cos(a), 200 * np.sin(a), np.zeros(n)])
    ref = _write_tum(tmp_path / "ref.tum", pts)
    bump = np.column_stack([np.zeros(n), np.zeros(n), 5.0 * np.sin(3 * a)])
    r = gnss_ape.ape(ref, _write_tum(tmp_path / "est.tum", pts + bump))
    # A 5 m sinusoid has RMS 5/sqrt(2).
    assert r["gnss_ape_vert_rmse_m"] == pytest.approx(5 / math.sqrt(2), rel=0.05)
    assert r["gnss_ape_horiz_rmse_m"] < 0.1
    assert r["gnss_ape_max_m"] == pytest.approx(5.0, rel=0.05)


def test_ape_absorbs_a_rigid_offset_rather_than_charging_it_as_vertical_error(tmp_path):
    n = 400
    a = np.linspace(0, 2 * np.pi, n)
    pts = np.column_stack([200 * np.cos(a), 200 * np.sin(a), np.zeros(n)])
    ref = _write_tum(tmp_path / "ref.tum", pts)
    est = _write_tum(tmp_path / "est.tum", pts + np.array([0.0, 0.0, 5.0]))
    assert gnss_ape.ape(ref, est)["gnss_ape_rmse_m"] == pytest.approx(0.0, abs=1e-6)


def test_ape_returns_nothing_rather_than_a_number_it_cannot_support(tmp_path):
    pts = np.random.default_rng(4).normal(size=(10, 3))
    ref = _write_tum(tmp_path / "ref.tum", pts)
    est = _write_tum(tmp_path / "est.tum", pts)
    assert gnss_ape.ape(ref, est) == {}


def test_ape_drops_a_trajectory_that_does_not_overlap_the_reference_in_time(tmp_path):
    pts = np.random.default_rng(5).normal(size=(300, 3)) * 20
    ref = _write_tum(tmp_path / "ref.tum", pts, t0=0.0)
    est = _write_tum(tmp_path / "est.tum", pts, t0=10_000.0)
    assert gnss_ape.ape(ref, est) == {}


def test_end_error_is_the_last_pose_not_the_average(tmp_path):
    # What M-DRIFT wants: how far off is it when it finishes. Clean for most of the run
    # and wrong at the end, so an RMS would understate it and only the endpoint sees it.
    n = 400
    a = np.linspace(0, 2 * np.pi, n)
    pts = np.column_stack([200 * np.cos(a), 200 * np.sin(a), np.zeros(n)])
    ref = _write_tum(tmp_path / "ref.tum", pts)
    drift = np.zeros((n, 3))
    drift[-20:, 2] = np.linspace(0, 12, 20)
    r = gnss_ape.ape(ref, _write_tum(tmp_path / "est.tum", pts + drift))
    assert r["gnss_end_err_vert_m"] > 5 * r["gnss_ape_vert_rmse_m"]
    assert r["gnss_end_err_horiz_m"] < 1.0


def test_every_ape_key_has_a_null_counterpart():
    # A run that could not be measured must read null on every key, never 0 — which would
    # claim a perfect run.
    n = 400
    a = np.linspace(0, 2 * np.pi, n)
    pts = np.column_stack([200 * np.cos(a), 200 * np.sin(a), np.zeros(n)])
    import tempfile, pathlib
    d = pathlib.Path(tempfile.mkdtemp())
    produced = gnss_ape.ape(_write_tum(d / "r.tum", pts), _write_tum(d / "e.tum", pts))
    assert set(produced) == set(gnss_ape.NO_APE)
    assert all(v is None for v in gnss_ape.NO_APE.values())


# --- the opening alignment ---------------------------------------------------------

import numpy as np  # noqa: E402

import plot_gnss  # noqa: E402


def _straightish(n=600, rise=0.005):
    """A route heading east while climbing gently, as a ground vehicle does."""
    t = np.arange(n, dtype=float)
    return np.stack([t, 0.05 * t, rise * t], axis=1)


def test_the_opening_correction_never_tips_the_trajectory(tmp_path):
    # The whole point: pitch and roll come from the whole-run fit, which has kilometres to
    # determine them with. The opening may turn the result and shift it, nothing else.
    ref = _straightish()
    rot, shift = plot_gnss._opening_correction(ref + [3.0, -2.0, 7.0], ref, 40)
    # A yaw-only rotation leaves the vertical axis exactly where it was.
    assert rot[2, 2] == pytest.approx(1.0)
    assert rot[2, :2] == pytest.approx([0.0, 0.0])
    assert rot[:2, 2] == pytest.approx([0.0, 0.0])


def test_the_opening_correction_removes_a_pure_yaw(tmp_path):
    ref = _straightish()
    th = np.radians(30.0)
    spin = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    turned = (spin @ ref.T).T + [10.0, -5.0, 2.0]
    rot, shift = plot_gnss._opening_correction(turned, ref, 60)
    assert (rot @ turned.T).T + shift == pytest.approx(ref, abs=1e-6)


def test_a_vertical_offset_is_removed_without_fitting_a_slope(tmp_path):
    # Up is defined by gravity and both frames already agree on it; the opening is only
    # allowed to say where zero is, not which way is up.
    ref = _straightish()
    rot, shift = plot_gnss._opening_correction(ref + [0.0, 0.0, 12.0], ref, 50)
    assert np.allclose(rot, np.eye(3), atol=1e-9)
    assert shift[2] == pytest.approx(-12.0, abs=1e-6)


def test_the_correction_does_not_mirror_a_poorly_spread_opening(tmp_path):
    # Kabsch without the reflection guard can "fit" by mirroring, which is not a pose.
    ref = _straightish()
    rot, _ = plot_gnss._opening_correction(ref * [1, -1, 1], ref, 30)
    assert np.linalg.det(rot) == pytest.approx(1.0)
