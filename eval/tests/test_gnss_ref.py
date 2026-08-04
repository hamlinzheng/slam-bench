import math

import pytest

import gnss_ref


# A mid-latitude fixture, not a real site. The latitude is what matters: the whole point
# of doing ECEF -> ENU properly is that a single-radius approximation is 0.17% off at 43
# degrees, and the longitude is arbitrary.
LAT0, LON0, ALT0 = 43.0, 0.0, 300.0


def fix(t, dlat=0.0, dlon=0.0, dalt=0.0):
    return (t, LAT0 + dlat, LON0 + dlon, ALT0 + dalt)


def test_enu_origin_is_the_first_fix():
    e = gnss_ref.to_enu([fix(0), fix(1, dlat=0.001)])
    assert e[0][1:] == (0.0, 0.0, 0.0)


def test_enu_axes_point_east_north_up():
    east, north, up = (
        gnss_ref.to_enu([fix(0), fix(1, dlon=0.001)])[1],
        gnss_ref.to_enu([fix(0), fix(1, dlat=0.001)])[1],
        gnss_ref.to_enu([fix(0), fix(1, dalt=10.0)])[1],
    )
    assert east[1] > 0 and abs(east[2]) < 1e-3       # +lon -> +E, no N
    assert north[2] > 0 and abs(north[1]) < 1e-3     # +lat -> +N, no E
    assert up[3] == pytest.approx(10.0, abs=1e-6)    # +alt -> +U, metre for metre


def test_enu_uses_the_local_radius_not_the_equatorial_one():
    # One degree of latitude at 43 N is ~111.1 km along the meridian. Scaling by the
    # equatorial radius instead would give ~111.3 km — 0.17% high, which is 12 m over a
    # 7 km route, the size of the errors this reference is meant to resolve.
    north = gnss_ref.to_enu([fix(0), fix(1, dlat=1.0)])[1][2]
    assert north == pytest.approx(111_100, rel=2e-4)
    assert north < math.radians(1.0) * 6378137.0     # strictly below the equatorial value


def test_gap_stats_reports_the_hole_not_just_the_rate():
    # 100 samples at 1 Hz with one 30 s hole: a fix *rate* would call this 77% and say
    # nothing about the stretch of road that has no reference at all.
    times = [float(i) for i in range(50)] + [float(80 + i) for i in range(50)]
    g = gnss_ref.gap_stats(times)
    assert g["gaps"] == [pytest.approx(31.0)]
    assert g["gap_total_s"] == pytest.approx(31.0)
    assert g["median_dt_s"] == pytest.approx(1.0)


def test_gap_stats_ignores_ordinary_sample_spacing():
    assert gnss_ref.gap_stats([float(i) for i in range(100)])["gaps"] == []


def test_approximated_covariance_is_flagged_untrustworthy():
    # Measured on this rig: an NMEA-derived fix advertised 8.5 cm sigma where the truth
    # was metres, because the driver back-computes it from HDOP.
    enu = gnss_ref.to_enu([fix(0), fix(1, dlat=0.001)])
    r = gnss_ref.assess(enu, [1, 1], [gnss_ref.COVARIANCE_APPROXIMATED] * 2)
    assert r["covariance"]["trustworthy"] is False


def test_known_covariance_is_not_flagged():
    enu = gnss_ref.to_enu([fix(0), fix(1, dlat=0.001)])
    assert gnss_ref.assess(enu, [1, 1], [3, 3])["covariance"]["trustworthy"] is True


def test_written_tum_carries_an_identity_quaternion(tmp_path):
    # The dummy is load-bearing: a motion-derived yaw here would make -r full return a
    # plausible, meaningless number instead of an obviously absent one.
    out = tmp_path / "gnss_ref.tum"
    gnss_ref.write_tum(out, gnss_ref.to_enu([fix(0), fix(1, dlat=0.001)]))
    for line in out.read_text().splitlines():
        assert line.split()[4:] == ["0", "0", "0", "1"]


def test_assess_reports_closure_split_by_axis(tmp_path):
    # Same split as aggregate.py's end_pos: on a flat route the 3D scalar is dominated by
    # the vertical component and ranks runs by the axis with the weakest constraint.
    enu = gnss_ref.to_enu([fix(0), fix(1, dlat=0.001, dalt=5.0)])
    r = gnss_ref.assess(enu, [1, 1], [3, 3])
    assert r["closure_horiz_m"] == pytest.approx(111.1, rel=1e-3)
    # Not exactly 5: the tangent plane drops away from the ellipsoid, so a point 111 m
    # north sits ~1 mm below the origin's up axis. Real, and the reason the tolerance is
    # millimetres rather than machine epsilon.
    assert r["closure_vert_m"] == pytest.approx(5.0, abs=2e-3)


def moving_then_parked(n_move=200, n_park=200, dt=0.2, noise=0.0):
    """A run that drives east and then stops, with optional noise while parked."""
    import random

    rng = random.Random(0)
    fixes = []
    for i in range(n_move):
        fixes.append(fix(i * dt, dlon=i * 1e-4))
    lon_end = (n_move - 1) * 1e-4
    for j in range(n_park):
        jitter = rng.gauss(0, noise) if noise else 0.0
        fixes.append(fix((n_move + j) * dt, dlon=lon_end, dalt=jitter))
    return fixes


def test_stationary_spans_find_the_stop_and_not_the_drive():
    enu = gnss_ref.to_enu(moving_then_parked())
    spans = gnss_ref.stationary_spans(enu)
    assert len(spans) == 1
    a, b = spans[0]
    assert a >= 195 and b == len(enu) - 1


def test_noise_floor_is_measured_where_the_vehicle_stopped():
    # 0.5 m of jitter while parked must come back as ~0.5 m, and the 1400 m of driving
    # before it must not contribute — that was the defect a rolling median over the
    # moving track had, reporting 8.2 m east where the truth was the driving.
    enu = gnss_ref.to_enu(moving_then_parked(noise=0.5))
    nf = gnss_ref.stationary_noise(enu, windows=(10.0,))
    assert nf["n_spans"] == 1
    assert nf["w10s"]["vertical"]["rmse_m"] == pytest.approx(0.5, rel=0.25)
    # Sub-millimetre rather than zero: the stop is 1400 m east of the ENU origin, so the
    # altitude jitter runs along that point's normal, not the origin's up axis, and
    # leaks jitter x 1400/R ~ 1e-4 m into east. Tangent-plane geometry, not a defect.
    assert nf["w10s"]["east"]["rmse_m"] == pytest.approx(0.0, abs=1e-3)


def test_noise_floor_says_why_when_the_vehicle_never_stopped():
    enu = gnss_ref.to_enu([fix(i * 0.2, dlon=i * 1e-4) for i in range(300)])
    nf = gnss_ref.stationary_noise(enu)
    assert nf["n_spans"] == 0
    assert "stationary" in nf["reason"]


def test_a_window_longer_than_any_stop_is_simply_absent():
    # No fabricated figure for a timescale the data cannot support.
    enu = gnss_ref.to_enu(moving_then_parked(n_park=100, noise=0.3))  # 20 s parked
    nf = gnss_ref.stationary_noise(enu, windows=(10.0, 120.0))
    assert "w10s" in nf
    assert "w120s" not in nf
