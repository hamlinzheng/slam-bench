import sys
import types

import pytest

import aggregate


def _record_frames():
    """Import the recorder on a host that has no ROS.

    Only the header parser is testable off-robot, but it is the one piece whose failure
    mode is silent: a wrong offset or endianness yields plausible-looking stamps and
    every latency below becomes fiction.
    """
    if "rospy" not in sys.modules:
        sys.modules["rospy"] = types.ModuleType("rospy")
        nav = types.ModuleType("nav_msgs")
        nav.msg = types.ModuleType("nav_msgs.msg")
        nav.msg.Odometry = object
        sys.modules["nav_msgs"] = nav
        sys.modules["nav_msgs.msg"] = nav.msg
    import record_frames

    return record_frames


def test_header_stamp_is_read_off_the_wire_without_deserializing_the_payload():
    recorder = _record_frames()
    # std_msgs/Header on the wire: uint32 seq, uint32 secs, uint32 nsecs, then frame_id
    # and, behind it, the point data this must never touch. Little-endian.
    #   seq   = 7          -> 07 00 00 00
    #   secs  = 1731912345 -> 0x673AE299 -> 99 e2 3a 67
    #   nsecs = 100000000  -> 0x05F5E100 -> 00 e1 f5 05
    wire = bytes.fromhex("07000000") + bytes.fromhex("99e23a67") + bytes.fromhex("00e1f505")
    payload = b"\x04\x00\x00\x00body" + b"\xff" * 4096
    assert recorder.parse_header_stamp(wire + payload) == pytest.approx(1731912345.1)


def write_tum(path, n):
    """n poses one metre apart — only the count matters to the tests here."""
    path.write_text("".join("{:.6f} {} 0 0 0 0 0 1\n".format(i * 0.1, i) for i in range(n)))
    return path


def write_events(path, events):
    """events: [(kind, stamp, wall), ...] — written in the order given.

    Order matters to one test: record_frames.py writes from two callbacks, so `in` and
    `out` rows interleave however the two topics happened to fire.
    """
    path.write_text(
        "kind,stamp,wall\n"
        + "".join("{},{:.9f},{:.6f}\n".format(k, s, w) for k, s, w in events)
    )
    return path


# A scan is stamped when it begins; the odometry it produces is stamped at the last point
# in the sweep, a hair short of where the next scan begins. That gap is the whole margin
# the stamp match has to work with, so the fixture reproduces it rather than rounding it
# away. Stamps always advance by SENSOR_PERIOD — only the wall clock is compressed by
# faster playback.
SENSOR_PERIOD = 0.1
SWEEP_END = 0.0998


def steady(n, compute_s, period_s=SENSOR_PERIOD, start_stamp=1000.0, start_wall=5000.0):
    """A run of n frames fed every `period_s` of wall clock, `compute_s` of work each."""
    events, finished = [], None
    for i in range(n):
        stamp = start_stamp + i * SENSOR_PERIOD
        wall = start_wall + i * period_s
        events.append(("in", stamp, wall))
        began = wall if finished is None else max(wall, finished)
        finished = began + compute_s
        events.append(("out", stamp + SWEEP_END, finished))
    return events


# --- pairing ---------------------------------------------------------------------


def test_outputs_consume_inputs_in_order(tmp_path):
    events = write_events(tmp_path / "e.csv", steady(3, compute_s=0.04))
    ins, outs = aggregate.read_frame_events(events)
    pairs, skipped, unmatched = aggregate.pair_frames(ins, outs, SENSOR_PERIOD)
    assert len(pairs) == 3
    assert (skipped, unmatched) == (0, 0)
    # 40 ms of work every 100 ms: each frame is idle-waiting when it arrives.
    assert [round(p[1], 6) for p in pairs] == [40.0] * 3
    assert [p[2] for p in pairs] == [False] * 3


def test_scan_jitter_narrower_than_the_sweep_does_not_shift_the_pairing(tmp_path):
    """The regression that a stamp-interval match got wrong on real data.

    Scans are not evenly spaced: measured on MID-360 the interval between input stamps
    wanders over 0.086–0.115 s. Whenever the next scan begins sooner than the sweep the
    previous odometry was stamped at, that odometry's stamp lands past it — and a rule
    that asks "which scan's interval is this stamp inside" hands it to a scan the system
    has not processed yet, then finds the next output has nowhere to go. On an 11 729
    frame NORCAT run that misattributed 1 514 frames and claimed a 13 % frame drop where
    the true count was zero.
    """
    events, wall = [], 5000.0
    stamp = 1000.0
    gaps = [0.086, 0.115, 0.092, 0.108, 0.099, 0.087, 0.114]
    for gap in gaps:
        events.append(("in", stamp, wall))
        events.append(("out", stamp + SWEEP_END, wall + 0.04))
        stamp += gap
        wall += gap
    ins, outs = aggregate.read_frame_events(write_events(tmp_path / "e.csv", events))
    pairs, skipped, unmatched = aggregate.pair_frames(ins, outs, SENSOR_PERIOD)
    assert len(pairs) == len(gaps)
    assert (skipped, unmatched) == (0, 0)
    assert [round(p[1], 6) for p in pairs] == [40.0] * len(gaps)


def test_scans_consumed_without_a_pose_are_walked_past(tmp_path):
    """FAST-LIO publishes nothing while its IMU initialises — measured, the first 3 scans.

    Those scans are skipped, not paired, and everything after them still lines up.
    """
    events = steady(8, compute_s=0.04)
    events = [e for i, e in enumerate(events) if e[0] == "in" or i // 2 >= 3]
    ins, outs = aggregate.read_frame_events(write_events(tmp_path / "e.csv", events))
    pairs, skipped, unmatched = aggregate.pair_frames(ins, outs, SENSOR_PERIOD)
    assert len(pairs) == 5
    assert (skipped, unmatched) == (3, 0)
    assert [round(p[1], 6) for p in pairs] == [40.0] * 5


def test_outputs_with_no_scan_left_to_attribute_them_to_are_unmatched(tmp_path):
    events = write_events(
        tmp_path / "e.csv",
        [("in", 1000.0, 5000.0)]
        + [("out", 1000.05, 5000.04 + i * 0.002) for i in range(5)],
    )
    ins, outs = aggregate.read_frame_events(events)
    pairs, skipped, unmatched = aggregate.pair_frames(ins, outs, SENSOR_PERIOD)
    assert len(pairs) == 1
    assert unmatched == 4
    assert skipped == 0


def test_output_published_before_any_input_is_unmatched(tmp_path):
    events = write_events(
        tmp_path / "e.csv",
        [("out", 999.0, 4999.0), ("in", 1000.0, 5000.0), ("out", 1000.1, 5000.04)],
    )
    ins, outs = aggregate.read_frame_events(events)
    pairs, skipped, unmatched = aggregate.pair_frames(ins, outs, SENSOR_PERIOD)
    assert len(pairs) == 1
    assert unmatched == 1
    assert skipped == 0


def test_a_system_publishing_per_imu_sample_yields_no_timing_at_all(tmp_path):
    """Point-LIO with publish_odometry_without_downsample, were a preset to set it.

    One pose per scan is what pairing rests on. Ten per scan is not a frame drop and not
    a duplicate — it is a different contract, so nothing is derived rather than confident
    numbers over frames matched wrongly.
    """
    events = []
    for i in range(20):
        stamp, wall = 1000.0 + i * 0.1, 5000.0 + i * 0.1
        events.append(("in", stamp, wall))
        for k in range(10):
            events.append(("out", stamp + k * 0.01, wall + k * 0.01))
    stats = aggregate.frame_stats(write_events(tmp_path / "e.csv", events))
    assert stats["unmatched_out"] == 200
    assert stats["out_ratio"] is None
    assert stats["lat_p50_ms"] is None
    assert stats["sensor_hz"] is not None  # the input stream was still readable


def test_pairing_does_not_depend_on_the_order_rows_were_written(tmp_path):
    events = steady(5, compute_s=0.04)
    grouped = [e for e in events if e[0] == "in"] + [e for e in events if e[0] == "out"]
    interleaved = aggregate.frame_stats(write_events(tmp_path / "a.csv", events))
    separated = aggregate.frame_stats(write_events(tmp_path / "b.csv", grouped))
    assert interleaved == separated


# --- saturation ------------------------------------------------------------------


def test_latency_is_the_per_frame_cost_while_the_system_keeps_up(tmp_path):
    # 40 ms of work every 100 ms: no frame ever waits behind another.
    stats = aggregate.frame_stats(
        write_events(tmp_path / "e.csv", steady(50, compute_s=0.04))
    )
    assert round(stats["lat_p50_ms"], 3) == 40.0
    assert stats["saturated_frac"] == 0.0
    assert stats["lag_growth_ms"] == 0.0


def test_a_system_that_cannot_keep_up_shows_it_in_sat_and_lag_growth(tmp_path):
    # 150 ms of work every 100 ms: the backlog grows by 50 ms per frame forever, so the
    # latencies stop being the system's own cost. sat and lag_growth are what say so.
    stats = aggregate.frame_stats(
        write_events(tmp_path / "e.csv", steady(50, compute_s=0.15))
    )
    assert stats["saturated_frac"] == 49 / 50
    assert stats["lat_p99_ms"] > 2000
    assert stats["lag_growth_ms"] > 1000


def test_sat_separates_a_queue_from_a_slow_frame(tmp_path):
    """Both runs do 60 ms of work per frame; only the second is fed faster than that.

    Their lat_p50 differ for a reason that is nothing to do with the system, which is why
    no latency here is read without sat beside it.
    """
    unsaturated = aggregate.frame_stats(
        write_events(tmp_path / "a.csv", steady(60, compute_s=0.06, period_s=0.1))
    )
    saturated = aggregate.frame_stats(
        write_events(tmp_path / "b.csv", steady(60, compute_s=0.06, period_s=0.02))
    )
    assert unsaturated["saturated_frac"] == 0.0
    assert saturated["saturated_frac"] > 0.9
    assert round(unsaturated["lat_p50_ms"], 3) == 60.0
    assert saturated["lat_p99_ms"] > 10 * unsaturated["lat_p99_ms"]


def test_the_first_frame_never_counts_as_saturated(tmp_path):
    """It has no predecessor to have been waiting behind, however backed up the run is."""
    stats = aggregate.frame_stats(
        write_events(tmp_path / "e.csv", steady(50, compute_s=0.15))
    )
    assert stats["saturated_frac"] == 49 / 50


# --- derived quantities ----------------------------------------------------------


def test_sensor_rate_comes_off_the_stamp_axis_and_ignores_playback_speed(tmp_path):
    stats = aggregate.frame_stats(
        write_events(tmp_path / "e.csv", steady(30, compute_s=0.005, period_s=0.05))
    )
    # Stamps still advance 0.1 s per frame in `steady`; only the wall clock is compressed,
    # so the sensor's own rate is unmoved and the ratio is what reports the 2x.
    assert round(stats["sensor_hz"], 6) == 10.0
    assert round(stats["rate_actual"], 6) == 2.0


def test_parallelism_is_cpu_time_over_wall_time(tmp_path):
    """Two cores busy for 50 ms of wall clock bills 100 ms of CPU.

    Neither input says this on its own: the same 100 ms of CPU could be one core for
    100 ms, which is the distinction between "computes less" and "uses fewer cores".
    """
    write_tum(tmp_path / "trajectory.tum", 2)
    # 2 samples of 100% over 1 s each = 2 CPU-seconds over 2 frames = 1000 ms/frame.
    (tmp_path / "resource.csv").write_text(
        "wall_s,cpu_pct,rss_mb\n1.000,100,10\n2.000,100,10\n"
    )
    write_events(tmp_path / "frame_events.csv", steady(2, compute_s=0.5))
    derived = aggregate._derive(tmp_path)
    assert derived["cpu_ms_per_frame"] == pytest.approx(1000.0)
    assert derived["lat_p50_ms"] == pytest.approx(500.0)
    assert derived["parallelism"] == pytest.approx(2.0)


def test_parallelism_is_absent_when_either_half_is(tmp_path):
    write_tum(tmp_path / "trajectory.tum", 2)
    # A trajectory but no frame trace: no wall-clock latency to divide by.
    assert aggregate._derive(tmp_path)["parallelism"] is None


def test_out_ratio_is_one_when_every_scan_produced_a_pose(tmp_path):
    stats = aggregate.frame_stats(
        write_events(tmp_path / "e.csv", steady(40, compute_s=0.04))
    )
    assert stats["out_ratio"] == 1.0


def test_out_ratio_counts_scans_that_produced_nothing(tmp_path):
    events = steady(10, compute_s=0.04)
    # Drop the odd frames' outputs — a system skipping scans to keep up.
    kept = [e for i, e in enumerate(events) if e[0] == "in" or (i // 2) % 2 == 0]
    stats = aggregate.frame_stats(write_events(tmp_path / "e.csv", kept))
    assert stats["out_ratio"] == 0.5


def test_lag_growth_needs_both_tenths_to_be_distinct(tmp_path):
    short = aggregate.frame_stats(
        write_events(tmp_path / "a.csv", steady(19, compute_s=0.15))
    )
    assert short["lag_growth_ms"] is None
    just_enough = aggregate.frame_stats(
        write_events(tmp_path / "b.csv", steady(20, compute_s=0.15))
    )
    # 20 frames -> a tenth is 2. Frames 0,1 waited 150/200 ms, frames 18,19 waited
    # 150 + 50*18 = 1050 and 1100; low medians 150 and 1050.
    assert round(just_enough["lag_growth_ms"], 3) == 900.0


def test_quantiles_are_real_observations_not_interpolated():
    assert aggregate._nearest_rank([7.0], 0.99) == 7.0
    assert aggregate._nearest_rank([1.0, 9.0], 0.50) == 1.0
    assert aggregate._nearest_rank([1.0, 9.0], 0.99) == 9.0
    hundred = [float(i) for i in range(100)]
    assert aggregate._nearest_rank(hundred, 0.99) == 98.0


# --- the three different kinds of nothing ----------------------------------------


def test_an_empty_trace_leaves_every_latency_quantity_null(tmp_path):
    assert aggregate.frame_stats(write_events(tmp_path / "e.csv", [])) == aggregate.NO_FRAMES


def test_a_run_recorded_before_this_existed_reads_as_not_measured(tmp_path):
    """Every run under results/ predating frame_events.csv must still aggregate."""
    (tmp_path / "trajectory.tum").write_text(
        "1000.0 0 0 0 0 0 0 1\n1000.1 1 0 0 0 0 0 1\n"
    )
    derived = aggregate._derive(tmp_path)
    assert derived["path_len_m"] == 1.0
    assert all(derived[key] is None for key in aggregate.NO_FRAMES)


def test_scans_that_produced_no_output_at_all_are_a_measured_zero(tmp_path):
    events = write_events(
        tmp_path / "e.csv", [("in", 1000.0 + i * 0.1, 5000.0 + i * 0.1) for i in range(5)]
    )
    stats = aggregate.frame_stats(events)
    assert stats["out_ratio"] == 0.0  # measured: the system emitted nothing
    assert stats["lat_p50_ms"] is None
    assert stats["sensor_hz"] is not None


def test_output_with_no_input_at_all_is_not_measured(tmp_path):
    events = write_events(
        tmp_path / "e.csv", [("out", 1000.0 + i * 0.1, 5000.0 + i * 0.1) for i in range(5)]
    )
    stats = aggregate.frame_stats(events)
    assert stats["out_ratio"] is None  # the input topic was wrong; nothing was measured
    assert stats["unmatched_out"] == 5
    assert stats["sensor_hz"] is None


# --- grouping --------------------------------------------------------------------


def _run(rate, **kw):
    rec = {
        "system": "fast_lio",
        "preset": "default",
        "rate": rate,
        "status": "ok",
        "preset_sha": "aaa",
        "binary_sha": "bbb",
        "system_commit": "ccc",
        "derived": {},
    }
    rec.update(kw)
    return rec


def test_runs_played_at_different_rates_are_never_one_sample():
    groups = aggregate.group_runs([_run(1.0), _run(1.0), _run(5.0)])
    assert len(groups) == 2
    assert sorted(len(g["runs"]) for g in groups) == [1, 2]
    assert all(not g["consistent"] for g in groups)


def test_runs_at_the_same_rate_still_group_together():
    groups = aggregate.group_runs([_run(1.0), _run(1.0)])
    assert len(groups) == 1
    assert groups[0]["consistent"]


def test_the_rate_warning_names_only_the_groups_played_off_speed():
    stats = [
        aggregate.group_stats(g)
        for g in aggregate.group_runs([_run(1.0), _run(5.0)])
    ]
    text = aggregate.render_text("ds", stats)
    assert "⚠ NOT 1.0x" in text
    assert "(5x)" in text
    assert "## accuracy" in text and "## real-time" in text
