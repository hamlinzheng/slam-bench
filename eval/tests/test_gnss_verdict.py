import pytest

import gnss_ref


def track(times, spacing_deg=1e-5):
    """A straight run of fixes at the given times."""
    return [(t, 43.0 + i * spacing_deg, 0.0, 300.0) for i, t in enumerate(times)]


def test_a_continuous_track_is_usable():
    enu = gnss_ref.to_enu(track([i * 0.2 for i in range(1000)]))
    r = gnss_ref.assess(enu, [1] * len(enu), [3] * len(enu))
    assert r["usable"] is True
    assert r["unusable_because"] == []


def test_a_track_with_a_hole_over_most_of_the_run_is_not_a_reference():
    # The underground case: the receiver holds lock at the portal, loses it inside, and
    # the resulting 780 m "reference" would be drawn against a 2849 m trajectory.
    before = [i * 0.2 for i in range(400)]
    after = [before[-1] + 900 + i * 0.2 for i in range(400)]
    enu = gnss_ref.to_enu(track(before + after))
    r = gnss_ref.assess(enu, [1] * len(enu), [3] * len(enu))
    assert r["usable"] is False
    assert "no fix" in r["unusable_because"][0]


def test_a_handful_of_fixes_is_not_a_reference():
    enu = gnss_ref.to_enu(track([i * 0.2 for i in range(20)]))
    r = gnss_ref.assess(enu, [1] * len(enu), [3] * len(enu))
    assert r["usable"] is False
    assert "fixes" in r["unusable_because"][0]


def test_short_dropouts_do_not_disqualify_a_track():
    # Four gaps of ~3 s in a 24 minute run — measured on a dataset whose track is a
    # perfectly good reference.
    times = []
    t = 0.0
    for i in range(6000):
        t += 3.0 if i and i % 1500 == 0 else 0.2
        times.append(t)
    enu = gnss_ref.to_enu(track(times))
    assert gnss_ref.assess(enu, [1] * len(enu), [3] * len(enu))["usable"] is True


def test_verdict_survives_json_round_trip():
    import json

    enu = gnss_ref.to_enu(track([i * 0.2 for i in range(1000)]))
    r = json.loads(json.dumps(gnss_ref.assess(enu, [1] * len(enu), [3] * len(enu))))
    assert r["usable"] is True
