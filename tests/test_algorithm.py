"""
pytest suite for the EDR core.   run:  pytest -q

Split into three groups:
  test_math_*        integer primitives against a float reference
  test_scenario_*    the must-fire / must-not-fire matrix
  test_property_*    invariants that must hold for ANY input, which is
                     where the interesting bugs actually live
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))

import pulses  # noqa: E402
from edr_core import EdrSim  # noqa: E402

TRG_LOOPA = 0b0110
TRG_LOOPB = 0b0001
G = 9.80665


def run(name, **cfg):
    fs = cfg.pop("fs", 833)
    cfg["fast_hz"] = fs          # the sim must be told the rate it is fed
    t, ax, ay, az, sp, s = pulses.render(name, fs=fs)
    res = EdrSim(cfg).run(t, ax, ay, az, sp)
    return s, res


def fired_A(res):
    return any(e.trigger_mask & TRG_LOOPA for e in res.events)


def fired_B(res):
    return any(e.trigger_mask & TRG_LOOPB for e in res.events)


# ------------------------------------------------------------ math ----
def test_math_boundary_monotonic_and_interpolated():
    sim = EdrSim()
    v = [sim.boundary_at(t) for t in range(0, 420, 5)]
    assert all(b <= a + 1e-9 for b, a in zip(v, v[1:])), "boundary must not decrease"
    assert sim.boundary_at(150) == pytest.approx(6.944, abs=0.002), \
        "the 150 ms knee is the contractual 25 km/h point"
    # linear between knees
    assert sim.boundary_at(135) == pytest.approx((6.600 + 6.944) / 2, abs=0.03)
    # clamped, never extrapolated
    assert sim.boundary_at(9999) == pytest.approx(sim.boundary_at(400), abs=1e-9)
    assert sim.boundary_at(0) == pytest.approx(sim.boundary_at(10), abs=1e-9)


def test_math_delta_v_integration_accuracy():
    """A clean haversine has dV = A*T/2 in closed form. The pipeline must
    land within 3% of it, which is the budget for LPF loss plus the
    arming delay that the seed window is there to recover."""
    fs = 833
    t = np.arange(0, 2.0, 1 / fs)
    peak_g, dur = 9.44, 0.150
    ax = -pulses.haversine(t, 0.5, dur, peak_g)
    expect = peak_g * G * dur / 2
    res = EdrSim().run(t, ax, np.zeros_like(t), np.zeros_like(t),
                       pulses._speed_from_accel(ax, fs, 40))
    assert res.events, "clean 25 km/h pulse must fire"
    got = res.events[0].dv_res
    assert got == pytest.approx(expect, rel=0.03), f"dV {got:.3f} vs {expect:.3f}"


def test_math_pdof_matches_atan2():
    """PDOF is computed with an integer atan2; it must agree with the
    float reference to better than a degree."""
    fs = 833
    for deg in (0, 15, 30, 45, 60, 90):
        t = np.arange(0, 1.5, 1 / fs)
        r = math.radians(deg)
        ax = -pulses.haversine(t, 0.4, 0.12, 14.0 * math.cos(r))
        ay = -pulses.haversine(t, 0.4, 0.12, 14.0 * math.sin(r))
        res = EdrSim().run(t, ax, ay)
        assert res.events, f"no event at {deg} deg"
        assert res.events[0].pdof_deg == pytest.approx(deg, abs=1.0)


# -------------------------------------------------------- scenarios ---
@pytest.mark.parametrize("name", pulses.all_names())
def test_scenario_matrix(name):
    s, res = run(name)
    assert fired_A(res) == s.expect_loopA, \
        f"{name}: Loop A expected {s.expect_loopA}, got {fired_A(res)} ({s.note})"
    assert fired_B(res) == s.expect_loopB, \
        f"{name}: Loop B expected {s.expect_loopB}, got {fired_B(res)} ({s.note})"


@pytest.mark.parametrize("fs", [416, 833, 1667])
@pytest.mark.parametrize("name", ["truck_frontal_rigid_40kph", "hard_braking_0g8",
                                  "marginal_25kph_150ms", "sudden_decel_3g5_1100ms"])
def test_scenario_sample_rate_independent(name, fs):
    s, res = run(name, fs=fs)
    assert fired_A(res) == s.expect_loopA
    assert fired_B(res) == s.expect_loopB


@pytest.mark.parametrize("seed", range(1, 9))
def test_scenario_hard_braking_never_fires_loop_a(seed):
    """The single most important negative case: 0.8 g for 1.5 s banks more
    total dV than the 25 km/h criterion, and must still never be recorded
    as a collision."""
    t, ax, ay, az, sp, s = pulses.render("hard_braking_0g8", seed=seed)
    res = EdrSim().run(t, ax, ay, az, sp)
    assert not fired_A(res)
    assert fired_B(res), "but it IS a mandated sudden-deceleration event"
    total = abs(np.trapezoid(ax, t)) * G
    assert total > 6.944, f"the scenario only banks {total:.2f} m/s - it stopped being a test"


def test_scenario_multi_event_sequencing():
    s, res = run("multi_impact_800ms")
    a = [e for e in res.events if e.trigger_mask & TRG_LOOPA]
    assert len(a) == 2, f"expected two sub-events, got {len(a)}"
    assert a[0].seq_index == 0 and a[1].seq_index == 1
    assert 0.6 < (a[1].t_s - a[0].t_s) < 1.0


def test_scenario_grade_immunity():
    """Constant speed on a 6% downgrade. VSS sees no deceleration; only
    the accelerometer fallback would be fooled."""
    s, res = run("downgrade_6pct_constant_speed")
    assert not fired_B(res)
    assert np.nanmax(res.decel) < 0.5


def test_scenario_clipping_is_flagged():
    s, res = run("truck_frontal_rigid_40kph")
    e = res.events[0]
    assert e.dv_is_lower_bound, \
        "a 17.5 g pulse into a +/-16 g sensor MUST be flagged as clipped"


def test_scenario_srs_locks_and_loop_b_does_not():
    t, ax, ay, az, sp, s = pulses.render("sudden_decel_3g5_1100ms")
    res = EdrSim().run(t, ax, ay, az, sp)
    b = [e for e in res.events if e.trigger_mask & TRG_LOOPB]
    assert b and not any(e.locked for e in b), \
        "AIS-220 5.3.2: sudden-deceleration events are recorded, not locked"


def test_scenario_conformant_mode_never_locks_without_srs():
    """With lock_on_delta_v disabled the build is strictly AIS-220
    conformant - and on a truck with no SRS, nothing ever locks."""
    t, ax, ay, az, sp, s = pulses.render("truck_frontal_rigid_40kph")
    res = EdrSim({"lock_on_delta_v": 0}).run(t, ax, ay, az, sp)
    assert res.events
    assert not any(e.locked for e in res.events)


# -------------------------------------------------------- properties --
def test_property_no_events_on_silence():
    fs = 833
    t = np.arange(0, 5, 1 / fs)
    z = np.zeros_like(t)
    res = EdrSim().run(t, z, z, z, np.full_like(t, 50.0))
    assert not res.events


def test_property_bias_does_not_leak_into_delta_v():
    """A 0.05 g static mounting offset held for 20 s must not integrate
    into a spurious dV. Without a gated bias tracker this is 9.8 m/s of
    phantom velocity - comfortably past every threshold in the file."""
    fs = 833
    t = np.arange(0, 20, 1 / fs)
    ax = np.full_like(t, -0.05)
    res = EdrSim().run(t, ax, np.zeros_like(t), np.zeros_like(t),
                       np.full_like(t, 50.0))
    assert not res.events
    assert np.abs(res.dv_main).max() < 0.5, \
        f"phantom dV {np.abs(res.dv_main).max():.2f} m/s from a static offset"


def test_property_delta_v_never_exceeds_physics():
    """Recorded dV can never exceed the true integral of the input."""
    for name in pulses.all_names("crash"):
        t, ax, ay, az, sp, s = pulses.render(name)
        res = EdrSim().run(t, ax, ay, az, sp)
        bound = np.abs(np.trapezoid(np.hypot(ax, ay), t)) * G
        for e in res.events:
            assert e.dv_res <= bound * 1.05, \
                f"{name}: dV {e.dv_res:.2f} exceeds the physical bound {bound:.2f}"


def test_property_no_duplicate_events():
    """One physical event, one record. Regression guard for the Loop B
    edge being consumed once per fast tick instead of once per slow tick."""
    for name in pulses.all_names():
        t, ax, ay, az, sp, s = pulses.render(name)
        res = EdrSim().run(t, ax, ay, az, sp)
        ts = sorted(e.t_s for e in res.events)
        for a, b in zip(ts, ts[1:]):
            assert b - a > 0.05, f"{name}: duplicate records {a:.3f} / {b:.3f}"


def test_property_events_are_bounded():
    for name in pulses.all_names():
        t, ax, ay, az, sp, s = pulses.render(name)
        res = EdrSim().run(t, ax, ay, az, sp)
        assert len(res.events) <= 4, f"{name}: {len(res.events)} events, buffer holds 4"


def test_property_invalid_calibration_is_rejected():
    with pytest.raises(ValueError):
        EdrSim({"dv_safing_win_ms": 5000})        # window exceeds the ring
    with pytest.raises(ValueError):
        EdrSim({"decel_rearm_mm_s2": 9000})       # hysteresis inverted
    with pytest.raises(ValueError):
        EdrSim({"fast_hz": 5})                    # out of range
    with pytest.raises(ValueError):
        EdrSim(boundary=[(100, 5000), (50, 6000)])  # not increasing in time
    with pytest.raises(KeyError):
        EdrSim({"no_such_field": 1})


def test_property_annexure4_record_is_quantised():
    """The regulated record is +/-1.5 g at 0.1 g. Anything outside that
    saturates and must say so."""
    s, res = run("truck_frontal_rigid_40kph")
    rec = res.record
    assert rec and len(rec["a_long_g"]) > 0
    q = rec["a_long_g"]
    assert np.all(np.abs(q) <= 1.5 + 1e-9)
    assert np.allclose(q * 10, np.round(q * 10)), "not on a 0.1 g grid"
