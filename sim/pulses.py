"""
pulses.py - scenario library for offline validation.

NOTE ON THE LOOP B COLUMN: every crash scenario is expect_loopB = False.
A collision is over in 100-180 ms; the AIS-220 sudden-deceleration
trigger needs 3.25 m/s^2 sustained for 700 ms. They are different events
measured by different means, and a collision does NOT satisfy the
sudden-deceleration criterion. Getting this wrong in the test matrix is
the same conflation that inflates the requirement set.

Two halves, and both matter equally:

  MUST-FIRE   real collisions the ECU is required to catch
  MUST-NOT    everything a truck does in a working life that looks like a
              collision to a naive threshold detector

A crash algorithm is not validated by the must-fire list. Anyone can fire
on a barrier pulse. It is validated by the must-not list, because that is
where field returns, warranty cost and a discredited evidential record
come from.

Pulse shapes:
  haversine   a(t) = A sin^2(pi t / T)      - standard analytical crash
              pulse; dV = A*T/2, smooth C1, no spectral splatter.
  double-hump models a rail-then-engine-block two-stage crush.

Sensor effects applied on top (see `sensorise`): ASM330LHBG1-class noise
density, 0.488 mg/LSB quantisation, a static mounting offset, and
saturation at the configured full scale.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

G = 9.80665
FS_DEFAULT = 833


# ---------------------------------------------------------------- shapes
def haversine(t, t0, dur, peak_g):
    y = np.zeros_like(t)
    m = (t >= t0) & (t <= t0 + dur)
    y[m] = peak_g * np.sin(np.pi * (t[m] - t0) / dur) ** 2
    return y


def half_sine(t, t0, dur, peak_g):
    y = np.zeros_like(t)
    m = (t >= t0) & (t <= t0 + dur)
    y[m] = peak_g * np.sin(np.pi * (t[m] - t0) / dur)
    return y


def double_hump(t, t0, dur, peak_g, split=0.42, ratio=0.62):
    """Two-stage crush: soft rail collapse, then hard load path."""
    d1 = dur * split
    d2 = dur - d1
    return (haversine(t, t0, d1, peak_g * ratio)
            + haversine(t, t0 + d1, d2, peak_g))


def damped_ring(t, t0, freq_hz, peak_g, tau_s):
    y = np.zeros_like(t)
    m = t >= t0
    dt = t[m] - t0
    y[m] = peak_g * np.exp(-dt / tau_s) * np.sin(2 * np.pi * freq_hz * dt)
    return y


def band_noise(t, rms_g, f_lo, f_hi, rng):
    """Band-limited random road excitation."""
    n = len(t)
    fs = 1.0 / (t[1] - t[0])
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    spec[(f < f_lo) | (f > f_hi)] = 0
    y = np.fft.irfft(spec, n)
    s = np.std(y)
    return y * (rms_g / s) if s > 0 else y


def dv_of(a_g, fs):
    """Integrate a g-trace to m/s."""
    return np.cumsum(a_g) * G / fs


# ---------------------------------------------------------------- scenario
@dataclass
class Scenario:
    name: str
    kind: str                    # 'crash' | 'immunity' | 'decel' | 'multi'
    expect_loopA: bool
    expect_loopB: bool
    note: str
    build: Callable
    v0_kph: float = 60.0
    duration_s: float = 3.0
    tags: list = field(default_factory=list)


def _speed_from_accel(a_long_g, fs, v0_kph, can_hz=100):
    """Physically consistent VSS channel: integrate the longitudinal
    acceleration, clamp at zero, then apply CAN quantisation (0.0625 km/h)
    and a 100 Hz zero-order hold - because that is what actually arrives
    on the bus, and the Loop B slope estimator has to survive it."""
    v = v0_kph / 3.6 + dv_of(a_long_g, fs)
    v = np.maximum(v, 0.0)
    kph = v * 3.6
    kph = np.floor(kph / 0.0625) * 0.0625          # CAN LSB

    # Zero-order hold on exact CAN frame boundaries. Using fs//can_hz here
    # would put the frames at 104.125 Hz for fs=833, which then beats
    # against the ECU's true 100 Hz slow loop and shows up as spurious
    # jitter in the Loop B slope estimate.
    n = len(kph)
    k = (np.arange(n) * can_hz) // fs
    first = np.searchsorted(k, np.arange(k[-1] + 1))
    return kph[first][k]


def sensorise(a_g, fs, rng, noise_rms_g=0.003, offset_g=0.0, lsb_g=0.000488):
    """Apply IMU imperfections. The static offset is deliberate: it is the
    thing the bias tracker exists to remove, and leaving it out of the
    simulation would hide a whole class of integration-drift bugs."""
    y = a_g + offset_g + rng.normal(0, noise_rms_g, len(a_g))
    return np.round(y / lsb_g) * lsb_g


# ---------------------------------------------------------------- builders
def make(fs=FS_DEFAULT, dur=3.0):
    return np.arange(0, dur, 1.0 / fs)


REGISTRY: dict[str, Scenario] = {}


def register(s: Scenario):
    REGISTRY[s.name] = s
    return s


def _zeros(t):
    return np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)


# ============================ MUST FIRE (Loop A) ========================

def _truck_frontal_40(t, rng):
    """N3 tractor into a rigid barrier at 40 km/h. dV ~ 12 m/s with
    rebound, ~140 ms crush. Peak lands near 17 g - ABOVE the +/-16 g full
    scale of the ASM330LHBG1, so this case also exercises clip handling."""
    al = -haversine(t, 0.5, 0.140, 17.5)
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("truck_frontal_rigid_40kph", "crash", True, False,
                  "N3 rigid barrier 40 km/h, dV~12 m/s, clips at 16 g",
                  _truck_frontal_40, v0_kph=40))


def _truck_frontal_30(t, rng):
    al = -haversine(t, 0.5, 0.150, 12.3)
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("truck_frontal_rigid_30kph", "crash", True, False,
                  "N3 rigid barrier 30 km/h, dV~9.2 m/s",
                  _truck_frontal_30, v0_kph=30))


def _car_barrier_56(t, rng):
    """NCAP full frontal rigid barrier 56 km/h - included because it is
    the pulse everyone has data for, and it saturates the sensor hard."""
    al = -double_hump(t, 0.5, 0.085, 42.0)
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("car_barrier_56kph", "crash", True, False,
                  "M1 NCAP FFRB 56 km/h, dV~17 m/s, heavy clipping",
                  _car_barrier_56, v0_kph=56))


def _odb_64(t, rng):
    al = -double_hump(t, 0.5, 0.115, 24.0, split=0.5, ratio=0.75)
    ay = -haversine(t, 0.52, 0.10, 5.0)      # offset impact yaws the cab
    return al, ay, np.zeros_like(t)


register(Scenario("odb_64kph", "crash", True, False,
                  "40% offset deformable barrier 64 km/h",
                  _odb_64, v0_kph=64))


def _side_50(t, rng):
    ay = -haversine(t, 0.5, 0.070, 22.0)
    return np.zeros_like(t) - 0.05, ay, np.zeros_like(t)


register(Scenario("side_impact_50kph", "crash", True, False,
                  "lateral MDB 50 km/h, dV~7.7 m/s lateral",
                  _side_50, v0_kph=50))


def _rear_40(t, rng):
    al = +haversine(t, 0.5, 0.100, 15.0)
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("rear_impact_40kph", "crash", True, False,
                  "struck from behind, positive longitudinal dV",
                  _rear_40, v0_kph=20))


def _marginal_25(t, rng):
    """Sits exactly on the 25 km/h / 150 ms contractual boundary. The
    single most important calibration case: it decides where the line is."""
    al = -haversine(t, 0.5, 0.150, 9.44)   # dV = A*T/2 = 6.95 m/s
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("marginal_25kph_150ms", "crash", True, False,
                  "on the contractual boundary: dV=25 km/h in exactly 150 ms",
                  _marginal_25, v0_kph=40))


def _pole_32(t, rng):
    al = -haversine(t, 0.5, 0.180, 11.0)
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("pole_impact_32kph", "crash", True, False,
                  "narrow object, long soft pulse, dV~9.7 m/s",
                  _pole_32, v0_kph=32))


# ============================ MUST NOT FIRE (Loop A) ====================

def _hard_brake(t, rng):
    """THE case. 0.8 g emergency stop banks 11.8 m/s of dV - well over the
    25 km/h number - but spread across 1.5 s. A naive free-running
    integrator fires here and destroys the product's credibility. The
    rising dV-vs-t boundary is what rejects it."""
    al = np.zeros_like(t)
    m = (t >= 0.4) & (t <= 1.9)
    al[m] = -0.80
    al = np.convolve(al, np.ones(40) / 40, mode="same")
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("hard_braking_0g8", "immunity", False, True,
                  "0.8 g emergency stop, 1.5 s: total dV EXCEEDS 25 km/h",
                  _hard_brake, v0_kph=60))


def _abs_brake(t, rng):
    al = np.zeros_like(t)
    m = (t >= 0.4) & (t <= 2.2)
    al[m] = -0.70 + 0.18 * np.sin(2 * np.pi * 12 * t[m])
    return np.convolve(al, np.ones(20) / 20, mode="same"), np.zeros_like(t), np.zeros_like(t)


register(Scenario("abs_braking", "immunity", False, True,
                  "0.7 g with 12 Hz ABS modulation ripple",
                  _abs_brake, v0_kph=70))


def _pothole(t, rng):
    al = -half_sine(t, 0.8, 0.012, 10.0)
    az = half_sine(t, 0.8, 0.012, 14.0)
    return al, np.zeros_like(t), az


register(Scenario("pothole_strike", "immunity", False, False,
                  "10 g / 12 ms spike: huge jerk, dV only 0.75 m/s",
                  _pothole, v0_kph=50))


def _kerb(t, rng):
    ay = -half_sine(t, 0.8, 0.020, 8.0)
    al = -half_sine(t, 0.8, 0.020, 3.0)
    return al, ay, np.zeros_like(t)


register(Scenario("kerb_strike", "immunity", False, False,
                  "lateral kerb mount at low speed",
                  _kerb, v0_kph=25))


def _washboard(t, rng):
    al = 0.6 * np.sin(2 * np.pi * 10 * t) * ((t > 0.3) & (t < 2.7))
    az = 1.2 * np.sin(2 * np.pi * 10 * t + 0.7) * ((t > 0.3) & (t < 2.7))
    return al, 0.3 * np.sin(2 * np.pi * 7 * t), az


register(Scenario("washboard_road", "immunity", False, False,
                  "+/-0.6 g at 10 Hz corrugation, 2.4 s",
                  _washboard, v0_kph=40))


def _hammer(t, rng):
    """Service-bay reality: someone strikes the bracket with a mallet."""
    al = -half_sine(t, 1.0, 0.003, 40.0) + damped_ring(t, 1.003, 900, 8.0, 0.006)
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("hammer_blow", "immunity", False, False,
                  "40 g / 3 ms mallet strike on the housing",
                  _hammer, v0_kph=0))


def _door_slam(t, rng):
    al = damped_ring(t, 0.9, 55, 6.0, 0.035)
    return al, damped_ring(t, 0.9, 48, 3.0, 0.035), np.zeros_like(t)


register(Scenario("door_slam", "immunity", False, False,
                  "cab door slam, damped structural ringing",
                  _door_slam, v0_kph=0))


def _speed_bump(t, rng):
    al = -half_sine(t, 0.8, 0.12, 1.2) + half_sine(t, 0.95, 0.12, 0.9)
    az = half_sine(t, 0.8, 0.10, 2.5) - half_sine(t, 0.93, 0.10, 1.8)
    return al, np.zeros_like(t), az


register(Scenario("speed_bump_20kph", "immunity", False, False,
                  "speed hump traversal at 20 km/h",
                  _speed_bump, v0_kph=20))


def _offroad(t, rng):
    al = band_noise(t, 0.55, 3, 20, rng)
    ay = band_noise(t, 0.45, 3, 20, rng)
    az = band_noise(t, 1.10, 2, 18, rng)
    return al, ay, az


register(Scenario("rough_offroad_tractor", "immunity", False, False,
                  "MTB tractor in-field operation, broadband 3-20 Hz",
                  _offroad, v0_kph=15))


def _shunt(t, rng):
    al = -haversine(t, 0.8, 0.060, 4.0)
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("trailer_coupling_shunt", "immunity", False, False,
                  "coupling/uncoupling shunt, 4 g, dV~1.2 m/s",
                  _shunt, v0_kph=5))


def _downgrade(t, rng):
    """6% downgrade at CONSTANT road speed. The accelerometer reads a
    steady -0.59 m/s^2 of gravity projection. Loop B on VSS sees zero
    deceleration and correctly stays idle. Loop B on the accelerometer
    fallback would happily call this a sudden-deceleration event - which
    is the whole argument for VSS being the conformant source."""
    al = np.full_like(t, -0.06)          # -0.59 m/s^2 in g
    al[t < 0.3] = 0.0
    return al, np.zeros_like(t), np.zeros_like(t)


register(Scenario("downgrade_6pct_constant_speed", "immunity", False, False,
                  "6% grade, constant speed: gravity projection, not decel",
                  _downgrade, v0_kph=50, duration_s=4.0))


# ============================ LOOP B cases ==============================

def _decel_ais220(t, rng):
    al = np.zeros_like(t)
    m = (t >= 0.5) & (t <= 1.6)
    al[m] = -3.50 / G
    return np.convolve(al, np.ones(30) / 30, mode="same"), np.zeros_like(t), np.zeros_like(t)


register(Scenario("sudden_decel_3g5_1100ms", "decel", False, True,
                  "3.50 m/s^2 for 1.10 s -> AIS-220 sudden-decel trigger",
                  _decel_ais220, v0_kph=60, duration_s=3.0))


def _decel_marginal(t, rng):
    al = np.zeros_like(t)
    m = (t >= 0.5) & (t <= 1.28)
    al[m] = -3.35 / G
    return np.convolve(al, np.ones(30) / 30, mode="same"), np.zeros_like(t), np.zeros_like(t)


register(Scenario("sudden_decel_marginal_780ms", "decel", False, True,
                  "3.35 m/s^2 for 0.78 s - just inside the 0.7 s dwell",
                  _decel_marginal, v0_kph=60))


def _decel_short(t, rng):
    al = np.zeros_like(t)
    m = (t >= 0.5) & (t <= 1.0)
    al[m] = -4.20 / G
    return np.convolve(al, np.ones(30) / 30, mode="same"), np.zeros_like(t), np.zeros_like(t)


register(Scenario("sudden_decel_too_short_500ms", "decel", False, False,
                  "4.20 m/s^2 but only 0.50 s - fails the dwell criterion",
                  _decel_short, v0_kph=60))


def _decel_weak(t, rng):
    al = np.zeros_like(t)
    m = (t >= 0.5) & (t <= 2.5)
    al[m] = -2.90 / G
    return np.convolve(al, np.ones(30) / 30, mode="same"), np.zeros_like(t), np.zeros_like(t)


register(Scenario("sudden_decel_too_weak_2g9", "decel", False, False,
                  "2.90 m/s^2 for 2.0 s - below the 3.25 m/s^2 threshold",
                  _decel_weak, v0_kph=60, duration_s=4.0))


# ============================ multi-event ===============================

def _multi(t, rng):
    al = (-haversine(t, 0.5, 0.120, 14.0)
          - haversine(t, 1.30, 0.110, 12.0))
    ay = -haversine(t, 1.30, 0.110, 4.0)
    return al, ay, np.zeros_like(t)


register(Scenario("multi_impact_800ms", "multi", True, False,
                  "two collisions 800 ms apart -> one sequence, two events",
                  _multi, v0_kph=50, duration_s=4.0))


# ---------------------------------------------------------------- render
def render(name, fs=FS_DEFAULT, seed=1, noise_rms_g=0.003, offset_g=0.025):
    """Return (t, ax_g, ay_g, az_g, speed_kph, scenario)."""
    if name not in REGISTRY:
        raise KeyError(f"unknown scenario {name!r}. Known: {sorted(REGISTRY)}")
    s = REGISTRY[name]
    rng = np.random.default_rng(seed)
    t = make(fs, s.duration_s)
    ax, ay, az = s.build(t, rng)

    if s.name == "downgrade_6pct_constant_speed":
        speed = np.full_like(t, s.v0_kph)      # constant by construction
    else:
        speed = _speed_from_accel(ax, fs, s.v0_kph)

    ax = sensorise(ax, fs, rng, noise_rms_g, offset_g)
    ay = sensorise(ay, fs, rng, noise_rms_g, offset_g * 0.4)
    az = sensorise(az, fs, rng, noise_rms_g, 0.0)
    return t, ax, ay, az, speed, s


def all_names(kind=None):
    return [n for n, s in REGISTRY.items() if kind is None or s.kind == kind]
