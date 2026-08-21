# EDR crash detection core — ΔV + sustained deceleration

Two-loop crash/event detection for the AIS-220 + UN R169 EDR programme.
Detection core in portable C99, PC simulator in Python driving the **same
compiled binary** that goes into the ECU image.

```
firmware/inc   edr_types.h  edr_config.h  edr_filter.h  edr_loops.h  edr_api.h
firmware/src   edr_filter.c edr_config.c  edr_loopA.c   edr_loopB.c  edr_api.c
               edr_sim_shim.c        <- simulator only, NOT in the ECU build
sim/           build.py  edr_core.py  pulses.py  run_sim.py
               validate.py  plot_results.py  fetch_nhtsa.py
tests/         test_algorithm.py
```

## Quick start

```bash
pip install -r requirements.txt
python sim/build.py                       # ~1 s, needs gcc/clang/MinGW
python sim/validate.py                    # the pass/fail matrix
python sim/run_sim.py --list              # 24 built-in scenarios
python sim/run_sim.py --scenario truck_frontal_rigid_40kph --plot
python sim/run_sim.py --input data/sample_capture.csv --plot
python -m pytest tests/ -q
```

In VS Code: **Ctrl+Shift+B** builds, **F5** offers six launch configs
(scenario picker, CSV input, validation matrix, gdb into the C core).

---

## The two loops, and why they are separate

```
 IMU ASM330LHBG1  ──833 Hz──▶  LOOP A: ΔV discrimination  ──┐
                                                            ├─▶ ARBITER ─▶ record / lock
 VSS / ABS (CAN)  ──100 Hz──▶  LOOP B: decel rate       ────┤
 SRS ECU (CAN)    ──event───▶  lock path (§5.3.2)       ────┘
```

| | Loop A | Loop B |
|---|---|---|
| Source | IMU, longitudinal + lateral | **Vehicle speed**, not the IMU |
| Rate | 833 Hz | 100 Hz |
| Measures | ΔV over 150 ms, and cumulative ΔV vs a time-varying boundary | deceleration sustained ≥ 0.7 s |
| Criterion | ΔV ≥ 25 km/h in 150 ms, **or** boundary crossing | ≥ 3.25 m/s² for ≥ 700 ms |
| Status | **product-level value-add** | **AIS-220 mandated** |
| Locks the record? | only if `lock_on_delta_v` (a product decision) | never |

They are not redundant paths to the same answer. A collision is over in
100–180 ms and **never** satisfies the 700 ms dwell. A 0.8 g emergency
stop banks more total ΔV than 25 km/h and must **never** be recorded as a
collision. Each loop exists to catch what the other structurally cannot.

### Regulatory provenance

Every threshold in `edr_config.c` carries a tag. Keeping these separate
is what stops a compliance argument from being contaminated by product
features:

- **`[AIS-220]`** — 3.25 m/s² / 0.7 s from VSS; ±1.5 g / 0.1 g / 4 Hz
  recorded output; 5 s pre-crash; SRS-only locking per §5.3.2.
- **`[PRODUCT]`** — the ΔV 25 km/h / 150 ms criterion and the ΔV lock
  path. These solve the no-SRS locking gap on N2/N3 trucks. They are not
  compliance requirements, and `lock_on_delta_v = 0` gives a strictly
  conformant build (which, on a truck with no airbag, never locks
  anything — `test_scenario_conformant_mode_never_locks_without_srs`).
- **`[ENG]`** — arming levels, filter corners, immunity thresholds. Tune
  on vehicle.

---

## Algorithm

### Front end (per axis, per sample)

```
raw ─▶ clip detect ─▶ LPF 100 Hz ─▶ bias removal ─▶ jerk, oscillation count
                   └▶ LPF  60 Hz ─▶ bias removal ─▶ integrate ─▶ ΔV ring
```

- **Two filter paths.** ~CFC 60 for integration (the vehicle-level pulse),
  100 Hz for discrimination (keeps the onset edge sharp for arming).
- **Bias tracker frozen while armed.** A leaky integrator, gated so a real
  pulse can never bend the estimate toward itself, and held fixed once
  armed. Without this a 0.05 g mounting offset integrates to 9.8 m/s of
  phantom ΔV over 20 s — past every threshold in the file. Guarded by
  `test_property_bias_does_not_leak_into_delta_v`.
- **One ring, four windows.** Per-sample ΔV increments in a single
  1024-entry ring with four independent running sums (30 / 60 / 150 /
  300 ms). O(1) per sample, no re-summation.
- **Integer only.** Acceleration mm/s², velocity mm/s, integration
  accumulated in **µm/s** so per-sample truncation is <1 µm/s instead of
  ~0.5 mm/s (which at 833 Hz would be 400 mm/s² of drift). No FPU
  dependency, deterministic, bit-identical sim to target.

### Loop A firing logic

```
   ( ΔV₁₅₀ₘₛ ≥ 25 km/h          ← contractual spec
     OR ΔV_cum > boundary(t) )   ← severity discrimination
   AND ΔV₃₀₀ₘₛ ≥ 2.0 m/s         ← safing, independent concurrence
   AND NOT rough-road AND NOT jerk-spike
```

The **ΔV-vs-time boundary** is the piece that does the real work. It is
monotonically *increasing*: a violent pulse banks ΔV fast and crosses
early; a slow accumulation can never catch a rising threshold. That is
what separates a collision from hard braking without needing a second
sensor. `plot_results.plot_boundary()` draws it for the review pack.

The **safing gate** is the classic two-out-of-two. A pothole produces
10 g and enormous jerk but only 0.75 m/s of ΔV — it arms, then fails
safing. The **rough-road oscillation counter** and **jerk limiter** cover
corrugation and mallet strikes.

### Loop A state machine

```
IDLE ──arm──▶ ARMED ──criteria met──▶ CAPTURE ──+250 ms──▶ FIRED ──▶ LOCKOUT
                 └──── eval timeout ─────────────────────────────────┘
```

`CAPTURE` matters: the trigger *decision* is taken at (say) 63 ms, but
the record is finalised 250 ms later so the stored ΔV is the ΔV of the
**collision**, not the ΔV at the instant a threshold happened to be
crossed. `t0_ms` still carries the decision instant, which is what a
reconstruction has to line up against the rest of the vehicle's data.

### Loop B

Least-squares slope over a 200 ms window of vehicle speed. **Not** a
two-point difference: CAN speed is quantised at 0.0625 km/h, and
differentiating that at 100 Hz turns one LSB into 1.74 m/s² of noise —
over half the trigger threshold. The LS estimator brings the noise on the
plateau down to ±0.04 m/s² (measured).

Hysteresis band (3.25 fire / 3.00 hold) plus a 40 ms dropout tolerance so
ABS cycling and gear shifts do not discard accumulated dwell.

---

## Findings from building this

Five things surfaced during bring-up that are worth carrying into the HLA.

**1. ±16 g is marginal for this application, and the record has to say so.**
An N3 tractor into a rigid barrier at 40 km/h peaks near 17.5 g. The
ASM330LHBG1 clips at 16 g and the recorded ΔV comes out 11.76 m/s against
a true 12.01 — a 2% under-report that grows sharply with severity. The
core detects clipping and sets `dv_is_lower_bound` on the event, because
a reconstructionist must never read a clipped ΔV as the true ΔV. This
does not by itself justify a second higher-range sensor — an N2/N3 crash
pulse is far softer than the M1 case — but it is a decision that should
be taken explicitly rather than inherited.

**2. Loop B systematically under-measured dwell by 100 ms.** A centred
least-squares slope estimates the derivative at the *window centre*, so
it lags by `regress_ms/2`. Starting the dwell clock on the first
threshold crossing therefore measured a genuine 780 ms deceleration as
680 ms and missed it — a **false negative on a mandated trigger**. Fixed
by crediting the half-window back at dwell entry
(`sudden_decel_marginal_780ms` is the regression guard).

**3. An integrator that starts at zero on arming under-reports ΔV by ~6%.**
Arming needs a level *and* a short-window ΔV, which on a 150 ms truck
pulse costs 20–40 ms of onset. Fixed with a 60 ms pre-arm seed window.
The seed is deliberately short: longer, and a collision following hard
braking folds the braking ΔV into the collision ΔV.

**4. Sample-rate ratios must not be integer-divided.** 833/100 truncates
to 8, running the slow loop at 104.125 Hz while Loop B scales its slope
by the configured 100 Hz — a silent 4% error on every deceleration
reading. Same class of bug on the 4 Hz record clock (833/4 → 4.0048 Hz,
6 ms of timebase drift across a 5 s buffer). Both now use Bresenham
accumulators. On target the loops have independent timers, but the
simulator has to reproduce the ECU's arithmetic or the calibration is
tuned against a lie.

**5. The 25 km/h flat threshold is close to redundant.** In the threshold
sweep, the matrix passes 24/24 for `dv_win_thresh_mm_s` anywhere from
5000 to 10000 mm/s — because the boundary curve fires first in every
scenario. Worth knowing: the contractual number is being satisfied, but
it is not what is doing the discriminating, and the boundary table is
where calibration effort should go.

---

## Validation

`python sim/validate.py` runs 24 scenarios. The must-not-fire half is
what makes it credible — anyone can fire on a barrier pulse.

```
scenario                           kind      expA gotA expB gotB  margin  result
truck_frontal_rigid_40kph          crash        1    1    0    0   1.69x  PASS
marginal_25kph_150ms               crash        1    1    0    0   1.11x  PASS
hard_braking_0g8                   immunity     0    0    1    1   5.90x  PASS
pothole_strike                     immunity     0    0    0    0   4.52x  PASS
hammer_blow                        immunity     0    0    0    0   5.96x  PASS
downgrade_6pct_constant_speed      immunity     0    0    0    0  79.82x  PASS
sudden_decel_marginal_780ms        decel        0    0    1    1  13.83x  PASS
multi_impact_800ms                 multi        1    1    0    0   1.43x  PASS
                                                    ... 24/24 pass
```

`margin` is the ratio to the wrong answer, so >1.0 is always comfortable.

- `--seeds 20` — Monte Carlo over noise realisations
- `--rates` — verdicts must be identical at 416 / 833 / 1667 Hz
- `--sweep` — threshold sensitivity; if the matrix only passes in a narrow
  band, the calibration is fitted to the scenario set, not to physics

The immunity set covers hard braking, ABS modulation, potholes, kerb
strikes, corrugation, mallet blows, door slams, speed humps, in-field
tractor operation, coupling shunts, and a 6% downgrade at constant speed.
That last one is the argument for VSS: the accelerometer reads a steady
−0.59 m/s² of gravity projection and would happily call it a
sudden-deceleration event. The VSS path reads 0.000.

---

## Real crash data

Synthetic haversines are smooth and single-peaked. Real pulses have rail
buckling spikes, engine-block contact and 300–900 Hz mount ringing. A
boundary calibrated only on analytical pulses looks excellent in the
report and behaves differently on the sled.

```bash
python sim/fetch_nhtsa.py --list-guides
python sim/fetch_nhtsa.py --convert test.001 --fs 10000 --invert \
                          --v0-kph 40 --out data/nhtsa_v12345.csv
python sim/run_sim.py --input data/nhtsa_v12345.csv --plot
```

NHTSA runs public vehicle/biomechanics/component crash-test databases
with per-channel time histories; entry point and Swagger specs at
<https://nrd.api.nhtsa.dot.gov/>. The databases were reworked in 2024, so
confirm endpoint paths before scripting `--api` — the offline `--convert`
path needs no network and is the dependable route. Prefer
vehicle-mounted longitudinal channels (B-pillar, tunnel) over occupant
channels, and the heaviest classes available: light-vehicle barrier
pulses peak two to three times higher than a loaded tractor unit and will
pull a calibration the wrong way.

**Note:** a crash test article has no wheel-speed trace. `--v0-kph`
synthesises one by integrating the pulse; any Loop B result from it is an
inference, not a measurement, and must not be presented as VSS-sourced.

### Input CSV schema

```
t_s , ax_g , ay_g , az_g , speed_kph , srs
```

`t_s` may be irregularly spaced (it is resampled). `ax_g` is +X forward,
so **braking and frontal impact are negative**. Common alternative column
names are auto-detected. Without `speed_kph`, Loop B falls back to the
accelerometer and every record is tagged `decel_src_is_vss = false`.

---

## Target integration (S32K312)

```c
edr_ctx_t ctx;
edr_config_t cfg;
edr_config_defaults(&cfg);
if (edr_config_validate(&cfg) != 0) { /* calibration blob is bad */ }
edr_init(&ctx, &cfg);

/* IMU data-ready ISR or 1.2 ms task */
edr_fast_tick(&ctx, &imu_sample);

/* 100 Hz CAN Rx task */
edr_slow_tick(&ctx, &vehicle_sample);

/* background */
n = edr_poll_events(&ctx, evbuf, 4);
```

- **Static allocation only.** No malloc, no recursion, no variable-bound
  loops in `edr_fast_tick`. `sizeof(edr_ctx_t)` = **9832 bytes** measured
  on x86-64; 8320 of that is the two 1024-entry ΔV rings. Re-measure on
  the ARM toolchain (alignment differs) and budget it against S32K312
  SRAM before layout. If it is tight, `EDR_DV_RING_LEN` can drop to 512
  provided `fast_hz × dv_safing_win_ms / 1000 < 500` —
  `edr_config_validate()` enforces that and returns −3 if violated.
- **Concurrency.** Loop B publishes through a seqlock
  (`edr_decel_publish` / `edr_decel_read`). Add `__DMB()` at the two
  marked points when the loops become separate tasks — the simulator is
  single-threaded and will not catch a missing barrier.
- **Float usage is confined to init.** `edr_biquad_design_lp()` uses
  `double` and runs once; for a strictly FPU-free build, replace it with
  a const coefficient table generated offline.
- **ASIL-B.** Two redundant ASM330LHBG1 devices plus ST's certified
  library sit *below* this layer; the core takes one conditioned sample
  pair and a validity flag. Cross-channel comparison and the safety
  reaction are not implemented here.

---

## Open items

1. **Boundary table is engineering judgement, not vehicle data.** The
   knees in `cfg->boundary[]` were set to bracket the scenario set and
   pin the 150 ms contractual point. They need sled or real-pulse
   calibration per platform (MTB tractor vs AL N3) before they mean
   anything.
2. **±16 g clipping** — decide explicitly whether the clip flag is
   sufficient or a higher-range channel is warranted (see Finding 1).
3. **Confirm the AIS-220 recording window** — `precrash_ms` / 
   `postcrash_ms` are set to 5000/250 from the summary understanding of
   Annexure 4; verify against the published table before freezing the
   record format.
4. **Anti-alias before 4 Hz decimation** is implemented at `record_hz/2`
   (2 Hz). Confirm this is acceptable versus whatever filter class the
   standard specifies for the recorded channel.
5. **AUTOSAR** — if AEPL assumes Classic, `edr_fast_tick` becomes a
   runnable and the record path goes through MemIf/Fee. That changes the
   event-drain interface, not the algorithm.
6. **Multi-event window** is 5000 ms with a 4-event buffer. Confirm the
   grouping rule and maximum sub-event count against the standard.
7. **Timing not yet measured on silicon.** Worst-case `edr_fast_tick`
   needs a real measurement on the S32K312 with I-cache configured as
   shipped, not an estimate.
8. **`lock_on_srs_only` vs `lock_on_delta_v`** — the shipped combination
   is a customer decision and should be recorded as one in the safety
   case, per platform.
