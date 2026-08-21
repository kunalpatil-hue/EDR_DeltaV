#!/usr/bin/env python3
"""
validate.py - run the whole scenario matrix and print a pass/fail table.

    python sim/validate.py
    python sim/validate.py --sweep          # threshold sensitivity
    python sim/validate.py --seeds 20       # noise robustness (Monte Carlo)

This is the artefact to put in front of the customer. A crash algorithm
is not credible because it fires on a barrier pulse; it is credible
because it does NOT fire on the other eleven things in this table, across
noise realisations, with margin you can quote.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pulses  # noqa: E402
from edr_core import EdrSim  # noqa: E402

TRG_LOOPA = 0b0110
TRG_LOOPB = 0b0001


def run_one(name, fs=833, seed=1, cfg=None):
    t, ax, ay, az, sp, s = pulses.render(name, fs=fs, seed=seed)
    res = EdrSim(dict(cfg or {}, fast_hz=fs)).run(t, ax, ay, az, sp)
    a = any(e.trigger_mask & TRG_LOOPA for e in res.events)
    b = any(e.trigger_mask & TRG_LOOPB for e in res.events)
    return s, res, a, b


def margin(res, s):
    """How close did we come to the WRONG answer?

    Per sample, ratio = how far the severity measures got toward their
    criteria, taking whichever is closer to firing:

        max( dV_150ms / 6.944 ,  dV_cum / boundary(t) )

    A must-fire case reports the peak of that ratio (>1 means it fired
    with headroom). A must-not-fire case reports its reciprocal (>1 means
    it stayed clear). So >1.0 is always "comfortable", and the number is
    directly quotable as a calibration margin either way."""
    r1 = res.dv_main / 6.944
    b = np.where(res.boundary > 1e-6, res.boundary, np.inf)
    r2 = res.dv_cum / b
    ratio = np.maximum(r1, r2)
    peak = float(ratio.max()) if len(ratio) else 0.0
    if s.expect_loopA:
        return peak
    return (1.0 / peak) if peak > 1e-6 else 99.0


def matrix(fs=833, seeds=(1,), cfg=None, quiet=False):
    rows, npass = [], 0
    for name in pulses.all_names():
        results = [run_one(name, fs, sd, cfg) for sd in seeds]
        s = results[0][0]
        a_all = [r[2] for r in results]
        b_all = [r[3] for r in results]
        okA = all(x == s.expect_loopA for x in a_all)
        okB = all(x == s.expect_loopB for x in b_all)
        ok = okA and okB
        npass += ok
        m = min(margin(r[1], s) for r in results)
        rows.append((name, s.kind, s.expect_loopA, s.expect_loopB,
                     a_all[0], b_all[0], ok, m, s.note,
                     sum(a_all), sum(b_all), len(seeds)))

    if not quiet:
        hdr = f"{'scenario':34s} {'kind':9s} {'expA':>4s} {'gotA':>4s} {'expB':>4s} {'gotB':>4s} {'margin':>7s}  result"
        print(hdr)
        print("-" * len(hdr))
        for (n, k, ea, eb, ga, gb, ok, m, note, sa, sb, ns) in rows:
            flag = "PASS" if ok else "**FAIL**"
            mm = f"{m:6.2f}x" if m < 90 else "   inf"
            print(f"{n:34s} {k:9s} {int(ea):>4d} {int(ga):>4d} {int(eb):>4d} {int(gb):>4d} {mm:>7s}  {flag}")
            if not ok and ns > 1:
                print(f"{'':34s} {'':9s} loopA fired {sa}/{ns} seeds, loopB {sb}/{ns}")
        print("-" * len(hdr))
        print(f"{npass}/{len(rows)} pass"
              + (f"   ({len(seeds)} noise seeds each)" if len(seeds) > 1 else ""))
        worst = min((r for r in rows if r[7] < 90), key=lambda r: r[7], default=None)
        if worst:
            print(f"tightest margin: {worst[0]} at {worst[7]:.2f}x")
    return rows, npass


def sweep():
    """Sensitivity of the pass rate to the two headline thresholds. If the
    matrix only passes inside a narrow band, the calibration is fitted to
    the scenario set rather than to the physics."""
    print("\ndV window threshold sweep (nominal 6944 mm/s = 25 km/h)")
    print(f"{'thresh mm/s':>12s} {'km/h':>6s}  {'pass':>6s}")
    for thr in (5000, 5500, 6000, 6500, 6944, 7500, 8000, 9000, 10000):
        _, n = matrix(cfg={"dv_win_thresh_mm_s": thr}, quiet=True)
        print(f"{thr:12d} {thr * 0.0036:6.1f}  {n:3d}/24")

    print("\ndecel threshold sweep (nominal 3250 mm/s^2, AIS-220)")
    print(f"{'thresh mm/s2':>13s}  {'pass':>6s}")
    for thr in (2500, 2800, 3000, 3250, 3500, 3800, 4200):
        _, n = matrix(cfg={"decel_thresh_mm_s2": thr,
                           "decel_rearm_mm_s2": int(thr * 0.92)}, quiet=True)
        print(f"{thr:13d}  {n:3d}/24")

    print("\narming threshold sweep (nominal 19614 mm/s^2 = 2.0 g)")
    print(f"{'arm mm/s2':>10s} {'g':>5s}  {'pass':>6s}")
    for g in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
        _, n = matrix(cfg={"armA_accel_mm_s2": int(g * 9807)}, quiet=True)
        print(f"{int(g * 9807):10d} {g:5.1f}  {n:3d}/24")


def rate_check():
    """The algorithm must give the same verdict at 833 Hz and 1667 Hz. If
    it does not, something is sample-rate dependent that should not be."""
    print("\nsample-rate independence")
    for fs in (416, 833, 1667):
        rows, n = matrix(fs=fs, quiet=True)
        fails = [r[0] for r in rows if not r[6]]
        print(f"  {fs:5d} Hz : {n}/{len(rows)} pass" + (f"   fails: {fails}" if fails else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--rates", action="store_true")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--fs", type=int, default=833)
    args = ap.parse_args()

    seeds = tuple(range(1, args.seeds + 1))
    print(f"\nEDR crash-detection validation matrix  (fast loop {args.fs} Hz)\n")
    rows, npass = matrix(fs=args.fs, seeds=seeds)

    if args.sweep:
        sweep()
    if args.rates:
        rate_check()

    return 0 if npass == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
