#!/usr/bin/env python3
"""
run_sim.py - run the EDR core against a file or a built-in scenario.

    python sim/run_sim.py --scenario truck_frontal_rigid_40kph --plot
    python sim/run_sim.py --input data/mycapture.csv --plot
    python sim/run_sim.py --list

INPUT CSV SCHEMA (header row required, extra columns ignored):

    t_s , ax_g , ay_g , az_g , speed_kph , srs

    t_s        seconds, monotonic. Irregular spacing is fine - the file
               is resampled onto the configured fast_hz grid.
    ax_g       longitudinal, +X forward. Braking is NEGATIVE.
    ay_g       lateral,      +Y left.        (optional)
    az_g       vertical.                     (optional)
    speed_kph  vehicle speed from VSS/ABS.   (optional but strongly
               recommended: without it Loop B runs on the degraded
               accelerometer fallback and every record is tagged as such)
    srs        0/1 SRS deployment flag.      (optional)

Common alternative column names are auto-detected (see COLUMN_ALIASES).
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pulses  # noqa: E402
from edr_core import EdrSim, decode_mask, INHIBIT_NAMES  # noqa: E402

COLUMN_ALIASES = {
    "t_s": ["t_s", "t", "time", "time_s", "time_sec", "timestamp"],
    "ax_g": ["ax_g", "ax", "accel_x", "a_long", "long_g", "x", "acc_x"],
    "ay_g": ["ay_g", "ay", "accel_y", "a_lat", "lat_g", "y", "acc_y"],
    "az_g": ["az_g", "az", "accel_z", "a_vert", "vert_g", "z", "acc_z"],
    "speed_kph": ["speed_kph", "speed", "vehicle_speed", "vss_kph", "v_kph"],
    "srs": ["srs", "srs_deploy", "airbag", "deploy"],
}


def load_csv(path: Path) -> dict:
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        raise ValueError(f"{path} is empty")

    header = [h.strip().lower() for h in rows[0]]
    try:
        float(rows[0][0])
        raise ValueError(f"{path}: first row looks numeric - a header row is required")
    except ValueError as e:
        if "header row is required" in str(e):
            raise

    idx = {}
    for canon, names in COLUMN_ALIASES.items():
        for n in names:
            if n in header:
                idx[canon] = header.index(n)
                break

    missing = [c for c in ("t_s", "ax_g") if c not in idx]
    if missing:
        raise ValueError(
            f"{path}: could not find column(s) {missing}. Header was {header}. "
            f"Recognised aliases: {COLUMN_ALIASES}")

    data = np.array([[float(r[i]) if i < len(r) and r[i].strip() else np.nan
                      for i in range(len(header))] for r in rows[1:] if r])

    out = {"t_s": data[:, idx["t_s"]], "ax_g": data[:, idx["ax_g"]]}
    for c in ("ay_g", "az_g", "speed_kph", "srs"):
        out[c] = data[:, idx[c]] if c in idx else None
    return out


def resample(d: dict, fs: int) -> dict:
    """Onto a uniform fast_hz grid. Linear for continuous channels,
    nearest-hold for the SRS flag so a one-sample pulse is not smeared."""
    t = d["t_s"]
    order = np.argsort(t)
    t = t[order]
    tg = np.arange(t[0], t[-1], 1.0 / fs)
    out = {"t_s": tg}
    for c in ("ax_g", "ay_g", "az_g", "speed_kph"):
        v = d.get(c)
        out[c] = np.interp(tg, t, np.nan_to_num(v[order])) if v is not None else None
    if d.get("srs") is not None:
        v = np.nan_to_num(d["srs"][order])
        out["srs"] = (np.interp(tg, t, v) > 0.5).astype(np.int32)
    else:
        out["srs"] = None
    return out


def report(res, scen=None, verbose=True) -> dict:
    lines = []
    fired_A = any(e.trigger_mask & 0b0110 for e in res.events)
    fired_B = any(e.trigger_mask & 0b0001 for e in res.events)

    lines.append(f"  samples        : {len(res.t)} @ {res.fs_hz} Hz "
                 f"({res.t[-1] - res.t[0]:.2f} s)")
    lines.append(f"  peak |a| long  : {np.abs(res.a_long_filt).max():6.2f} g "
                 f"(filtered, bias removed)")
    lines.append(f"  peak dV 150 ms : {res.dv_main.max():6.2f} m/s "
                 f"({res.dv_main.max() * 3.6:5.1f} km/h)")
    lines.append(f"  peak decel     : {np.nanmax(res.decel):6.2f} m/s^2   "
                 f"(threshold 3.25, dwell {res.dwell.max():.0f} ms / 700 ms)")
    lines.append(f"  bias estimate  : {res.bias_long[-1] * 1000:6.1f} mg long")

    inh = int(np.bitwise_or.reduce(res.inhibit.astype(np.int64))) if len(res.inhibit) else 0
    if inh:
        lines.append(f"  inhibits seen  : {decode_mask(inh, INHIBIT_NAMES)}")

    lines.append("")
    if not res.events:
        lines.append("  NO EVENTS")
    for e in res.events:
        flags = []
        if e.locked:
            flags.append("LOCKED")
        if e.dv_is_lower_bound:
            flags.append("dV IS A LOWER BOUND (sensor clipped)")
        if not e.decel_src_is_vss:
            flags.append("decel from ACCEL FALLBACK - degraded")
        lines.append(f"  [{e.seq_index}] t0={e.t_s:7.3f}s  {e.triggers}")
        lines.append(f"       dV  {e.dv_res:6.2f} m/s ({e.dv_kph:5.1f} km/h)   "
                     f"long {e.dv_long:6.2f}  lat {e.dv_lat:6.2f}  PDOF {e.pdof_deg:6.1f} deg")
        lines.append(f"       peak {e.peak_a_long_g:6.2f} g long / {e.peak_a_lat_g:6.2f} g lat   "
                     f"dVmax {e.dv_max:5.2f} m/s at {e.t_to_dv_max_ms:.0f} ms")
        lines.append(f"       decel {e.decel_ms2:5.2f} m/s^2 dwell {e.decel_dwell_ms:.0f} ms   "
                     f"speed {e.speed_kph:5.1f} km/h")
        if flags:
            lines.append(f"       ** {' | '.join(flags)}")

    verdict = None
    if scen is not None:
        okA = fired_A == scen.expect_loopA
        okB = fired_B == scen.expect_loopB
        verdict = okA and okB
        lines.append("")
        lines.append(f"  expected  A={int(scen.expect_loopA)} B={int(scen.expect_loopB)}   "
                     f"actual A={int(fired_A)} B={int(fired_B)}   "
                     f"-> {'PASS' if verdict else 'FAIL'}")

    if verbose:
        print("\n".join(lines))
    return {"fired_A": fired_A, "fired_B": fired_B, "verdict": verdict,
            "text": "\n".join(lines)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--input", type=Path, help="CSV file")
    src.add_argument("--scenario", help="built-in scenario name")
    ap.add_argument("--list", action="store_true", help="list built-in scenarios")
    ap.add_argument("--fs", type=int, default=833, help="fast loop rate (Hz)")
    ap.add_argument("--config", type=Path, help="JSON calibration overrides")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--save-csv", type=Path, help="dump internal signals")
    ap.add_argument("--save-json", type=Path, help="dump event records")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    if args.list or (not args.input and not args.scenario):
        print("Built-in scenarios:\n")
        for kind in ("crash", "immunity", "decel", "multi"):
            print(f"  --- {kind} ---")
            for n in pulses.all_names(kind):
                s = pulses.REGISTRY[n]
                print(f"    {n:34s} A={int(s.expect_loopA)} B={int(s.expect_loopB)}  {s.note}")
            print()
        return 0

    cfg = {"fast_hz": args.fs}
    if args.config:
        cfg.update(json.loads(args.config.read_text()))

    scen = None
    if args.scenario:
        t, ax, ay, az, sp, scen = pulses.render(args.scenario, fs=args.fs, seed=args.seed)
        srs = None
        title = args.scenario
    else:
        d = resample(load_csv(args.input), args.fs)
        t, ax, ay, az = d["t_s"], d["ax_g"], d["ay_g"], d["az_g"]
        sp, srs = d["speed_kph"], d["srs"]
        title = args.input.name
        if sp is None:
            print("  ! no speed column: Loop B falls back to the accelerometer "
                  "and every record will be tagged decel_src_is_vss = false\n")

    print(f"\n=== {title} ===")
    if scen:
        print(f"  {scen.note}\n")
    res = EdrSim(cfg).run(t, ax, ay, az, sp, srs)
    r = report(res, scen)

    if args.save_csv:
        args.save_csv.parent.mkdir(parents=True, exist_ok=True)
        with open(args.save_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "ax_in_g", "a_long_filt_g", "a_lat_filt_g", "bias_g",
                        "dv_main_ms", "dv_cum_ms", "dv_safing_ms", "boundary_ms",
                        "decel_ms2", "dwell_ms", "stateA", "stateB",
                        "trigger", "inhibit", "speed_kph"])
            for i in range(len(res.t)):
                w.writerow([f"{res.t[i]:.6f}", f"{res.a_long_in[i]:.5f}",
                            f"{res.a_long_filt[i]:.5f}", f"{res.a_lat_filt[i]:.5f}",
                            f"{res.bias_long[i]:.5f}", f"{res.dv_main[i]:.4f}",
                            f"{res.dv_cum[i]:.4f}", f"{res.dv_safing[i]:.4f}",
                            f"{res.boundary[i]:.4f}", f"{res.decel[i]:.4f}",
                            int(res.dwell[i]), int(res.stateA[i]), int(res.stateB[i]),
                            int(res.trigger[i]), int(res.inhibit[i]),
                            f"{res.speed_kph[i]:.3f}"])
        print(f"\n  signals -> {args.save_csv}")

    if args.save_json:
        args.save_json.parent.mkdir(parents=True, exist_ok=True)
        args.save_json.write_text(json.dumps(
            {"source": title,
             "events": [vars(e) | {"triggers": e.triggers} for e in res.events],
             "annexure4_record": {k: list(map(float, v))
                                  for k, v in (res.record or {}).items()}},
            indent=2))
        print(f"  events  -> {args.save_json}")

    if args.plot:
        import plot_results
        plot_results.plot(res, title)

    return 0 if (r["verdict"] is None or r["verdict"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
