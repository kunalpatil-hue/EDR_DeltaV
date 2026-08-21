#!/usr/bin/env python3
"""
fetch_nhtsa.py - pull real crash-test acceleration traces into the sim.

WHY REAL DATA MATTERS
Synthetic haversines are smooth, symmetric and single-peaked. Real crash
pulses have rail buckling spikes, engine-block contact, sensor mount
ringing at 300-900 Hz, and DC offsets from the mount. A boundary curve
calibrated only on analytical pulses will look excellent in the report
and behave differently on the sled.

SOURCE
NHTSA runs public crash-test databases (Vehicle / Biomechanics /
Component / Crash Avoidance) with per-channel time-history signal data.
The API entry point and Swagger specs are documented at:

    https://nrd.api.nhtsa.dot.gov/

    Vehicle DB (VEHDB) API:
      https://nrd.api.nhtsa.dot.gov/nhtsa/vehicle/swagger-ui/index.html

The databases were substantially reworked in 2024, so CONFIRM THE CURRENT
ENDPOINT PATHS against that Swagger UI before relying on --api. The
offline converters below (--convert) do not depend on the API at all and
are the reliable path: download the test's signal files from the web UI,
then convert.

SIGNAL FILE FORMAT
NHTSA test submissions carry unfiltered digitised signal data in files
indexed by curve number as extensions .001, .002, .003 ... matching the
order of the instrumentation table. They are ASCII, one value per line,
preceded by a header block giving the sample rate and scale factors.
`--convert` handles that layout and the newer CSV exports.

WHAT TO PICK
For an N2/N3 programme, prefer:
  - vehicle-mounted longitudinal channels at the B-pillar or tunnel
    (not occupant or dummy channels)
  - the heaviest vehicle classes available; light-vehicle barrier pulses
    peak two to three times higher than a loaded tractor unit and will
    push a calibration in the wrong direction
  - both rigid barrier and offset deformable, since crush rate differs

USAGE
    python sim/fetch_nhtsa.py --list-guides
    python sim/fetch_nhtsa.py --convert path/to/test.001 --fs 10000 \
                              --out data/nhtsa_v12345.csv
    python sim/fetch_nhtsa.py --convert-csv path/to/export.csv \
                              --time-col Time --accel-col "X Acceleration"
    python sim/run_sim.py --input data/nhtsa_v12345.csv --plot

NOTE: this container has no route to nhtsa.dot.gov. Run --api from your
own machine.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

API_BASE = "https://nrd.api.nhtsa.dot.gov"
DOC_URLS = {
    "API index / Swagger specs": "https://nrd.api.nhtsa.dot.gov/",
    "Vehicle DB Swagger": "https://nrd.api.nhtsa.dot.gov/nhtsa/vehicle/swagger-ui/index.html",
    "Component DB Swagger": "https://nrd.api.nhtsa.dot.gov/swagger-ui/index.html",
    "Crash test database (NHTSA)": "https://www.nhtsa.gov/research-data/research-testing-databases/",
    "Vehicle Crash Test Database (data.gov)": "https://catalog.data.gov/dataset/vehicle-crash-test-database",
}
G = 9.80665


# ------------------------------------------------------------------ API
def api_get(path: str, params: dict | None = None, timeout=30):
    import urllib.parse
    import urllib.request
    url = API_BASE.rstrip("/") + "/" + path.lstrip("/")
    if params:
        url += "?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers={"Accept": "application/json",
                                               "User-Agent": "edr-sim/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8", "replace"))


# ------------------------------------------------------- signal parsing
def parse_signal_file(path: Path, fs_hz: float | None = None,
                      scale: float | None = None, units: str = "g"):
    """Parse an NHTSA-style ASCII signal file (.001/.002/...).

    The header is key-ish text before the numeric block; we scrape a
    sample rate and scale factor from it where present, and otherwise
    require them on the command line. Everything that parses as a float
    on its own line is treated as a sample."""
    text = path.read_text(errors="replace").splitlines()

    vals, header = [], []
    for line in text:
        t = line.strip()
        if not t:
            continue
        try:
            vals.append(float(t.split()[0]) if len(t.split()) == 1 else float(t))
        except ValueError:
            header.append(t)

    if not vals:
        raise ValueError(f"{path}: no numeric samples found")

    hdr = " ".join(header)
    if fs_hz is None:
        m = re.search(r"(?:sample\s*rate|sampling\s*rate|SAMP)[^\d]{0,12}([\d.]+)", hdr, re.I)
        if m:
            fs_hz = float(m.group(1))
    if fs_hz is None:
        m = re.search(r"(?:time\s*(?:step|increment)|DELTA_T)[^\d]{0,12}([\d.eE+-]+)", hdr, re.I)
        if m and float(m.group(1)) > 0:
            fs_hz = 1.0 / float(m.group(1))
    if fs_hz is None:
        raise ValueError(
            f"{path}: could not find a sample rate in the header. "
            f"Pass --fs explicitly (crash-test channels are commonly "
            f"10000 or 20000 Hz). Header seen:\n  " + hdr[:400])

    if scale is None:
        m = re.search(r"(?:scale|SCALE_FACTOR|sensitivity)[^\d-]{0,12}([\d.eE+-]+)", hdr, re.I)
        scale = float(m.group(1)) if m else 1.0

    a = np.asarray(vals, dtype=float) * scale
    if units.lower() in ("m/s2", "m/s^2", "mps2"):
        a = a / G
    elif units.lower() in ("mg",):
        a = a / 1000.0

    return np.arange(len(a)) / fs_hz, a, fs_hz, hdr[:400]


def write_sim_csv(out: Path, t, ax, ay=None, az=None, speed=None):
    out.parent.mkdir(parents=True, exist_ok=True)
    n = len(t)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "ax_g", "ay_g", "az_g", "speed_kph"])
        for i in range(n):
            w.writerow([f"{t[i]:.7f}", f"{ax[i]:.5f}",
                        f"{ay[i]:.5f}" if ay is not None else "0",
                        f"{az[i]:.5f}" if az is not None else "0",
                        f"{speed[i]:.4f}" if speed is not None else ""])
    print(f"  wrote {out}  ({n} samples)")


def synth_speed(t, ax_g, v0_kph):
    """Derive a plausible VSS channel by integrating the pulse. This is a
    STAND-IN, not data: a real test article has no wheel-speed trace, and
    any Loop B result computed from it is an inference, not a measurement.
    Records produced this way must not be presented as VSS-sourced."""
    v = v0_kph / 3.6 + np.cumsum(ax_g) * G * (t[1] - t[0])
    return np.maximum(v, 0) * 3.6


# ----------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list-guides", action="store_true",
                    help="print the documentation and database URLs")
    ap.add_argument("--api", metavar="PATH",
                    help="raw GET against the NHTSA API, e.g. "
                         "'nhtsa/vehicle/tests?make=FREIGHTLINER'")
    ap.add_argument("--convert", type=Path, help="NHTSA .001-style signal file")
    ap.add_argument("--convert-csv", type=Path, help="CSV export from the web UI")
    ap.add_argument("--time-col", default="Time")
    ap.add_argument("--accel-col", default=None)
    ap.add_argument("--fs", type=float, help="sample rate if not in the header")
    ap.add_argument("--scale", type=float, help="scale factor override")
    ap.add_argument("--units", default="g", choices=["g", "m/s2", "mg"])
    ap.add_argument("--invert", action="store_true",
                    help="flip sign (NHTSA sign conventions vary by channel; "
                         "braking/frontal MUST come out NEGATIVE for this sim)")
    ap.add_argument("--v0-kph", type=float, help="synthesise a speed channel")
    ap.add_argument("--out", type=Path, default=Path("data/nhtsa_pulse.csv"))
    args = ap.parse_args()

    if args.list_guides or not any((args.api, args.convert, args.convert_csv)):
        print("\nNHTSA crash test data - entry points\n")
        for k, v in DOC_URLS.items():
            print(f"  {k:42s} {v}")
        print("\n  The 2024 rework changed endpoint paths; confirm against the")
        print("  Swagger UI above before scripting --api. The --convert path")
        print("  needs no network access and is the dependable route.\n")
        return 0

    if args.api:
        try:
            print(json.dumps(api_get(args.api), indent=2)[:8000])
        except Exception as e:
            print(f"  request failed: {e}", file=sys.stderr)
            print("  Confirm the path against the Swagger UI "
                  "(--list-guides) and check network access.", file=sys.stderr)
            return 1
        return 0

    if args.convert:
        t, a, fs, hdr = parse_signal_file(args.convert, args.fs, args.scale, args.units)
        if args.invert:
            a = -a
        print(f"  {args.convert.name}: {len(a)} samples @ {fs:.0f} Hz, "
              f"peak {np.abs(a).max():.1f} g, dV {np.cumsum(a).max() * G / fs:.2f} m/s")
        if a[np.argmax(np.abs(a))] > 0:
            print("  ! peak is POSITIVE. For a frontal impact this sim expects a "
                  "NEGATIVE longitudinal pulse - consider --invert.")
        sp = synth_speed(t, a, args.v0_kph) if args.v0_kph else None
        write_sim_csv(args.out, t, a, speed=sp)
        return 0

    if args.convert_csv:
        with open(args.convert_csv, newline="") as f:
            rows = list(csv.reader(f))
        hdr = [h.strip() for h in rows[0]]
        ti = hdr.index(args.time_col) if args.time_col in hdr else 0
        if args.accel_col and args.accel_col in hdr:
            ai = hdr.index(args.accel_col)
        else:
            cand = [i for i, h in enumerate(hdr)
                    if re.search(r"accel|\bacc\b|\bg\b", h, re.I) and i != ti]
            if not cand:
                print(f"  no acceleration column found. Columns: {hdr}", file=sys.stderr)
                return 1
            ai = cand[0]
            print(f"  using column {hdr[ai]!r}")
        d = np.array([[float(r[ti]), float(r[ai])] for r in rows[1:]
                      if len(r) > max(ti, ai) and r[ti].strip()])
        t, a = d[:, 0], d[:, 1]
        if t.max() > 100:          # milliseconds
            t = t / 1000.0
            print("  time column looked like ms; converted to s")
        if args.units != "g":
            a = a / (G if args.units.startswith("m/s") else 1000.0)
        if args.invert:
            a = -a
        sp = synth_speed(t, a, args.v0_kph) if args.v0_kph else None
        write_sim_csv(args.out, t, a, speed=sp)
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
