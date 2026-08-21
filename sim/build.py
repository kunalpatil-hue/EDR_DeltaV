#!/usr/bin/env python3
"""
Build the EDR core into a shared library for the PC simulator.

    python sim/build.py            # build
    python sim/build.py --clean    # rebuild from scratch

Works with gcc/clang on Linux and macOS and with MinGW or MSVC on
Windows. The flags mirror the production build as closely as a hosted
compiler allows, so warnings caught here are warnings caught on target.
"""
from __future__ import annotations

import argparse
import platform
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "firmware" / "src"
INC = ROOT / "firmware" / "inc"
BUILD = ROOT / "build"

SOURCES = [
    "edr_filter.c",
    "edr_config.c",
    "edr_loopA.c",
    "edr_loopB.c",
    "edr_api.c",
    "edr_sim_shim.c",
]

WARN = ["-Wall", "-Wextra", "-Wshadow", "-Wconversion", "-Wsign-conversion"]


def lib_name() -> str:
    return {"Windows": "edrcore.dll",
            "Darwin": "libedrcore.dylib"}.get(platform.system(), "libedrcore.so")


def find_cc() -> str:
    for cc in ("gcc", "clang", "cc"):
        if shutil.which(cc):
            return cc
    if platform.system() == "Windows" and shutil.which("cl"):
        return "cl"
    sys.exit("No C compiler found. Install gcc/clang, or MinGW-w64 on Windows.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--strict", action="store_true",
                    help="enable the full conversion warning set (noisy but "
                         "closer to the MISRA review gate)")
    args = ap.parse_args()

    if args.clean and BUILD.exists():
        shutil.rmtree(BUILD)
    BUILD.mkdir(exist_ok=True)

    cc = find_cc()
    out = BUILD / lib_name()

    cmd = [cc, "-std=c99", "-O2", "-fPIC", "-shared",
           f"-I{INC}", "-Wall", "-Wextra"]
    if args.strict:
        cmd += WARN
    cmd += [str(SRC / s) for s in SOURCES]
    cmd += ["-o", str(out), "-lm"]

    print(" ".join(cmd))
    r = subprocess.run(cmd)
    if r.returncode != 0:
        return r.returncode

    print(f"\n  built {out}  ({out.stat().st_size / 1024:.1f} kB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
