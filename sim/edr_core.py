"""
edr_core.py - ctypes binding to the compiled EDR detection core.

The Python side does I/O, resampling and plotting. It does NOT reimplement
any part of the algorithm: every decision in the simulator output came out
of the same C that goes into the ECU image.
"""
from __future__ import annotations

import ctypes
import platform
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"

EV_STRIDE = 22  # keep in sync with EV_STRIDE in edr_sim_shim.c

TRIGGER_NAMES = {
    1 << 0: "SUDDEN_DECEL",
    1 << 1: "DELTA_V_WINDOW",
    1 << 2: "DELTA_V_CURVE",
    1 << 3: "SRS_DEPLOY",
    1 << 4: "ROLLOVER",
}
INHIBIT_NAMES = {
    1 << 0: "ROUGH_ROAD",
    1 << 1: "JERK_SPIKE",
    1 << 2: "NO_SAFING",
    1 << 3: "LOCKOUT",
    1 << 4: "NOT_VALID",
}
STATE_A = ["IDLE", "ARMED", "FIRED", "LOCKOUT"]
STATE_B = ["IDLE", "DWELL", "FIRED", "LATCHED"]


def decode_mask(mask: int, names: dict[int, str]) -> str:
    hits = [n for bit, n in names.items() if mask & bit]
    return "+".join(hits) if hits else "-"


def _libpath() -> Path:
    name = {"Windows": "edrcore.dll",
            "Darwin": "libedrcore.dylib"}.get(platform.system(), "libedrcore.so")
    p = BUILD / name
    if not p.exists():
        raise FileNotFoundError(
            f"{p} not found. Run:  python sim/build.py")
    return p


_D = ctypes.POINTER(ctypes.c_double)
_I = ctypes.POINTER(ctypes.c_int)


def _load() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_libpath()))
    lib.edr_sim_create.restype = ctypes.c_void_p
    lib.edr_sim_destroy.argtypes = [ctypes.c_void_p]
    lib.edr_sim_set.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double]
    lib.edr_sim_set.restype = ctypes.c_int
    lib.edr_sim_set_boundary.argtypes = [ctypes.c_void_p, _I, _I, ctypes.c_int]
    lib.edr_sim_set_boundary.restype = ctypes.c_int
    lib.edr_sim_init.argtypes = [ctypes.c_void_p]
    lib.edr_sim_init.restype = ctypes.c_int
    lib.edr_sim_boundary_at.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.edr_sim_boundary_at.restype = ctypes.c_double
    lib.edr_sim_run.argtypes = [
        ctypes.c_void_p, ctypes.c_int,
        _D, _D, _D, _D, ctypes.c_int, _I,
        _D, _D, _D, _D, _D, _D, _D, _D, _D,
        _I, _I, _I, _I,
        ctypes.c_int, _D, _I,
    ]
    lib.edr_sim_run.restype = ctypes.c_int
    lib.edr_sim_get_record.argtypes = [ctypes.c_void_p, _D, _D, _D, _I, ctypes.c_int]
    lib.edr_sim_get_record.restype = ctypes.c_int
    return lib


_LIB = None


def lib() -> ctypes.CDLL:
    global _LIB
    if _LIB is None:
        _LIB = _load()
    return _LIB


# --------------------------------------------------------------------------
@dataclass
class Event:
    t_s: float
    seq_index: int
    trigger_mask: int
    inhibit_mask: int
    dv_long: float
    dv_lat: float
    dv_res: float
    dv_max: float
    t_to_dv_max_ms: float
    peak_a_long_g: float
    peak_a_lat_g: float
    pdof_deg: float
    decel_ms2: float
    decel_dwell_ms: float
    speed_kph: float
    locked: bool
    dv_is_lower_bound: bool
    decel_src_is_vss: bool

    @property
    def triggers(self) -> str:
        return decode_mask(self.trigger_mask, TRIGGER_NAMES)

    @property
    def dv_kph(self) -> float:
        return self.dv_res * 3.6


@dataclass
class SimResult:
    t: np.ndarray
    a_long_in: np.ndarray
    a_long_filt: np.ndarray
    a_lat_filt: np.ndarray
    bias_long: np.ndarray
    dv_main: np.ndarray
    dv_cum: np.ndarray
    dv_safing: np.ndarray
    boundary: np.ndarray
    decel: np.ndarray
    dwell: np.ndarray
    stateA: np.ndarray
    stateB: np.ndarray
    trigger: np.ndarray
    inhibit: np.ndarray
    speed_kph: np.ndarray
    events: list[Event] = field(default_factory=list)
    record: dict | None = None
    fs_hz: int = 833


class EdrSim:
    """One instance == one ECU. Not reusable across runs (state is latched
    on purpose, exactly as on target)."""

    def __init__(self, config: dict | None = None, boundary: list | None = None):
        self._h = lib().edr_sim_create()
        if not self._h:
            raise MemoryError("edr_sim_create failed")
        self.config = dict(config or {})
        for k, v in self.config.items():
            rc = lib().edr_sim_set(self._h, k.encode(), float(v))
            if rc == -2:
                raise KeyError(f"unknown config key: {k!r}")
        if boundary:
            t = (ctypes.c_int * len(boundary))(*[int(p[0]) for p in boundary])
            d = (ctypes.c_int * len(boundary))(*[int(p[1]) for p in boundary])
            if lib().edr_sim_set_boundary(self._h, t, d, len(boundary)) != 0:
                raise ValueError("invalid boundary table")
        rc = lib().edr_sim_init(self._h)
        if rc != 0:
            raise ValueError(
                f"edr_config_validate rejected the calibration (code {rc}); "
                "see edr_config.c for the field each code maps to")
        self.fs_hz = int(self.config.get("fast_hz", 833))

    def __del__(self):
        try:
            if getattr(self, "_h", None):
                lib().edr_sim_destroy(self._h)
                self._h = None
        except Exception:
            pass

    def boundary_at(self, t_ms: int) -> float:
        return lib().edr_sim_boundary_at(self._h, int(t_ms)) / 1000.0

    def run(self, t, ax_g, ay_g=None, az_g=None, speed_kph=None,
            srs=None, max_events=64) -> SimResult:
        n = len(ax_g)
        ax = np.ascontiguousarray(ax_g, dtype=np.float64)
        ay = np.ascontiguousarray(ay_g if ay_g is not None else np.zeros(n), dtype=np.float64)
        az = np.ascontiguousarray(az_g if az_g is not None else np.zeros(n), dtype=np.float64)
        has_speed = speed_kph is not None
        sp = np.ascontiguousarray(speed_kph if has_speed else np.zeros(n), dtype=np.float64)
        sr = np.ascontiguousarray(srs if srs is not None else np.zeros(n), dtype=np.int32)

        outs = {k: np.zeros(n, dtype=np.float64) for k in
                ("a_long", "a_lat", "bias_long", "dv_main", "dv_cum",
                 "dv_safing", "boundary", "decel", "dwell")}
        iouts = {k: np.zeros(n, dtype=np.int32) for k in
                 ("stateA", "stateB", "trigger", "inhibit")}
        ev = np.zeros(max_events * EV_STRIDE, dtype=np.float64)
        n_ev = ctypes.c_int(0)

        def dp(a):
            return a.ctypes.data_as(_D)

        def ip(a):
            return a.ctypes.data_as(_I)

        rc = lib().edr_sim_run(
            self._h, n, dp(ax), dp(ay), dp(az),
            dp(sp), 1 if has_speed else 0, ip(sr),
            dp(outs["a_long"]), dp(outs["a_lat"]), dp(outs["bias_long"]),
            dp(outs["dv_main"]), dp(outs["dv_cum"]), dp(outs["dv_safing"]),
            dp(outs["boundary"]), dp(outs["decel"]), dp(outs["dwell"]),
            ip(iouts["stateA"]), ip(iouts["stateB"]),
            ip(iouts["trigger"]), ip(iouts["inhibit"]),
            max_events, dp(ev), ctypes.byref(n_ev))
        if rc != 0:
            raise RuntimeError(f"edr_sim_run returned {rc}")

        events = []
        for i in range(n_ev.value):
            e = ev[i * EV_STRIDE:(i + 1) * EV_STRIDE]
            events.append(Event(
                t_s=e[0], seq_index=int(e[1]), trigger_mask=int(e[2]),
                inhibit_mask=int(e[3]), dv_long=e[4], dv_lat=e[5],
                dv_res=e[6], dv_max=e[7], t_to_dv_max_ms=e[8],
                peak_a_long_g=e[9], peak_a_lat_g=e[10], pdof_deg=e[11],
                decel_ms2=e[12], decel_dwell_ms=e[13], speed_kph=e[14],
                locked=bool(e[15]), dv_is_lower_bound=bool(e[16]),
                decel_src_is_vss=bool(e[17])))

        # Annexure 4 record readback
        m = 24
        ra, rl, rs = (np.zeros(m) for _ in range(3))
        rsat = np.zeros(m, dtype=np.int32)
        got = lib().edr_sim_get_record(self._h, dp(ra), dp(rl), dp(rs), ip(rsat), m)
        record = {"a_long_g": ra[:got], "a_lat_g": rl[:got],
                  "speed_kph": rs[:got], "saturated": rsat[:got]}

        return SimResult(
            t=np.asarray(t, dtype=np.float64), a_long_in=ax,
            a_long_filt=outs["a_long"], a_lat_filt=outs["a_lat"],
            bias_long=outs["bias_long"], dv_main=outs["dv_main"],
            dv_cum=outs["dv_cum"], dv_safing=outs["dv_safing"],
            boundary=outs["boundary"], decel=outs["decel"], dwell=outs["dwell"],
            stateA=iouts["stateA"], stateB=iouts["stateB"],
            trigger=iouts["trigger"], inhibit=iouts["inhibit"],
            speed_kph=sp if has_speed else np.full(n, np.nan),
            events=events, record=record, fs_hz=self.fs_hz)
