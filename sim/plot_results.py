#!/usr/bin/env python3
"""
plot_results.py - four-pane diagnostic view for calibration review.

  1  raw vs filtered longitudinal acceleration, with the bias estimate
  2  the three dV measures against the boundary curve and the flat spec
  3  Loop B: deceleration, threshold and dwell accumulation
  4  state of both loops, with trigger and inhibit markers

Pane 2 is the one to look at first: if the cumulative trace crosses the
boundary well before the sliding-window trace reaches the flat line, the
boundary curve is doing the discrimination and the flat threshold is
close to redundant.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from edr_core import STATE_A, STATE_B, decode_mask, INHIBIT_NAMES  # noqa: E402


def plot(res, title="EDR", save: Path | None = None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = res.t
    fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True,
                           gridspec_kw={"height_ratios": [2, 2.4, 2, 1.2]})
    fig.suptitle(f"EDR crash detection - {title}", fontsize=13, y=0.98)

    # ---- 1 acceleration -------------------------------------------------
    ax[0].plot(t, res.a_long_in, lw=0.5, color="0.75", label="raw longitudinal")
    ax[0].plot(t, res.a_long_filt, lw=1.1, color="#c0392b", label="filtered, bias removed")
    ax[0].plot(t, res.a_lat_filt, lw=0.9, color="#2980b9", alpha=0.8, label="lateral")
    ax[0].plot(t, res.bias_long, lw=1.0, color="#27ae60", ls="--", label="bias estimate")
    ax[0].set_ylabel("acceleration [g]")
    ax[0].legend(loc="upper right", fontsize=8, ncol=2)
    ax[0].grid(alpha=0.25)

    # ---- 2 delta-V ------------------------------------------------------
    ax[1].plot(t, res.dv_main, lw=1.4, color="#c0392b", label="dV sliding 150 ms")
    ax[1].plot(t, res.dv_cum, lw=1.4, color="#8e44ad", label="dV cumulative since arm")
    ax[1].plot(t, res.dv_safing, lw=1.0, color="#16a085", alpha=0.8, label="dV safing 300 ms")
    bd = np.where(res.boundary > 0, res.boundary, np.nan)
    ax[1].plot(t, bd, lw=1.6, color="k", ls="--", label="dV-vs-t boundary")
    ax[1].axhline(6.944, color="#d35400", ls=":", lw=1.4, label="25 km/h spec")
    ax[1].axhline(2.0, color="#16a085", ls=":", lw=1.0, label="safing threshold")
    ax[1].set_ylabel("delta-V [m/s]")
    ax[1].legend(loc="upper left", fontsize=8, ncol=2)
    ax[1].grid(alpha=0.25)

    # ---- 3 Loop B -------------------------------------------------------
    ax[2].plot(t, res.decel, lw=1.2, color="#2c3e50", label="decel (LS slope of VSS)")
    ax[2].axhline(3.25, color="#c0392b", ls="--", lw=1.3, label="3.25 m/s^2 (AIS-220)")
    ax[2].set_ylabel("deceleration [m/s$^2$]")
    ax[2].set_ylim(-1, max(6, float(np.nanmax(res.decel)) * 1.15))
    ax[2].grid(alpha=0.25)
    a2 = ax[2].twinx()
    a2.plot(t, res.dwell, lw=1.1, color="#e67e22", alpha=0.9, label="dwell")
    a2.axhline(700, color="#e67e22", ls=":", lw=1.1)
    a2.set_ylabel("dwell [ms]", color="#e67e22")
    a2.tick_params(axis="y", colors="#e67e22")
    h1, l1 = ax[2].get_legend_handles_labels()
    h2, l2 = a2.get_legend_handles_labels()
    ax[2].legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8)

    if not np.all(np.isnan(res.speed_kph)):
        a2b = ax[2].twinx()
        a2b.spines["right"].set_position(("outward", 44))
        a2b.plot(t, res.speed_kph, lw=0.9, color="#7f8c8d", alpha=0.6)
        a2b.set_ylabel("speed [km/h]", color="#7f8c8d")
        a2b.tick_params(axis="y", colors="#7f8c8d")

    # ---- 4 states -------------------------------------------------------
    ax[3].step(t, res.stateA, where="post", lw=1.3, color="#c0392b", label="Loop A")
    ax[3].step(t, res.stateB, where="post", lw=1.3, color="#2c3e50",
               ls="--", alpha=0.8, label="Loop B")
    ax[3].set_yticks(range(4))
    ax[3].set_yticklabels([f"{a}/{b}" for a, b in zip(STATE_A, STATE_B)], fontsize=7)
    ax[3].set_ylabel("state")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(loc="upper left", fontsize=8)
    ax[3].grid(alpha=0.25)

    inh = res.inhibit != 0
    if inh.any():
        ax[3].fill_between(t, 0, 3, where=inh, color="#f39c12", alpha=0.12, step="post")

    for e in res.events:
        col = "#c0392b" if e.trigger_mask & 0b0110 else "#2980b9"
        for a in ax:
            a.axvline(e.t_s, color=col, lw=1.1, alpha=0.75)
        ax[1].annotate(f"{e.triggers}\n{e.dv_kph:.1f} km/h"
                       + ("\nLOCKED" if e.locked else "")
                       + ("\nCLIPPED" if e.dv_is_lower_bound else ""),
                       xy=(e.t_s, res.dv_main.max() * 0.92),
                       fontsize=7.5, color=col,
                       ha="left" if e.t_s < t[-1] * 0.7 else "right")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if save:
        Path(save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save, dpi=130)
        print(f"  plot -> {save}")
    if show:
        plt.show()
    return fig


def plot_boundary(sim, save=None, show=True):
    """The calibration curve on its own, for the design review pack."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(0, 420, 2)
    v = [sim.boundary_at(int(x)) for x in t]
    fig, a = plt.subplots(figsize=(7.5, 4.5))
    a.plot(t, v, lw=2, color="k")
    a.axhline(6.944, color="#d35400", ls=":", lw=1.4)
    a.annotate("25 km/h @ 150 ms\n(contractual spec point)", xy=(150, 6.944),
               xytext=(190, 5.0), fontsize=8,
               arrowprops=dict(arrowstyle="->", lw=0.8))
    a.fill_between(t, v, 13, alpha=0.10, color="#c0392b")
    a.fill_between(t, 0, v, alpha=0.10, color="#27ae60")
    a.text(60, 10.5, "FIRE", fontsize=11, color="#c0392b", weight="bold")
    a.text(230, 2.2, "NO FIRE", fontsize=11, color="#27ae60", weight="bold")
    a.set_xlabel("time since Loop A armed [ms]")
    a.set_ylabel("cumulative delta-V [m/s]")
    a.set_title("dV-vs-time deployment boundary")
    a.set_ylim(0, 13)
    a.grid(alpha=0.25)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
        print(f"  plot -> {save}")
    if show:
        plt.show()
    return fig


if __name__ == "__main__":
    import pulses
    from edr_core import EdrSim
    name = sys.argv[1] if len(sys.argv) > 1 else "truck_frontal_rigid_40kph"
    t, ax_, ay, az, sp, s = pulses.render(name)
    plot(EdrSim().run(t, ax_, ay, az, sp), name)
