"""Paper figure: per-sample lateral error |e_y| for all four methods on
seed 12, encoded as color along the executed planned trajectory.

Layout: 2x2 small multiples sharing a single horizontal colorbar.
Each panel overlays the lap reference (dashed) and the planned path
colored by |e_y(t)| with a fixed shared scale across all four methods.

Source data:
    analysis_results/full_lap_bag_20260528_080853/
        M1_vanilla_seed12_timeseries.csv
        M0_intent_mpc_seed12_timeseries.csv
        M5_dra_mppi_seed12_timeseries.csv
        M4_im2_full_seed12_timeseries.csv
    autonomous_flight/cfg/mpc_navigation/ref_trajectory.txt

Outputs:
    analysis_results/tracking_error_four_methods_seed12.{pdf,png}
"""
from __future__ import annotations

import csv
import math
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

REPO_ROOT = Path(__file__).resolve().parents[2]
SEED_DIR = (REPO_ROOT / "analysis_results"
            / "full_lap_bag_20260528_080853"
            / "topdown_plots" / "seed12_with_obstacles")
sys.path.insert(0, str(SEED_DIR))
from plot_seed12_with_obstacles import (  # type: ignore
    DATA_DIR,
    read_reference_path,
)

SEED = 12
OUT_DIR = REPO_ROOT / "analysis_results"

METHODS = [
    ("M1_vanilla",     "MPPI"),
    ("M0_intent_mpc",  "Intent-MPC"),
    ("M5_dra_mppi",    "DRA-MPPI"),
    ("M4_im2_full",    "Ours (IM2-MPPI)"),
]


# --------------------------------------------------------------------------
def read_xy(config: str, seed: int) -> np.ndarray:
    """Return (N, 2): x, y."""
    path = os.path.join(DATA_DIR, f"{config}_seed{seed}_timeseries.csv")
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                rows.append((x, y))
    return np.asarray(rows, dtype=float)


def reference_xy_tangent() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pts = np.asarray(read_reference_path(), dtype=float)
    rx, ry = pts[:, 0], pts[:, 1]
    psi = np.arctan2(np.gradient(ry), np.gradient(rx))
    return rx, ry, psi


def signed_lateral(traj: np.ndarray, rx: np.ndarray, ry: np.ndarray,
                   psi_ref: np.ndarray) -> np.ndarray:
    """For each row of traj (x, y) return signed cross-track error to the
    nearest reference point (+ve = left of reference heading)."""
    out = np.empty(len(traj))
    for i, (x, y) in enumerate(traj):
        j = int(np.argmin((rx - x) ** 2 + (ry - y) ** 2))
        dx, dy = x - rx[j], y - ry[j]
        nx, ny = -math.sin(psi_ref[j]), math.cos(psi_ref[j])
        out[i] = dx * nx + dy * ny
    return out


def colored_path(ax, xy: np.ndarray, vals: np.ndarray, *, vmin: float,
                 vmax: float, cmap, lw: float = 1.9):
    """Draw xy as a polyline whose color encodes vals."""
    segs = np.stack([xy[:-1], xy[1:]], axis=1)         # (N-1, 2, 2)
    cvals = 0.5 * (vals[:-1] + vals[1:])               # per-segment colour
    lc = LineCollection(segs, cmap=cmap, linewidths=lw,
                        capstyle="round")
    lc.set_array(cvals)
    lc.set_clim(vmin, vmax)
    ax.add_collection(lc)
    return lc


# --------------------------------------------------------------------------
def main() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.titlesize": 10,
        "legend.fontsize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "axes.linewidth": 0.8,
        "mathtext.fontset": "cm",
    })

    rx, ry, psi_ref = reference_xy_tangent()

    # ---- load all methods and compute errors first (for shared colour limits) ----
    data = {}
    for cfg, label in METHODS:
        try:
            xy = read_xy(cfg, SEED)
        except FileNotFoundError:
            print(f"skip {cfg}: csv missing")
            continue
        if len(xy) < 2:
            print(f"skip {cfg}: too few points")
            continue
        ey = signed_lateral(xy, rx, ry, psi_ref)
        rmse = float(np.sqrt(np.mean(ey ** 2)))
        emax = float(np.max(np.abs(ey)))
        print(f"{cfg:14s}  N={len(xy):5d}  RMSE={rmse:.3f}  max={emax:.3f}")
        data[cfg] = (xy, ey, rmse, emax, label)

    abs_max = max(d[3] for d in data.values())
    # use a hard-clipped scale just slightly above the per-method max, so the
    # colour ramp is well used; round up to nearest 0.1
    vmax = math.ceil(abs_max * 10) / 10.0
    vmin = 0.0
    cmap = plt.get_cmap("viridis")
    print(f"shared colour scale: vmin={vmin}, vmax={vmax}")

    # ---- shared plot bounds (over all methods + reference) ----
    all_xy = np.vstack([d[0] for d in data.values()] +
                       [np.column_stack([rx, ry])])
    pad = 0.6
    xmin, xmax = all_xy[:, 0].min() - pad, all_xy[:, 0].max() + pad
    ymin, ymax = all_xy[:, 1].min() - pad, all_xy[:, 1].max() + pad

    # ---- figure ----
    fig, axes = plt.subplots(2, 2, figsize=(8.0, 8.4),
                             sharex=True, sharey=True)
    axes = axes.ravel()

    last_lc = None
    for ax, (cfg, _) in zip(axes, METHODS):
        if cfg not in data:
            ax.set_axis_off()
            continue
        xy, ey, rmse, emax, label = data[cfg]

        ax.plot(rx, ry, color="#37474f", linestyle=(0, (6, 3)),
                linewidth=1.2, alpha=0.9, zorder=1)
        last_lc = colored_path(ax, xy, np.abs(ey),
                               vmin=vmin, vmax=vmax, cmap=cmap, lw=2.0)

        ax.set_title(label, loc="left", fontsize=10, pad=4)
        ax.text(
            0.985, 0.985,
            (f"$\\mathrm{{RMSE}}_y$={rmse:.3f} m\n"
             f"$e_{{y,\\max}}$={emax:.3f} m"),
            transform=ax.transAxes, va="top", ha="right", fontsize=8,
            bbox=dict(boxstyle="round,pad=0.30", facecolor="white",
                      edgecolor="#90a4ae", linewidth=0.6, alpha=0.94),
        )
        ax.set_aspect("equal")
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.55)

    # shared axis labels: only put labels on outer axes
    for ax in (axes[2], axes[3]):
        ax.set_xlabel("$x$ [m]")
    for ax in (axes[0], axes[2]):
        ax.set_ylabel("$y$ [m]")

    fig.suptitle("Tracking error along the lap — seed 12, four methods",
                 fontsize=11, y=0.995)

    fig.tight_layout(rect=(0, 0.09, 1, 0.98))

    # ---- shared horizontal colorbar at the bottom ----
    cbar_ax = fig.add_axes([0.18, 0.058, 0.66, 0.022])
    cb = fig.colorbar(last_lc, cax=cbar_ax, orientation="horizontal")
    cb.set_label(r"lateral error magnitude $|e_y|$  [m]", labelpad=6)
    cb.ax.tick_params(labelsize=8, pad=2)
    cb.outline.set_edgecolor("#90a4ae")
    cb.outline.set_linewidth(0.6)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    pdf = OUT_DIR / "tracking_error_four_methods_seed12.pdf"
    png = OUT_DIR / "tracking_error_four_methods_seed12.png"
    fig.savefig(pdf, bbox_inches="tight")
    fig.savefig(png, dpi=300, bbox_inches="tight")
    print(f"wrote {pdf}")
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
