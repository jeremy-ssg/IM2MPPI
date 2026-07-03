"""Paper figure: tracking-error illustration computed on real bag data.

Source data:
    analysis_results/full_lap_bag_20260528_080853/
        M4_im2_full_seed12_timeseries.csv      (planned trajectory of Ours)
        ...                                    (other methods, unused here)
    autonomous_flight/cfg/mpc_navigation/ref_trajectory.txt  (reference lap)

Definition convention shown in the figure (standard Stanley/pure-pursuit
cross-track formulation):

    For each planned sample p_k, project it onto the reference path to get
    the nearest point p_k^ref. The error vector p_k - p_k^ref is
    perpendicular to the reference tangent t_ref at p_k^ref by construction,
    so its magnitude equals the lateral / cross-track error
        e_y = sign * || p_k - p_k^ref ||
    with the sign chosen to be positive when p_k lies to the LEFT of the
    reference travel direction.  The heading error is
        e_psi = atan2( sin(psi_plan - psi_ref), cos(psi_plan - psi_ref) )
    where psi_plan comes from the planned velocity direction (vx, vy) and
    psi_ref is the local reference tangent angle.

Outputs:
    analysis_results/tracking_error_illustration_seed12.{pdf,png}
"""

from __future__ import annotations

import csv
import math
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, FancyArrowPatch

# ---------------------------------------------------------------------------
# Reuse the project's existing loaders
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[2]
SEED_DIR = (REPO_ROOT / "analysis_results"
            / "full_lap_bag_20260528_080853"
            / "topdown_plots" / "seed12_with_obstacles")
sys.path.insert(0, str(SEED_DIR))
from plot_seed12_with_obstacles import (  # type: ignore
    DATA_DIR,
    REF_PATH,
    read_path,
    read_reference_path,
)

SEED = 12
METHOD = "M4_im2_full"
OUT_DIR = REPO_ROOT / "analysis_results"


# ---------------------------------------------------------------------------
# Trajectory + heading from the timeseries CSV
# ---------------------------------------------------------------------------

def read_timeseries(config: str, seed: int) -> np.ndarray:
    """Return (N, 4): t, x, y, psi_from_velocity."""
    path = os.path.join(DATA_DIR, f"{config}_seed{seed}_timeseries.csv")
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["t"])
                x = float(row["x"])
                y = float(row["y"])
                vx = float(row["vx"])
                vy = float(row["vy"])
            except (KeyError, ValueError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            psi = math.atan2(vy, vx) if (vx * vx + vy * vy) > 1e-6 else math.nan
            rows.append((t, x, y, psi))
    return np.asarray(rows, dtype=float)


def reference_with_tangent() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (rx, ry, psi_ref) plus cumulative arclength as a side product."""
    pts = read_reference_path()
    arr = np.asarray(pts, dtype=float)
    rx, ry = arr[:, 0], arr[:, 1]
    # closed loop → use wrap-around finite differences
    dx = np.gradient(rx)
    dy = np.gradient(ry)
    psi_ref = np.arctan2(dy, dx)
    return rx, ry, psi_ref


# ---------------------------------------------------------------------------
# Error computation
# ---------------------------------------------------------------------------

def nearest_index(px: float, py: float,
                  rx: np.ndarray, ry: np.ndarray) -> int:
    return int(np.argmin((rx - px) ** 2 + (ry - py) ** 2))


def signed_lateral_error(px: float, py: float,
                         rxk: float, ryk: float,
                         psi_ref_k: float) -> float:
    """Cross-track error, +ve when (px,py) is left of the reference travel
    direction at (rxk, ryk)."""
    dx = px - rxk
    dy = py - ryk
    # left-normal of tangent (cos, sin) is (-sin, cos)
    nx = -math.sin(psi_ref_k)
    ny = math.cos(psi_ref_k)
    return dx * nx + dy * ny


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

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
        "lines.linewidth": 1.4,
        "mathtext.fontset": "cm",
    })

    # ---- load real data ----
    ts = read_timeseries(METHOD, SEED)             # (N, 4)
    rx, ry, psi_ref = reference_with_tangent()     # closed-loop reference
    print(f"loaded planned: {ts.shape[0]} samples; reference: {len(rx)} pts")

    # ---- per-station nearest-projection error ----
    px_full = ts[:, 1]
    py_full = ts[:, 2]
    psi_plan_full = ts[:, 3]

    # subsample planned trajectory for plotting (every K) — but compute error
    # densely first for accurate RMSE
    err_y_dense = np.empty(len(px_full))
    for i in range(len(px_full)):
        j = nearest_index(px_full[i], py_full[i], rx, ry)
        err_y_dense[i] = signed_lateral_error(
            px_full[i], py_full[i], rx[j], ry[j], psi_ref[j])
    rmse = float(np.sqrt(np.mean(err_y_dense ** 2)))
    emax = float(np.max(np.abs(err_y_dense)))
    print(f"e_y dense:  RMSE = {rmse:.4f} m   max = {emax:.4f} m")

    # evenly-spaced display stations (arclength-based)
    cum = np.concatenate(([0.0], np.cumsum(np.hypot(
        np.diff(px_full), np.diff(py_full)))))
    total_len = cum[-1]
    n_bars = 26
    target_s = np.linspace(0.04 * total_len, 0.96 * total_len, n_bars)
    bar_idx = np.searchsorted(cum, target_s)
    bar_idx = np.clip(bar_idx, 1, len(px_full) - 2)

    # ---- main figure ----
    fig, ax = plt.subplots(figsize=(7.4, 5.6))

    ax.plot(rx, ry, color="#37474f", linestyle=(0, (6, 3)),
            linewidth=1.5, label="reference path")
    ax.plot(px_full, py_full, color="#c0392b",
            linewidth=1.7, label="planned path (Ours, seed 12)")

    for ii, k in enumerate(bar_idx):
        j = nearest_index(px_full[k], py_full[k], rx, ry)
        ax.plot([px_full[k], rx[j]], [py_full[k], ry[j]],
                color="#1f6feb", linewidth=1.3, alpha=0.95,
                solid_capstyle="round",
                label="lateral error $e_y$" if ii == 0 else None)
        ax.plot(px_full[k], py_full[k], "o",
                color="#c0392b", markersize=3.6, markeredgewidth=0)
        ax.plot(rx[j], ry[j], "s",
                color="#37474f", markersize=3.0, markeredgewidth=0)

    ax.text(
        0.015, 0.985,
        (f"seed {SEED}, method: Ours (M4)\n"
         f"$\\mathrm{{RMSE}}_y = {rmse:.3f}\\,\\mathrm{{m}}$\n"
         f"$e_{{y,\\max}} = {emax:.3f}\\,\\mathrm{{m}}$"),
        transform=ax.transAxes, va="top", ha="left", fontsize=8,
        bbox=dict(boxstyle="round,pad=0.32", facecolor="white",
                  edgecolor="#90a4ae", linewidth=0.6),
    )

    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.6)
    ax.legend(loc="lower right", frameon=True, framealpha=0.95,
              edgecolor="#90a4ae", borderpad=0.4, handlelength=1.8)

    # ---- inset: zoom on the sample with the largest |e_y| ----
    candidates = bar_idx
    abs_err = np.array([abs(err_y_dense[k]) for k in candidates])
    k_zoom = int(candidates[int(np.argmax(abs_err))])
    j_zoom = nearest_index(px_full[k_zoom], py_full[k_zoom], rx, ry)

    cx, cy = rx[j_zoom], ry[j_zoom]
    tx_ref = math.cos(psi_ref[j_zoom])
    ty_ref = math.sin(psi_ref[j_zoom])

    # planned tangent: average over a short window for stability
    win = 5
    a = max(k_zoom - win, 0)
    b = min(k_zoom + win, len(px_full) - 1)
    dxp = px_full[b] - px_full[a]
    dyp = py_full[b] - py_full[a]
    nrm = math.hypot(dxp, dyp) or 1.0
    tx_pl, ty_pl = dxp / nrm, dyp / nrm

    # inset window size: scale to the local error magnitude
    e_mag = math.hypot(px_full[k_zoom] - cx, py_full[k_zoom] - cy)
    L = max(0.35, 2.4 * e_mag)

    ax_in = ax.inset_axes([0.012, 0.012, 0.36, 0.40])
    ax_in.set_xlim(cx - L, cx + L)
    ax_in.set_ylim(cy - L * 0.85, cy + L * 0.85)
    ax_in.set_aspect("equal")
    ax_in.set_xticks([])
    ax_in.set_yticks([])
    for sp in ax_in.spines.values():
        sp.set_edgecolor("#90a4ae")
        sp.set_linewidth(0.8)

    # local stretches
    rwin = 40
    aa = max(j_zoom - rwin, 0)
    bb = min(j_zoom + rwin, len(rx) - 1)
    ax_in.plot(rx[aa:bb], ry[aa:bb], color="#37474f",
               linestyle=(0, (6, 3)), linewidth=1.3)
    pwin = 60
    aa = max(k_zoom - pwin, 0)
    bb = min(k_zoom + pwin, len(px_full) - 1)
    ax_in.plot(px_full[aa:bb], py_full[aa:bb],
               color="#c0392b", linewidth=1.5)

    def arrow(ax_, x0, y0, x1, y1, color, lw=1.0):
        ax_.add_patch(FancyArrowPatch(
            (x0, y0), (x1, y1), arrowstyle="-|>",
            mutation_scale=9, color=color, linewidth=lw,
            shrinkA=0, shrinkB=0))

    # lateral error arrow from ref point to planned point
    arrow(ax_in, cx, cy, px_full[k_zoom], py_full[k_zoom],
          "#1f6feb", lw=1.3)

    # tangents
    Lt = 0.45 * L
    arrow(ax_in, cx, cy, cx + Lt * tx_ref, cy + Lt * ty_ref, "#37474f", 0.95)
    arrow(ax_in, px_full[k_zoom], py_full[k_zoom],
          px_full[k_zoom] + Lt * tx_pl, py_full[k_zoom] + Lt * ty_pl,
          "#c0392b", 0.95)

    # heading-error arc
    a_ref = math.degrees(math.atan2(ty_ref, tx_ref))
    a_pl = math.degrees(math.atan2(ty_pl, tx_pl))
    a0, a1 = sorted([a_ref, a_pl])
    arc_r = 0.45 * Lt
    ax_in.add_patch(Arc((px_full[k_zoom], py_full[k_zoom]),
                        width=2 * arc_r, height=2 * arc_r,
                        angle=0.0, theta1=a0, theta2=a1,
                        color="#c0392b", linewidth=1.1))

    # markers
    ax_in.plot(cx, cy, "s", color="#37474f", markersize=4.5)
    ax_in.plot(px_full[k_zoom], py_full[k_zoom], "o",
               color="#c0392b", markersize=4.5)

    # labels with small offsets (in inset data units)
    off = 0.05 * L
    ax_in.text(cx + Lt * tx_ref + off, cy + Lt * ty_ref - off,
               r"$\mathbf{t}_{\mathrm{ref}}$",
               color="#37474f", fontsize=9)
    ax_in.text(px_full[k_zoom] + Lt * tx_pl + off,
               py_full[k_zoom] + Lt * ty_pl + off,
               r"$\mathbf{t}_{\mathrm{plan}}$",
               color="#c0392b", fontsize=9)
    mid_x = 0.5 * (cx + px_full[k_zoom])
    mid_y = 0.5 * (cy + py_full[k_zoom])
    ax_in.text(mid_x + off, mid_y + 0.2 * off,
               r"$e_y$", color="#1f6feb", fontsize=10)
    # heading error label, just past the arc
    a_mid = math.radians(0.5 * (a0 + a1))
    ax_in.text(px_full[k_zoom] + 1.4 * arc_r * math.cos(a_mid),
               py_full[k_zoom] + 1.4 * arc_r * math.sin(a_mid),
               r"$e_\psi$", color="#c0392b", fontsize=10,
               ha="center", va="center")
    ax_in.text(cx - 0.45 * off, cy - 1.6 * off,
               r"$\mathbf{p}^{\mathrm{ref}}_k$",
               color="#37474f", fontsize=9,
               ha="right", va="top")
    ax_in.text(px_full[k_zoom] + 0.6 * off,
               py_full[k_zoom] + 1.2 * off,
               r"$\mathbf{p}_k$",
               color="#c0392b", fontsize=9)

    ax.indicate_inset_zoom(ax_in, edgecolor="#90a4ae",
                           linewidth=0.7, alpha=0.9)

    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    pdf = OUT_DIR / "tracking_error_illustration_seed12.pdf"
    png = OUT_DIR / "tracking_error_illustration_seed12.png"
    fig.savefig(pdf, bbox_inches="tight")
    fig.savefig(png, dpi=300, bbox_inches="tight")
    print(f"wrote {pdf}")
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
