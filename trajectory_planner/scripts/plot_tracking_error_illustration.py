"""Generate a paper-quality illustration of the tracking error between a
planned path and a reference path.

Outputs two files next to this script (or under analysis_results/):
    tracking_error_illustration.pdf
    tracking_error_illustration.png

The figure consists of:
    * a main axes showing a reference path (dashed) and a planned path
      (solid), with thin perpendicular segments visualising the per-sample
      lateral error e_y at evenly spaced stations;
    * an inset axes that zooms onto a single sample and labels the formal
      decomposition of the error vector (e_x longitudinal, e_y lateral,
      e_psi heading mismatch) with respect to the local reference frame.
"""

from __future__ import annotations

import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, FancyArrowPatch


# ----------------------------------------------------------------------------
# Synthetic paths
# ----------------------------------------------------------------------------

def reference_path(num: int = 600) -> tuple[np.ndarray, np.ndarray]:
    """A smooth S-shaped reference path."""
    s = np.linspace(0.0, 14.0, num)
    x = s
    y = 1.6 * np.sin(0.55 * s) + 0.25 * np.sin(0.18 * s)
    return x, y


def planned_path(num: int = 600, seed: int = 7) -> tuple[np.ndarray, np.ndarray]:
    """A planned path that follows the reference but with mild deviation."""
    rng = np.random.default_rng(seed)
    s = np.linspace(0.0, 14.0, num)
    x_ref = s
    y_ref = 1.6 * np.sin(0.55 * s) + 0.25 * np.sin(0.18 * s)

    # low-frequency systematic offset (curvature-induced lag)
    lag = 0.18 * np.sin(0.35 * s + 0.6) + 0.10 * np.cos(0.18 * s)
    # small high-frequency noise smoothed by a moving average
    noise = rng.normal(scale=0.04, size=num)
    kernel = np.ones(21) / 21
    noise = np.convolve(noise, kernel, mode="same")

    # offset applied along the local normal of the reference
    dx = np.gradient(x_ref, s)
    dy = np.gradient(y_ref, s)
    tnorm = np.hypot(dx, dy)
    nx = -dy / tnorm
    ny = dx / tnorm
    offset = lag + noise

    x = x_ref + offset * nx
    y = y_ref + offset * ny
    return x, y


# ----------------------------------------------------------------------------
# Error computation
# ----------------------------------------------------------------------------

def nearest_projection(px: float, py: float, rx: np.ndarray, ry: np.ndarray) -> int:
    return int(np.argmin((rx - px) ** 2 + (ry - py) ** 2))


def reference_tangent(idx: int, rx: np.ndarray, ry: np.ndarray) -> tuple[float, float]:
    i0 = max(idx - 1, 0)
    i1 = min(idx + 1, len(rx) - 1)
    tx = rx[i1] - rx[i0]
    ty = ry[i1] - ry[i0]
    n = np.hypot(tx, ty)
    return tx / n, ty / n


# ----------------------------------------------------------------------------
# Plot
# ----------------------------------------------------------------------------

def main() -> None:
    plt.rcParams.update(
        {
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
        }
    )

    rx, ry = reference_path()
    px, py = planned_path()

    # evenly spaced sample stations on the planned path
    sample_idx = np.linspace(20, len(px) - 20, 11, dtype=int)

    fig, ax = plt.subplots(figsize=(7.2, 4.3))

    # tolerance band around the reference
    dxr = np.gradient(rx)
    dyr = np.gradient(ry)
    tn = np.hypot(dxr, dyr)
    nxr, nyr = -dyr / tn, dxr / tn
    band = 0.35
    ax.fill(
        np.concatenate([rx + band * nxr, (rx - band * nxr)[::-1]]),
        np.concatenate([ry + band * nyr, (ry - band * nyr)[::-1]]),
        color="#cfd8dc",
        alpha=0.55,
        linewidth=0,
        label=r"tolerance band $\pm\,\delta$",
    )

    ax.plot(rx, ry, color="#37474f", linestyle=(0, (6, 3)),
            linewidth=1.4, label="reference path")
    ax.plot(px, py, color="#c0392b", linewidth=1.7, label="planned path")

    # per-sample lateral error segments
    err_vals = []
    for i, k in enumerate(sample_idx):
        j = nearest_projection(px[k], py[k], rx, ry)
        ax.plot([px[k], rx[j]], [py[k], ry[j]],
                color="#1f6feb", linewidth=1.4, alpha=0.95,
                solid_capstyle="round",
                label="lateral error $e_y$" if i == 0 else None)
        ax.plot(px[k], py[k], "o", color="#c0392b",
                markersize=4.2, markeredgewidth=0)
        ax.plot(rx[j], ry[j], "s", color="#37474f",
                markersize=3.6, markeredgewidth=0)
        err_vals.append(np.hypot(px[k] - rx[j], py[k] - ry[j]))

    rmse = float(np.sqrt(np.mean(np.square(err_vals))))
    emax = float(np.max(np.abs(err_vals)))

    ax.text(
        0.015, 0.965,
        f"$\\mathrm{{RMSE}}_y = {rmse:.3f}\\,\\mathrm{{m}}$\n"
        f"$e_{{y,\\max}} = {emax:.3f}\\,\\mathrm{{m}}$",
        transform=ax.transAxes, va="top", ha="left",
        fontsize=8,
        bbox=dict(boxstyle="round,pad=0.32", facecolor="white",
                  edgecolor="#90a4ae", linewidth=0.6),
    )

    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.6)
    ax.set_xlim(-0.6, 14.6)
    ax.set_ylim(-3.4, 3.2)
    ax.legend(loc="upper right", frameon=True, framealpha=0.95,
              edgecolor="#90a4ae", borderpad=0.4, handlelength=1.8)

    # ---- inset: zoom on a single sample and define e_x, e_y, e_psi ----
    # use the station-matched (same arc-length parameter) reference point so
    # that the longitudinal component e_x is non-zero and the decomposition
    # is pedagogically meaningful.  pick the sample with the largest
    # composite station-matched error to give the inset visible structure.
    cand = sample_idx
    comp_err = np.hypot(px[cand] - rx[cand], py[cand] - ry[cand])
    k_zoom = int(cand[int(np.argmax(comp_err))])
    j_zoom = k_zoom
    tx_ref, ty_ref = reference_tangent(j_zoom, rx, ry)
    nx_ref, ny_ref = -ty_ref, tx_ref

    dxv = px[k_zoom] - rx[j_zoom]
    dyv = py[k_zoom] - ry[j_zoom]
    e_x = dxv * tx_ref + dyv * ty_ref
    e_y = dxv * nx_ref + dyv * ny_ref

    # planned heading from a small forward difference
    k1 = min(k_zoom + 5, len(px) - 1)
    tx_pl = px[k1] - px[k_zoom]
    ty_pl = py[k1] - py[k_zoom]
    tn_pl = np.hypot(tx_pl, ty_pl)
    tx_pl /= tn_pl
    ty_pl /= tn_pl

    ax_in = ax.inset_axes([0.555, 0.035, 0.44, 0.54])
    L = 0.55  # local window half-size, tuned to error magnitude
    cx, cy = rx[j_zoom], ry[j_zoom]
    ax_in.set_xlim(cx - L, cx + L)
    ax_in.set_ylim(cy - L * 0.85, cy + L * 0.85)
    ax_in.set_aspect("equal")
    ax_in.set_xticks([])
    ax_in.set_yticks([])
    for spine in ax_in.spines.values():
        spine.set_edgecolor("#90a4ae")
        spine.set_linewidth(0.8)

    # local stretches of the two paths
    win = 30
    a = max(j_zoom - win, 0)
    b = min(j_zoom + win, len(rx))
    ax_in.plot(rx[a:b], ry[a:b], color="#37474f",
               linestyle=(0, (6, 3)), linewidth=1.3)
    a = max(k_zoom - win, 0)
    b = min(k_zoom + win, len(px))
    ax_in.plot(px[a:b], py[a:b], color="#c0392b", linewidth=1.5)

    # decomposition: from reference point first along tangent (e_x) then
    # along normal (e_y); both arrows are shown.
    mid_x = cx + e_x * tx_ref
    mid_y = cy + e_x * ty_ref

    def arrow(ax_, x0, y0, x1, y1, color, lw=1.0):
        ax_.add_patch(FancyArrowPatch((x0, y0), (x1, y1),
                                      arrowstyle="-|>", mutation_scale=8,
                                      color=color, linewidth=lw,
                                      shrinkA=0, shrinkB=0))

    arrow(ax_in, cx, cy, mid_x, mid_y, "#2e7d32", lw=1.1)               # e_x
    arrow(ax_in, mid_x, mid_y, px[k_zoom], py[k_zoom], "#1f6feb", 1.1)  # e_y
    arrow(ax_in, cx, cy, px[k_zoom], py[k_zoom], "#6a1b9a", 1.1)        # Δp

    # tangent unit vectors
    Lt = 0.32
    arrow(ax_in, cx, cy, cx + Lt * tx_ref, cy + Lt * ty_ref, "#37474f", 0.9)
    arrow(ax_in, px[k_zoom], py[k_zoom],
          px[k_zoom] + Lt * tx_pl, py[k_zoom] + Lt * ty_pl,
          "#c0392b", 0.9)

    # heading-error arc between the two tangents, drawn at the planned point
    ang_ref = np.degrees(np.arctan2(ty_ref, tx_ref))
    ang_pl = np.degrees(np.arctan2(ty_pl, tx_pl))
    a0, a1 = sorted([ang_ref, ang_pl])
    arc = Arc((px[k_zoom], py[k_zoom]),
              width=0.22, height=0.22, angle=0.0, theta1=a0, theta2=a1,
              color="#c0392b", linewidth=1.1)
    ax_in.add_patch(arc)

    # markers
    ax_in.plot(cx, cy, "s", color="#37474f", markersize=4)
    ax_in.plot(px[k_zoom], py[k_zoom], "o", color="#c0392b", markersize=4)

    # labels
    ax_in.text(cx + Lt * tx_ref + 0.02,
               cy + Lt * ty_ref - 0.07,
               r"$\mathbf{t}_{\mathrm{ref}}$",
               color="#37474f", fontsize=9)
    ax_in.text(px[k_zoom] + Lt * tx_pl + 0.02,
               py[k_zoom] + Lt * ty_pl + 0.03,
               r"$\mathbf{t}_{\mathrm{plan}}$",
               color="#c0392b", fontsize=9)
    ax_in.text((cx + mid_x) / 2,
               (cy + mid_y) / 2 - 0.09,
               r"$e_x$", color="#2e7d32", fontsize=10)
    ax_in.text((mid_x + px[k_zoom]) / 2 + 0.025,
               (mid_y + py[k_zoom]) / 2,
               r"$e_y$", color="#1f6feb", fontsize=10)
    ax_in.text((cx + px[k_zoom]) / 2 - 0.15,
               (cy + py[k_zoom]) / 2 + 0.03,
               r"$\Delta\mathbf{p}$", color="#6a1b9a", fontsize=10)
    ax_in.text(px[k_zoom] + 0.14, py[k_zoom] - 0.02,
               r"$e_\psi$", color="#c0392b", fontsize=10)
    ax_in.text(cx - 0.02, cy - 0.12,
               r"$\mathbf{p}^{\mathrm{ref}}_k$", color="#37474f", fontsize=9)
    ax_in.text(px[k_zoom] + 0.03, py[k_zoom] + 0.07,
               r"$\mathbf{p}_k$", color="#c0392b", fontsize=9)

    # connector lines from inset to source sample on the main plot
    ax.indicate_inset_zoom(ax_in, edgecolor="#90a4ae", linewidth=0.7,
                           alpha=0.9)

    fig.tight_layout()

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "..", "analysis_results")
    out_dir = os.path.normpath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    pdf = os.path.join(out_dir, "tracking_error_illustration.pdf")
    png = os.path.join(out_dir, "tracking_error_illustration.png")
    fig.savefig(pdf, bbox_inches="tight")
    fig.savefig(png, dpi=300, bbox_inches="tight")
    print(f"wrote {pdf}")
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
