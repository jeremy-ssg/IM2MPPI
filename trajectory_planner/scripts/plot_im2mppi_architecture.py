"""Paper figure: IM2-MPPI method architecture / module diagram (detailed).

Layout follows the user-provided reference figure:
    - dashed outer border + title
    - top row: gray I/O + preprocessing modules (3 + 2 + 3 cells)
    - bottom row: two orange "core" module groups, 2 x 3 numbered cells each
    - orthogonal data-flow arrows between groups, labelled with data names
    - notation legend strip below the orange row

Outputs:
    analysis_results/im2mppi_architecture.{pdf,png}
"""
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = REPO_ROOT / "analysis_results"


# ----- visual constants ---------------------------------------------------
GRAY_HEAD = "#bfbfbf"
GRAY_FILL = "#9a9a9a"
GRAY_BODY = "#eaeaea"
GRAY_EDGE = "#6a6a6a"

ORANGE_HEAD = "#f6c89a"
ORANGE_FILL = "#f0a35d"
ORANGE_BODY = "#fde0c8"
ORANGE_EDGE = "#c47a36"

LEGEND_BODY = "#f1f3f7"
LEGEND_EDGE = "#9aa3b2"

BORDER = "#1f3a7a"
ARROW = "#0d0d0d"
LOOP_COLOR = "#c0392b"


# ----- helpers ------------------------------------------------------------

def group_box(ax, x, y, w, h, title, *, header_color, body_color, edge):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=body_color, edgecolor=edge,
                           linewidth=1.2, zorder=1))
    head_h = 0.46
    ax.add_patch(Rectangle((x, y + h - head_h), w, head_h,
                           facecolor=header_color, edgecolor=edge,
                           linewidth=1.2, zorder=1))
    ax.text(x + 0.22, y + h - head_h / 2, title,
            ha="left", va="center", fontsize=11.5, fontweight="bold",
            color="#1d1d1d", zorder=3)


def cell(ax, x, y, w, h, text, *, facecolor, edgecolor, textcolor="#1d1d1d",
         number=None, fontsize=9.5):
    patch = FancyBboxPatch((x, y), w, h,
                           boxstyle="round,pad=0.02,rounding_size=0.07",
                           facecolor=facecolor, edgecolor=edgecolor,
                           linewidth=1.1, zorder=2)
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h / 2, text,
            ha="center", va="center", fontsize=fontsize, color=textcolor,
            fontweight="bold", zorder=3)
    if number is not None:
        ax.text(x + w - 0.12, y + 0.08, str(number),
                ha="right", va="bottom", fontsize=9, fontstyle="italic",
                color=textcolor, zorder=3)


def arrow(ax, x0, y0, x1, y1, *, color=ARROW, lw=1.5, style="-|>",
          mutation=16, connectionstyle="arc3,rad=0"):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle=style,
                                 mutation_scale=mutation, color=color,
                                 linewidth=lw, zorder=4,
                                 connectionstyle=connectionstyle,
                                 shrinkA=0, shrinkB=0))


def label(ax, x, y, text, *, fontsize=10, color=ARROW, ha="center",
          va="center", italic=True, weight="bold", bg=None):
    kw = dict(ha=ha, va=va, fontsize=fontsize, color=color,
              fontweight=weight,
              fontstyle="italic" if italic else "normal", zorder=5)
    if bg is not None:
        kw["bbox"] = dict(boxstyle="round,pad=0.22", facecolor=bg,
                          edgecolor="none", alpha=0.95)
    ax.text(x, y, text, **kw)


# ----- main figure --------------------------------------------------------

def main() -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "mathtext.fontset": "cm",
    })

    # canvas (data units) — wider figsize so each cell has visual room
    W, H = 22.0, 14.6
    fig, ax = plt.subplots(figsize=(16.0, 10.6))
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.set_axis_off()

    # outer dashed frame
    ax.add_patch(Rectangle((0.10, 0.10), W - 0.20, H - 0.20,
                           fill=False, edgecolor=BORDER, linewidth=2.0,
                           linestyle=(0, (8, 5)), zorder=0))

    # ===== title =====
    ax.text(W / 2, H - 0.40,
            "IM2-MPPI : Intent-Modal Risk-Aware MPPI Planner — Pipeline",
            ha="center", va="top", fontsize=14, fontweight="bold",
            color="#1f3a7a")

    # ===== TOP ROW : I/O + preprocessing =====
    top_y = 10.10
    top_h = 3.40
    g_in_w  = 5.20
    g_pp_w  = 6.80
    g_out_w = 7.40
    gap_top = 0.65
    x_in  = 0.55
    x_pp  = x_in + g_in_w + gap_top
    x_out = W - 0.55 - g_out_w

    # Input
    group_box(ax, x_in, top_y, g_in_w, top_h,
              "Input Module",
              header_color=GRAY_HEAD, body_color=GRAY_BODY, edge=GRAY_EDGE)
    cw = g_in_w - 0.50
    ch = 0.78
    cy0 = top_y + 1.84
    cy1 = top_y + 0.96
    cy2 = top_y + 0.08
    cell(ax, x_in + 0.25, cy0, cw, ch,
         "Current State  x₀ = (p, v)",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)
    cell(ax, x_in + 0.25, cy1, cw, ch,
         "Goal  g  &  Ref. path  Γ_ref",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)
    cell(ax, x_in + 0.25, cy2, cw, ch,
         "Static Map  &  Static Obs.",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)

    # Preprocessing
    group_box(ax, x_pp, top_y, g_pp_w, top_h,
              "Preprocessing Module",
              header_color=GRAY_HEAD, body_color=GRAY_BODY, edge=GRAY_EDGE)
    cw2 = g_pp_w - 0.50
    pch = 1.18
    cell(ax, x_pp + 0.25, top_y + 1.52, cw2, pch,
         "Dynamic Obstacle Tracking\n(KF / occupancy · AABB)",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)
    cell(ax, x_pp + 0.25, top_y + 0.12, cw2, pch,
         "Intent-Modal Prediction\n{ π_m,  μ_{m,k},  Σ_{m,k} }",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)

    # Output
    group_box(ax, x_out, top_y, g_out_w, top_h,
              "Output Module",
              header_color=GRAY_HEAD, body_color=GRAY_BODY, edge=GRAY_EDGE)
    cw3 = g_out_w - 0.50
    cell(ax, x_out + 0.25, cy0, cw3, ch,
         "Planned Trajectory  {p, v, a, ψ}_{0:H}",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)
    cell(ax, x_out + 0.25, cy1, cw3, ch,
         "Nominal Control Sequence  u*_{0:H-1}",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)
    cell(ax, x_out + 0.25, cy2, cw3, ch,
         "Rollout Visualization (positions, weights)",
         facecolor=GRAY_FILL, edgecolor="#5a5a5a", fontsize=10)

    # input -> preprocess (horizontal, top row)
    arrow(ax, x_in + g_in_w, top_y + top_h / 2,
          x_pp,             top_y + top_h / 2)

    # ===== BOTTOM ROW : two orange core groups (2x3 cells each) =====
    bot_y = 1.55
    bot_h = 7.10
    g_left_w  = 9.20
    g_right_w = 9.20
    bot_gap   = 2.10
    x_left  = 0.55
    x_right = x_left + g_left_w + bot_gap

    # Left core: 2 rows x 3 cols
    group_box(ax, x_left, bot_y, g_left_w, bot_h,
              "Sampling, Rollout & Per-Mode Cost Evaluation",
              header_color=ORANGE_HEAD, body_color=ORANGE_BODY,
              edge=ORANGE_EDGE)
    grid_cols, grid_rows = 3, 2
    inner_pad = 0.30
    inter_gap_x = 0.30
    inter_gap_y = 0.35
    inner_w = g_left_w - 2 * inner_pad
    inner_h = bot_h - 0.46 - 2 * inner_pad
    ccw = (inner_w - (grid_cols - 1) * inter_gap_x) / grid_cols
    cch = (inner_h - (grid_rows - 1) * inter_gap_y) / grid_rows

    def left_xy(col, row):
        cx = x_left + inner_pad + col * (ccw + inter_gap_x)
        cy = bot_y + inner_pad + (grid_rows - 1 - row) * (cch + inter_gap_y)
        return cx, cy

    L_TEXT = [
        # (col, row, text, num)
        (0, 0, "Control Init / Shift\n(warm start,\nreceding horizon)", 1),
        (1, 0, "Noise Sampling\nε_n ~ N(0, Σ_u)\nN rollouts × H steps", 2),
        (2, 0, "Dynamics Rollout\n3-D point mass\nx_{k+1} = f(x_k, u_k+ε)", 3),
        (0, 1, "Joint Intent Modes\nbuild · prune\nM = ⊗_i M_i", 4),
        (1, 1, "Tracking Costs\ngoal · path · smooth\n‖p_H−g‖² + Σd(p_k,Γ)²", 5),
        (2, 1, "Collision Costs\nstatic · map · dyn\nAABB hinge per mode", 6),
    ]
    for col, row, txt, num in L_TEXT:
        cx, cy = left_xy(col, row)
        cell(ax, cx, cy, ccw, cch, txt,
             facecolor=ORANGE_FILL, edgecolor=ORANGE_EDGE,
             number=num, fontsize=9.5)

    # Right core
    group_box(ax, x_right, bot_y, g_right_w, bot_h,
              "Risk-Aware Fusion & MPPI Control Update",
              header_color=ORANGE_HEAD, body_color=ORANGE_BODY,
              edge=ORANGE_EDGE)
    inner_w_r = g_right_w - 2 * inner_pad
    rcw = (inner_w_r - (grid_cols - 1) * inter_gap_x) / grid_cols
    rch = cch

    def right_xy(col, row):
        rx = x_right + inner_pad + col * (rcw + inter_gap_x)
        ry = bot_y + inner_pad + (grid_rows - 1 - row) * (rch + inter_gap_y)
        return rx, ry

    R_TEXT = [
        (0, 0, "Obstacle CVaR\nR samples per (n, m)\nworst-α hinge²", 1),
        (1, 0, "Intent CVaR Premium\nCVaR_α over joint\nmode posterior π_m", 2),
        (2, 0, "Hard Safety Floor\nclearance < d_min\n⇒ cost = +∞", 3),
        (0, 1, "Mode Fusion π_eff\nsoft · sharpened ·\nadaptive · argmax", 4),
        (1, 1, "MPPI Softmax Update\nw_n ∝ Σ_m π_eff[m] ·\n  exp(−S_{n,m}/λ)", 5),
        (2, 1, "Yaw Reference Gen.\ntangent-aligned\nψ = atan2(v_y, v_x)", 6),
    ]
    for col, row, txt, num in R_TEXT:
        rx, ry = right_xy(col, row)
        cell(ax, rx, ry, rcw, rch, txt,
             facecolor=ORANGE_FILL, edgecolor=ORANGE_EDGE,
             number=num, fontsize=9.5)

    # ===== VERTICAL CONNECTORS BETWEEN ROWS =====
    top_of_bot = bot_y + bot_h
    bot_of_top = top_y
    mid_y = (bot_of_top + top_of_bot) / 2

    # 1) Input → left-core
    x_a1 = x_in + g_in_w * 0.55
    x_b1 = x_left + g_left_w * 0.14
    arrow(ax, x_a1, bot_of_top, x_a1, mid_y + 0.10)
    arrow(ax, x_a1, mid_y + 0.10, x_b1, top_of_bot)
    label(ax, (x_a1 + x_b1) / 2, mid_y + 0.55,
          "x₀,  g,  Γ_ref,  static obs.", fontsize=10, bg=GRAY_BODY)

    # 2) Preprocess → left-core (predictions)
    x_a2 = x_pp + g_pp_w * 0.55
    x_b2 = x_left + g_left_w * 0.55
    arrow(ax, x_a2, bot_of_top, x_a2, mid_y - 0.10)
    arrow(ax, x_a2, mid_y - 0.10, x_b2, top_of_bot)
    label(ax, (x_a2 + x_b2) / 2 + 0.20, mid_y + 0.55,
          "{ π_m,  μ_{m,k},  Σ_{m,k} }",
          fontsize=10, bg=GRAY_BODY)

    # 3) Right-core → output
    x_a3 = x_right + g_right_w * 0.62
    x_b3 = x_out + g_out_w * 0.5
    arrow(ax, x_a3, top_of_bot, x_a3, mid_y + 0.10)
    arrow(ax, x_a3, mid_y + 0.10, x_b3, bot_of_top)
    label(ax, (x_a3 + x_b3) / 2, mid_y + 0.55,
          "planned trajectory,  u*",
          fontsize=10, bg=GRAY_BODY)

    # ===== HORIZONTAL ARROWS BETWEEN CORES =====
    gap_mid_x = (x_left + g_left_w + x_right) / 2
    # arrow centred on rows of cells (use cell centres)
    y_top_arrow = bot_y + inner_pad + (cch + inter_gap_y) + cch / 2
    y_bot_arrow = bot_y + inner_pad + cch / 2

    arrow(ax, x_left + g_left_w, y_top_arrow,
          x_right,              y_top_arrow)
    label(ax, gap_mid_x, y_top_arrow + 0.55,
          "rollouts +\nbase costs S_{n,m}",
          fontsize=10, bg=ORANGE_BODY)

    arrow(ax, x_right,             y_bot_arrow,
          x_left + g_left_w,       y_bot_arrow)
    label(ax, gap_mid_x, y_bot_arrow - 0.55,
          "π_eff,  u*\n(receding-horizon\nfeedback)",
          fontsize=10, bg=ORANGE_BODY, color=LOOP_COLOR)

    # ===== NOTATION STRIP at the very bottom =====
    leg_x, leg_y, leg_w, leg_h = 0.55, 0.30, W - 1.10, 0.90
    ax.add_patch(FancyBboxPatch((leg_x, leg_y), leg_w, leg_h,
                                boxstyle="round,pad=0.02,rounding_size=0.08",
                                facecolor=LEGEND_BODY, edgecolor=LEGEND_EDGE,
                                linewidth=1.0, zorder=1))
    notation = (
        r"$\bf{N}$ rollouts  ·  "
        r"$\bf{H}$ horizon  ·  "
        r"$\bf{M}_i$ intent modes per obstacle  ·  "
        r"$\bf{R}$ obstacle samples per (n,m)  ·  "
        r"$\alpha$ CVaR tail  ·  "
        r"$\lambda$ MPPI temperature  ·  "
        r"$\lambda_r$ CVaR weight  ·  "
        r"$d_{\min}$ safety-floor clearance"
    )
    ax.text(leg_x + leg_w / 2, leg_y + leg_h / 2, notation,
            ha="center", va="center", fontsize=10.5, color="#1d1d1d",
            zorder=3)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    pdf = OUT_DIR / "im2mppi_architecture.pdf"
    png = OUT_DIR / "im2mppi_architecture.png"
    fig.savefig(pdf, bbox_inches="tight")
    fig.savefig(png, dpi=300, bbox_inches="tight")
    print(f"wrote {pdf}")
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
