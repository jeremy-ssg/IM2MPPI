#!/usr/bin/env python3
"""
Analyze a sweep_summary.csv produced by sweep_im2_full.py.

Outputs (all alongside the input CSV):
  1.  per_parameter_best.csv   – for each parameter, the value that optimises
                                  each metric (one row per (param, metric)).
  2.  recommended_config.yaml  – yaml diff suggesting the best value per param
                                  according to a weighted composite score.
  3.  plots/<param>.png        – per-parameter response: 6 subplots of the
                                  most important metrics vs the swept value.
  4.  corr_param_vs_metric.png – Pearson correlation heat-map between every
                                  parameter (rows) and every metric (cols).
  5.  corr_metric_vs_metric.png – metric-metric Pearson correlation heat-map.

Usage:
  rosrun trajectory_planner analyze_sweep.py PATH/TO/sweep_summary.csv
  rosrun trajectory_planner analyze_sweep.py        # auto-pick most recent
"""

import argparse
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ─── Metric directions (True = higher is better) ──────────────────────────
DIRECTION = {
    "SR":         True,
    "Time":       False,
    "Length":     False,
    "FlightDur":  False,
    "FinalDist":  False,
    "MinClr":     True,
    "MeanClr":    True,
    "p05Clr":     True,
    "CVaR5":      True,
    "CVaR10":     True,
    "CR":         False,
    "CR_nm":      False,
    "CR_tail":    False,
    "CR_time_s":  False,
    "AccRMS":     False,
    "Jerk":       False,
    "TrkRMS":     False,
    "TrkP95":     False,
    "Lat_avg":    False,
    "Lat_p95":    False,
    "Lat_max":    False,
}

# Composite-score weights (sum = 1.0). Tune to taste.
COMPOSITE_WEIGHTS = {
    "MinClr":  0.18,
    "CVaR5":   0.12,
    "CR":     -0.15,   # negative weight = penalty
    "CR_nm":  -0.10,
    "CR_tail": -0.05,
    "Time":   -0.10,
    "Length": -0.05,
    "Jerk":   -0.10,
    "TrkRMS": -0.10,
    "SR":      0.05,
}

# Per-parameter plot grid: which 6 metrics to show.
PLOT_METRICS = ["MinClr", "CVaR5", "Time", "Length", "Jerk", "TrkRMS"]


# ─── Helpers ──────────────────────────────────────────────────────────────
def find_latest_csv() -> Path:
    """Pick the most-recent sweep_summary.csv under ~/IM2MPPI/results/."""
    root = Path.home() / "IM2MPPI" / "results"
    candidates = sorted(root.glob("sweep_*/sweep_summary.csv"),
                        key=lambda p: p.stat().st_mtime)
    if not candidates:
        sys.exit(f"No sweep_summary.csv under {root}")
    return candidates[-1]


def normalize_metric(s: pd.Series, higher_is_better: bool) -> pd.Series:
    """Linearly scale to [0,1] with NaN-safe handling. Constant → 0.5."""
    v = pd.to_numeric(s, errors="coerce")
    lo, hi = v.min(skipna=True), v.max(skipna=True)
    if pd.isna(lo) or pd.isna(hi) or hi == lo:
        return pd.Series(0.5, index=v.index)
    n = (v - lo) / (hi - lo)
    return n if higher_is_better else 1.0 - n


def metric_columns(df: pd.DataFrame) -> list:
    return [c for c in DIRECTION if c in df.columns]


# ─── 1. Per-parameter best ────────────────────────────────────────────────
def per_parameter_best(df: pd.DataFrame, out_csv: Path):
    rows = []
    for param, sub in df.groupby("param", sort=False):
        for metric in metric_columns(df):
            v = pd.to_numeric(sub[metric], errors="coerce")
            if v.dropna().empty:
                continue
            best_is_max = DIRECTION[metric]
            idx = v.idxmax() if best_is_max else v.idxmin()
            rows.append({
                "param":         param,
                "metric":        metric,
                "best_value":    sub.loc[idx, "value"],
                "metric_value":  v.loc[idx],
                "metric_min":    v.min(),
                "metric_max":    v.max(),
                "metric_mean":   v.mean(),
            })
    out = pd.DataFrame(rows)
    out.to_csv(out_csv, index=False)
    print(f"  wrote {out_csv}")
    return out


# ─── 2. Recommended config (composite score) ──────────────────────────────
def recommended_config(df: pd.DataFrame, out_yaml: Path) -> dict:
    # Build composite score per row.
    score = pd.Series(0.0, index=df.index)
    used_weight = 0.0
    for metric, w in COMPOSITE_WEIGHTS.items():
        if metric not in df.columns:
            continue
        norm = normalize_metric(df[metric], higher_is_better=DIRECTION[metric])
        score = score + w * norm
        used_weight += abs(w)
    df = df.assign(_score=score)

    picks = {}
    lines = ["# Recommended IM2-MPPI parameter values.",
             "# Pick = value that maximised the composite score for each",
             "# parameter while holding all others at the sweep baseline.",
             f"# Composite weights: {COMPOSITE_WEIGHTS}",
             "",
             "im2_mppi:"]
    rows = []
    for param, sub in df.groupby("param", sort=False):
        sub = sub.dropna(subset=["_score"])
        if sub.empty:
            continue
        best = sub.loc[sub["_score"].idxmax()]
        v = best["value"]
        picks[param] = v
        # cast value type for yaml-y rendering
        try:
            v_str = str(int(v)) if float(v).is_integer() else str(v)
        except Exception:
            v_str = f'"{v}"'
        lines.append(f"  {param}: {v_str}")
        rows.append({"param": param, "value": v, "score": float(best["_score"])})

    out_yaml.write_text("\n".join(lines) + "\n")
    print(f"  wrote {out_yaml}")
    pd.DataFrame(rows).to_csv(out_yaml.with_suffix(".csv"), index=False)
    return picks


# ─── 3. Per-parameter PNG plots ───────────────────────────────────────────
def plot_param_responses(df: pd.DataFrame, out_dir: Path):
    out_dir.mkdir(exist_ok=True)
    for param, sub in df.groupby("param", sort=False):
        # Sort by value so line plot makes sense for numeric params.
        try:
            sub = sub.sort_values(by="value",
                                  key=lambda s: pd.to_numeric(s, errors="coerce"))
        except Exception:
            pass

        fig, axes = plt.subplots(2, 3, figsize=(13, 7))
        fig.suptitle(f"M4_im2_full response: {param}", fontsize=14)
        for ax, metric in zip(axes.flat, PLOT_METRICS):
            if metric not in sub.columns:
                ax.set_visible(False)
                continue
            y = pd.to_numeric(sub[metric], errors="coerce")
            x_raw = sub["value"]
            x_num = pd.to_numeric(x_raw, errors="coerce")

            if x_num.notna().all():
                ax.plot(x_num, y, "o-", linewidth=1.4, markersize=5)
                # Highlight the best point.
                if y.dropna().empty:
                    pass
                else:
                    idx = y.idxmax() if DIRECTION[metric] else y.idxmin()
                    ax.plot(x_num.loc[idx], y.loc[idx],
                            "*", markersize=15, color="red", zorder=5)
            else:
                # Categorical x.
                ax.bar(range(len(x_raw)), y.values)
                ax.set_xticks(range(len(x_raw)))
                ax.set_xticklabels([str(v) for v in x_raw],
                                   rotation=30, ha="right")

            ax.set_title(metric)
            ax.set_xlabel(param)
            ax.grid(alpha=0.3)

        fig.tight_layout(rect=(0, 0, 1, 0.96))
        path = out_dir / f"{param}.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
    print(f"  wrote {len(list(out_dir.glob('*.png')))} plots → {out_dir}")


# ─── 4. Correlation heat-maps ─────────────────────────────────────────────
def _heatmap(ax, M: np.ndarray, row_labels: list, col_labels: list,
             title: str, vmin=-1.0, vmax=1.0):
    im = ax.imshow(M, cmap="RdBu_r", aspect="auto", vmin=vmin, vmax=vmax)
    ax.set_xticks(range(len(col_labels)))
    ax.set_xticklabels(col_labels, rotation=45, ha="right", fontsize=8)
    ax.set_yticks(range(len(row_labels)))
    ax.set_yticklabels(row_labels, fontsize=9)
    for (i, j), v in np.ndenumerate(M):
        if not np.isnan(v):
            ax.text(j, i, f"{v:+.2f}", ha="center", va="center",
                    fontsize=7,
                    color="white" if abs(v) > 0.5 else "black")
    ax.set_title(title)
    return im


def corr_param_vs_metric(df: pd.DataFrame, out_png: Path):
    params  = list(df["param"].unique())
    metrics = metric_columns(df)
    M = np.full((len(params), len(metrics)), np.nan)

    for i, p in enumerate(params):
        sub = df[df["param"] == p]
        x = pd.to_numeric(sub["value"], errors="coerce")
        if x.dropna().nunique() < 2:
            continue
        for j, m in enumerate(metrics):
            y = pd.to_numeric(sub[m], errors="coerce")
            if y.dropna().nunique() < 2:
                continue
            valid = x.notna() & y.notna()
            if valid.sum() < 3:
                continue
            r = np.corrcoef(x[valid], y[valid])[0, 1]
            M[i, j] = r

    fig, ax = plt.subplots(figsize=(max(8, 0.55 * len(metrics)),
                                    max(4, 0.45 * len(params))))
    im = _heatmap(ax, M, params, metrics,
                  "Pearson r:  parameter value  vs  metric  (within each row)")
    fig.colorbar(im, ax=ax, fraction=0.025)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"  wrote {out_png}")


def corr_metric_vs_metric(df: pd.DataFrame, out_png: Path):
    metrics = metric_columns(df)
    M = df[metrics].apply(pd.to_numeric, errors="coerce").corr().values
    fig, ax = plt.subplots(figsize=(max(8, 0.55 * len(metrics)),
                                    max(8, 0.55 * len(metrics))))
    im = _heatmap(ax, M, metrics, metrics, "Pearson r:  metric vs metric")
    fig.colorbar(im, ax=ax, fraction=0.025)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"  wrote {out_png}")


# ─── Main ─────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default=None,
                    help="Path to sweep_summary.csv (auto-picked if omitted).")
    args = ap.parse_args()

    csv_path = Path(args.csv).expanduser() if args.csv else find_latest_csv()
    if not csv_path.exists():
        sys.exit(f"Not found: {csv_path}")
    out_dir = csv_path.parent

    print(f"Loading {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"  {len(df)} trials, {len(df['param'].unique())} params")

    # 1. Per-parameter best
    per_parameter_best(df, out_dir / "per_parameter_best.csv")

    # 2. Recommended config
    recommended_config(df, out_dir / "recommended_config.yaml")

    # 3. Per-parameter plots
    plot_param_responses(df, out_dir / "plots")

    # 4. Correlation heat-maps
    corr_param_vs_metric (df, out_dir / "corr_param_vs_metric.png")
    corr_metric_vs_metric(df, out_dir / "corr_metric_vs_metric.png")

    print("\nDone. All outputs in:", out_dir)


if __name__ == "__main__":
    main()
