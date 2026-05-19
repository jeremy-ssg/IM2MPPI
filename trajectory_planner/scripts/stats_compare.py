#!/usr/bin/env python3
"""
Paired statistical comparison between two planner algorithms across N seeds.

Given two directories (or two glob patterns) of *_summary.json files indexed
by seed, runs Wilcoxon signed-rank tests on per-seed metrics and writes a
table of (mean_A, mean_B, mean_diff, Wilcoxon W, p-value, significant?).

Example:
  python3 stats_compare.py \
      --baseline-glob 'results/intent_mpc_seed*_summary.json' \
      --treatment-glob 'results/im2_mppi_seed*_summary.json' \
      --out stats.csv

Requires scipy. Skips tests if fewer than 5 paired samples available.
"""

import argparse
import csv
import glob
import json
import os
import re
import sys


METRICS = [
    # (dotted path in summary.json, friendly label, lower|higher better)
    ("task.success",                          "success_rate",                 "higher"),
    ("task.mission_time_s",                   "mission_time_s",               "lower"),
    ("task.completed_path_length_m",          "completed_path_length_m",      "lower"),
    # Tiered collision EVENT counters — primary safety KPI
    ("safety.collision_strict_events",        "collision_strict_events",      "lower"),
    ("safety.collision_near_miss_events",     "collision_near_miss_events",   "lower"),
    ("safety.collision_tail_events",          "collision_tail_events",        "lower"),
    # Time-in-collision proxies (samples)
    ("safety.collision_strict_samples",       "collision_strict_samples",     "lower"),
    ("safety.collision_near_miss_samples",    "collision_near_miss_samples",  "lower"),
    # Clearance distribution
    ("safety.min_clearance_m",                "min_clearance_m",              "higher"),
    ("safety.mean_clearance_m",               "mean_clearance_m",             "higher"),
    ("safety.p05_clearance_m",                "p05_clearance_m",              "higher"),
    ("safety.empirical_cvar_5pct_m",          "empirical_cvar_5pct_m",        "higher"),
    ("safety.empirical_cvar_10pct_m",         "empirical_cvar_10pct_m",       "higher"),
    # Tracking
    ("tracking.rms_target_error_m",           "rms_target_error_m",           "lower"),
    # Smoothness
    ("smoothness.cmd_rms_accel_mps2",         "cmd_rms_accel_mps2",           "lower"),
    ("smoothness.cmd_rms_jerk_mps3",          "cmd_rms_jerk_mps3",            "lower"),
    # Planning latency
    ("planner.plan_latency_mean_ms",          "plan_latency_mean_ms",         "lower"),
    ("planner.plan_latency_p95_ms",           "plan_latency_p95_ms",          "lower"),
    ("planner.plan_latency_max_ms",           "plan_latency_max_ms",          "lower"),
]


_SEED_RE = re.compile(r"seed(\d+)")


def nested_get(data, dotted):
    cur = data
    for key in dotted.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def index_by_seed(paths):
    """Map seed_id -> file path. Files without a seed in the name are skipped."""
    out = {}
    for p in paths:
        m = _SEED_RE.search(os.path.basename(p))
        if m:
            out[int(m.group(1))] = p
    return out


def load_metric(path, dotted):
    with open(path) as f:
        data = json.load(f)
    v = nested_get(data, dotted)
    if v is None:
        return None
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline-glob",  required=True,
                    help="glob for baseline summary.json files (e.g. intent_mpc_seed*.json)")
    ap.add_argument("--treatment-glob", required=True,
                    help="glob for treatment summary.json files (e.g. im2_mppi_seed*.json)")
    ap.add_argument("--out", default="stats.csv", help="output CSV path")
    ap.add_argument("--baseline-name",  default="baseline")
    ap.add_argument("--treatment-name", default="treatment")
    ap.add_argument("--alpha", type=float, default=0.01,
                    help="significance threshold (default 0.01)")
    args = ap.parse_args()

    try:
        from scipy.stats import wilcoxon
    except ImportError:
        print("ERROR: scipy is required. pip install scipy", file=sys.stderr)
        return 2

    base_files = index_by_seed(glob.glob(args.baseline_glob))
    treat_files = index_by_seed(glob.glob(args.treatment_glob))

    common_seeds = sorted(set(base_files) & set(treat_files))
    if not common_seeds:
        print("ERROR: no overlapping seeds between baseline and treatment globs.",
              file=sys.stderr)
        print("  baseline seeds: ", sorted(base_files))
        print("  treatment seeds:", sorted(treat_files))
        return 1
    print(f"Paired comparison on {len(common_seeds)} seeds: {common_seeds[:10]}{'...' if len(common_seeds)>10 else ''}")

    rows = []
    for dotted, label, direction in METRICS:
        a, b = [], []
        for seed in common_seeds:
            va = load_metric(base_files[seed], dotted)
            vb = load_metric(treat_files[seed], dotted)
            if va is not None and vb is not None:
                a.append(va)
                b.append(vb)
        if len(a) < 5:
            rows.append({
                "metric": label, "n": len(a), "direction": direction,
                "baseline_mean": None, "treatment_mean": None,
                "mean_diff": None, "W": None, "p_value": None,
                "significant": False, "winner": "n/a",
            })
            continue

        mean_a = sum(a) / len(a)
        mean_b = sum(b) / len(b)
        diff = [b[i] - a[i] for i in range(len(a))]
        # Wilcoxon will raise if all diffs are zero; guard it.
        if all(abs(d) < 1e-12 for d in diff):
            W, p = None, 1.0
        else:
            try:
                W, p = wilcoxon(diff)
                W, p = float(W), float(p)
            except Exception as e:  # noqa: BLE001
                W, p = None, None

        sig = (p is not None and p < args.alpha)
        if not sig:
            winner = "tie"
        else:
            if direction == "lower":
                winner = args.treatment_name if mean_b < mean_a else args.baseline_name
            else:
                winner = args.treatment_name if mean_b > mean_a else args.baseline_name

        rows.append({
            "metric": label, "n": len(a), "direction": direction,
            "baseline_mean":  mean_a,
            "treatment_mean": mean_b,
            "mean_diff":      mean_b - mean_a,
            "W": W, "p_value": p,
            "significant": sig, "winner": winner,
        })

    fields = ["metric", "n", "direction",
              "baseline_mean", "treatment_mean", "mean_diff",
              "W", "p_value", "significant", "winner"]
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # Pretty-print to terminal
    print(f"\n{'Metric':<32} {'n':>4} {'baseline':>12} {'treatment':>12} "
          f"{'diff':>10} {'p':>10}  winner")
    print("-" * 100)
    for r in rows:
        bm = f"{r['baseline_mean']:.4f}"  if r['baseline_mean']  is not None else "  n/a   "
        tm = f"{r['treatment_mean']:.4f}" if r['treatment_mean'] is not None else "  n/a   "
        dm = f"{r['mean_diff']:+.4f}"     if r['mean_diff']      is not None else "  n/a   "
        pv = f"{r['p_value']:.4f}"        if r['p_value']        is not None else "  n/a   "
        flag = "*" if r["significant"] else " "
        print(f"{r['metric']:<32} {r['n']:>4} {bm:>12} {tm:>12} {dm:>10} {pv:>10}{flag} {r['winner']}")

    print(f"\nWrote {args.out}  ({sum(1 for r in rows if r['significant'])}/{len(rows)} "
          f"metrics significant at α={args.alpha})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
