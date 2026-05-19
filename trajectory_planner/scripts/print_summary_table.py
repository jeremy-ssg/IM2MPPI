#!/usr/bin/env python3
"""
print_summary_table.py
----------------------
Reads every *_summary.json in a results directory, groups by configuration
(file pattern <CONFIG>_seed<N>_summary.json), and prints a console-friendly
per-config mean ± std table for the headline IM2-MPPI metrics.

Best value per metric is annotated with a ★ for quick visual comparison.

Usage:
    python3 print_summary_table.py <results_dir>
"""

import json
import os
import re
import statistics
import sys
from collections import defaultdict

NAME_RE = re.compile(r"(?P<cfg>.+)_seed(?P<seed>\d+)_summary\.json$")

# (dotted_path, label, format_spec, lower_is_better)
# Tight headline table — collision EVENT counts are the primary safety metric.
METRICS = [
    ("task.executed_path_length_m",        "Length",     ".1f", True ),
    ("safety.collision_strict_events",     "CR#",        ".1f", True ),
    ("safety.collision_near_miss_events",  "CR_nm#",     ".1f", True ),
    ("safety.collision_tail_events",       "CR_tail#",   ".1f", True ),
    ("safety.min_clearance_m",             "MinClr",     ".3f", False),
    ("safety.empirical_cvar_5pct_m",       "CVaR@5%",    ".3f", False),
    ("safety.empirical_cvar_10pct_m",      "CVaR@10%",   ".3f", False),
    ("tracking.rms_target_error_m",        "TrkRMS",     ".3f", True ),
    ("smoothness.cmd_rms_jerk_mps3",       "Jerk",       ".2f", True ),
    ("planner.plan_latency_mean_ms",       "Lat_avg",    ".2f", True ),
    ("planner.plan_latency_p95_ms",        "Lat_p95",    ".2f", True ),
    ("planner.plan_latency_max_ms",        "Lat_max",    ".2f", True ),
]


def nested_get(data, dotted):
    cur = data
    for key in dotted.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def coerce(v):
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    if v is None:
        return None
    try:
        return float(v)
    except (ValueError, TypeError):
        return None


def fmt(val, spec):
    if val is None:
        return "—"
    try:
        return format(val, spec)
    except (ValueError, TypeError):
        return str(val)


def collect(results_dir):
    """Returns: {config_name: [summary_dict, ...]}"""
    groups = defaultdict(list)
    for fname in sorted(os.listdir(results_dir)):
        m = NAME_RE.match(fname)
        if not m:
            continue
        with open(os.path.join(results_dir, fname)) as f:
            data = json.load(f)
        groups[m.group("cfg")].append(data)
    return groups


def stats_per_config(groups):
    """Returns: [(cfg_name, n, {label: (mean, std)})]"""
    rows = []
    for cfg in sorted(groups):
        runs = groups[cfg]
        agg = {}
        for path, label, spec, lower in METRICS:
            vals = [coerce(nested_get(r, path)) for r in runs]
            vals = [v for v in vals if v is not None]
            if not vals:
                agg[label] = None
                continue
            m = statistics.mean(vals)
            s = statistics.stdev(vals) if len(vals) > 1 else 0.0
            agg[label] = (m, s)
        rows.append((cfg, len(runs), agg))
    return rows


def find_winners(rows):
    """Returns: {label: cfg_index} of the best mean per metric."""
    winners = {}
    for path, label, spec, lower in METRICS:
        best_val = None
        best_idx = None
        for i, (_cfg, _n, agg) in enumerate(rows):
            v = agg.get(label)
            if v is None:
                continue
            mean_val = v[0]
            if best_val is None:
                best_val, best_idx = mean_val, i
            elif lower and mean_val < best_val:
                best_val, best_idx = mean_val, i
            elif (not lower) and mean_val > best_val:
                best_val, best_idx = mean_val, i
        winners[label] = best_idx
    return winners


def print_table(rows, winners):
    if not rows:
        print("No summary files found.")
        return

    # Column widths
    name_w = max(len("Config"), max(len(cfg) for cfg, _, _ in rows))
    cell_w = 11
    n_w    = 3

    # Header
    print()
    print("=" * (name_w + 4 + n_w + 3 + (cell_w + 3) * len(METRICS)))
    hdr = f"{'Config':<{name_w}}  | {'n':>{n_w}}"
    for _, label, _, _ in METRICS:
        hdr += f" | {label:>{cell_w}}"
    print(hdr)
    print("-" * (name_w + 4 + n_w + 3 + (cell_w + 3) * len(METRICS)))

    # Rows
    for i, (cfg, n, agg) in enumerate(rows):
        line = f"{cfg:<{name_w}}  | {n:>{n_w}}"
        for path, label, spec, lower in METRICS:
            v = agg.get(label)
            if v is None:
                cell = "—"
            else:
                mean_v, std_v = v
                cell = fmt(mean_v, spec)
                if std_v > 0 and not spec.endswith("%"):
                    cell = f"{cell}±{fmt(std_v, spec)}"
            if winners.get(label) == i and v is not None:
                cell = f"★{cell}"
            line += f" | {cell:>{cell_w}}"
        print(line)

    print("=" * (name_w + 4 + n_w + 3 + (cell_w + 3) * len(METRICS)))
    print("★  = best mean across configs for that metric.")
    print()


def print_legend():
    print("Metric legend (mean ± std across seeds):")
    print("  Length    = executed path length (m)                     (lower  better)")
    print("  CR#       = strict collision events <0.15m (count)       (lower  better)")
    print("  CR_nm#    = near-miss events <0.30m (count)              (lower  better)")
    print("  CR_tail#  = tail events <0.50m (count)                   (lower  better)")
    print("  MinClr    = min obstacle clearance (m)                   (higher better)")
    print("  CVaR@5%   = empirical CVaR, worst 5% tail clearance (m)  (higher better)  ← core IM2-MPPI claim")
    print("  CVaR@10%  = empirical CVaR, worst 10% tail clearance (m) (higher better)")
    print("  TrkRMS    = tracking RMS error (m)                       (lower  better)")
    print("  Jerk      = commanded RMS jerk (m/s³)                    (lower  better)")
    print("  Lat_avg   = mean planning latency (ms)                   (lower  better)")
    print("  Lat_p95   = p95 planning latency (ms)                    (lower  better)")
    print("  Lat_max   = max planning latency spike (ms)              (lower  better)")
    print()


def main():
    if len(sys.argv) < 2:
        print("usage: print_summary_table.py <results_dir>", file=sys.stderr)
        return 2
    results_dir = sys.argv[1]
    if not os.path.isdir(results_dir):
        print(f"not a directory: {results_dir}", file=sys.stderr)
        return 2

    groups = collect(results_dir)
    rows = stats_per_config(groups)
    winners = find_winners(rows)

    print_legend()
    print_table(rows, winners)

    # Persist a clean stats CSV alongside.
    out_csv = os.path.join(results_dir, "summary_stats.csv")
    with open(out_csv, "w") as f:
        cols = ["config", "n"] + [lbl for _, lbl, _, _ in METRICS] \
            + [lbl + "_std" for _, lbl, _, _ in METRICS]
        f.write(",".join(cols) + "\n")
        for cfg, n, agg in rows:
            cells = [cfg, str(n)]
            for _, label, _, _ in METRICS:
                v = agg.get(label)
                cells.append(f"{v[0]:.6g}" if v else "")
            for _, label, _, _ in METRICS:
                v = agg.get(label)
                cells.append(f"{v[1]:.6g}" if v else "")
            f.write(",".join(cells) + "\n")
    print(f"Wrote stats CSV: {out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
