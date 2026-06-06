#!/usr/bin/env python3
"""
bench_table_i.py
----------------
Aggregate per-stage planning timings produced by the instrumented IM2-MPPI
planner and emit Table I (the per-plan complexity / wall-clock breakdown
that appears in the RA-L paper).

Usage
-----
1) Apply the C++ instrumentation described in `bench_table_i_instrument.md`
   to im2_mppi_planner.{h,cpp} and im2_mppi_kernels.cu, then rebuild:

       cd ~/catkin_ws && catkin_make && source devel/setup.bash

   The instrumentation writes per-tick stage timings to a ROS-side file
   whose path is exposed via the parameter `im2_mppi/stage_timing_log`.

2) Run any benchmark scenario long enough to collect >= 200 plan() calls
   (the production benchmark suite already meets this — e.g.
   `run_all_experiments.sh 50 90` produces ~9000 ticks per planner).

3) Run this aggregator on the resulting log directory:

       python3 bench_table_i.py <log_dir> --yaml ../cfg/im2_mppi.yaml

   Multiple log files in the directory are pooled.

Output
------
- Console table with stage, complexity formula, derived count, and
  measured mean ± std (ms) plus p95.
- A LaTeX block (`--latex` to print only the LaTeX, suitable for
  pasting straight into the paper Table I).

Honesty contract
----------------
This script does NOT invent numbers. If the timing log is missing or
empty for a stage, the corresponding row prints `n/a` so the missing
measurement is obvious. The complexity column is derived from the
YAML parameters and is always shown.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import statistics
import sys
from typing import Dict, List, Optional


# Stage names must match the C++ tags emitted by the instrumentation
# (see bench_table_i_instrument.md). Order here = display order in the
# table.
STAGES = [
    # (stage_tag,         pretty_name,            complexity_formula,                  device)
    ("joint_mode_tree",   "Joint-mode tree",      lambda p: p["Kbar"] * p["K"] * p["J"] * p["H"],                   "CPU"),
    ("noise_sampling",    "Noise sampling",       lambda p: p["N"] * p["H"],                                        "CPU"),
    ("rollout_cost",      "K1 rollout + cost",    lambda p: p["N"] * p["H"] * p["J"] * p["Kbar"],                   "GPU"),
    ("cvar_mc",           "K2 CVaR Monte-Carlo",  lambda p: p["N"] * p["Kbar"] * p["J"] * p["R"] * p["H"],          "GPU"),
    ("voxel_map",         "Voxel-map collision",  lambda p: p["N"] * p["H"] // 6,                                   "CPU"),
    ("reduction_fusion",  "K3 reduction+fusion",  lambda p: p["Kbar"] * p["H"] * p["N"],                            "GPU"),
    ("yaw_warmstart",     "Yaw + warm-start",     lambda p: p["H"],                                                 "CPU"),
]

COMPLEXITY_FORMULA = {
    "joint_mode_tree":   r"\mathcal{O}(\bar K K J H)",
    "noise_sampling":    r"\mathcal{O}(N H)",
    "rollout_cost":      r"\mathcal{O}(N H J \bar K)",
    "cvar_mc":           r"\mathcal{O}(N \bar K J R H)",
    "voxel_map":         r"\mathcal{O}(N H / 6)",
    "reduction_fusion":  r"\mathcal{O}(\bar K H N)",
    "yaw_warmstart":     r"\mathcal{O}(H)",
}


# ---------------------------------------------------------------------------
#  Parameter loading
# ---------------------------------------------------------------------------

def load_params(yaml_path: Optional[str]) -> Dict[str, int]:
    """Return the integer parameters used by the complexity formulas.

    Reads im2_mppi.yaml if provided; otherwise falls back to the production
    defaults documented in the paper.
    """
    defaults = dict(N=512, H=38, Kbar=4, J=25, R=16, K=3)
    if yaml_path is None:
        return defaults

    try:
        import yaml  # type: ignore
    except ImportError:
        print("[WARN] PyYAML not installed; using paper defaults for parameters.",
              file=sys.stderr)
        return defaults

    with open(yaml_path) as f:
        raw = yaml.safe_load(f) or {}

    cfg = raw.get("im2_mppi", raw)

    out = dict(defaults)
    out["N"]    = int(cfg.get("num_rollouts",           defaults["N"]))
    out["H"]    = int(cfg.get("horizon_steps",          defaults["H"]))
    out["Kbar"] = int(cfg.get("num_joint_modes_keep",   defaults["Kbar"]))
    out["R"]    = int(cfg.get("cvar_num_obstacle_samples", defaults["R"]))
    out["K"]    = int(cfg.get("num_modes_per_obstacle", defaults["K"]))
    # J is scenario-dependent (number of dynamic obstacles); leave default
    # but allow override via env var.
    out["J"]    = int(os.environ.get("IM2MPPI_BENCH_J", defaults["J"]))
    return out


# ---------------------------------------------------------------------------
#  Log loading
# ---------------------------------------------------------------------------

def iter_timing_records(path: str):
    """Yield per-tick stage timing dicts from any JSON / JSONL file under path.

    Expected schema (one of):

    1) JSON Lines file, one dict per tick:
           {"joint_mode_tree": 0.21, "noise_sampling": 0.09, ...}

    2) JSON file with a top-level list of such dicts.

    3) Run summary JSON with a `planner.stage_timings_ms` field that
       contains either a list of per-tick dicts or a dict of pre-aggregated
       per-stage lists.
    """
    if os.path.isdir(path):
        files = sorted(
            glob.glob(os.path.join(path, "**", "*.json"), recursive=True) +
            glob.glob(os.path.join(path, "**", "*.jsonl"), recursive=True)
        )
    else:
        files = [path]

    stage_tags = {s[0] for s in STAGES}

    def _yield_from_obj(obj):
        """Yield per-tick dicts from a single deserialised JSON object."""
        if not isinstance(obj, dict):
            return
        stage_field = None
        if isinstance(obj.get("planner"), dict):
            stage_field = obj["planner"].get("stage_timings_ms")
        if stage_field is None:
            stage_field = obj.get("stage_timings_ms")

        if isinstance(stage_field, list):
            for rec in stage_field:
                if isinstance(rec, dict):
                    yield rec
        elif isinstance(stage_field, dict):
            keys = list(stage_field.keys())
            if keys and all(isinstance(stage_field[k], list) for k in keys):
                n = min(len(stage_field[k]) for k in keys)
                for i in range(n):
                    yield {k: stage_field[k][i] for k in keys}
        elif any(k in obj for k in stage_tags) or "total" in obj:
            # Single-tick dict at top level
            yield obj

    for fp in files:
        try:
            with open(fp) as f:
                raw = f.read()
        except Exception as e:
            print(f"[WARN] cannot read {fp}: {e}", file=sys.stderr)
            continue

        stripped = raw.lstrip()
        if not stripped:
            continue

        # First try whole-file JSON (handles arrays and single objects).
        # If that fails OR returns nothing useful, fall back to JSONL.
        used_jsonl = False
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            used_jsonl = True
            data = None

        emitted_any = False
        if not used_jsonl:
            if isinstance(data, list):
                for rec in data:
                    if isinstance(rec, dict):
                        emitted_any = True
                        yield rec
            elif isinstance(data, dict):
                for rec in _yield_from_obj(data):
                    emitted_any = True
                    yield rec
            if not emitted_any:
                # Whole-file parse succeeded but contained no timing rows;
                # treat as JSONL anyway in case it's a single-line JSONL file.
                used_jsonl = True

        if used_jsonl:
            for line in stripped.splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(rec, dict):
                    # Could be either a bare tick dict or a wrapped summary
                    if any(k in rec for k in stage_tags) or "total" in rec:
                        yield rec
                    else:
                        for sub in _yield_from_obj(rec):
                            yield sub


# ---------------------------------------------------------------------------
#  Statistics
# ---------------------------------------------------------------------------

def collect_samples(path: str) -> Dict[str, List[float]]:
    """Pool per-stage samples across all tick records under `path`."""
    samples: Dict[str, List[float]] = {tag: [] for tag, *_ in STAGES}
    samples["total"] = []

    n_ticks = 0
    for rec in iter_timing_records(path):
        n_ticks += 1
        for tag, *_ in STAGES:
            v = rec.get(tag)
            if isinstance(v, (int, float)) and math.isfinite(v):
                samples[tag].append(float(v))
        tot = rec.get("total")
        if isinstance(tot, (int, float)) and math.isfinite(tot):
            samples["total"].append(float(tot))

    samples["__n_ticks__"] = n_ticks  # type: ignore
    return samples


def percentile(values: List[float], q: float) -> Optional[float]:
    if not values:
        return None
    s = sorted(values)
    idx = max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))
    return s[idx]


def summarise(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"mean": None, "std": None, "p95": None, "n": 0}
    return {
        "mean": statistics.fmean(values),
        "std":  statistics.pstdev(values) if len(values) >= 2 else 0.0,
        "p95":  percentile(values, 0.95),
        "n":    len(values),
    }


def fmt_count(n: int) -> str:
    """Human-readable scientific notation like 1.9{\\times}10^{6}."""
    if n <= 0:
        return "0"
    exp = int(math.floor(math.log10(n)))
    if exp < 3:
        return f"{n}"
    mant = n / 10 ** exp
    return f"{mant:.1f}{{\\times}}10^{{{exp}}}"


def fmt_ms(stats: Dict[str, Optional[float]]) -> str:
    m = stats.get("mean")
    s = stats.get("std")
    if m is None:
        return "n/a"
    if s is None or s == 0.0:
        return f"{m:.2f}"
    return f"{m:.2f} $\\pm$ {s:.2f}"


# ---------------------------------------------------------------------------
#  Output
# ---------------------------------------------------------------------------

def print_console_table(stats: Dict[str, Dict[str, Optional[float]]],
                        counts: Dict[str, int],
                        params: Dict[str, int]) -> None:
    print()
    print(f"Parameters: N={params['N']}, H={params['H']}, "
          f"Kbar={params['Kbar']}, J={params['J']}, "
          f"R={params['R']}, K={params['K']}")
    print()
    header = f"{'Stage':<24}{'Count':<14}{'Mean (ms)':<14}{'Std (ms)':<12}{'p95 (ms)':<12}{'n_ticks':>8}"
    print(header)
    print("-" * len(header))
    for tag, name, _, device in STAGES:
        c = counts[tag]
        st = stats[tag]
        m  = f"{st['mean']:.3f}" if st['mean'] is not None else "n/a"
        sd = f"{st['std']:.3f}"  if st['std']  is not None else "n/a"
        p9 = f"{st['p95']:.3f}"  if st['p95']  is not None else "n/a"
        print(f"{name:<20}{device:<4}{c:<14}{m:<14}{sd:<12}{p9:<12}{st['n']:>8}")
    print("-" * len(header))
    tot = stats["total"]
    m  = f"{tot['mean']:.3f}" if tot['mean'] is not None else "n/a"
    sd = f"{tot['std']:.3f}"  if tot['std']  is not None else "n/a"
    p9 = f"{tot['p95']:.3f}"  if tot['p95']  is not None else "n/a"
    print(f"{'TOTAL':<24}{'':<14}{m:<14}{sd:<12}{p9:<12}{tot['n']:>8}")


def print_latex_table(stats: Dict[str, Dict[str, Optional[float]]],
                      counts: Dict[str, int],
                      params: Dict[str, int]) -> None:
    print("\\begin{table}[h]")
    print("  \\centering")
    print("  \\caption{Per-plan complexity and measured wall-clock time at the")
    print(f"    production parameter setting $(N{{=}}{params['N']}, "
          f"H{{=}}{params['H']}, \\bar{{K}}{{=}}{params['Kbar']}, "
          f"J{{=}}{params['J']}, R{{=}}{params['R']}, K{{=}}{params['K']})$.")
    print("    All wall-clock values are measured by chrono+cudaEvent")
    print("    instrumentation across all plan() ticks of the production")
    print("    benchmark.}")
    print("  \\label{tbl:complexity}")
    print("  \\setlength\\tabcolsep{3pt}")
    print("  \\scriptsize")
    print("  \\begin{tabular}{lll}")
    print("    \\toprule")
    print("    \\textbf{Stage} & \\textbf{Complexity} & \\textbf{Time (ms)} \\\\")
    print("    \\midrule")
    for tag, name, _, device in STAGES:
        c = counts[tag]
        formula = COMPLEXITY_FORMULA[tag]
        count_str = fmt_count(c)
        time_str = fmt_ms(stats[tag])
        print(f"    {name:<22} & ${formula} \\approx {count_str}$ "
              f"& {time_str} ({device}) \\\\")
    print("    \\midrule")
    tot = stats["total"]
    if tot["mean"] is not None:
        if tot["std"] is not None and tot["std"] > 0:
            tot_str = f"$\\mathbf{{{tot['mean']:.2f} \\pm {tot['std']:.2f}}}$\\,ms"
        else:
            tot_str = f"$\\mathbf{{{tot['mean']:.2f}}}$\\,ms"
    else:
        tot_str = "n/a"
    print(f"    \\textbf{{Total}}        &              & {tot_str} \\\\")
    print("    \\bottomrule")
    print("  \\end{tabular}")
    print("\\end{table}")


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?",
                    help="Directory or JSON/JSONL file with per-tick stage timing samples")
    ap.add_argument("--yaml", default=None,
                    help="Path to im2_mppi.yaml (default: paper production settings)")
    ap.add_argument("--latex", action="store_true",
                    help="Print only the LaTeX table block")
    ap.add_argument("--print-patch", action="store_true",
                    help="Print the path to the C++ instrumentation guide and exit")

    args = ap.parse_args(argv)

    if args.print_patch:
        here = os.path.dirname(os.path.abspath(__file__))
        guide = os.path.join(here, "bench_table_i_instrument.md")
        print(guide)
        return 0

    if args.path is None:
        ap.error("path argument required (unless --print-patch). "
                 "See `--help` for input format.")

    params = load_params(args.yaml)
    counts = {tag: int(formula(params)) for tag, _, formula, _ in STAGES}

    samples = collect_samples(args.path)
    n_ticks = samples.pop("__n_ticks__", 0)  # type: ignore

    if n_ticks == 0:
        print(f"[ERROR] No timing records found under {args.path}.\n"
              f"Have you applied the C++ instrumentation and re-run the\n"
              f"benchmark? Run with --print-patch for the guide.",
              file=sys.stderr)
        return 2

    stats = {tag: summarise(samples[tag]) for tag, *_ in STAGES}
    stats["total"] = summarise(samples["total"])

    if not args.latex:
        print_console_table(stats, counts, params)
        print()
    print_latex_table(stats, counts, params)
    return 0


if __name__ == "__main__":
    sys.exit(main())
