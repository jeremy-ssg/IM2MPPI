#!/usr/bin/env python3
"""
Parameter sweep for IM2-MPPI (M4_im2_full ONLY).

Design: one-factor-at-a-time (OFAT) around the v0.2-vertical-escape baseline.
        ~200 trials = 10 parameters × 20 values each.

For each trial:
  1. Restore yaml to baseline, then patch ONE parameter to the swept value.
  2. roslaunch autonomous_flight im2_mppi_demo.launch (rviz off).
  3. Run evaluate_intent_mpc_im2mppi.py for EVAL_DURATION_S seconds.
  4. Tear down ROS / Gazebo cleanly.
  5. Read summary*.json from the trial directory.
  6. Append a row to sweep_summary.csv with the param + every metric.

Output:
  ~/IM2MPPI/results/sweep_<timestamp>/
      sweep_summary.csv              (one row per trial — main artifact)
      im2_mppi.yaml.baseline         (snapshot of yaml at sweep start)
      trial_<NNN>_<param>=<value>/   (per-trial logs + summary)

Usage:
  rosrun trajectory_planner sweep_im2_full.py
  rosrun trajectory_planner sweep_im2_full.py --dry-run
  rosrun trajectory_planner sweep_im2_full.py --start 75            # resume
  rosrun trajectory_planner sweep_im2_full.py --eval-duration 60    # shorter
"""

import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np


# ─── Defaults (override via CLI) ──────────────────────────────────────────
EVAL_DURATION_S = 90        # how long the evaluator runs (sets internal timer)
TRIAL_TIMEOUT_S = 150       # hard kill on the eval process after this
SIM_WARMUP_S    = 12        # wait after roslaunch start.launch for gazebo
STACK_WARMUP_S  = 6         # additional wait after launching the planner stack
INTER_TRIAL_PAUSE_S = 4     # let OS reclaim ports / shared mem

# Simulator (gazebo + drone urdf spawn). Headless via gui:=false.
SIM_PKG    = "uav_simulator"
SIM_LAUNCH = "start.launch"

# Planner stack (params + nav node + tracking controller, no gazebo).
NAV_PKG    = "autonomous_flight"
NAV_LAUNCH = "im2_mppi_demo.launch"


# ─── Sweep definitions (OFAT around v0.2 baseline) ────────────────────────
def make_sweeps():
    """
    Each entry: (param_name_under_im2_mppi, list_of_values, kind).
    kind = "num" → bare numeric in yaml,  "cat" → quoted string.
    Total trial count = sum of len(values).
    """
    return [
        ("d_safe",
         np.linspace(0.10, 1.00, 20).round(3).tolist(),
         "num"),

        ("hard_floor_clearance",
         np.linspace(0.00, 0.40, 20).round(3).tolist(),
         "num"),

        ("lambda",
         np.logspace(0.0, 1.3, 20).round(3).tolist(),     # 1 → 20
         "num"),

        ("w_dyn",
         np.logspace(0.5, 2.0, 20).round(2).tolist(),     # 3 → 100
         "num"),

        ("cvar_lambda_r",
         np.linspace(0.0, 10.0, 20).round(2).tolist(),
         "num"),

        ("sigma_az",
         np.linspace(0.10, 1.50, 20).round(3).tolist(),
         "num"),

        ("w_path",
         np.linspace(0.10, 20.0, 20).round(2).tolist(),
         "num"),

        ("horizon_steps",
         [10, 12, 15, 18, 20, 22, 25, 28, 30, 33,
          35, 38, 40, 43, 45, 48, 50, 55, 60, 70],
         "num"),

        ("w_static",
         np.logspace(0.5, 2.0, 20).round(2).tolist(),
         "num"),

        ("fusion_mode",
         ["soft", "sharpened", "adaptive", "argmax"],
         "cat"),
    ]


# ─── Path helpers ─────────────────────────────────────────────────────────
def resolve_yaml_path() -> Path:
    pkg = subprocess.check_output(
        ["rospack", "find", "trajectory_planner"], text=True
    ).strip()
    p = Path(pkg) / "cfg" / "im2_mppi.yaml"
    if not p.exists():
        raise FileNotFoundError(f"Expected yaml at {p}")
    return p


# ─── YAML in-place mutation (regex; preserves comments/formatting) ────────
def set_yaml_param(yaml_path: Path, param_name: str, value, kind: str):
    txt = yaml_path.read_text()

    # Match indented key under im2_mppi:, with any value + optional comment
    pat = rf'^(\s+){re.escape(param_name)}:\s*("[^"]*"|\S+)([ \t]*)(#.*)?$'

    if kind == "cat":
        new_value_repr = f'"{value}"'
    elif isinstance(value, bool):
        new_value_repr = "true" if value else "false"
    elif isinstance(value, (int, np.integer)):
        new_value_repr = str(int(value))
    else:
        new_value_repr = repr(float(value))

    def _sub(m):
        indent = m.group(1)
        gap    = m.group(3) or ""
        comment = m.group(4) or ""
        # Keep at least one space before comment.
        if comment and not gap:
            gap = "  "
        return f"{indent}{param_name}: {new_value_repr}{gap}{comment}"

    new_txt, n = re.subn(pat, _sub, txt, count=1, flags=re.MULTILINE)
    if n == 0:
        raise RuntimeError(
            f"YAML key '{param_name}' not found in {yaml_path}.")
    yaml_path.write_text(new_txt)


# ─── ROS lifecycle ────────────────────────────────────────────────────────
_KILL_TARGETS = [
    "roslaunch", "rosmaster", "rosout",
    "gzserver", "gzclient", "gazebo",
    "spawn_gazebo_model",
    "im2_mppi_navigation_node",
    "tracking_controller_node",
    "evaluate_intent_mpc_im2mppi",
    "keyboard_control",
    "rviz",
]


def kill_ros():
    """Best-effort: nuke every project / ROS / gazebo process by name."""
    # Two passes: TERM then KILL, so any state flushes finish cleanly.
    for sig in ("-15", "-9"):
        for t in _KILL_TARGETS:
            subprocess.run(
                ["pkill", sig, "-f", t],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        time.sleep(0.5)


def run_trial(idx: int, param_name: str, value, kind: str,
              baseline_yaml_text: str, yaml_path: Path,
              results_root: Path,
              eval_duration: int, trial_timeout: int) -> Optional[dict]:
    safe_value = str(value).replace("/", "_")
    trial_dir = results_root / f"trial_{idx:03d}_{param_name}={safe_value}"
    trial_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*72}")
    print(f"  [Trial {idx}]  {param_name} = {value}")
    print(f"  {trial_dir}")
    print(f"{'='*72}", flush=True)

    # 1. Reset yaml to baseline, then apply just THIS override.
    yaml_path.write_text(baseline_yaml_text)
    set_yaml_param(yaml_path, param_name, value, kind)

    # 2a. Background-launch the simulator (Gazebo) HEADLESS.
    sim_log = open(trial_dir / "sim.log", "wb")
    sim_proc = subprocess.Popen(
        ["roslaunch", SIM_PKG, SIM_LAUNCH, "gui:=false"],
        stdout=sim_log, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    time.sleep(SIM_WARMUP_S)

    # 2b. Background-launch the planner stack (nav node + tracking controller).
    nav_log = open(trial_dir / "nav.log", "wb")
    nav_proc = subprocess.Popen(
        ["roslaunch", NAV_PKG, NAV_LAUNCH, "enable_rviz:=false"],
        stdout=nav_log, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    time.sleep(STACK_WARMUP_S)

    # 3. Run evaluator (foreground) for `eval_duration` seconds.
    eval_log = open(trial_dir / "eval.log", "wb")
    eval_cmd = [
        "rosrun", "trajectory_planner", "evaluate_intent_mpc_im2mppi.py",
        "_algorithm:=im2_mppi",
        f"_duration:={eval_duration}",
        f"_output_dir:={trial_dir}",
    ]
    eval_proc = subprocess.Popen(
        eval_cmd, stdout=eval_log, stderr=subprocess.STDOUT,
    )
    try:
        eval_proc.wait(timeout=trial_timeout)
    except subprocess.TimeoutExpired:
        print("  ! eval timed out — killing", flush=True)
        eval_proc.kill()
        try:
            eval_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    finally:
        eval_log.close()

    # 4. Tear down everything: nav stack first, then sim, then nuke stragglers.
    for proc in (nav_proc, sim_proc):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=6)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    kill_ros()
    nav_log.close()
    sim_log.close()
    time.sleep(INTER_TRIAL_PAUSE_S)

    # 5. Locate + parse summary JSON.
    summaries = sorted(trial_dir.glob("summary*.json"))
    if not summaries:
        print("  ! no summary*.json produced", flush=True)
        return None
    try:
        return json.loads(summaries[-1].read_text())
    except Exception as e:
        print(f"  ! summary parse failed: {e}", flush=True)
        return None


# ─── Metric extraction ────────────────────────────────────────────────────
# Dot-paths into summary.json → CSV column header.
# Anything missing is recorded as blank — robust to evaluator schema drift.
METRIC_PATHS = [
    ("task.success",                       "SR"),
    ("task.time_to_goal_s",                "Time"),
    ("task.executed_path_length_m",        "Length"),
    ("task.flight_duration_s",             "FlightDur"),
    ("task.final_goal_distance_m",         "FinalDist"),

    ("safety.min_clearance_m",             "MinClr"),
    ("safety.mean_clearance_m",            "MeanClr"),
    ("safety.p05_clearance_m",             "p05Clr"),
    ("safety.empirical_cvar_5pct_m",       "CVaR5"),
    ("safety.empirical_cvar_10pct_m",      "CVaR10"),
    ("safety.collision_strict_events",     "CR"),
    ("safety.collision_near_miss_events",  "CR_nm"),
    ("safety.collision_tail_events",       "CR_tail"),
    ("safety.collision_strict_time_s",     "CR_time_s"),

    ("smoothness.cmd_accel_rms_norm",      "AccRMS"),
    ("smoothness.cmd_jerk_rms_norm",       "Jerk"),

    ("tracking.target_error_rms_m",        "TrkRMS"),
    ("tracking.target_error_p95_m",        "TrkP95"),

    ("planning_latency.mean_ms",           "Lat_avg"),
    ("planning_latency.p95_ms",            "Lat_p95"),
    ("planning_latency.max_ms",            "Lat_max"),
]


def _dotget(d, dotted: str):
    cur = d
    for k in dotted.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def append_row(master_csv: Path, idx: int, param_name: str, value,
               summary: Optional[dict]):
    base = {"trial": idx, "param": param_name, "value": value}
    metrics = {
        col: (_dotget(summary, path) if summary is not None else None)
        for path, col in METRIC_PATHS
    }
    row = {**base, **metrics}

    write_header = not master_csv.exists()
    with master_csv.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            w.writeheader()
        w.writerow(row)


# ─── Main ─────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start",          type=int, default=1,
                    help="Resume from trial index N (1-based).")
    ap.add_argument("--end",            type=int, default=None,
                    help="Stop after this trial index (inclusive).")
    ap.add_argument("--eval-duration",  type=int, default=EVAL_DURATION_S)
    ap.add_argument("--trial-timeout",  type=int, default=TRIAL_TIMEOUT_S)
    ap.add_argument("--out",            type=str, default=None,
                    help="Override output directory.")
    ap.add_argument("--dry-run",        action="store_true",
                    help="Print the plan and exit.")
    args = ap.parse_args()

    yaml_path = resolve_yaml_path()

    if args.out:
        results_root = Path(args.out).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_root = Path.home() / "IM2MPPI" / "results" / f"sweep_{stamp}"
    results_root.mkdir(parents=True, exist_ok=True)
    master_csv = results_root / "sweep_summary.csv"

    baseline_yaml_text = yaml_path.read_text()
    (results_root / "im2_mppi.yaml.baseline").write_text(baseline_yaml_text)

    sweeps = make_sweeps()
    total = sum(len(v) for _, v, _ in sweeps)

    print(f"YAML file       : {yaml_path}")
    print(f"Output dir      : {results_root}")
    print(f"Summary CSV     : {master_csv}")
    print(f"Total trials    : {total}")
    print(f"Start at trial  : {args.start}")
    if args.end:
        print(f"Stop after trial: {args.end}")
    print(f"Eval duration   : {args.eval_duration}s")

    if args.dry_run:
        idx = 0
        for name, vals, kind in sweeps:
            for v in vals:
                idx += 1
                print(f"  trial {idx:03d}  {name} = {v}  ({kind})")
        return

    # Restore yaml on Ctrl-C / kill so we never leave the repo in a weird state.
    def _restore_and_exit(*_):
        yaml_path.write_text(baseline_yaml_text)
        kill_ros()
        print("\n[interrupted] yaml restored, ROS killed; exiting.", flush=True)
        sys.exit(130)
    signal.signal(signal.SIGINT, _restore_and_exit)
    signal.signal(signal.SIGTERM, _restore_and_exit)

    try:
        idx = 0
        for name, vals, kind in sweeps:
            for v in vals:
                idx += 1
                if idx < args.start:
                    continue
                if args.end is not None and idx > args.end:
                    raise StopIteration

                summary = run_trial(
                    idx, name, v, kind,
                    baseline_yaml_text, yaml_path,
                    results_root,
                    args.eval_duration, args.trial_timeout,
                )
                append_row(master_csv, idx, name, v, summary)
    except StopIteration:
        pass
    finally:
        yaml_path.write_text(baseline_yaml_text)
        kill_ros()
        print(f"\nDone. {master_csv}", flush=True)


if __name__ == "__main__":
    main()
