#!/usr/bin/env python3
"""Compare two or more planner evaluation summary JSON files into a CSV."""

import csv
import json
import os
import sys


METRICS = [
    # Task completion: one predefined-lap completion is the primary boundary.
    ("task.success",                          "success"),
    ("task.completion_mode",                  "completion_mode"),
    ("task.completion_reason",                "completion_reason"),
    ("task.mission_time_s",                   "mission_time_s"),
    ("task.completion_time_s",                "completion_time_s"),
    ("task.time_to_goal_s",                   "time_to_goal_s"),
    ("task.final_goal_distance_m",            "final_goal_distance_m"),
    ("task.completed_path_length_m",          "completed_path_length_m"),
    ("task.executed_path_length_m",           "executed_path_length_m"),
    ("task.flight_duration_s",                "flight_duration_s"),
    ("task.lap_reference_length_m",           "lap_reference_length_m"),
    ("task.lap_progress_m",                   "lap_progress_m"),
    ("task.lap_progress_fraction",            "lap_progress_fraction"),
    # Safety — clearance stats
    ("safety.min_clearance_m",                "min_clearance_m"),
    ("safety.mean_clearance_m",               "mean_clearance_m"),
    ("safety.p05_clearance_m",                "p05_clearance_m"),
    # Safety — tiered collision counters (samples == time-in-collision proxy)
    ("safety.collision_strict_samples",       "collision_strict_samples"),
    ("safety.collision_near_miss_samples",    "collision_near_miss_samples"),
    ("safety.collision_tail_samples",         "collision_tail_samples"),
    ("safety.collision_strict_rate",          "collision_strict_rate"),
    ("safety.collision_near_miss_rate",       "collision_near_miss_rate"),
    ("safety.collision_tail_rate",            "collision_tail_rate"),
    ("safety.collision_strict_time_s",        "collision_strict_time_s"),
    # Safety — tiered collision EVENT counters (rising edge — primary KPI)
    ("safety.collision_strict_events",        "collision_strict_events"),
    ("safety.collision_near_miss_events",     "collision_near_miss_events"),
    ("safety.collision_tail_events",          "collision_tail_events"),
    # Safety — empirical CVaR
    ("safety.empirical_cvar_5pct_m",          "empirical_cvar_5pct_m"),
    ("safety.empirical_cvar_10pct_m",         "empirical_cvar_10pct_m"),
    ("safety.planned_min_clearance_m",        "planned_min_clearance_m"),
    # Tracking
    ("tracking.rms_target_error_m",           "rms_target_error_m"),
    ("tracking.max_target_error_m",           "max_target_error_m"),
    # Smoothness (commanded — preferred)
    ("smoothness.mean_speed_mps",             "mean_speed_mps"),
    ("smoothness.max_speed_mps",              "max_speed_mps"),
    ("smoothness.cmd_rms_accel_mps2",         "cmd_rms_accel_mps2"),
    ("smoothness.cmd_max_accel_mps2",         "cmd_max_accel_mps2"),
    ("smoothness.cmd_rms_jerk_mps3",          "cmd_rms_jerk_mps3"),
    ("smoothness.cmd_max_jerk_mps3",          "cmd_max_jerk_mps3"),
    # Planner latency
    ("planner.publish_rate_hz",               "publish_rate_hz"),
    ("planner.plan_latency_mean_ms",          "plan_latency_mean_ms"),
    ("planner.plan_latency_p95_ms",           "plan_latency_p95_ms"),
    ("planner.plan_latency_max_ms",           "plan_latency_max_ms"),
    ("planner.mean_planned_path_length_m",    "mean_planned_path_length_m"),
]


def nested_get(data, dotted):
    cur = data
    for key in dotted.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def load_summary(path):
    with open(path, "r") as f:
        data = json.load(f)
    row = {
        "file": os.path.basename(path),
        "algorithm": data.get("algorithm", os.path.basename(path)),
    }
    for dotted, label in METRICS:
        row[label] = nested_get(data, dotted)
    return row


def main(argv):
    if len(argv) < 2:
        print("usage: compare_planner_eval.py OUT.csv SUMMARY1.json SUMMARY2.json [...]",
              file=sys.stderr)
        return 2

    out_csv = argv[0]
    rows = [load_summary(path) for path in argv[1:]]
    fields = ["file", "algorithm"] + [label for _dotted, label in METRICS]

    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print("wrote {} ({} rows)".format(out_csv, len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
