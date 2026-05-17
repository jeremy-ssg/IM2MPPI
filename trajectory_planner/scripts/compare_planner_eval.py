#!/usr/bin/env python3
"""Compare two or more planner evaluation summary JSON files."""

import csv
import json
import os
import sys


METRICS = [
    ("task.success", "success"),
    ("task.time_to_goal_s", "time_to_goal_s"),
    ("task.final_goal_distance_m", "final_goal_distance_m"),
    ("task.executed_path_length_m", "executed_path_length_m"),
    ("safety.min_clearance_m", "min_clearance_m"),
    ("safety.p05_clearance_m", "p05_clearance_m"),
    ("safety.collision_samples", "collision_samples"),
    ("safety.collision_time_s", "collision_time_s"),
    ("safety.planned_min_clearance_m", "planned_min_clearance_m"),
    ("tracking.rms_target_error_m", "rms_target_error_m"),
    ("tracking.max_target_error_m", "max_target_error_m"),
    ("smoothness.max_speed_mps", "max_speed_mps"),
    ("smoothness.rms_accel_mps2", "rms_accel_mps2"),
    ("smoothness.rms_jerk_mps3", "rms_jerk_mps3"),
    ("planner.publish_rate_hz", "planner_publish_rate_hz"),
    ("planner.mean_planned_path_length_m", "mean_planned_path_length_m"),
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
        print("usage: compare_planner_eval.py OUT.csv SUMMARY1.json SUMMARY2.json [...]", file=sys.stderr)
        return 2

    out_csv = argv[0]
    rows = [load_summary(path) for path in argv[1:]]
    fields = ["file", "algorithm"] + [label for _dotted, label in METRICS]

    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print("wrote {}".format(out_csv))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
