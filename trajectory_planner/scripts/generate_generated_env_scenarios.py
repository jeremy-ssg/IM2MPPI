#!/usr/bin/env python3
"""Generate fixed-trajectory dynamic-obstacle benchmark worlds.

The source world is generated_env.world:
  - 20 static cylinders
  - 20 static boxes
  - 40 dynamic cylinders and 40 dynamic boxes with libobstaclePathPlugin

This script keeps the static part unchanged and derives scenario worlds by
retaining a balanced first N/2 cylinders + N/2 boxes and rewriting their
plugin speeds.
"""

import argparse
import os
import re
from pathlib import Path


SCENARIOS = {
    "generated_sparse_slow.world": {
        "label": "Sparse-Slow",
        "keep_dynamic": 20,
        "speed_range": (0.3, 0.5),
    },
    "generated_sparse_medium.world": {
        # NEW: between Sparse-Slow (20) and Medium-Normal (40)
        "label": "Sparse-Medium",
        "keep_dynamic": 25,
        "speed_range": (0.4, 0.6),
    },
    "generated_medium_light.world": {
        # NEW: between Sparse-Medium (25) and Medium-Normal (40)
        "label": "Medium-Light",
        "keep_dynamic": 35,
        "speed_range": (0.5, 0.7),
    },
    "generated_medium_normal.world": {
        "label": "Medium-Normal",
        "keep_dynamic": 40,
        "speed_range": (0.5, 0.8),
    },
    "generated_dense_normal.world": {
        "label": "Dense-Normal",
        "keep_dynamic": 80,
        "speed_range": (0.5, 0.8),
    },
    "generated_dense_fast.world": {
        "label": "Dense-Fast",
        "keep_dynamic": 80,
        "speed_range": (0.8, 1.1),
    },
}

MODEL_RE = re.compile(
    r"(?P<block>\s*<model\s+name=(?P<quote>['\"])dynamic_(?P<kind>cylinder|box)_(?P<idx>\d+)[^'\"]*(?P=quote)>.*?</model>\s*)",
    re.DOTALL,
)
VELOCITY_RE = re.compile(r"<velocity>\s*[-+0-9.eE]+\s*</velocity>")


def deterministic_speed(idx, lo, hi):
    """Stable pseudo-random speed in [lo, hi] based only on obstacle index."""
    if hi <= lo:
        return lo
    frac = ((idx * 37 + 17) % 100) / 99.0
    return lo + (hi - lo) * frac


def rewrite_world(text, label, keep_dynamic, speed_range):
    lo, hi = speed_range
    # Odd keep_dynamic (e.g. 35 or 25): give one extra to cylinders so totals
    # are exact instead of off-by-one.
    keep_cylinders = (keep_dynamic + 1) // 2
    keep_boxes     = keep_dynamic - keep_cylinders
    kept = 0
    removed = 0
    speeds = []

    def repl(match):
        nonlocal kept, removed
        block = match.group("block")
        kind = match.group("kind")
        idx = int(match.group("idx"))
        keep_n = keep_cylinders if kind == "cylinder" else keep_boxes
        if idx >= keep_n:
            removed += 1
            return "\n"
        global_idx = idx if kind == "cylinder" else 40 + idx
        speed = deterministic_speed(global_idx, lo, hi)
        speeds.append(speed)
        kept += 1
        block = VELOCITY_RE.sub(f"<velocity>{speed:.6f}</velocity>", block, count=1)
        return block

    out = MODEL_RE.sub(repl, text)
    note = (
        f"\n                <!-- Generated benchmark scenario: {label}. "
        f"Dynamic cylinders: {keep_dynamic}; speed range: {lo:.1f}-{hi:.1f} m/s. -->\n"
    )
    out = out.replace("<world name='default'>", "<world name='default'>" + note, 1)
    out = "\n".join(line.rstrip() for line in out.splitlines()).rstrip() + "\n"
    return out, kept, removed, speeds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base",
        default="uav_simulator/worlds/generated_env/generated_env.world",
        help="source generated_env world",
    )
    parser.add_argument(
        "--out-dir",
        default="uav_simulator/worlds/generated_env",
        help="directory for generated scenario worlds",
    )
    args = parser.parse_args()

    base = Path(args.base)
    out_dir = Path(args.out_dir)
    text = base.read_text()
    out_dir.mkdir(parents=True, exist_ok=True)

    for name, cfg in SCENARIOS.items():
        out_text, kept, removed, speeds = rewrite_world(
            text,
            cfg["label"],
            cfg["keep_dynamic"],
            cfg["speed_range"],
        )
        out_path = out_dir / name
        out_path.write_text(out_text)
        speed_msg = "n/a"
        if speeds:
            speed_msg = f"{min(speeds):.3f}-{max(speeds):.3f}"
        print(
            f"{cfg['label']}: wrote {os.fspath(out_path)} "
            f"(kept={kept}, removed={removed}, speeds={speed_msg})"
        )


if __name__ == "__main__":
    main()
