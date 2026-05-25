#!/usr/bin/env python3
"""Create fixed-count variants of the intent-uncertain pedestrian world.

The source world currently contains a larger pedestrian set.  For controlled
benchmarks we keep the world footer, physics, branch-point paths, and obstacle
plugin settings intact, then retain only the first N pedestrian models.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


PERSON_START_RE = re.compile(r'^\s*<model name="person')


def build_count_world(base: Path, out_path: Path, count: int) -> int:
    lines = base.read_text(encoding="utf-8").splitlines(keepends=True)
    out = []
    pending_ped_comment = []
    in_person = False
    keep_person = False
    seen = 0

    for line in lines:
        stripped = line.strip()

        if not in_person and stripped.startswith("<!-- Pedestrian "):
            pending_ped_comment.append(line)
            continue

        if not in_person and PERSON_START_RE.search(line):
            seen += 1
            keep_person = seen <= count
            in_person = True
            if keep_person:
                out.extend(pending_ped_comment)
                out.append(line)
            pending_ped_comment.clear()
            continue

        if in_person:
            if keep_person:
                out.append(line)
            if stripped == "</model>":
                in_person = False
                keep_person = False
            continue

        if pending_ped_comment:
            out.extend(pending_ped_comment)
            pending_ped_comment.clear()
        out.append(line)

    if seen < count:
        raise ValueError(f"{base} only has {seen} pedestrian models; requested {count}")
    if in_person:
        raise ValueError(f"{base} ended inside a pedestrian model block")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("".join(out), encoding="utf-8")
    return seen


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate intent-uncertain worlds with selected pedestrian counts."
    )
    parser.add_argument("--base", required=True, type=Path, help="Base 45-pedestrian world")
    parser.add_argument("--out-dir", required=True, type=Path, help="Directory for generated worlds")
    parser.add_argument(
        "--counts",
        nargs="+",
        type=int,
        default=[35, 25],
        help="Pedestrian counts to generate",
    )
    args = parser.parse_args()

    if not args.base.is_file():
        raise FileNotFoundError(args.base)

    for count in args.counts:
        if count <= 0:
            raise ValueError(f"count must be positive, got {count}")
        out_path = args.out_dir / f"intent_branch_{count}.world"
        total = build_count_world(args.base, out_path, count)
        print(f"wrote {out_path} ({count}/{total} pedestrians)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
