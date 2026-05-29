#!/usr/bin/env python3
"""Replay saved benchmark trajectories in RViz.

The node reads *_timeseries.csv files produced by the benchmark scripts and
publishes one MarkerArray containing full paths, animated trails, moving vehicle
markers, direction arrows, and labels for each selected method.
"""

import bisect
import csv
import math
import os
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


METHODS = [
    ("M0_intent_mpc", "Intent-MPC", (0.88, 0.43, 0.16, 1.0)),
    ("M1_vanilla", "MPPI", (0.40, 0.42, 0.46, 1.0)),
    ("M5_dra_mppi", "DRA-MPPI", (0.30, 0.43, 0.78, 1.0)),
    ("M4_im2_full", "Ours", (0.06, 0.62, 0.29, 1.0)),
]


class Series:
    def __init__(
        self,
        method_key: str,
        label: str,
        color: Tuple[float, float, float, float],
        rows: List[Tuple[float, float, float, float]],
    ):
        self.method_key = method_key
        self.label = label
        self.color = color
        self.rows = rows
        self.times = [row[0] for row in rows]

    @property
    def duration(self) -> float:
        return self.times[-1] if self.times else 0.0

    def pose_at(self, t: float) -> Tuple[int, Point]:
        if not self.rows:
            return 0, Point()
        if t <= self.times[0]:
            return 0, make_point(self.rows[0])
        if t >= self.times[-1]:
            return len(self.rows) - 1, make_point(self.rows[-1])

        right = bisect.bisect_right(self.times, t)
        left = max(0, right - 1)
        right = min(right, len(self.rows) - 1)
        t0, x0, y0, z0 = self.rows[left]
        t1, x1, y1, z1 = self.rows[right]
        if t1 <= t0:
            return left, make_point(self.rows[left])

        ratio = (t - t0) / (t1 - t0)
        point = Point()
        point.x = x0 + ratio * (x1 - x0)
        point.y = y0 + ratio * (y1 - y0)
        point.z = z0 + ratio * (z1 - z0)
        return left, point

    def next_point_after(self, index: int, min_dist: float = 0.05) -> Optional[Point]:
        if not self.rows or index >= len(self.rows) - 1:
            return None
        base = make_point(self.rows[index])
        for row in self.rows[index + 1 : min(len(self.rows), index + 20)]:
            point = make_point(row)
            if dist_xy(base, point) >= min_dist:
                return point
        return make_point(self.rows[-1])

    def sampled_points(self, end_index: Optional[int] = None, max_points: int = 1600) -> List[Point]:
        if not self.rows:
            return []
        rows = self.rows if end_index is None else self.rows[: max(1, end_index + 1)]
        if not rows:
            return []
        stride = max(1, int(math.ceil(float(len(rows)) / float(max(1, max_points)))))
        points = [make_point(row) for row in rows[::stride]]
        last = make_point(rows[-1])
        if not points or dist_xyz(points[-1], last) > 1e-6:
            points.append(last)
        return points


def make_point(row: Tuple[float, float, float, float]) -> Point:
    _, x, y, z = row
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def dist_xy(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def dist_xyz(a: Point, b: Point) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def color_rgba(color: Tuple[float, float, float, float], alpha: Optional[float] = None) -> ColorRGBA:
    rgba = ColorRGBA()
    rgba.r, rgba.g, rgba.b, rgba.a = color
    if alpha is not None:
        rgba.a = alpha
    return rgba


def parse_seed_list(value: str) -> List[int]:
    seeds: List[int] = []
    for item in value.replace(";", ",").split(","):
        item = item.strip()
        if item:
            seeds.append(int(item))
    return seeds


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def parse_float(row: Dict[str, str], names: Sequence[str], default: Optional[float] = None) -> Optional[float]:
    for name in names:
        value = row.get(name, "")
        if value is None or value == "":
            continue
        try:
            return float(value)
        except ValueError:
            continue
    return default


def read_series(
    results_dir: str,
    method_key: str,
    label: str,
    color: Tuple[float, float, float, float],
    seed: int,
    z_offset: float,
) -> Optional[Series]:
    path = os.path.join(results_dir, f"{method_key}_seed{seed}_timeseries.csv")
    if not os.path.isfile(path):
        rospy.logwarn("Missing timeseries CSV: %s", path)
        return None

    rows: List[Tuple[float, float, float, float]] = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t = parse_float(raw, ("lap_time_s", "t"))
            x = parse_float(raw, ("x",))
            y = parse_float(raw, ("y",))
            z = parse_float(raw, ("z",), 1.0)
            if t is None or x is None or y is None or z is None:
                continue
            if not (math.isfinite(t) and math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            rows.append((t, x, y, z + z_offset))

    rows.sort(key=lambda item: item[0])
    if not rows:
        rospy.logwarn("No valid trajectory rows in %s", path)
        return None

    t0 = rows[0][0]
    rows = [(max(0.0, t - t0), x, y, z) for (t, x, y, z) in rows]
    return Series(method_key, label, color, rows)


def make_marker(ns: str, marker_id: int, marker_type: int, frame_id: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time(0)
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration(0.0)
    return marker


def add_line_marker(
    markers: MarkerArray,
    ns: str,
    marker_id: int,
    frame_id: str,
    points: Iterable[Point],
    color: Tuple[float, float, float, float],
    width: float,
    alpha: float,
) -> None:
    marker = make_marker(ns, marker_id, Marker.LINE_STRIP, frame_id)
    marker.points = list(points)
    marker.scale.x = width
    marker.color = color_rgba(color, alpha)
    markers.markers.append(marker)


def add_sphere_marker(
    markers: MarkerArray,
    ns: str,
    marker_id: int,
    frame_id: str,
    point: Point,
    color: Tuple[float, float, float, float],
) -> None:
    marker = make_marker(ns, marker_id, Marker.SPHERE, frame_id)
    marker.pose.position = point
    marker.scale.x = 0.34
    marker.scale.y = 0.34
    marker.scale.z = 0.18
    marker.color = color_rgba(color, 1.0)
    markers.markers.append(marker)


def add_arrow_marker(
    markers: MarkerArray,
    ns: str,
    marker_id: int,
    frame_id: str,
    start: Point,
    end: Optional[Point],
    color: Tuple[float, float, float, float],
) -> None:
    if end is None or dist_xy(start, end) < 0.03:
        return
    marker = make_marker(ns, marker_id, Marker.ARROW, frame_id)
    marker.points = [start, end]
    marker.scale.x = 0.08
    marker.scale.y = 0.22
    marker.scale.z = 0.22
    marker.color = color_rgba(color, 0.95)
    markers.markers.append(marker)


def add_text_marker(
    markers: MarkerArray,
    ns: str,
    marker_id: int,
    frame_id: str,
    point: Point,
    text: str,
    color: Tuple[float, float, float, float],
    scale: float = 0.34,
) -> None:
    marker = make_marker(ns, marker_id, Marker.TEXT_VIEW_FACING, frame_id)
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z + 0.42
    marker.scale.z = scale
    marker.color = color_rgba(color, 1.0)
    marker.text = text
    markers.markers.append(marker)


def delete_all_marker() -> MarkerArray:
    marker = Marker()
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


def load_all_series(results_dir: str, seeds: Sequence[int], z_offset: float) -> Dict[int, List[Series]]:
    loaded: Dict[int, List[Series]] = {}
    for seed in seeds:
        series_for_seed: List[Series] = []
        for method_key, label, color in METHODS:
            series = read_series(results_dir, method_key, label, color, seed, z_offset)
            if series is not None:
                series_for_seed.append(series)
        if series_for_seed:
            loaded[seed] = series_for_seed
    return loaded


def first_point(series_list: Sequence[Series]) -> Optional[Point]:
    for series in series_list:
        if series.rows:
            return make_point(series.rows[0])
    return None


def build_markers(seed: int, series_list: Sequence[Series], play_t: float, frame_id: str, max_path_points: int) -> MarkerArray:
    markers = MarkerArray()
    duration = max(series.duration for series in series_list)

    for method_idx, series in enumerate(series_list):
        index, point = series.pose_at(play_t)
        full_points = series.sampled_points(max_points=max_path_points)
        trail_points = series.sampled_points(end_index=index, max_points=max_path_points)
        if trail_points:
            trail_points[-1] = point

        add_line_marker(markers, "seed_traj_full_paths", method_idx, frame_id, full_points, series.color, 0.045, 0.22)
        add_line_marker(markers, "seed_traj_trails", 100 + method_idx, frame_id, trail_points, series.color, 0.095, 0.95)
        add_sphere_marker(markers, "seed_traj_agents", 200 + method_idx, frame_id, point, series.color)
        add_arrow_marker(markers, "seed_traj_arrows", 300 + method_idx, frame_id, point, series.next_point_after(index), series.color)
        add_text_marker(markers, "seed_traj_labels", 400 + method_idx, frame_id, point, series.label, series.color)

    anchor = first_point(series_list)
    if anchor is not None:
        label_point = Point()
        label_point.x = anchor.x - 2.6
        label_point.y = anchor.y + 2.0
        label_point.z = max(anchor.z + 1.2, 1.7)
        add_text_marker(
            markers,
            "seed_traj_seed_label",
            900,
            frame_id,
            label_point,
            f"seed {seed}   {min(play_t, duration):.1f}s / {duration:.1f}s",
            (0.92, 0.92, 0.92, 1.0),
            0.46,
        )

    return markers


def main() -> None:
    rospy.init_node("seed_trajectory_rviz_replay", anonymous=False)

    default_results = os.path.expanduser("~/IM2MPPI/results/full_lap_bag_20260528_080853")
    results_dir = os.path.expanduser(rospy.get_param("~results_dir", default_results))
    seed_text = rospy.get_param("~seeds", "12,19,21,24,25,28,29")
    seeds = parse_seed_list(seed_text)
    frame_id = rospy.get_param("~frame_id", "map")
    playback_speed = float(rospy.get_param("~playback_speed", 1.0))
    rate_hz = float(rospy.get_param("~rate", 20.0))
    pause_between_seeds = float(rospy.get_param("~pause_between_seeds", 2.0))
    max_path_points = int(rospy.get_param("~max_path_points", 1600))
    z_offset = float(rospy.get_param("~z_offset", 0.05))
    loop = parse_bool(rospy.get_param("~loop", True))

    if not os.path.isdir(results_dir):
        rospy.logerr("Results directory does not exist: %s", results_dir)
        return
    if not seeds:
        rospy.logerr("No seeds requested.")
        return

    loaded = load_all_series(results_dir, seeds, z_offset)
    seeds = [seed for seed in seeds if seed in loaded]
    if not seeds:
        rospy.logerr("No usable timeseries CSV files found in %s", results_dir)
        return

    publisher = rospy.Publisher("/seed_trajectory_viz/markers", MarkerArray, queue_size=2, latch=True)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("Seed trajectory RViz replay loaded %d seed(s) from %s", len(seeds), results_dir)
    rospy.loginfo("Publishing MarkerArray on /seed_trajectory_viz/markers")

    seed_index = 0
    play_t = 0.0
    last_wall = time.monotonic()
    sent_clear = False

    while not rospy.is_shutdown():
        seed = seeds[seed_index]
        series_list = loaded[seed]
        duration = max(series.duration for series in series_list)

        if not sent_clear:
            publisher.publish(delete_all_marker())
            sent_clear = True

        now = time.monotonic()
        dt = now - last_wall
        last_wall = now
        if dt < 0.0 or dt > 1.0:
            dt = 1.0 / max(rate_hz, 1.0)
        play_t += dt * playback_speed

        publisher.publish(build_markers(seed, series_list, play_t, frame_id, max_path_points))

        if play_t >= duration + pause_between_seeds:
            seed_index += 1
            if seed_index >= len(seeds):
                if not loop:
                    break
                seed_index = 0
            play_t = 0.0
            sent_clear = False

        rate.sleep()


if __name__ == "__main__":
    main()
