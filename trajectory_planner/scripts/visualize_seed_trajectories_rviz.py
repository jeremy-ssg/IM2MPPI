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
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

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

BAG_TOPICS = [
    "/dynamic_map/inflated_voxel_map",
    "/dynamic_map/voxel_map",
    "/dynamic_map/explored_voxel_map",
    "/onboard_detector/GT_obstacle_bbox",
    "/onboard_detector/dynamic_bboxes",
    "/onboard_detector/tracked_bboxes",
    "/onboard_detector/history_trajectories",
    "/onboard_detector/velocity_visualizaton",
    "/im2mppi/dynamic_obstacle_predictions",
]


try:
    import rosbag
except ImportError:  # pragma: no cover - only available inside the ROS runtime.
    rosbag = None

try:
    from rviz.srv import SendFilePath
except ImportError:  # pragma: no cover - only available inside the ROS runtime.
    SendFilePath = None


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


def parse_string_list(value: str) -> List[str]:
    items: List[str] = []
    for item in value.replace(";", ",").split(","):
        item = item.strip()
        if item:
            items.append(item)
    return items


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


def has_timeseries_csv(results_dir: str) -> bool:
    if not os.path.isdir(results_dir):
        return False
    return any(name.endswith("_timeseries.csv") for name in os.listdir(results_dir))


def normalize_results_dir(results_dir: str) -> str:
    if not os.path.isdir(results_dir):
        return results_dir
    if has_timeseries_csv(results_dir):
        return results_dir
    nested = os.path.join(results_dir, os.path.basename(os.path.normpath(results_dir)))
    if has_timeseries_csv(nested):
        rospy.loginfo("Using nested results directory: %s", nested)
        return nested
    return results_dir


def results_root_for_bags(results_dir: str) -> str:
    candidates = [
        results_dir,
        os.path.dirname(results_dir),
        os.path.dirname(os.path.dirname(results_dir)),
    ]
    for candidate in candidates:
        if candidate and os.path.isdir(os.path.join(candidate, "bags")):
            return candidate
    return results_dir


def default_screenshot_dir(results_dir: str) -> str:
    return os.path.join(results_root_for_bags(results_dir), "rviz_seed_screenshots")


def find_bag_for_seed(results_dir: str, seed: int, method_order: Sequence[str]) -> Optional[str]:
    root = results_root_for_bags(results_dir)
    bag_dirs = [
        os.path.join(root, "bags"),
        os.path.join(results_dir, "bags"),
        os.path.join(os.path.dirname(results_dir), "bags"),
    ]

    seen: Set[str] = set()
    bag_dirs = [d for d in bag_dirs if d and not (d in seen or seen.add(d))]
    for bag_dir in bag_dirs:
        if not os.path.isdir(bag_dir):
            continue
        for method in method_order:
            candidate = os.path.join(bag_dir, f"{method}_seed{seed}.bag")
            if os.path.isfile(candidate):
                return candidate
        for method in method_order:
            matches = [
                os.path.join(bag_dir, name)
                for name in os.listdir(bag_dir)
                if name.endswith(".bag") and f"{method}_seed{seed}" in name
            ]
            if matches:
                return sorted(matches)[0]
        matches = [
            os.path.join(bag_dir, name)
            for name in os.listdir(bag_dir)
            if name.endswith(".bag") and f"seed{seed}" in name
        ]
        if matches:
            return sorted(matches)[0]
    return None


class BagReplay:
    def __init__(self, topics: Sequence[str]):
        self.topics = list(topics)
        self.publishers = {}
        self.bag = None
        self.iterator = None
        self.next_item = None
        self.first_time = None
        self.path = None
        self.exhausted = True

    def close(self) -> None:
        if self.bag is not None:
            self.bag.close()
        self.bag = None
        self.iterator = None
        self.next_item = None
        self.first_time = None
        self.path = None
        self.exhausted = True

    def open(self, path: Optional[str]) -> None:
        self.close()
        if not path:
            return
        if rosbag is None:
            rospy.logwarn("rosbag Python module is unavailable; obstacle bag replay is disabled.")
            return
        try:
            self.bag = rosbag.Bag(path, "r")
            available = set(self.bag.get_type_and_topic_info().topics.keys())
            topics = [topic for topic in self.topics if topic in available]
            if not topics:
                rospy.logwarn("No configured obstacle/map topics found in bag: %s", path)
                self.close()
                return
            self.iterator = self.bag.read_messages(topics=topics)
            self.path = path
            self.exhausted = False
            rospy.loginfo("Replaying obstacle/map bag: %s", path)
        except Exception as exc:  # pragma: no cover - depends on local bag files.
            rospy.logwarn("Failed to open bag %s: %s", path, exc)
            self.close()

    def publish_until(self, play_t: float) -> None:
        if self.exhausted or self.iterator is None:
            return
        while not rospy.is_shutdown():
            if self.next_item is None:
                try:
                    self.next_item = next(self.iterator)
                except StopIteration:
                    self.exhausted = True
                    return
                except Exception as exc:  # pragma: no cover - depends on local bag files.
                    rospy.logwarn("Bag replay stopped for %s: %s", self.path, exc)
                    self.exhausted = True
                    return

            topic, msg, stamp = self.next_item
            stamp_s = stamp.to_sec()
            if self.first_time is None:
                self.first_time = stamp_s
            rel_t = stamp_s - self.first_time
            if rel_t > play_t:
                return

            self.publish(topic, msg)
            self.next_item = None

    def publish(self, topic: str, msg) -> None:
        normalize_message_stamp(msg)
        if topic not in self.publishers:
            self.publishers[topic] = rospy.Publisher(topic, msg.__class__, queue_size=20, latch=True)
            rospy.sleep(0.01)
        self.publishers[topic].publish(msg)


def normalize_message_stamp(msg) -> None:
    zero = rospy.Time(0)
    if hasattr(msg, "header"):
        msg.header.stamp = zero
    if hasattr(msg, "markers"):
        for marker in msg.markers:
            marker.header.stamp = zero


def save_rviz_screenshot(service_name: str, filename: str, timeout: float) -> bool:
    if SendFilePath is None:
        rospy.logwarn("rviz/SendFilePath service type is unavailable; cannot save RViz screenshot.")
        return False
    try:
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        rospy.wait_for_service(service_name, timeout=timeout)
        save_image = rospy.ServiceProxy(service_name, SendFilePath)
        response = save_image(filename)
        if hasattr(response, "success") and not response.success:
            rospy.logwarn("RViz refused to save screenshot: %s", filename)
            return False
        rospy.loginfo("Saved RViz screenshot: %s", filename)
        return True
    except Exception as exc:  # pragma: no cover - depends on local RViz service.
        rospy.logwarn("Failed to save RViz screenshot via %s: %s", service_name, exc)
        return False


def first_point(series_list: Sequence[Series]) -> Optional[Point]:
    for series in series_list:
        if series.rows:
            return make_point(series.rows[0])
    return None


def build_markers(seed: int, series_list: Sequence[Series], play_t: float, frame_id: str, max_path_points: int, show_labels: bool) -> MarkerArray:
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
        if show_labels:
            add_text_marker(markers, "seed_traj_labels", 400 + method_idx, frame_id, point, series.label, series.color)

    anchor = first_point(series_list)
    if show_labels and anchor is not None:
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
    results_dir = normalize_results_dir(os.path.expanduser(rospy.get_param("~results_dir", default_results)))
    seed_text = rospy.get_param("~seeds", "12,19,21,24,25,28,29")
    seeds = parse_seed_list(seed_text)
    frame_id = rospy.get_param("~frame_id", "map")
    playback_speed = float(rospy.get_param("~playback_speed", 1.0))
    rate_hz = float(rospy.get_param("~rate", 20.0))
    pause_between_seeds = float(rospy.get_param("~pause_between_seeds", 10.0))
    max_path_points = int(rospy.get_param("~max_path_points", 1600))
    z_offset = float(rospy.get_param("~z_offset", 0.05))
    loop = parse_bool(rospy.get_param("~loop", True))
    replay_bags = parse_bool(rospy.get_param("~replay_bags", True))
    bag_method_order = parse_string_list(rospy.get_param("~bag_method_order", "M4_im2_full,M5_dra_mppi,M1_vanilla,M0_intent_mpc"))
    bag_topics = parse_string_list(rospy.get_param("~bag_topics", ",".join(BAG_TOPICS)))
    save_screenshots = parse_bool(rospy.get_param("~save_rviz_screenshots", True))
    screenshot_dir_param = os.path.expanduser(rospy.get_param("~screenshot_dir", ""))
    screenshot_dir = screenshot_dir_param if screenshot_dir_param else default_screenshot_dir(results_dir)
    rviz_save_image_service = rospy.get_param("~rviz_save_image_service", "/rviz/save_image")
    screenshot_delay = float(rospy.get_param("~screenshot_delay", 0.5))
    screenshot_once_per_seed = parse_bool(rospy.get_param("~screenshot_once_per_seed", True))
    show_labels = parse_bool(rospy.get_param("~show_labels", False))

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
    current_bag_seed = None
    bag_replay = BagReplay(bag_topics)
    screenshot_saved: Set[int] = set()

    while not rospy.is_shutdown():
        seed = seeds[seed_index]
        series_list = loaded[seed]
        duration = max(series.duration for series in series_list)

        if not sent_clear:
            publisher.publish(delete_all_marker())
            sent_clear = True

        if replay_bags and current_bag_seed != seed:
            current_bag_seed = seed
            bag_path = find_bag_for_seed(results_dir, seed, bag_method_order)
            if bag_path is None:
                rospy.logwarn("No bag found for seed %d; only CSV trajectories will be shown.", seed)
                bag_replay.close()
            else:
                bag_replay.open(bag_path)

        now = time.monotonic()
        dt = now - last_wall
        last_wall = now
        if dt < 0.0 or dt > 1.0:
            dt = 1.0 / max(rate_hz, 1.0)
        play_t += dt * playback_speed
        display_t = min(play_t, duration)

        if replay_bags:
            bag_replay.publish_until(display_t)
        publisher.publish(build_markers(seed, series_list, display_t, frame_id, max_path_points, show_labels))

        should_save_screenshot = save_screenshots and play_t >= duration
        if should_save_screenshot and (not screenshot_once_per_seed or seed not in screenshot_saved):
            publisher.publish(build_markers(seed, series_list, duration, frame_id, max_path_points, show_labels))
            if replay_bags:
                bag_replay.publish_until(duration)
            rospy.sleep(max(0.0, screenshot_delay))
            screenshot_path = os.path.join(screenshot_dir, f"seed{seed:02d}_rviz.png")
            if save_rviz_screenshot(rviz_save_image_service, screenshot_path, 2.0):
                screenshot_saved.add(seed)

        if play_t >= duration + pause_between_seeds:
            seed_index += 1
            if seed_index >= len(seeds):
                if not loop:
                    break
                seed_index = 0
            play_t = 0.0
            sent_clear = False

        rate.sleep()

    bag_replay.close()


if __name__ == "__main__":
    main()
