#!/usr/bin/env python3
"""
Runtime evaluator for Intent-MPC and IM2-MPPI.

Records safety / tracking / smoothness / planning-latency metrics for one
planner per launch. Outputs JSON summary + CSV time-series for downstream
statistical comparison.

Improvements over the initial version:
  * Tiered collision metrics (strict + near-miss + tail).
  * Empirical CVaR over the worst-α clearance tail (matches the theoretical
    CVaR objective the planner optimizes).
  * Acceleration / jerk computed from the COMMANDED target (no odom noise).
  * Stops sampling once the configured task is complete. For the predefined
    circle experiment this means one reference lap, not a fixed time window.
  * Subscribes to /im2mppi/plan_time_ms (or any configured plan-time topic)
    to record actual planning latency (mean / p95 / max).
"""

import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from tracking_controller.msg import Target
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────────────────────────
#  Math helpers
# ─────────────────────────────────────────────────────────────────────────────

def norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dist3(a, b):
    return norm3(sub3(a, b))


def point_from_pose(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def path_points(path_msg):
    return [point_from_pose(ps.pose) for ps in path_msg.poses]


def aabb_clearance(point, center, size):
    """Same SDF as the C++ planner (aabbSDF in im2_mppi_planner.cpp)."""
    half = (max(size[0], 1e-6) * 0.5,
            max(size[1], 1e-6) * 0.5,
            max(size[2], 1e-6) * 0.5)
    q = (abs(point[0] - center[0]) - half[0],
         abs(point[1] - center[1]) - half[1],
         abs(point[2] - center[2]) - half[2])
    outside = norm3((max(q[0], 0.0), max(q[1], 0.0), max(q[2], 0.0)))
    inside = min(max(q[0], q[1], q[2]), 0.0)
    return outside + inside


def mean(values):
    return sum(values) / len(values) if values else None


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def percentile(values, q):
    if not values:
        return None
    vals = sorted(values)
    idx = int(round((len(vals) - 1) * q))
    return vals[max(0, min(idx, len(vals) - 1))]


def empirical_cvar(values, alpha, worst="low"):
    """
    Empirical CVaR over the worst α fraction of `values`.
      worst="low"  : low values are bad (e.g., clearance — small = dangerous)
                     returns mean of the smallest α fraction.
      worst="high" : high values are bad (e.g., cost, latency)
                     returns mean of the largest α fraction.
    """
    if not values:
        return None
    n = len(values)
    k = max(1, int(round(n * alpha)))
    s = sorted(values)
    tail = s[:k] if worst == "low" else s[-k:]
    return sum(tail) / len(tail)


def path_length(points):
    return sum(dist3(points[i], points[i - 1]) for i in range(1, len(points)))


def dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def project_onto_polyline(points, point, min_s=None, backtrack=2.0):
    """Project a point onto a polyline and return (arc_length_s, distance)."""
    if len(points) < 2:
        return None, None

    candidates = []
    accum = 0.0
    for i in range(1, len(points)):
        a = points[i - 1]
        b = points[i]
        ab = sub3(b, a)
        seg_len = norm3(ab)
        if seg_len < 1e-9:
            continue

        ap = sub3(point, a)
        u = max(0.0, min(1.0, dot3(ap, ab) / (seg_len * seg_len)))
        proj = (a[0] + u * ab[0], a[1] + u * ab[1], a[2] + u * ab[2])
        candidates.append((accum + u * seg_len, dist3(point, proj)))
        accum += seg_len

    if not candidates:
        return None, None

    # A closed or near-closed trajectory has two physically close places near
    # the lap boundary. Once progress is high, avoid snapping back to s=0.
    if min_s is not None:
        filtered = [c for c in candidates if c[0] >= min_s - backtrack]
        if filtered:
            candidates = filtered

    best_s, best_dist = min(candidates, key=lambda x: x[1])
    return best_s, best_dist


def unwrap_closed_progress(raw_s, previous_unwrapped_s, reference_length):
    """Lift a closed-path arc length into a monotonic lap-progress frame."""
    if previous_unwrapped_s is None or reference_length <= 1e-6:
        return raw_s

    k0 = round((previous_unwrapped_s - raw_s) / reference_length)
    candidates = [
        raw_s + (k0 + dk) * reference_length
        for dk in (-1, 0, 1)
    ]
    return min(candidates, key=lambda s: abs(s - previous_unwrapped_s))


def finite_diff_norms(samples):
    out = []
    for i in range(1, len(samples)):
        t0, v0 = samples[i - 1]
        t1, v1 = samples[i]
        dt = t1 - t0
        if dt > 1e-6:
            out.append(norm3(sub3(v1, v0)) / dt)
    return out


# ─────────────────────────────────────────────────────────────────────────────
#  Evaluator
# ─────────────────────────────────────────────────────────────────────────────

class Evaluator:
    def __init__(self):
        self.algorithm = rospy.get_param("~algorithm", "im2_mppi")
        self.duration = float(rospy.get_param("~duration", 120.0))
        self.output_dir = rospy.get_param(
            "~output_dir",
            os.path.join(os.path.expanduser("~"), ".ros", "mppi_eval"),
        )
        self.goal_radius = float(rospy.get_param("~goal_radius", 0.5))
        self.sample_hz = float(rospy.get_param("~sample_hz", 20.0))
        self.requested_completion_mode = rospy.get_param("~completion_mode", "auto")
        self.shutdown_on_success = bool(rospy.get_param("~shutdown_on_success", True))

        lap_completion_radius = float(rospy.get_param("~lap_completion_radius", -1.0))
        self.lap_completion_radius = (
            self.goal_radius if lap_completion_radius <= 0.0 else lap_completion_radius)
        self.lap_finish_fraction = float(
            rospy.get_param("~lap_finish_fraction", 0.98))
        self.lap_min_path_fraction = float(
            rospy.get_param("~lap_min_path_fraction", 0.85))
        self.lap_progress_max_deviation = float(
            rospy.get_param("~lap_progress_max_deviation", 3.0))

        # Tiered collision thresholds (m).
        self.collision_strict   = float(rospy.get_param("~collision_strict",   0.15))
        self.collision_near_miss = float(rospy.get_param("~collision_near_miss", 0.30))
        self.collision_tail     = float(rospy.get_param("~collision_tail",     0.50))

        # CVaR α for the safety tail (worst α fraction of clearances).
        self.cvar_alpha = float(rospy.get_param("~cvar_alpha", 0.05))

        # Topics
        self.odom_topic     = rospy.get_param("~odom_topic", "/CERLAB/quadcopter/odom")
        self.target_topic   = rospy.get_param("~target_topic", "/autonomous_flight/target_state")
        self.goal_topic     = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/onboard_detector/GT_obstacle_bbox")
        self.path_topic     = rospy.get_param("~path_topic", "")
        self.plan_time_topic = rospy.get_param(
            "~plan_time_topic", "/im2mppi/plan_time_ms")
        if not self.path_topic:
            self.path_topic = self.default_path_topic(self.algorithm)

        ref_param = rospy.get_param("~lap_reference_path", "")
        if not ref_param:
            ref_param = rospy.get_param("/autonomous_flight/predefined_goal_directory", "")
        self.lap_reference_path = self.resolve_reference_path(ref_param)
        self.lap_reference_points = self.load_reference_points(self.lap_reference_path)
        self.lap_reference_length = path_length(self.lap_reference_points)

        if self.requested_completion_mode == "auto":
            self.completion_mode = "lap" if self.lap_reference_length > 1e-6 else "goal"
        else:
            self.completion_mode = self.requested_completion_mode
        if self.completion_mode == "lap" and self.lap_reference_length <= 1e-6:
            rospy.logwarn("[eval] lap completion requested but reference path is unavailable; falling back to goal mode.")
            self.completion_mode = "goal"

        self.lock = threading.Lock()
        self.start_time = None
        self.goal = None
        self.latest_odom = None
        self.latest_target = None
        self.latest_obstacles = []
        self.latest_path = None

        # Per-sample storage (in-flight only — stops at arrival).
        self.odom_samples = []
        self.target_errors = []
        self.clearances = []
        self.target_samples = []      # full /autonomous_flight/target_state stream
        self.cmd_accel_samples = []   # (t, (ax,ay,az)) from target msg
        self.path_metrics = []
        self.path_arrival_times = []
        self.plan_time_samples_ms = []
        self.plan_time_samples = []   # (t, plan_time_ms)

        # Tiered collision counters.
        self.collision_strict_samples    = 0
        self.collision_near_miss_samples = 0
        self.collision_tail_samples      = 0
        self.collision_strict_time = 0.0
        self.last_sample_time = None

        # Tiered collision EVENT counters (rising-edge detection — each
        # entry into a tier counts as one event regardless of how many
        # samples the drone stays in that tier afterwards).
        self.collision_strict_events    = 0
        self.collision_near_miss_events = 0
        self.collision_tail_events      = 0
        self.prev_in_strict    = False
        self.prev_in_near_miss = False
        self.prev_in_tail      = False
        # Per-event log (timestamp, tier, clearance_at_entry) for forensics.
        self.collision_event_log = []

        # Flight phase control.
        self.flight_ended = False
        self.time_to_goal = None
        self.completion_time = None
        self.completion_reason = None
        self.success = False
        self.finish_requested = False
        self.outputs_written = False
        self.completed_path_length = None
        self.completion_wall_time = None
        self.executed_path_length_live = 0.0
        self.last_odom_position_for_length = None
        self.lap_executed_path_length_live = 0.0
        self.last_lap_odom_position_for_length = None
        self.lap_progress_m = 0.0
        self.lap_progress_fraction = None
        self.lap_nearest_distance = None
        self.lap_reference_s = None
        self.lap_reference_fraction = None
        self.lap_start_reference_s = None
        self.lap_unwrapped_reference_s = None
        self.lap_started = False
        self.lap_start_time = None

        os.makedirs(self.output_dir, exist_ok=True)

        rospy.Subscriber(self.odom_topic,     Odometry,    self.odom_cb,     queue_size=50)
        rospy.Subscriber(self.target_topic,   Target,      self.target_cb,   queue_size=50)
        rospy.Subscriber(self.goal_topic,     PoseStamped, self.goal_cb,     queue_size=5)
        rospy.Subscriber(self.path_topic,     Path,        self.path_cb,     queue_size=20)
        rospy.Subscriber(self.obstacle_topic, MarkerArray, self.obstacle_cb, queue_size=10)
        rospy.Subscriber(self.plan_time_topic, Float64,    self.plan_time_cb, queue_size=50)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.sample_hz, 1.0)),
                                 self.sample_cb)
        self.shutdown_timer = rospy.Timer(rospy.Duration(max(self.duration, 1.0)),
                                          self.finish_cb, oneshot=True)

    # ── Topic defaults ────────────────────────────────────────────────────

    @staticmethod
    def default_path_topic(algorithm):
        if algorithm in ("intent_mpc", "intend_mpc", "mpc"):
            return "/mpcNavigation/mpc_trajectory"
        return "/im2mppi/best_trajectory"

    def now_rel(self):
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now
        return (now - self.start_time).to_sec()

    def ros_package_path(self, package):
        try:
            import rospkg  # ROS dependency, available on the robot/WSL side.
            return rospkg.RosPack().get_path(package)
        except Exception:  # noqa: BLE001
            return None

    def resolve_reference_path(self, raw_path):
        if not raw_path or raw_path == "None":
            return None

        expanded = os.path.expandvars(os.path.expanduser(raw_path))
        candidates = []
        if os.path.isabs(expanded) and os.path.isfile(expanded):
            return expanded
        if not os.path.isabs(expanded):
            candidates.append(os.path.abspath(expanded))

        pkg_path = self.ros_package_path("autonomous_flight")
        if pkg_path:
            candidates.append(os.path.join(pkg_path, expanded.lstrip("/\\")))

        for path in candidates:
            if os.path.isfile(path):
                return path
        return expanded

    def load_reference_points(self, path):
        points = []
        if not path or not os.path.isfile(path):
            return points
        with open(path) as f:
            for line in f:
                parts = line.split()
                if len(parts) < 4:
                    continue
                try:
                    points.append((float(parts[1]), float(parts[2]), float(parts[3])))
                except ValueError:
                    continue
        return points

    # ── Callbacks ─────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg

    def target_cb(self, msg):
        # Record COMMANDED acceleration — noise-free, matches what the
        # tracking controller is asked to execute.
        t = self.now_rel()
        with self.lock:
            self.latest_target = msg
            if not self.flight_ended:
                self.target_samples.append({
                    "t": t,
                    "x": msg.position.x, "y": msg.position.y, "z": msg.position.z,
                    "vx": msg.velocity.x, "vy": msg.velocity.y, "vz": msg.velocity.z,
                    "ax": msg.acceleration.x, "ay": msg.acceleration.y, "az": msg.acceleration.z,
                    "acc_norm": norm3((msg.acceleration.x,
                                       msg.acceleration.y,
                                       msg.acceleration.z)),
                    "yaw": msg.yaw,
                })
                a = (msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)
                self.cmd_accel_samples.append((t, a))

    def goal_cb(self, msg):
        with self.lock:
            self.goal = point_from_pose(msg.pose)

    def path_cb(self, msg):
        t = self.now_rel()
        points = path_points(msg)
        with self.lock:
            if self.flight_ended:
                return
            self.latest_path = points
            self.path_arrival_times.append(t)
            if points:
                metric = self.compute_path_metric(t, points)
                self.path_metrics.append(metric)

    def plan_time_cb(self, msg):
        t = self.now_rel()
        with self.lock:
            if self.flight_ended:
                return
            v = float(msg.data)
            self.plan_time_samples_ms.append(v)
            self.plan_time_samples.append((t, v))

    def obstacle_cb(self, msg):
        boxes = []
        line_groups = {}
        for marker in msg.markers:
            if marker.type == Marker.LINE_LIST and marker.points:
                key = marker.ns if marker.ns else str(marker.id)
                line_groups.setdefault(key, []).extend(marker.points)
                if len(marker.points) >= 24:
                    boxes.extend(self.line_points_to_boxes(marker.points))
                continue
            box = self.marker_to_box(marker)
            if box is not None:
                boxes.append(box)
        for points in line_groups.values():
            boxes.extend(self.line_points_to_boxes(points))
        with self.lock:
            self.latest_obstacles = boxes

    def line_points_to_boxes(self, points):
        boxes = []
        if not points:
            return boxes
        chunk_size = 24 if len(points) >= 24 else len(points)
        for start in range(0, len(points), chunk_size):
            chunk = points[start:start + chunk_size]
            if len(chunk) < 2:
                continue
            xs = [p.x for p in chunk]
            ys = [p.y for p in chunk]
            zs = [p.z for p in chunk]
            size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            if max(size) < 1e-6:
                continue
            center = ((min(xs) + max(xs)) * 0.5,
                      (min(ys) + max(ys)) * 0.5,
                      (min(zs) + max(zs)) * 0.5)
            boxes.append((center, size))
        return boxes

    def marker_to_box(self, marker):
        if marker.type == Marker.CUBE:
            center = point_from_pose(marker.pose)
            size = (marker.scale.x, marker.scale.y, marker.scale.z)
            return center, size

        if marker.type == Marker.LINE_LIST and marker.points:
            xs = [p.x for p in marker.points]
            ys = [p.y for p in marker.points]
            zs = [p.z for p in marker.points]
            if not xs:
                return None
            center = ((min(xs) + max(xs)) * 0.5,
                      (min(ys) + max(ys)) * 0.5,
                      (min(zs) + max(zs)) * 0.5)
            size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            return center, size

        if marker.type in (Marker.SPHERE, Marker.CYLINDER):
            center = point_from_pose(marker.pose)
            size = (marker.scale.x, marker.scale.y, marker.scale.z)
            return center, size

        return None

    def compute_path_metric(self, t, points):
        clearances = []
        for p in points:
            c = self.min_clearance_locked(p)
            if c is not None:
                clearances.append(c)
        segment_lengths = [dist3(points[i], points[i - 1]) for i in range(1, len(points))]
        return {
            "t": t,
            "num_points": len(points),
            "path_length": path_length(points),
            "mean_segment_length": mean(segment_lengths),
            "min_obstacle_clearance": min(clearances) if clearances else None,
            "near_miss_points": sum(1 for c in clearances if c < self.collision_near_miss),
            "strict_collision_points": sum(1 for c in clearances if c < self.collision_strict),
        }

    def min_clearance_locked(self, point):
        if not self.latest_obstacles:
            return None
        return min(aabb_clearance(point, center, size)
                   for center, size in self.latest_obstacles)

    def update_lap_progress_locked(self, t, pos):
        if not self.lap_reference_points or self.lap_reference_length <= 1e-6:
            return

        s, nearest = project_onto_polyline(self.lap_reference_points, pos)
        if s is None or nearest is None:
            return

        self.lap_reference_s = s
        self.lap_reference_fraction = min(1.0, s / self.lap_reference_length)
        self.lap_nearest_distance = nearest
        if nearest <= self.lap_progress_max_deviation:
            if not self.lap_started:
                self.lap_started = True
                self.lap_start_time = t
                self.lap_start_reference_s = s
                self.lap_unwrapped_reference_s = s
                self.lap_progress_m = 0.0
                self.lap_progress_fraction = 0.0
                self.lap_executed_path_length_live = 0.0
                self.last_lap_odom_position_for_length = pos
                return

            unwrapped_s = unwrap_closed_progress(
                s, self.lap_unwrapped_reference_s, self.lap_reference_length)
            self.lap_unwrapped_reference_s = unwrapped_s
            relative_s = max(0.0, unwrapped_s - self.lap_start_reference_s)
            self.lap_progress_m = max(self.lap_progress_m, relative_s)
            self.lap_progress_fraction = min(
                1.0, self.lap_progress_m / self.lap_reference_length)

    def update_lap_path_length_locked(self, pos):
        if not self.lap_started:
            return
        if self.last_lap_odom_position_for_length is not None:
            step_len = dist3(pos, self.last_lap_odom_position_for_length)
            if math.isfinite(step_len):
                self.lap_executed_path_length_live += step_len
        self.last_lap_odom_position_for_length = pos

    def completion_reached_locked(self, t, pos, goal_dist):
        if self.completion_mode == "lap":
            self.update_lap_progress_locked(t, pos)
            if self.lap_progress_fraction is None:
                return False, None
            min_path_len = self.lap_min_path_fraction * self.lap_reference_length
            path_len_ok = self.lap_executed_path_length_live >= min_path_len
            if (self.lap_progress_fraction >= self.lap_finish_fraction and
                    self.lap_nearest_distance is not None and
                    self.lap_nearest_distance <= self.lap_completion_radius and
                    path_len_ok):
                return True, "lap_complete"
            return False, None

        if self.completion_mode == "goal":
            if (self.goal is not None and goal_dist is not None and
                    goal_dist <= self.goal_radius):
                return True, "goal_reached"
            return False, None

        return False, None

    def request_finish(self):
        rospy.Timer(rospy.Duration(0.05), self.finish_cb, oneshot=True)

    # ── Periodic sample ───────────────────────────────────────────────────

    def sample_cb(self, _event):
        should_finish = False
        with self.lock:
            if self.flight_ended:
                return
            if self.latest_odom is None:
                return

            t = self.now_rel()
            odom = self.latest_odom
            pos = point_from_pose(odom.pose.pose)
            vel = (odom.twist.twist.linear.x,
                   odom.twist.twist.linear.y,
                   odom.twist.twist.linear.z)

            if self.last_odom_position_for_length is not None:
                step_len = dist3(pos, self.last_odom_position_for_length)
                if math.isfinite(step_len):
                    self.executed_path_length_live += step_len
            self.last_odom_position_for_length = pos

            goal_dist = dist3(pos, self.goal) if self.goal is not None else None
            if self.completion_mode == "lap":
                self.update_lap_progress_locked(t, pos)
                self.update_lap_path_length_locked(pos)

            in_metric_window = self.completion_mode != "lap" or self.lap_started
            lap_time = None
            metric_time = t
            metric_path_length = self.executed_path_length_live
            if self.completion_mode == "lap":
                metric_path_length = self.lap_executed_path_length_live
                if self.lap_started and self.lap_start_time is not None:
                    lap_time = max(0.0, t - self.lap_start_time)
                    metric_time = lap_time

            target_error = None
            target_age = None
            if self.latest_target is not None:
                tgt = (self.latest_target.position.x,
                       self.latest_target.position.y,
                       self.latest_target.position.z)
                target_error = dist3(pos, tgt)
                if in_metric_window:
                    self.target_errors.append(target_error)
                if self.target_samples:
                    target_age = max(0.0, t - self.target_samples[-1]["t"])

            clearance = self.min_clearance_locked(pos)
            if clearance is not None and in_metric_window:
                self.clearances.append(clearance)

                # Per-tier sample counts (time-in-collision proxy).
                in_strict    = clearance < self.collision_strict
                in_near_miss = clearance < self.collision_near_miss
                in_tail      = clearance < self.collision_tail

                if in_strict:
                    self.collision_strict_samples += 1
                    if self.last_sample_time is not None:
                        self.collision_strict_time += max(0.0, metric_time - self.last_sample_time)
                if in_near_miss:
                    self.collision_near_miss_samples += 1
                if in_tail:
                    self.collision_tail_samples += 1

                # Rising-edge event counters (count of DISTINCT collisions).
                if in_strict and not self.prev_in_strict:
                    self.collision_strict_events += 1
                    self.collision_event_log.append((metric_time, "strict", clearance))
                if in_near_miss and not self.prev_in_near_miss:
                    self.collision_near_miss_events += 1
                    self.collision_event_log.append((metric_time, "near_miss", clearance))
                if in_tail and not self.prev_in_tail:
                    self.collision_tail_events += 1
                    self.collision_event_log.append((metric_time, "tail", clearance))

                self.prev_in_strict    = in_strict
                self.prev_in_near_miss = in_near_miss
                self.prev_in_tail      = in_tail

            self.odom_samples.append({
                "t": t,
                "lap_time_s": lap_time,
                "x": pos[0], "y": pos[1], "z": pos[2],
                "vx": vel[0], "vy": vel[1], "vz": vel[2],
                "speed": norm3(vel),
                "target_error": target_error,
                "target_age_s": target_age,
                "obstacle_clearance": clearance,
                "goal_distance": goal_dist,
                "executed_path_length_m": metric_path_length,
                "run_executed_path_length_m": self.executed_path_length_live,
                "lap_progress_m": self.lap_progress_m,
                "lap_progress_fraction": self.lap_progress_fraction,
                "lap_reference_s_m": self.lap_reference_s,
                "lap_reference_fraction": self.lap_reference_fraction,
                "lap_start_reference_s_m": self.lap_start_reference_s,
                "lap_nearest_distance": self.lap_nearest_distance,
            })
            if in_metric_window:
                self.last_sample_time = metric_time

            complete, reason = self.completion_reached_locked(t, pos, goal_dist)
            if complete:
                completion_elapsed = t
                if reason == "lap_complete" and self.lap_start_time is not None:
                    completion_elapsed = max(0.0, t - self.lap_start_time)
                self.success = True
                self.time_to_goal = completion_elapsed
                self.completion_time = completion_elapsed
                self.completion_wall_time = t
                self.completion_reason = reason
                self.flight_ended = True
                if reason == "lap_complete":
                    self.completed_path_length = self.lap_executed_path_length_live
                else:
                    self.completed_path_length = path_length(
                        [(s["x"], s["y"], s["z"]) for s in self.odom_samples])
                should_finish = self.shutdown_on_success and not self.finish_requested
                self.finish_requested = self.finish_requested or should_finish
                rospy.loginfo(
                    "[eval] %s at t=%.2fs (lap time %.2fs); freezing sampling.",
                    reason, t, completion_elapsed)

        if should_finish:
            self.request_finish()

    def finish_cb(self, _event):
        if self.outputs_written:
            return
        self.outputs_written = True
        self.write_outputs()
        rospy.signal_shutdown("evaluation finished")

    # ── Summary ───────────────────────────────────────────────────────────

    def summarize(self):
        positions = [(s["x"], s["y"], s["z"]) for s in self.odom_samples]
        velocities = [(s["t"], (s["vx"], s["vy"], s["vz"])) for s in self.odom_samples]
        speeds = [s["speed"] for s in self.odom_samples]

        # Smoothness from COMMANDED target acceleration (preferred — no odom noise).
        cmd_accel_norms = [norm3(a) for _t, a in self.cmd_accel_samples]
        cmd_jerk_norms = finite_diff_norms(self.cmd_accel_samples)

        # Fallback smoothness from odom (kept as a secondary, noisy reference).
        odom_accel_samples = []
        for i in range(1, len(velocities)):
            dt = velocities[i][0] - velocities[i - 1][0]
            if dt > 1e-6:
                dv = sub3(velocities[i][1], velocities[i - 1][1])
                odom_accel_samples.append(
                    (velocities[i][0], (dv[0] / dt, dv[1] / dt, dv[2] / dt)))
        odom_accel_norms = [norm3(a) for _t, a in odom_accel_samples]
        odom_jerk_norms = finite_diff_norms(odom_accel_samples)

        final_goal_distance = None
        if self.goal is not None and self.odom_samples:
            final_goal_distance = self.odom_samples[-1]["goal_distance"]

        plan_periods = [self.path_arrival_times[i] - self.path_arrival_times[i - 1]
                        for i in range(1, len(self.path_arrival_times))]

        path_clearances = [m["min_obstacle_clearance"] for m in self.path_metrics
                           if m["min_obstacle_clearance"] is not None]
        path_lengths = [m["path_length"] for m in self.path_metrics]

        # Empirical CVaR over the worst α fraction of the clearance series —
        # directly comparable to the planner's CVaR objective.
        cvar_05 = empirical_cvar(self.clearances, 0.05, worst="low")
        cvar_10 = empirical_cvar(self.clearances, 0.10, worst="low")
        cvar_user = empirical_cvar(self.clearances, self.cvar_alpha, worst="low")

        # Planner latency (P95 / max are the meaningful numbers for real-time).
        plan_latency_mean = mean(self.plan_time_samples_ms)
        plan_latency_p95 = percentile(self.plan_time_samples_ms, 0.95)
        plan_latency_max = max(self.plan_time_samples_ms) if self.plan_time_samples_ms else None
        executed_path_length = path_length(positions)
        flight_duration = self.odom_samples[-1]["t"] if self.odom_samples else None
        completed_path_length = self.completed_path_length
        if completed_path_length is None and self.success:
            if self.completion_mode == "lap":
                completed_path_length = self.lap_executed_path_length_live
            else:
                completed_path_length = executed_path_length

        summary = {
            "algorithm": self.algorithm,
            "topics": {
                "odom": self.odom_topic,
                "target": self.target_topic,
                "planned_path": self.path_topic,
                "obstacles": self.obstacle_topic,
                "goal": self.goal_topic,
                "plan_time": self.plan_time_topic,
            },
            "samples": {
                "odom": len(self.odom_samples),
                "target": len(self.target_samples),
                "cmd_accel": len(self.cmd_accel_samples),
                "planned_paths": len(self.path_metrics),
                "obstacle_clearance": len(self.clearances),
                "plan_time_msgs": len(self.plan_time_samples_ms),
            },
            "task": {
                "success": self.success,
                "completion_mode_requested": self.requested_completion_mode,
                "completion_mode": self.completion_mode,
                "completion_reason": self.completion_reason,
                "goal_radius": self.goal_radius,
                "time_to_goal_s": self.time_to_goal,
                "completion_time_s": self.completion_time,
                "completion_wall_time_s": self.completion_wall_time,
                "mission_time_s": self.completion_time if self.success else None,
                "final_goal_distance_m": final_goal_distance,
                "completed_path_length_m": completed_path_length,
                "executed_path_length_m": executed_path_length,
                "lap_executed_path_length_m": self.lap_executed_path_length_live,
                "flight_duration_s": flight_duration,
                "lap_reference_path": self.lap_reference_path,
                "lap_reference_length_m": self.lap_reference_length,
                "lap_completion_radius_m": self.lap_completion_radius,
                "lap_finish_fraction": self.lap_finish_fraction,
                "lap_min_path_fraction": self.lap_min_path_fraction,
                "lap_min_path_length_m": self.lap_min_path_fraction * self.lap_reference_length,
                "lap_started": self.lap_started,
                "lap_start_time_s": self.lap_start_time,
                "lap_progress_m": self.lap_progress_m,
                "lap_progress_fraction": self.lap_progress_fraction,
                "lap_reference_s_m": self.lap_reference_s,
                "lap_reference_fraction": self.lap_reference_fraction,
                "lap_start_reference_s_m": self.lap_start_reference_s,
                "lap_unwrapped_reference_s_m": self.lap_unwrapped_reference_s,
                "lap_nearest_distance_m": self.lap_nearest_distance,
            },
            "safety": {
                # Raw clearance stats
                "min_clearance_m": min(self.clearances) if self.clearances else None,
                "mean_clearance_m": mean(self.clearances),
                "p05_clearance_m": percentile(self.clearances, 0.05),
                # Tiered collision metrics
                "collision_strict_threshold_m":    self.collision_strict,
                "collision_near_miss_threshold_m": self.collision_near_miss,
                "collision_tail_threshold_m":      self.collision_tail,
                "collision_strict_samples":        self.collision_strict_samples,
                "collision_near_miss_samples":     self.collision_near_miss_samples,
                "collision_tail_samples":          self.collision_tail_samples,
                "collision_strict_rate":           (self.collision_strict_samples /
                                                    len(self.clearances)) if self.clearances else None,
                "collision_near_miss_rate":        (self.collision_near_miss_samples /
                                                    len(self.clearances)) if self.clearances else None,
                "collision_tail_rate":             (self.collision_tail_samples /
                                                    len(self.clearances)) if self.clearances else None,
                "collision_strict_time_s":         self.collision_strict_time,
                # ── EVENT counters (rising-edge, count of distinct collisions) ──
                "collision_strict_events":         self.collision_strict_events,
                "collision_near_miss_events":      self.collision_near_miss_events,
                "collision_tail_events":           self.collision_tail_events,
                # Empirical CVaR
                "cvar_alpha":               self.cvar_alpha,
                "empirical_cvar_5pct_m":    cvar_05,
                "empirical_cvar_10pct_m":   cvar_10,
                "empirical_cvar_user_m":    cvar_user,
                # Planned-trajectory safety (offline check of planner's intent)
                "planned_min_clearance_m":      min(path_clearances) if path_clearances else None,
                "planned_near_miss_points":     sum(m["near_miss_points"] for m in self.path_metrics),
                "planned_strict_collision_points": sum(m["strict_collision_points"] for m in self.path_metrics),
            },
            "tracking": {
                "mean_target_error_m": mean(self.target_errors),
                "rms_target_error_m":  rms(self.target_errors),
                "max_target_error_m":  max(self.target_errors) if self.target_errors else None,
            },
            "smoothness": {
                # Speed
                "mean_speed_mps": mean(speeds),
                "max_speed_mps":  max(speeds) if speeds else None,
                # Acceleration / jerk from COMMANDED target (preferred)
                "cmd_mean_accel_mps2": mean(cmd_accel_norms),
                "cmd_max_accel_mps2":  max(cmd_accel_norms) if cmd_accel_norms else None,
                "cmd_rms_accel_mps2":  rms(cmd_accel_norms),
                "cmd_mean_jerk_mps3":  mean(cmd_jerk_norms),
                "cmd_max_jerk_mps3":   max(cmd_jerk_norms) if cmd_jerk_norms else None,
                "cmd_rms_jerk_mps3":   rms(cmd_jerk_norms),
                # Acceleration / jerk from odom finite-difference (noisy reference)
                "odom_rms_accel_mps2": rms(odom_accel_norms),
                "odom_rms_jerk_mps3":  rms(odom_jerk_norms),
            },
            "planner": {
                # Output cadence (publish rate of planned path)
                "publish_rate_hz":         (1.0 / mean(plan_periods))
                                            if plan_periods and mean(plan_periods) else None,
                "mean_publish_period_s":   mean(plan_periods),
                "max_publish_period_s":    max(plan_periods) if plan_periods else None,
                # Actual planning latency (from /im2mppi/plan_time_ms)
                "plan_latency_mean_ms":    plan_latency_mean,
                "plan_latency_p95_ms":     plan_latency_p95,
                "plan_latency_max_ms":     plan_latency_max,
                # Planned path geometry
                "mean_planned_path_length_m": mean(path_lengths),
                "max_planned_path_length_m":  max(path_lengths) if path_lengths else None,
            },
        }
        return summary

    def build_diagnostics(self, summary):
        """Return a diagnosis dictionary plus timestamped events for debugging fly-away runs."""
        events = []

        def add_event(t, kind, value, threshold, detail):
            events.append({
                "t": t,
                "kind": kind,
                "value": value,
                "threshold": threshold,
                "detail": detail,
            })

        # Planner latency spikes.
        plan_vals = [v for _t, v in self.plan_time_samples]
        for t, v in self.plan_time_samples:
            if v > 100.0:
                add_event(t, "plan_latency_gt_100ms", v, 100.0, "planning over one 10Hz period")
            elif v > 50.0:
                add_event(t, "plan_latency_gt_50ms", v, 50.0, "large planning spike")
            elif v > 20.0:
                add_event(t, "plan_latency_gt_20ms", v, 20.0, "planning spike")

        # Target stream cadence from /autonomous_flight/target_state.
        target_gaps = []
        target_jumps = []
        for i in range(1, len(self.target_samples)):
            prev = self.target_samples[i - 1]
            cur = self.target_samples[i]
            gap = cur["t"] - prev["t"]
            target_gaps.append(gap)
            if gap > 0.10:
                add_event(cur["t"], "target_gap_gt_100ms", gap, 0.10, "target stream stalled")
            elif gap > 0.05:
                add_event(cur["t"], "target_gap_gt_50ms", gap, 0.05, "target stream gap")
            elif gap > 0.02:
                add_event(cur["t"], "target_gap_gt_20ms", gap, 0.02, "target stream jitter")

            jump = dist3((cur["x"], cur["y"], cur["z"]),
                         (prev["x"], prev["y"], prev["z"]))
            target_jumps.append(jump)
            if jump > 1.0:
                add_event(cur["t"], "target_jump_gt_1m", jump, 1.0, "trajectory switch discontinuity")
            elif jump > 0.3:
                add_event(cur["t"], "target_jump_gt_0p3m", jump, 0.3, "target position jump")

        # Tracking divergence and command spikes.
        last_odom_t = self.odom_samples[-1]["t"] if self.odom_samples else None
        last_target_t = self.target_samples[-1]["t"] if self.target_samples else None
        last_plan_t = self.plan_time_samples[-1][0] if self.plan_time_samples else None
        target_stopped_age = None
        plan_stopped_age = None
        target_stream_absent = last_odom_t is not None and not self.target_samples
        plan_time_absent = last_odom_t is not None and not self.plan_time_samples
        if target_stream_absent:
            add_event(last_odom_t, "target_stream_absent",
                      len(self.target_samples), 1,
                      "odom continued but no target_state messages were recorded; navigation node likely died before publishing")
        if plan_time_absent:
            add_event(last_odom_t, "plan_time_absent",
                      len(self.plan_time_samples), 1,
                      "odom continued but no plan_time messages were recorded; planner loop likely never ran or node died")
        if last_odom_t is not None and last_target_t is not None:
            target_stopped_age = last_odom_t - last_target_t
            if target_stopped_age > 1.0:
                add_event(last_target_t, "target_stream_stopped_before_eval_end",
                          target_stopped_age, 1.0,
                          "no target_state messages near the end; navigation node may have died")
        if last_odom_t is not None and last_plan_t is not None:
            plan_stopped_age = last_odom_t - last_plan_t
            if plan_stopped_age > 1.0:
                add_event(last_plan_t, "plan_time_stopped_before_eval_end",
                          plan_stopped_age, 1.0,
                          "no plan_time messages near the end; planner loop may have died")

        first_err_gt_2 = None
        first_err_gt_5 = None
        for row in self.odom_samples:
            err = row.get("target_error")
            if err is None:
                continue
            if err > 2.0 and first_err_gt_2 is None:
                first_err_gt_2 = row["t"]
                add_event(row["t"], "tracking_error_gt_2m", err, 2.0, "tracking has diverged")
            if err > 5.0 and first_err_gt_5 is None:
                first_err_gt_5 = row["t"]
                add_event(row["t"], "tracking_error_gt_5m", err, 5.0, "fly-away likely")

        cmd_accel_norms = [norm3(a) for _t, a in self.cmd_accel_samples]
        cmd_jerk_norms = finite_diff_norms(self.cmd_accel_samples)
        for t, a in self.cmd_accel_samples:
            an = norm3(a)
            if an > 6.0:
                add_event(t, "cmd_accel_gt_6mps2", an, 6.0, "aggressive commanded acceleration")
            elif an > 3.5:
                add_event(t, "cmd_accel_gt_3p5mps2", an, 3.5, "above nominal planner a_max")

        verdict = "no_clear_single_cause"
        if plan_vals and max(plan_vals) > 100.0:
            verdict = "planning_latency_spike"
        if target_gaps and max(target_gaps) > 0.10:
            verdict = "target_stream_stall"
        if target_jumps and max(target_jumps) > 1.0:
            verdict = "target_switch_discontinuity"
        if ((target_stopped_age is not None and target_stopped_age > 1.0) or
                (plan_stopped_age is not None and plan_stopped_age > 1.0) or
                target_stream_absent or plan_time_absent):
            verdict = "navigation_node_stopped_or_crashed"
        if first_err_gt_5 is not None and verdict == "no_clear_single_cause":
            verdict = "tracking_diverged_without_obvious_timing_spike"

        task = summary["task"]
        lap_path_length = task.get("lap_executed_path_length_m")
        if lap_path_length is None:
            lap_path_length = task.get("completed_path_length_m")
        if lap_path_length is None:
            lap_path_length = task.get("executed_path_length_m")
        if (not task["success"] and
                task["lap_progress_fraction"] is not None and
                task["lap_progress_fraction"] >= task["lap_finish_fraction"] and
                lap_path_length is not None and
                lap_path_length < task["lap_min_path_length_m"]):
            add_event(last_odom_t, "lap_completion_blocked_by_short_path",
                      lap_path_length, task["lap_min_path_length_m"],
                      "reference progress reached the finish band but actual path length was too short for one lap")

        diagnostics = {
            "verdict": verdict,
            "plan_latency": {
                "count": len(plan_vals),
                "mean_ms": mean(plan_vals),
                "p95_ms": percentile(plan_vals, 0.95),
                "p99_ms": percentile(plan_vals, 0.99),
                "max_ms": max(plan_vals) if plan_vals else None,
                "count_gt_20ms": sum(1 for v in plan_vals if v > 20.0),
                "count_gt_50ms": sum(1 for v in plan_vals if v > 50.0),
                "count_gt_100ms": sum(1 for v in plan_vals if v > 100.0),
            },
            "target_stream": {
                "count": len(self.target_samples),
                "mean_gap_s": mean(target_gaps),
                "p95_gap_s": percentile(target_gaps, 0.95),
                "max_gap_s": max(target_gaps) if target_gaps else None,
                "count_gap_gt_20ms": sum(1 for v in target_gaps if v > 0.02),
                "count_gap_gt_50ms": sum(1 for v in target_gaps if v > 0.05),
                "count_gap_gt_100ms": sum(1 for v in target_gaps if v > 0.10),
            },
            "target_command": {
                "max_position_jump_m": max(target_jumps) if target_jumps else None,
                "p95_position_jump_m": percentile(target_jumps, 0.95),
                "count_position_jump_gt_0p3m": sum(1 for v in target_jumps if v > 0.3),
                "count_position_jump_gt_1m": sum(1 for v in target_jumps if v > 1.0),
                "max_cmd_accel_mps2": max(cmd_accel_norms) if cmd_accel_norms else None,
                "max_cmd_jerk_mps3": max(cmd_jerk_norms) if cmd_jerk_norms else None,
            },
            "tracking": {
                "max_target_error_m": summary["tracking"]["max_target_error_m"],
                "first_error_gt_2m_s": first_err_gt_2,
                "first_error_gt_5m_s": first_err_gt_5,
            },
            "end_of_run": {
                "last_odom_t_s": last_odom_t,
                "last_target_t_s": last_target_t,
                "last_plan_time_t_s": last_plan_t,
                "target_stream_absent": target_stream_absent,
                "plan_time_absent": plan_time_absent,
                "target_stopped_age_s": target_stopped_age,
                "plan_time_stopped_age_s": plan_stopped_age,
            },
            "num_events": len(events),
        }
        events.sort(key=lambda e: e["t"])
        return diagnostics, events

    def write_outputs(self):
        with self.lock:
            summary = self.summarize()
            diagnostics, diagnostic_events = self.build_diagnostics(summary)
            summary["diagnostics"] = diagnostics
            prefix = os.path.join(self.output_dir, self.algorithm)

            with open(prefix + "_summary.json", "w") as f:
                json.dump(summary, f, indent=2, sort_keys=True)

            with open(prefix + "_timeseries.csv", "w", newline="") as f:
                fields = ["t", "lap_time_s",
                          "x", "y", "z", "vx", "vy", "vz", "speed",
                          "target_error", "target_age_s",
                          "obstacle_clearance", "goal_distance",
                          "executed_path_length_m", "run_executed_path_length_m",
                          "lap_progress_m", "lap_progress_fraction",
                          "lap_reference_s_m", "lap_reference_fraction",
                          "lap_start_reference_s_m",
                          "lap_nearest_distance"]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in self.odom_samples:
                    writer.writerow(row)

            with open(prefix + "_target_state.csv", "w", newline="") as f:
                fields = ["t", "x", "y", "z", "vx", "vy", "vz",
                          "ax", "ay", "az", "acc_norm", "yaw"]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in self.target_samples:
                    writer.writerow(row)

            with open(prefix + "_path_metrics.csv", "w", newline="") as f:
                fields = ["t", "num_points", "path_length", "mean_segment_length",
                          "min_obstacle_clearance",
                          "near_miss_points", "strict_collision_points"]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in self.path_metrics:
                    writer.writerow(row)

            with open(prefix + "_plan_time.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["plan_time_ms"])
                for v in self.plan_time_samples_ms:
                    writer.writerow([v])

            with open(prefix + "_plan_time_timeline.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t", "plan_time_ms"])
                for t, v in self.plan_time_samples:
                    writer.writerow([t, v])

            # Raw commanded acceleration (from tracking_controller::Target),
            # the noise-free signal jerk metrics are computed from.
            with open(prefix + "_cmd_accel.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t", "ax", "ay", "az"])
                for (t, a) in self.cmd_accel_samples:
                    writer.writerow([t, a[0], a[1], a[2]])

            # Per-event log of every distinct rising-edge collision entry.
            with open(prefix + "_collision_events.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t", "tier", "clearance_at_entry"])
                for (t, tier, clr) in self.collision_event_log:
                    writer.writerow([t, tier, clr])

            with open(prefix + "_diagnostics.json", "w") as f:
                json.dump(diagnostics, f, indent=2, sort_keys=True)

            with open(prefix + "_diagnostic_events.csv", "w", newline="") as f:
                fields = ["t", "kind", "value", "threshold", "detail"]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in diagnostic_events:
                    writer.writerow(row)

            rospy.loginfo("[eval] Wrote summary to %s_summary.json", prefix)
            rospy.loginfo(
                "[eval] CR_strict=%d  CR_near=%d  CR_tail=%d  "
                "min_clr=%.3f  cvar5=%s  p95_lat=%s ms  diag=%s",
                summary["safety"]["collision_strict_events"],
                summary["safety"]["collision_near_miss_events"],
                summary["safety"]["collision_tail_events"],
                summary["safety"]["min_clearance_m"] or float("nan"),
                summary["safety"]["empirical_cvar_5pct_m"],
                summary["planner"]["plan_latency_p95_ms"],
                diagnostics["verdict"])


if __name__ == "__main__":
    rospy.init_node("planner_evaluator")
    Evaluator()
    rospy.spin()
