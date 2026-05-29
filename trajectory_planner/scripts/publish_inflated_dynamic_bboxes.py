#!/usr/bin/env python3
"""Republish Gazebo GT obstacle boxes with visualization-only inflation."""

from __future__ import annotations

from typing import Dict, Iterable, List, Tuple

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def parse_vec3_param(name: str, default: Iterable[float]) -> List[float]:
    value = rospy.get_param(name, list(default))
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        rospy.logwarn("Invalid %s=%r; using %s", name, value, list(default))
        return list(default)
    return [float(value[0]), float(value[1]), float(value[2])]


def point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


def bbox_edges(min_x: float, max_x: float, min_y: float, max_y: float, min_z: float, max_z: float) -> List[Point]:
    p1 = point(max_x, max_y, max_z)
    p2 = point(min_x, max_y, max_z)
    p3 = point(max_x, min_y, max_z)
    p4 = point(min_x, min_y, max_z)
    p5 = point(max_x, max_y, min_z)
    p6 = point(min_x, max_y, min_z)
    p7 = point(max_x, min_y, min_z)
    p8 = point(min_x, min_y, min_z)
    edges = [
        (p1, p2), (p1, p3), (p2, p4), (p3, p4),
        (p1, p5), (p2, p6), (p3, p7), (p4, p8),
        (p5, p6), (p5, p7), (p6, p8), (p7, p8),
    ]
    pts: List[Point] = []
    for a, b in edges:
        pts.extend([a, b])
    return pts


class InflatedBBoxPublisher:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param("~input_topic", "/onboard_detector/GT_obstacle_bbox")
        self.output_topic = rospy.get_param("~output_topic", "/onboard_detector/GT_obstacle_bbox_inflated")
        self.frame_fallback = rospy.get_param("~frame_fallback", "map")
        self.namespace = rospy.get_param("~namespace", "inflated_gt_obstacles")

        default_robot_size = rospy.get_param("/dynamic_map/robot_size", [0.5, 0.5, 0.3])
        self.add_size = parse_vec3_param("~add_size", default_robot_size)
        self.extra_margin_xy = float(rospy.get_param("~extra_margin_xy", 0.0))
        self.extra_margin_z = float(rospy.get_param("~extra_margin_z", 0.0))
        self.line_width = float(rospy.get_param("~line_width", 0.08))
        self.lifetime = float(rospy.get_param("~lifetime", 0.3))

        self.color_r = float(rospy.get_param("~color_r", 1.0))
        self.color_g = float(rospy.get_param("~color_g", 0.58))
        self.color_b = float(rospy.get_param("~color_b", 0.05))
        self.color_a = float(rospy.get_param("~color_a", 0.9))

        self.publisher = rospy.Publisher(self.output_topic, MarkerArray, queue_size=2)
        self.subscriber = rospy.Subscriber(self.input_topic, MarkerArray, self.callback, queue_size=2)

        rospy.loginfo(
            "Inflated dynamic bbox visualizer: %s -> %s, add_size=%s, extra_margin_xy=%.3f",
            self.input_topic,
            self.output_topic,
            self.add_size,
            self.extra_margin_xy,
        )

    def callback(self, msg: MarkerArray) -> None:
        groups: Dict[Tuple[str, str], List[Point]] = {}
        for marker in msg.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            if marker.type != Marker.LINE_LIST or not marker.points:
                continue
            frame_id = marker.header.frame_id or self.frame_fallback
            groups.setdefault((frame_id, marker.ns), []).extend(marker.points)

        output = MarkerArray()
        for marker_id, ((frame_id, _source_ns), points) in enumerate(sorted(groups.items())):
            xs = [p.x for p in points]
            ys = [p.y for p in points]
            zs = [p.z for p in points]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            min_z, max_z = min(zs), max(zs)

            center_x = 0.5 * (min_x + max_x)
            center_y = 0.5 * (min_y + max_y)
            center_z = 0.5 * (min_z + max_z)

            size_x = (max_x - min_x) + self.add_size[0] + 2.0 * self.extra_margin_xy
            size_y = (max_y - min_y) + self.add_size[1] + 2.0 * self.extra_margin_xy
            size_z = (max_z - min_z) + self.add_size[2] + 2.0 * self.extra_margin_z

            inflated = Marker()
            inflated.header.frame_id = frame_id
            inflated.header.stamp = rospy.Time.now()
            inflated.ns = self.namespace
            inflated.id = marker_id
            inflated.type = Marker.LINE_LIST
            inflated.action = Marker.ADD
            inflated.lifetime = rospy.Duration(self.lifetime)
            inflated.scale.x = self.line_width
            inflated.color.r = self.color_r
            inflated.color.g = self.color_g
            inflated.color.b = self.color_b
            inflated.color.a = self.color_a
            inflated.points = bbox_edges(
                center_x - 0.5 * size_x,
                center_x + 0.5 * size_x,
                center_y - 0.5 * size_y,
                center_y + 0.5 * size_y,
                center_z - 0.5 * size_z,
                center_z + 0.5 * size_z,
            )
            output.markers.append(inflated)

        self.publisher.publish(output)


def main() -> None:
    rospy.init_node("inflated_dynamic_bbox_visualizer", anonymous=False)
    InflatedBBoxPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
