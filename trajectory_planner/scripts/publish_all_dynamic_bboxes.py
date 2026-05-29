#!/usr/bin/env python3
"""Publish all Gazebo dynamic obstacles using the detector bbox RViz style.

This node is visualization-only.  It reads Gazebo ground-truth model states and
republishes matching dynamic models as a MarkerArray.  By default the output is
``/onboard_detector/dynamic_bboxes`` so the existing RViz "Dynamic Obstacles"
display can show every dynamic obstacle, not only those inside the onboard
detector's local sensing range.
"""

from __future__ import annotations

import re
from typing import Iterable, List, Sequence

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


FLOAT_RE = re.compile(r"[-+]?(?:\d+\.\d+|\d+)")


def parse_vec3_param(name: str, default: Iterable[float]) -> List[float]:
    value = rospy.get_param(name, list(default))
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        rospy.logwarn("Invalid %s=%r; using %s", name, value, list(default))
        return list(default)
    return [float(value[0]), float(value[1]), float(value[2])]


def parse_string_list_param(name: str, default: Sequence[str]) -> List[str]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        return [part.strip() for part in value.split(",") if part.strip()]
    if isinstance(value, (list, tuple)):
        return [str(part).strip() for part in value if str(part).strip()]
    rospy.logwarn("Invalid %s=%r; using %s", name, value, list(default))
    return list(default)


def model_size_from_name(name: str, default_size: Sequence[float]) -> List[float]:
    values = [float(match.group(0)) for match in FLOAT_RE.finditer(name)]
    if len(values) >= 3:
        return values[-3:]
    return [float(default_size[0]), float(default_size[1]), float(default_size[2])]


def point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


def bbox_edges(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> List[Point]:
    min_x, max_x = cx - 0.5 * sx, cx + 0.5 * sx
    min_y, max_y = cy - 0.5 * sy, cy + 0.5 * sy
    min_z, max_z = cz - 0.5 * sz, cz + 0.5 * sz
    verts = [
        point(min_x, min_y, min_z),
        point(min_x, max_y, min_z),
        point(max_x, max_y, min_z),
        point(max_x, min_y, min_z),
        point(min_x, min_y, max_z),
        point(min_x, max_y, max_z),
        point(max_x, max_y, max_z),
        point(max_x, min_y, max_z),
    ]
    edge_indices = [
        (0, 1), (1, 2), (2, 3), (0, 3),
        (0, 4), (1, 5), (3, 7), (2, 6),
        (4, 5), (5, 6), (4, 7), (6, 7),
    ]
    pts: List[Point] = []
    for a, b in edge_indices:
        pts.extend([verts[a], verts[b]])
    return pts


class AllDynamicBBoxes:
    def __init__(self) -> None:
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.output_topic = rospy.get_param("~output_topic", "/onboard_detector/dynamic_bboxes")
        default_prefixes = rospy.get_param("/target_obstacle", ["dynamic_cylinder", "dynamic_box", "person"])
        self.target_prefixes = parse_string_list_param("~target_prefixes", default_prefixes)
        self.default_size = parse_vec3_param("~default_size", [0.5, 0.5, 1.7])
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.namespace = rospy.get_param("~namespace", "box3D")
        self.line_width = float(rospy.get_param("~line_width", 0.06))
        self.lifetime = float(rospy.get_param("~lifetime", 0.15))
        self.color_r = float(rospy.get_param("~color_r", 0.0))
        self.color_g = float(rospy.get_param("~color_g", 0.0))
        self.color_b = float(rospy.get_param("~color_b", 1.0))
        self.color_a = float(rospy.get_param("~color_a", 1.0))
        self.person_center_z = float(rospy.get_param("~person_center_z", 0.9))
        self.max_marker_count = 0

        self.publisher = rospy.Publisher(self.output_topic, MarkerArray, queue_size=2)
        self.subscriber = rospy.Subscriber(
            self.model_states_topic,
            ModelStates,
            self.callback,
            queue_size=1,
        )

        rospy.loginfo(
            "All dynamic bbox visualizer: %s -> %s, prefixes=%s, style=rgb(%.2f, %.2f, %.2f)",
            self.model_states_topic,
            self.output_topic,
            self.target_prefixes,
            self.color_r,
            self.color_g,
            self.color_b,
        )

    def is_target_model(self, name: str) -> bool:
        return any(name.startswith(prefix) for prefix in self.target_prefixes)

    def callback(self, msg: ModelStates) -> None:
        now = rospy.Time.now()
        markers: List[Marker] = []

        for model_idx, name in enumerate(msg.name):
            if not self.is_target_model(name):
                continue
            if model_idx >= len(msg.pose):
                continue

            pose = msg.pose[model_idx]
            sx, sy, sz = model_size_from_name(name, self.default_size)
            cx = pose.position.x
            cy = pose.position.y
            cz = pose.position.z + self.person_center_z if name.startswith("person") else pose.position.z

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = self.namespace
            marker.id = len(markers)
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.line_width
            marker.color.r = self.color_r
            marker.color.g = self.color_g
            marker.color.b = self.color_b
            marker.color.a = self.color_a
            marker.lifetime = rospy.Duration(self.lifetime)
            marker.text = name
            marker.points = bbox_edges(cx, cy, cz, sx, sy, sz)
            markers.append(marker)

        output = MarkerArray()
        output.markers.extend(markers)

        for marker_id in range(len(markers), self.max_marker_count):
            delete = Marker()
            delete.header.frame_id = self.frame_id
            delete.header.stamp = now
            delete.ns = self.namespace
            delete.id = marker_id
            delete.action = Marker.DELETE
            output.markers.append(delete)

        self.max_marker_count = max(self.max_marker_count, len(markers))
        self.publisher.publish(output)


def main() -> None:
    rospy.init_node("all_dynamic_bboxes_visualizer", anonymous=False)
    AllDynamicBBoxes()
    rospy.spin()


if __name__ == "__main__":
    main()
