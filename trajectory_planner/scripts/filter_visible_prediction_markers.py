#!/usr/bin/env python3

import copy
import math

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class VisiblePredictionMarkerFilter:
    def __init__(self):
        input_topic = rospy.get_param(
            "~input_topic", "/im2mppi/dynamic_obstacle_predictions"
        )
        output_topic = rospy.get_param(
            "~output_topic", "/im2mppi/visible_dynamic_obstacle_predictions"
        )
        odom_topic = rospy.get_param(
            "~odom_topic", "/CERLAB/quadcopter/odom"
        )
        self.fov = float(rospy.get_param("~fov", math.pi))
        self.max_distance = float(rospy.get_param("~max_distance", 4.0))
        self.robot_pose = None

        self.publisher = rospy.Publisher(output_topic, MarkerArray, queue_size=1)
        self.odom_subscriber = rospy.Subscriber(
            odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        self.marker_subscriber = rospy.Subscriber(
            input_topic, MarkerArray, self._marker_callback, queue_size=1
        )

        rospy.loginfo(
            "[visible prediction filter] %s -> %s, fov=%.1f deg, range=%.2f m",
            input_topic,
            output_topic,
            math.degrees(self.fov),
            self.max_distance,
        )

    def _odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        self.robot_pose = (
            position.x,
            position.y,
            math.atan2(sin_yaw, cos_yaw),
        )

    def _point_is_visible(self, x, y):
        if self.robot_pose is None:
            return False

        robot_x, robot_y, robot_yaw = self.robot_pose
        dx = x - robot_x
        dy = y - robot_y
        if math.hypot(dx, dy) > self.max_distance:
            return False

        bearing = math.atan2(dy, dx)
        return abs(normalize_angle(bearing - robot_yaw)) <= self.fov * 0.5

    def _marker_callback(self, message):
        output = MarkerArray()
        current_mode_visible = False

        for marker in message.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                output.markers.append(copy.deepcopy(marker))
                continue

            if marker.type == Marker.LINE_STRIP:
                if not marker.points:
                    current_mode_visible = False
                else:
                    anchor = marker.points[0]
                    current_mode_visible = self._point_is_visible(
                        anchor.x, anchor.y
                    )
            elif marker.type != Marker.CUBE:
                current_mode_visible = False

            if current_mode_visible:
                output.markers.append(copy.deepcopy(marker))

        self.publisher.publish(output)
        rospy.logdebug_throttle(
            2.0,
            "[visible prediction filter] publishing %d of %d markers",
            len(output.markers),
            len(message.markers),
        )


if __name__ == "__main__":
    rospy.init_node("visible_prediction_marker_filter")
    VisiblePredictionMarkerFilter()
    rospy.spin()
