#!/usr/bin/env python3

import copy

import rospy
from visualization_msgs.msg import Marker, MarkerArray


class VisibleDynamicBBoxFilter:
    def __init__(self):
        input_topic = rospy.get_param(
            "~input_topic", "/onboard_detector/GT_obstacle_bbox"
        )
        output_topic = rospy.get_param(
            "~output_topic", "/onboard_detector/visible_dynamic_bboxes"
        )
        self.red_min = float(rospy.get_param("~red_min", 0.8))
        self.green_max = float(rospy.get_param("~green_max", 0.2))
        self.blue_max = float(rospy.get_param("~blue_max", 0.2))

        self.publisher = rospy.Publisher(output_topic, MarkerArray, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, MarkerArray, self._callback, queue_size=1
        )

        rospy.loginfo(
            "[visible bbox filter] %s -> %s (preserving original bbox markers)",
            input_topic,
            output_topic,
        )

    def _is_visible_bbox(self, marker):
        return (
            marker.action == Marker.ADD
            and marker.type == Marker.LINE_LIST
            and marker.color.r >= self.red_min
            and marker.color.g <= self.green_max
            and marker.color.b <= self.blue_max
        )

    def _callback(self, message):
        output = MarkerArray()
        for marker in message.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                output.markers.append(copy.deepcopy(marker))
            elif self._is_visible_bbox(marker):
                output.markers.append(copy.deepcopy(marker))

        self.publisher.publish(output)
        rospy.logdebug_throttle(
            2.0,
            "[visible bbox filter] publishing %d of %d markers",
            len(output.markers),
            len(message.markers),
        )


if __name__ == "__main__":
    rospy.init_node("visible_dynamic_bbox_filter")
    VisibleDynamicBBoxFilter()
    rospy.spin()
