#!/usr/bin/env python3
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class ColorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_perception_node")

        self.declare_parameter("color_topic", "/overhead_camera/overhead_rgb/image_raw")
        self.declare_parameter("centroid_topic", "/detected_object_centroid")
        self.declare_parameter("min_area", 150.0)
        self.declare_parameter("red_hsv_lower1", [0, 120, 70])
        self.declare_parameter("red_hsv_upper1", [10, 255, 255])
        self.declare_parameter("red_hsv_lower2", [170, 120, 70])
        self.declare_parameter("red_hsv_upper2", [180, 255, 255])

        self.bridge = CvBridge()
        centroid_topic = self.get_parameter("centroid_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(PointStamped, centroid_topic, 10)

        color_topic = self.get_parameter("color_topic").get_parameter_value().string_value
        self.create_subscription(Image, color_topic, self.handle_color_image, 10)

        self.get_logger().info("Color detector node started.")

    def handle_color_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert color image: {exc}")
            return

        centroid = self.find_target_centroid(bgr)
        if centroid is None:
            return

        cx, cy = centroid
        output = PointStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = msg.header.frame_id
        output.point.x = float(cx)
        output.point.y = float(cy)
        output.point.z = 0.0
        self.publisher.publish(output)

    def find_target_centroid(self, bgr: np.ndarray) -> Optional[Tuple[int, int]]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower1 = np.array(
            self.get_parameter("red_hsv_lower1").get_parameter_value().integer_array_value, dtype=np.uint8
        )
        upper1 = np.array(
            self.get_parameter("red_hsv_upper1").get_parameter_value().integer_array_value, dtype=np.uint8
        )
        lower2 = np.array(
            self.get_parameter("red_hsv_lower2").get_parameter_value().integer_array_value, dtype=np.uint8
        )
        upper2 = np.array(
            self.get_parameter("red_hsv_upper2").get_parameter_value().integer_array_value, dtype=np.uint8
        )

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.medianBlur(mask, 5)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        min_area = float(self.get_parameter("min_area").get_parameter_value().double_value)
        if area < min_area:
            return None

        moments = cv2.moments(largest)
        if moments["m00"] == 0:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return cx, cy


def main() -> None:
    rclpy.init()
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
