#!/usr/bin/env python3
from typing import Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class PoseEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_pose_estimator")

        self.declare_parameter("centroid_topic", "/detected_object_centroid")
        self.declare_parameter("depth_topic", "/overhead_camera/overhead/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/overhead_camera/overhead_rgb/camera_info")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("publish_topic", "/detected_object_pose")
        self.declare_parameter("depth_window", 3)
        self.declare_parameter("max_depth_m", 2.0)

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.latest_depth: Optional[Image] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(PoseStamped, publish_topic, 10)

        centroid_topic = self.get_parameter("centroid_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        self.create_subscription(PointStamped, centroid_topic, self.handle_centroid, 10)
        self.create_subscription(Image, depth_topic, self.handle_depth_image, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.handle_camera_info, 10)

        self.get_logger().info("Pose estimator node started.")

    def handle_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def handle_depth_image(self, msg: Image) -> None:
        self.latest_depth = msg

    def handle_centroid(self, msg: PointStamped) -> None:
        if self.camera_info is None or self.latest_depth is None:
            return

        centroid = (int(msg.point.x), int(msg.point.y))
        depth_m = self.sample_depth(self.latest_depth, centroid)
        if depth_m is None:
            return

        camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        if not camera_frame:
            camera_frame = msg.header.frame_id
        pose = self.project_to_3d(centroid, depth_m, self.camera_info, camera_frame)
        if pose is None:
            return
        pose.header.stamp = msg.header.stamp

        target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        if target_frame and target_frame != pose.header.frame_id:
            pose = self.transform_pose(pose, target_frame)
            if pose is None:
                return

        self.publisher.publish(pose)

    def sample_depth(self, msg: Image, centroid: Tuple[int, int]) -> Optional[float]:
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert depth image: {exc}")
            return None

        cx, cy = centroid
        window = int(self.get_parameter("depth_window").get_parameter_value().integer_value)
        if window < 1:
            window = 1
        half = window // 2
        height, width = depth_image.shape[:2]
        x_min = max(cx - half, 0)
        x_max = min(cx + half + 1, width)
        y_min = max(cy - half, 0)
        y_max = min(cy + half + 1, height)
        patch = depth_image[y_min:y_max, x_min:x_max]

        if msg.encoding == "16UC1":
            patch = patch.astype(np.float32) / 1000.0
        depth_values = patch[np.isfinite(patch)]
        if depth_values.size == 0:
            return None

        depth_m = float(np.median(depth_values))
        max_depth = self.get_parameter("max_depth_m").get_parameter_value().double_value
        if depth_m <= 0.0 or depth_m > max_depth:
            return None
        return depth_m

    @staticmethod
    def project_to_3d(
        centroid: Tuple[int, int], depth_m: float, camera_info: CameraInfo, frame_id: str
    ) -> Optional[PoseStamped]:
        if len(camera_info.k) < 9:
            return None
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        if fx == 0.0 or fy == 0.0:
            return None

        u, v = centroid
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time.from_msg(pose.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warning(f"TF lookup failed: {exc}")
            return None
        try:
            transformed_pose = do_transform_pose(pose.pose, transform)
        except Exception as exc:
            self.get_logger().warning(f"TF transform failed: {exc}")
            return None
        output = PoseStamped()
        output.header.stamp = pose.header.stamp
        output.header.frame_id = target_frame
        output.pose = transformed_pose
        return output


def main() -> None:
    rclpy.init()
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
