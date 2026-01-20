#!/usr/bin/env python3
import math
from typing import List

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node


class GraspCandidateNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_grasp_candidates")

        self.declare_parameter("input_topic", "/detected_object_pose")
        self.declare_parameter("output_topic", "/grasp_candidates")
        self.declare_parameter("grasp_candidate_offsets", [0.0, 0.0, 0.02])
        self.declare_parameter("grasp_candidate_yaws", [0.0])
        self.declare_parameter("use_detected_orientation", False)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(PoseArray, output_topic, 10)
        self.create_subscription(PoseStamped, input_topic, self.handle_pose, 10)

        self.get_logger().info("Grasp candidate node started.")

    def handle_pose(self, msg: PoseStamped) -> None:
        offsets = self.get_parameter("grasp_candidate_offsets").get_parameter_value().double_array_value
        yaws = self.get_parameter("grasp_candidate_yaws").get_parameter_value().double_array_value
        if len(offsets) % 3 != 0:
            self.get_logger().warning("grasp_candidate_offsets length is not a multiple of 3.")
            return

        base_pose = msg.pose
        use_detected_orientation = (
            self.get_parameter("use_detected_orientation").get_parameter_value().bool_value
        )

        candidates: List[Pose] = []
        for i in range(0, len(offsets), 3):
            dx, dy, dz = offsets[i : i + 3]
            for yaw in yaws:
                pose = Pose()
                pose.position.x = base_pose.position.x + dx
                pose.position.y = base_pose.position.y + dy
                pose.position.z = base_pose.position.z + dz
                if use_detected_orientation:
                    pose.orientation = base_pose.orientation
                else:
                    pose.orientation = self.quaternion_from_rpy(0.0, 0.0, yaw)
                candidates.append(pose)

        output = PoseArray()
        output.header = msg.header
        output.poses = candidates
        self.publisher.publish(output)

    @staticmethod
    def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Pose().orientation
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main() -> None:
    rclpy.init()
    node = GraspCandidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
