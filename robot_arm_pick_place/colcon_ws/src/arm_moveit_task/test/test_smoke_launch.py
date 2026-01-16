import os
import time
import unittest

import launch
import launch_testing
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("arm_gazebo").find("arm_gazebo"),
                "launch",
                "bringup_all.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": "false",
            "enable_task": "false",
            "stress_test": "false",
        }.items(),
    )
    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node("smoke_test_node")
        self.received_joint_state = False
        self.node.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _joint_state_cb(self, msg):
        if msg.name:
            self.received_joint_state = True

    def test_joint_states_and_attach_service(self):
        end_time = time.time() + 20.0
        while time.time() < end_time and not self.received_joint_state:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertTrue(self.received_joint_state, "Did not receive /joint_states")

        end_time = time.time() + 10.0
        while time.time() < end_time:
            if self.node.count_services("/attach") > 0:
                break
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertGreater(self.node.count_services("/attach"), 0, "/attach service not found")
