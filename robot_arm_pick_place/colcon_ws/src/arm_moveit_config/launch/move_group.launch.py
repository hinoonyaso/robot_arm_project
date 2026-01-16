import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    description_share = get_package_share_directory("arm_description")
    moveit_share = get_package_share_directory("arm_moveit_config")
    robot_description = Command(
        [
            "xacro ",
            os.path.join(description_share, "urdf", "arm.urdf.xacro"),
        ]
    )

    robot_description_semantic = open(
        os.path.join(moveit_share, "config", "arm.srdf"),
        "r",
        encoding="utf-8",
    ).read()

    kinematics_yaml = os.path.join(moveit_share, "config", "kinematics.yaml")
    joint_limits_yaml = os.path.join(moveit_share, "config", "joint_limits.yaml")
    ompl_yaml = os.path.join(moveit_share, "config", "ompl_planning.yaml")
    controllers_yaml = os.path.join(moveit_share, "config", "controllers.yaml")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
            joint_limits_yaml,
            ompl_yaml,
            controllers_yaml,
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"planning_scene_monitor": {"publish_planning_scene": True}},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group])
