from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("pick_place_moveit_config")

    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_share, "config", "robot.urdf.xacro"]),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic = {
        "robot_description_semantic": Command([
            "cat ",
            PathJoinSubstitution([pkg_share, "config", "simple_arm.srdf"]),
        ])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            PathJoinSubstitution([pkg_share, "config", "kinematics.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "ompl_planning.yaml"]),
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group,
    ])
