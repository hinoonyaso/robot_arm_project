import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def load_yaml(file_path):
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    description_share = get_package_share_directory("arm_description")
    moveit_share = get_package_share_directory("arm_moveit_config")

    robot_description_content = Command(
        [
            "xacro ",
            os.path.join(description_share, "urdf", "arm.urdf.xacro"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    srdf_file = os.path.join(moveit_share, "config", "arm.srdf")
    with open(srdf_file, "r", encoding="utf-8") as file:
        robot_description_semantic = {"robot_description_semantic": file.read()}

    kinematics_yaml = load_yaml(os.path.join(moveit_share, "config", "kinematics.yaml"))
    ompl_config = load_yaml(os.path.join(moveit_share, "config", "ompl_planning.yaml"))
    moveit_controllers = load_yaml(os.path.join(moveit_share, "config", "controllers.yaml"))
    joint_limits_yaml = load_yaml(os.path.join(moveit_share, "config", "joint_limits.yaml"))

    planning_parameters = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "planning_scene_monitor": {"publish_planning_scene": True},
        "use_sim_time": True,
    }

    if ompl_config and "move_group" in ompl_config and "ros__parameters" in ompl_config["move_group"]:
        planning_parameters.update(ompl_config["move_group"]["ros__parameters"])

    if kinematics_yaml and "move_group" in kinematics_yaml and "ros__parameters" in kinematics_yaml["move_group"]:
        planning_parameters.update(kinematics_yaml["move_group"]["ros__parameters"])

    if (
        joint_limits_yaml
        and "move_group" in joint_limits_yaml
        and "ros__parameters" in joint_limits_yaml["move_group"]
    ):
        planning_parameters.update(joint_limits_yaml["move_group"]["ros__parameters"])

    if (
        moveit_controllers
        and "move_group" in moveit_controllers
        and "ros__parameters" in moveit_controllers["move_group"]
    ):
        planning_parameters.update(moveit_controllers["move_group"]["ros__parameters"])

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_parameters,
        ],
    )

    return LaunchDescription([move_group_node])
