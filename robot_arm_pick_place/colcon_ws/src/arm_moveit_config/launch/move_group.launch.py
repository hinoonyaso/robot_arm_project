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


def merge_config(target_dict, config_yaml):
    if config_yaml and "move_group" in config_yaml and "ros__parameters" in config_yaml["move_group"]:
        target_dict.update(config_yaml["move_group"]["ros__parameters"])


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
    joint_limits_yaml = load_yaml(os.path.join(moveit_share, "config", "joint_limits.yaml"))
    ompl_planning_yaml = load_yaml(os.path.join(moveit_share, "config", "ompl_planning.yaml"))
    moveit_controllers = load_yaml(os.path.join(moveit_share, "config", "controllers.yaml"))

    planning_pipelines_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {},
    }

    if ompl_planning_yaml and "move_group" in ompl_planning_yaml and "ros__parameters" in ompl_planning_yaml["move_group"]:
        planning_pipelines_config["ompl"] = ompl_planning_yaml["move_group"]["ros__parameters"]

    planning_parameters = {
        "use_sim_time": True,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
        "planning_scene_monitor": {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    merge_config(planning_parameters, kinematics_yaml)
    merge_config(planning_parameters, joint_limits_yaml)
    merge_config(planning_parameters, moveit_controllers)

    planning_parameters.update(planning_pipelines_config)
    planning_parameters.update(robot_description)
    planning_parameters.update(robot_description_semantic)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[planning_parameters],
    )

    return LaunchDescription([move_group_node])
