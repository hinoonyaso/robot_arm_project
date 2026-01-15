from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_id_arg = DeclareLaunchArgument("run_id", default_value="run_local")
    log_dir_arg = DeclareLaunchArgument("log_dir", default_value="data/runs")

    task_manager = Node(
        package="fail_aware_task_manager",
        executable="task_manager_node",
        name="fail_aware_task_manager",
        output="screen",
        parameters=[
            {"run_id": LaunchConfiguration("run_id")},
            {"log_dir": LaunchConfiguration("log_dir")},
        ],
    )

    return LaunchDescription([run_id_arg, log_dir_arg, task_manager])
