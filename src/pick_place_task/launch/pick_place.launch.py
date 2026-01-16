from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pick_place_task",
            executable="pick_place_state_machine",
            name="pick_place_state_machine",
            output="screen",
            parameters=["config/pick_place_params.yaml"],
        ),
    ])
