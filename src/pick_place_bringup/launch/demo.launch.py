from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        )
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("pick_place_moveit_config"),
                "launch",
                "moveit.launch.py",
            ])
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("pick_place_moveit_config"),
            "config",
            "moveit.rviz",
        ])],
    )

    pick_place_node = Node(
        package="pick_place_task",
        executable="pick_place_state_machine",
        name="pick_place_state_machine",
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare("pick_place_task"),
            "config",
            "pick_place_params.yaml",
        ])],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        rviz_node,
        static_tf,
        pick_place_node,
    ])
