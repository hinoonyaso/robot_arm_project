import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    enable_task = LaunchConfiguration("enable_task")
    stress_test = LaunchConfiguration("stress_test")
    iterations = LaunchConfiguration("iterations")
    csv_path = LaunchConfiguration("csv_path")

    description_share = get_package_share_directory("arm_description")
    gazebo_share = get_package_share_directory("arm_gazebo")
    moveit_share = get_package_share_directory("arm_moveit_config")

    controller_params = os.path.join(gazebo_share, "config", "ros2_controllers.yaml")
    robot_description_content = Command(
        [
            "xacro ",
            os.path.join(description_share, "urdf", "arm.urdf.xacro"),
            " ros2_control_params:=",
            controller_params,
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(gazebo_share, "worlds", "arm_world.world"),
        }.items(),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "simple_arm",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    spawn_object = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            os.path.join(gazebo_share, "models", "object_box", "model.sdf"),
            "-entity",
            "object_box",
            "-x",
            "0.55",
            "-y",
            "0.0",
            "-z",
            "0.72",
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "move_group.launch.py"))
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
    )

    task_node = Node(
        package="arm_moveit_task",
        executable="pick_place_task",
        output="screen",
        parameters=[
            {
                "enable_task": enable_task,
                "stress_test.enabled": stress_test,
                "stress_test.iterations": iterations,
                "metrics.csv_path": csv_path,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_task", default_value="true"),
            DeclareLaunchArgument("stress_test", default_value="false"),
            DeclareLaunchArgument("iterations", default_value="100"),
            DeclareLaunchArgument("csv_path", default_value="/tmp/arm_pick_place_metrics.csv"),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            spawn_object,
            joint_state_spawner,
            arm_spawner,
            gripper_spawner,
            move_group,
            rviz,
            task_node,
        ]
    )
