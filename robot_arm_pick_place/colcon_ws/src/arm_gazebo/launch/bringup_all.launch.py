import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    enable_task = LaunchConfiguration("enable_task")
    enable_perception = LaunchConfiguration("enable_perception")
    use_legacy_perception = LaunchConfiguration("use_legacy_perception")
    enable_color_cycle = LaunchConfiguration("enable_color_cycle")
    stress_test = LaunchConfiguration("stress_test")
    iterations = LaunchConfiguration("iterations")
    color_cycle_iterations = LaunchConfiguration("color_cycle_iterations")
    csv_path = LaunchConfiguration("csv_path")

    description_share = get_package_share_directory("arm_description")
    gazebo_share = get_package_share_directory("arm_gazebo")
    moveit_share = get_package_share_directory("arm_moveit_config")
    fastdds_profile = os.path.join(gazebo_share, "config", "fastdds_no_shm.xml")

    controller_params = os.path.join(gazebo_share, "config", "ros2_controllers.yaml")
    xacro_path = os.path.join(description_share, "urdf", "arm.urdf.xacro")
    try:
        robot_description_content = subprocess.check_output(
            ["xacro", xacro_path, f"ros2_control_params:={controller_params}"], text=True
        )
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"xacro failed: {exc}") from exc
    robot_description_semantic = open(
        os.path.join(moveit_share, "config", "arm.srdf"),
        "r",
        encoding="utf-8",
    ).read()

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
            "gui": gazebo_gui,
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

    spawn_red_box = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            os.path.join(gazebo_share, "models", "object_box_red", "model.sdf"),
            "-entity",
            "object_box_red",
            "-x",
            "0.55",
            "-y",
            "0.12",
            "-z",
            "0.72",
        ],
        output="screen",
    )

    spawn_blue_box = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            os.path.join(gazebo_share, "models", "object_box_blue", "model.sdf"),
            "-entity",
            "object_box_blue",
            "-x",
            "0.55",
            "-y",
            "0.0",
            "-z",
            "0.72",
        ],
        output="screen",
    )

    spawn_black_box = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            os.path.join(gazebo_share, "models", "object_box_black", "model.sdf"),
            "-entity",
            "object_box_black",
            "-x",
            "0.55",
            "-y",
            "-0.12",
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
        parameters=[
            {
                "robot_description": robot_description_content,
                "robot_description_semantic": robot_description_semantic,
                "use_sim_time": True,
            },
        ],
    )

    def create_task_node(context):
        def as_bool(value: str) -> bool:
            return value.lower() in ("1", "true", "yes", "on")

        enable_task_val = as_bool(enable_task.perform(context))
        enable_perception_val = as_bool(enable_perception.perform(context))
        stress_test_val = as_bool(stress_test.perform(context))
        enable_color_cycle_val = as_bool(enable_color_cycle.perform(context))
        iterations_val = int(iterations.perform(context))
        color_cycle_iterations_val = int(color_cycle_iterations.perform(context))
        csv_path_val = csv_path.perform(context)

        node = Node(
            package="arm_moveit_task",
            executable="pick_place_task",
            output="screen",
            parameters=[
                {
                    "enable_task": enable_task_val,
                    "perception.enabled": enable_perception_val,
                    "stress_test.enabled": stress_test_val,
                    "stress_test.iterations": iterations_val,
                    "color_cycle.enabled": enable_color_cycle_val,
                    "color_cycle.iterations": color_cycle_iterations_val,
                    "metrics.csv_path": csv_path_val,
                    "use_sim_time": True,
                }
            ],
        )
        return [node]

    overhead_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.5", "0.0", "1.15", "0.0", "1.5708", "0.0", "world", "overhead_camera_link"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    overhead_camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "-1.5708", "0.0", "-1.5708", "overhead_camera_link", "overhead_camera_optical_frame"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    color_detector = Node(
        condition=UnlessCondition(use_legacy_perception),
        package="arm_moveit_task",
        executable="color_detector",
        name="arm_perception_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "color_topic": "/overhead_camera/overhead_rgb/image_raw",
                "centroid_topic": "/detected_object_centroid",
            }
        ],
    )

    pose_estimator = Node(
        condition=UnlessCondition(use_legacy_perception),
        package="arm_moveit_task",
        executable="pose_estimator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "centroid_topic": "/detected_object_centroid",
                "depth_topic": "/overhead_camera/overhead/depth/image_raw",
                "camera_info_topic": "/overhead_camera/overhead_rgb/camera_info",
                "camera_frame": "overhead_camera_optical_frame",
                "target_frame": "world",
                "publish_topic": "/detected_object_pose",
            }
        ],
    )

    grasp_candidates = Node(
        condition=UnlessCondition(use_legacy_perception),
        package="arm_moveit_task",
        executable="grasp_candidates",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/detected_object_pose",
                "output_topic": "/grasp_candidates",
            }
        ],
    )

    perception_node = Node(
        condition=IfCondition(use_legacy_perception),
        package="arm_moveit_task",
        executable="perception_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "color_topic": "/overhead_camera/overhead_rgb/image_raw",
                "depth_topic": "/overhead_camera/overhead/depth/image_raw",
                "camera_info_topic": "/overhead_camera/overhead_rgb/camera_info",
                "camera_frame": "overhead_camera_optical_frame",
            }
        ],
    )

    perception_pipeline = GroupAction(
        condition=IfCondition(enable_perception),
        actions=[color_detector, pose_estimator, grasp_candidates, perception_node],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("enable_task", default_value="true"),
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("use_legacy_perception", default_value="false"),
            DeclareLaunchArgument("enable_color_cycle", default_value="false"),
            DeclareLaunchArgument("stress_test", default_value="false"),
            DeclareLaunchArgument("iterations", default_value="100"),
            DeclareLaunchArgument("color_cycle_iterations", default_value="10"),
            DeclareLaunchArgument("csv_path", default_value="/tmp/arm_pick_place_metrics.csv"),
            SetEnvironmentVariable("FASTDDS_DEFAULT_PROFILES_FILE", fastdds_profile),
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
            SetEnvironmentVariable("ENABLE_OCTOMAP", "false"),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            spawn_red_box,
            spawn_blue_box,
            spawn_black_box,
            joint_state_spawner,
            arm_spawner,
            gripper_spawner,
            move_group,
            rviz,
            world_to_base_tf,
            overhead_camera_tf,
            overhead_camera_optical_tf,
            OpaqueFunction(function=create_task_node),
            perception_pipeline,
        ]
    )
