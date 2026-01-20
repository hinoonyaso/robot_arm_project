# robot_arm_project

## Workspace

```bash
cd robot_arm_pick_place/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## One-click setup

```bash
./setup_env.sh
```

Add to `~/.bashrc` if you want auto-source:

```bash
source ~/dev_ws/robot_arm_vision/robot_arm_project/robot_arm_pick_place/colcon_ws/install/setup.bash
```

## Demo

```bash
ros2 launch arm_gazebo bringup_all.launch.py enable_task:=true stress_test:=false
```

## Perception (camera + 3D pose)

Goal: detect a red object, estimate (x, y, z) with depth, and publish `/detected_object_pose`.

- Perception node: `arm_moveit_task/perception_node.py`
- Publishes: `/detected_object_pose` (`geometry_msgs/PoseStamped`)
- Default topics (overhead depth camera):
  - Color: `/overhead_camera/overhead_rgb/image_raw`
  - Depth: `/overhead_camera/overhead/depth/image_raw`
  - Camera info: `/overhead_camera/overhead_rgb/camera_info`

Enable perception from launch:

```bash
ros2 launch arm_gazebo bringup_all.launch.py enable_task:=true enable_perception:=true
```

Color-cycle mode (random box colors per slot, always pick red, 10 iterations):

```bash
ros2 launch arm_gazebo bringup_all.launch.py enable_task:=true enable_perception:=true enable_color_cycle:=true color_cycle_iterations:=10
```

OctoMap (optional, enable via env var):

```bash
ENABLE_OCTOMAP=true ros2 launch arm_gazebo bringup_all.launch.py
```

Switch to EE camera or custom topics by overriding parameters on the perception node or remapping in launch.

## Grasping behavior

When perception is enabled, the task tries multiple grasp candidates around the detected pose.
If grasp succeeds, it places once and stops (no extra retreat/home loop).

Key parameters (set on `/arm_pick_place_task`):
- `perception.grasp_candidate_offsets`: flat list of `[dx, dy, dz]` triplets (meters).
- `perception.grasp_candidate_yaws`: list of yaw offsets (radians).
- `perception.grasp_offset`: fallback single offset for simple grasping.

Example:

```bash
ros2 param set /arm_pick_place_task perception.grasp_candidate_offsets "[0,0,0.02, 0.03,0,0.02, -0.03,0,0.02]"
ros2 param set /arm_pick_place_task perception.grasp_candidate_yaws "[0.0, 1.57, -1.57]"
```

## OctoMap (planning scene)

MoveIt ingests the overhead point cloud and builds an OctoMap for collision checking.
Config file: `robot_arm_pick_place/colcon_ws/src/arm_moveit_config/config/sensors_3d.yaml`.

Default topic: `/overhead_camera/overhead/points`

## Notes

- The overhead camera is a depth sensor in `arm_description/urdf/arm.urdf.xacro`.
- If depth topics are missing, verify the Gazebo depth camera plugin is present and loaded.

## Troubleshooting

Common checks:

```bash
# Perception node running?
ros2 node list | rg perception

# Detected pose available?
ros2 topic echo /detected_object_pose --once

# Overhead camera topics
ros2 topic list | rg overhead

# Point cloud for OctoMap
ros2 topic echo /overhead_camera/overhead/points --once
```

If the arm only moves vertically:
- Verify `/detected_object_pose` has x/y near the object position.
- Increase candidate offsets/yaws to allow lateral moves.

If perception does not publish:
- Check `rqt_image_view` on `/overhead_camera/overhead_rgb/image_raw`.
- Ensure `cv2` and `numpy<2` are installed in the ROS environment.
