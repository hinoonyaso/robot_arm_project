# Robot Arm Pick & Place (ROS 2 + Gazebo + MoveIt)

End-to-end simulation project for a 6-DOF robot arm performing pick-and-place with
vision-based perception, MoveIt planning, and Gazebo physics. This repo is set up
for repeatable experiments (stress tests, color cycles) and portfolio demos.

## Highlights
- Full ROS 2 pipeline: Gazebo simulation, ros2_control, MoveIt planning, RViz visualization.
- Vision-based grasp pipeline: color segmentation, depth projection, pose estimation, grasp candidates.
- Task execution with staged planning/execution, retries, and CSV metrics logging.
- Scene sync between Gazebo model states and MoveIt collision objects.
- Configurable launch toggles for tasks, perception, stress tests, and color cycles.

## Project Layout
- `src/arm_description`: URDF/xacro, ros2_control setup, links/joints/tool0/camera.
- `src/arm_gazebo`: Gazebo world, models, controllers, bringup launch.
- `src/arm_moveit_config`: SRDF, kinematics, OMPL planning, RViz config.
- `src/arm_moveit_task`: Task logic and perception nodes.

## Core Functionality
### Task Pipeline (PickPlaceTask)
- Staged flow: home -> pre-grasp -> grasp -> lift -> pre-place -> place -> retreat
- Planning and execution via MoveIt actions
- Recovery hooks and retry policy per stage
- Metrics logging to CSV
- Optional color cycling and stress testing loops

Key file: `src/arm_moveit_task/arm_moveit_task/pick_place_task.py`

### Perception Pipeline
Two options are provided:
1) Split pipeline (default):
   - `color_detector_node.py`: HSV segmentation for target color centroid
   - `pose_estimator_node.py`: depth-based 3D pose from centroid
   - `grasp_candidate_node.py`: candidate poses around target
2) Legacy combined node:
   - `perception_node.py`: color + depth + pose in one node

Key topics:
- `/overhead_camera/overhead_rgb/image_raw`
- `/overhead_camera/overhead/depth/image_raw`
- `/detected_object_pose`
- `/grasp_candidates`

## Running
### Build
```bash
colcon build --packages-select arm_description arm_gazebo arm_moveit_config arm_moveit_task
source install/setup.bash
```

### Launch (Gazebo + MoveIt + RViz + Task)
```bash
ros2 launch arm_gazebo bringup_all.launch.py \
  enable_task:=true enable_perception:=true \
  enable_color_cycle:=true color_cycle_iterations:=10
```

### Useful Launch Flags
- `use_rviz` (default: true)
- `gazebo_gui` (default: true)
- `enable_task` (default: true)
- `enable_perception` (default: true)
- `use_legacy_perception` (default: false)
- `enable_color_cycle` (default: false)
- `stress_test` (default: false)
- `iterations` (default: 100)
- `csv_path` (default: /tmp/arm_pick_place_metrics.csv)

## Notable Parameters
PickPlaceTask parameters (partial list):
- `home_pose_joint_values`
- `startup.force_home`, `startup.safe_joint_values`
- `planning.profile`, `planning.allowed_time`, `planning.num_attempts`
- `planning.scene_sync_enabled`, `planning.scene_sync_period_sec`
- `perception.enabled`, `perception.param_node`
- `color_cycle.*`, `stress_test.*`, `metrics.*`

## Architecture Notes
- The planning scene is kept aligned with Gazebo via `/model_states`.
- AllowedCollisionMatrix is inherited from MoveIt (SRDF) and extended only for visualization boxes.
- Startup includes an optional force-home and state validity gate before task execution.

## Demo Tips
- Use RViz Motion Planning to visualize start state collisions.
- Adjust `startup.safe_joint_values` if the arm spawns in collision with the table.
- The scene sync timer updates table/box poses if they move in Gazebo.

## Demo Assets
Add your own demo media and link it here.
- `docs/demo_overview.gif` (short loop)
- `docs/rviz_planning.png` (planning scene view)
- `docs/gazebo_pick_place.mp4` (full run)

## Metrics
CSV metrics include per-stage success, plan/exec time, and failure reasons:
```
iteration_id,stage_name,plan_success,exec_success,plan_time_ms,exec_time_ms,retries_used,fail_reason,timestamp
```

Example summary (sample numbers):
- Success rate: 92.3%
- Avg plan time: 120 ms
- Avg exec time: 310 ms
- Avg retries per stage: 0.3

## Portfolio Summary (1-2 lines)
Built a full ROS 2 pick-and-place pipeline integrating simulation, planning,
vision-based grasping, and automated experiment loops with metrics logging.

## Portfolio Story
**Problem**: Create a reproducible, end-to-end pick-and-place demo in ROS 2 that includes
perception, planning, and execution under simulation.

**Approach**: Built a Gazebo + MoveIt stack with a vision pipeline (color segmentation,
depth projection, pose estimation), staged task execution with retries, and continuous
scene synchronization between Gazebo and MoveIt.

**Result**: A configurable demo that supports stress testing, color cycling, and
metrics logging for iteration-level performance tracking.

## Roadmap Ideas
- Robust multi-color perception and confidence scoring
- Dynamic collision object updates from perception (not only Gazebo)
- Improved recovery policies and exception isolation between stages
