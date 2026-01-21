#!/usr/bin/env python3
import csv
import os
import random
import threading
import time
from dataclasses import dataclass
from enum import Enum
from math import cos, sin
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter as RclpyParameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StageResult(Enum):
    SUCCESS = "SUCCESS"
    PLAN_FAIL = "PLAN_FAIL"
    EXEC_FAIL = "EXEC_FAIL"
    TIMEOUT = "TIMEOUT"
    ATTACH_FAIL = "ATTACH_FAIL"
    DETACH_FAIL = "DETACH_FAIL"
    UNKNOWN = "UNKNOWN"


@dataclass
class MetricsRow:
    iteration_id: int
    stage_name: str
    plan_success: int
    exec_success: int
    plan_time_ms: float
    exec_time_ms: float
    retries_used: int
    fail_reason: str
    timestamp: float


class MetricsLogger:
    def __init__(self, node: Node, csv_path: str, enabled: bool) -> None:
        self._node = node
        self._csv_path = csv_path
        self._enabled = enabled
        self._rows: List[MetricsRow] = []
        self._header_written = False

    def log(self, row: MetricsRow) -> None:
        self._rows.append(row)
        if self._enabled:
            self._write_row(row)

    def _write_row(self, row: MetricsRow) -> None:
        directory = os.path.dirname(self._csv_path)
        if directory:
            try:
                os.makedirs(directory, exist_ok=True)
            except OSError:
                self._csv_path = "/tmp/arm_pick_place_metrics.csv"
                os.makedirs("/tmp", exist_ok=True)
        write_header = not self._header_written and not os.path.exists(self._csv_path)
        with open(self._csv_path, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            if write_header:
                writer.writerow(
                    [
                        "iteration_id",
                        "stage_name",
                        "plan_success",
                        "exec_success",
                        "plan_time_ms",
                        "exec_time_ms",
                        "retries_used",
                        "fail_reason",
                        "timestamp",
                    ]
                )
                self._header_written = True
            writer.writerow(
                [
                    row.iteration_id,
                    row.stage_name,
                    row.plan_success,
                    row.exec_success,
                    f"{row.plan_time_ms:.2f}",
                    f"{row.exec_time_ms:.2f}",
                    row.retries_used,
                    row.fail_reason,
                    f"{row.timestamp:.3f}",
                ]
            )

    def summarize(self) -> Dict[str, float]:
        plan_times = [r.plan_time_ms for r in self._rows if r.plan_success]
        exec_times = [r.exec_time_ms for r in self._rows if r.exec_success]
        success_count = sum(1 for r in self._rows if r.fail_reason == StageResult.SUCCESS.value)
        total = len(self._rows)
        return {
            "total": total,
            "success": success_count,
            "success_rate": (success_count / total) * 100.0 if total else 0.0,
            "plan_avg_ms": sum(plan_times) / len(plan_times) if plan_times else 0.0,
            "exec_avg_ms": sum(exec_times) / len(exec_times) if exec_times else 0.0,
        }


class PickPlaceTask(Node):
    def __init__(self) -> None:
        super().__init__("arm_pick_place_task")
        self.declare_parameter("enable_task", True)
        self.declare_parameter("home_pose_joint_values", [0.0, -0.7, 1.3, 0.0, 1.0, 0.0])
        self.declare_parameter("startup.force_home", True)
        self.declare_parameter("startup.force_home_duration_sec", 2.0)
        self.declare_parameter("startup.safe_joint_values", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("startup.wait_for_valid_state_sec", 10.0)
        self.declare_parameter("startup.state_check_period_sec", 0.5)
        self.declare_parameter("startup.abort_on_invalid_state", True)
        
        # 그리퍼 좌우 회전을 위해 Yaw = 0.0 으로 설정
        self.declare_parameter("pre_grasp_pose", [0.45, 0.0, 0.80, 3.14, 0.0, 0.0])
        self.declare_parameter("grasp_pose", [0.45, 0.0, 0.74, 3.14, 0.0, 0.0])
        self.declare_parameter("lift_pose", [0.45, 0.0, 0.88, 3.14, 0.0, 0.0])
        self.declare_parameter("pre_place_pose", [0.4, -0.25, 0.82, 3.14, 0.0, 0.0])
        self.declare_parameter("place_pose", [0.4, -0.25, 0.76, 3.14, 0.0, 0.0])
        self.declare_parameter("retreat_pose", [0.4, -0.25, 0.90, 3.14, 0.0, 0.0])

        self.declare_parameter("ee_to_object_offset", [0.0, 0.0, 0.08])
        self.declare_parameter("perception.enabled", False)
        self.declare_parameter("perception.topic", "/detected_object_pose")
        self.declare_parameter("perception.param_node", "/arm_color_detector")
        self.declare_parameter("perception.timeout_sec", 1.0)
        self.declare_parameter("perception.wait_timeout_sec", 3.0)
        self.declare_parameter("perception.use_detected_orientation", False)
        self.declare_parameter("perception.valid_bounds", [0.3, 0.75, -0.2, 0.2, 0.65, 0.85])
        self.declare_parameter(
            "perception.grasp_candidate_offsets",
            [0.0, 0.0, 0.0, 0.02, 0.0, 0.0, -0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, -0.02, 0.0],
        )
        self.declare_parameter("perception.grasp_candidate_yaws", [0.0])
        self.declare_parameter("perception.pre_grasp_offset", [0.0, 0.0, 0.08])
        self.declare_parameter("perception.grasp_offset", [0.0, 0.0, -0.01])
        self.declare_parameter("perception.lift_offset", [0.0, 0.0, 0.16])
        self.declare_parameter("perception.attach_distance_threshold", 0.05)
        self.declare_parameter("recovery.enabled", True)
        self.declare_parameter("recovery.max_retries", 1)
        self.declare_parameter("stress_test.enabled", False)
        self.declare_parameter("stress_test.iterations", 100)
        self.declare_parameter("stress_test.per_stage_timeout_sec", 12.0)
        self.declare_parameter("stress_test.max_retries_per_stage", 2)
        self.declare_parameter("stress_test.sleep_between_iterations_sec", 1.0)
        self.declare_parameter("color_cycle.enabled", False)
        self.declare_parameter("color_cycle.iterations", 10)
        self.declare_parameter("color_cycle.sleep_between_iterations_sec", 1.0)
        self.declare_parameter("color_cycle.colors", ["red", "blue", "black"])
        self.declare_parameter("metrics.log_csv", True)
        self.declare_parameter("metrics.csv_path", "/tmp/arm_pick_place_metrics.csv")
        self.declare_parameter("metrics.print_summary_every", 10)
        self.declare_parameter("planning.profile", "stable")
        self.declare_parameter("planning.visualize_boxes_in_perception", True)
        self.declare_parameter("planning.scene_sync_enabled", True)
        self.declare_parameter("planning.scene_sync_period_sec", 1.0)
        self.declare_parameter("planning.num_attempts", 1)
        self.declare_parameter("planning.allowed_time", 2.5)
        self.declare_parameter("planning.goal_timeout_sec", 4.0)
        self.declare_parameter("planning.result_timeout_sec", 4.0)

        self.ee_link = "tool0"
        self.planning_frame = "world"
        self.arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = ["finger_left_joint", "finger_right_joint"]
        self.robot_link_names = [
            "base_link",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
            "ee_link",
            "tool0",
            "finger_left",
            "finger_right",
        ]
        self.self_collision_allow_pairs = [
            ("base_link", "link1"),
            ("link1", "link2"),
            ("link2", "link3"),
            ("link3", "link4"),
            ("link4", "link5"),
            ("link5", "link6"),
            ("link6", "ee_link"),
            ("ee_link", "tool0"),
            ("ee_link", "finger_left"),
            ("ee_link", "finger_right"),
            ("base_link", "link2"),
            ("link2", "link4"),
            ("finger_left", "finger_right"),
            ("finger_left", "tool0"),
            ("finger_right", "tool0"),
        ]

        self.callback_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt move_group action server is "move_action" (not "move_group").
        self.move_group_client = ActionClient(self, MoveGroup, "move_action", callback_group=self.callback_group)
        self.execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory", callback_group=self.callback_group)
        self.arm_traj_client = ActionClient(
            self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory", callback_group=self.callback_group
        )
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene", callback_group=self.callback_group)
        self.get_scene_client = self.create_client(GetPlanningScene, "get_planning_scene", callback_group=self.callback_group)
        self.state_validity_client = self.create_client(
            GetStateValidity, "check_state_validity", callback_group=self.callback_group
        )
        perception_param_node = self.get_parameter("perception.param_node").get_parameter_value().string_value
        self.perception_param_client = self.create_client(
            SetParameters, f"{perception_param_node}/set_parameters", callback_group=self.callback_group
        )

        self.attach_service = self.create_service(Trigger, "/attach", self.handle_attach, callback_group=self.callback_group)
        self.detach_service = self.create_service(Trigger, "/detach", self.handle_detach, callback_group=self.callback_group)

        self.detected_pose: Optional[PoseStamped] = None
        self.latest_joint_state: Optional[JointState] = None

        self.model_poses: Dict[str, Pose] = {}
        self.create_subscription(ModelStates, "/model_states", self.handle_model_states, 10)
        self.create_subscription(JointState, "/joint_states", self.handle_joint_states, 10)
        scene_sync_period = self.get_parameter("planning.scene_sync_period_sec").get_parameter_value().double_value
        self.scene_sync_timer = self.create_timer(
            max(0.2, scene_sync_period), self.sync_scene_from_gazebo, callback_group=self.callback_group
        )
        perception_topic = self.get_parameter("perception.topic").get_parameter_value().string_value
        self.create_subscription(
            PoseStamped,
            perception_topic,
            self.handle_detected_pose,
            10,
            callback_group=self.callback_group,
        )

        self.entity_client = self.create_client(SetEntityState, "/set_entity_state", callback_group=self.callback_group)
        self.get_entity_client = self.create_client(GetEntityState, "/get_entity_state", callback_group=self.callback_group)
        self.spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity", callback_group=self.callback_group)
        self.delete_entity_client = self.create_client(DeleteEntity, "/delete_entity", callback_group=self.callback_group)
        self.attach_active = False
        self.attached_entity = "object_box_red"
        self.box_slots = [
            ("object_box_red", (0.55, 0.12, 0.72)),
            ("object_box_blue", (0.55, 0.0, 0.72)),
            ("object_box_black", (0.55, -0.12, 0.72)),
        ]
        self.box_models = self.load_box_models()
        self.planning_scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self.follow_timer = self.create_timer(1.0 / 30.0, self.follow_object, callback_group=self.callback_group)
        self.scene_applied = False
        self.scene_timer = self.create_timer(2.0, self.ensure_planning_scene, callback_group=self.callback_group)

        self.apply_planning_profile()

        self.metrics_logger = MetricsLogger(
            self,
            self.get_parameter("metrics.csv_path").get_parameter_value().string_value,
            self.get_parameter("metrics.log_csv").get_parameter_value().bool_value,
        )

        self.task_thread = threading.Thread(target=self.task_loop, daemon=True)
        self.task_thread.start()

    def load_box_models(self) -> Dict[str, str]:
        gazebo_share = get_package_share_directory("arm_gazebo")
        models: Dict[str, str] = {}
        for color in ["red", "blue", "black"]:
            model_path = os.path.join(gazebo_share, "models", f"object_box_{color}", "model.sdf")
            try:
                with open(model_path, "r", encoding="utf-8") as handle:
                    models[color] = handle.read()
            except OSError as exc:
                self.get_logger().warning(f"Failed to load model for {color}: {exc}")
        return models

        

    def handle_detected_pose(self, msg: PoseStamped) -> None:
        self.detected_pose = msg

    def handle_attach(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.attach_active = True
        response.success = True
        response.message = "Attached object follow enabled"
        return response

    def handle_detach(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.attach_active = False
        response.success = True
        response.message = "Detached object follow disabled"
        return response

    def follow_object(self) -> None:
        if not self.attach_active:
            return
        try:
            transform = self.tf_buffer.lookup_transform("world", self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.0))
        except Exception:
            return
        offset = self.get_parameter("ee_to_object_offset").get_parameter_value().double_array_value
        pose = Pose()
        pose.position.x = transform.transform.translation.x + offset[0]
        pose.position.y = transform.transform.translation.y + offset[1]
        pose.position.z = transform.transform.translation.z + offset[2]
        pose.orientation = transform.transform.rotation

        state = EntityState()
        state.name = self.attached_entity
        state.pose = pose
        self.entity_client.call_async(SetEntityState.Request(state=state))

    def ensure_planning_scene(self) -> None:
        if self.scene_applied:
            return
        self.scene_applied = self.add_collision_objects()

    def add_collision_objects(self) -> bool:
        if not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("apply_planning_scene service not available yet.")
            return False

        table = CollisionObject()
        table.id = "table"
        table.header.frame_id = self.planning_frame
        table.operation = CollisionObject.ADD
        table_primitive = SolidPrimitive()
        table_primitive.type = SolidPrimitive.BOX
        table_primitive.dimensions = [0.8, 0.6, 0.7]
        table_pose = Pose()
        table_pose.position.x = 0.52
        table_pose.position.y = 0.0
        table_pose.position.z = 0.35
        table_pose.orientation.w = 1.0
        table.primitives = [table_primitive]
        table.primitive_poses = [table_pose]

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects = [table]

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [table]

        perception_enabled = self.get_parameter("perception.enabled").get_parameter_value().bool_value
        visualize_boxes = self.get_parameter("planning.visualize_boxes_in_perception").get_parameter_value().bool_value
        if not perception_enabled or visualize_boxes:
            box_primitive = SolidPrimitive()
            box_primitive.type = SolidPrimitive.BOX
            box_primitive.dimensions = [0.04, 0.04, 0.04]

            red_box = CollisionObject()
            red_box.id = "object_box_red"
            red_box.header.frame_id = self.planning_frame
            red_box.operation = CollisionObject.ADD
            red_pose = Pose()
            red_pose.position.x = 0.55
            red_pose.position.y = 0.12
            red_pose.position.z = 0.72
            red_pose.orientation.w = 1.0
            red_box.primitives = [box_primitive]
            red_box.primitive_poses = [red_pose]

            blue_box = CollisionObject()
            blue_box.id = "object_box_blue"
            blue_box.header.frame_id = self.planning_frame
            blue_box.operation = CollisionObject.ADD
            blue_pose = Pose()
            blue_pose.position.x = 0.55
            blue_pose.position.y = 0.0
            blue_pose.position.z = 0.72
            blue_pose.orientation.w = 1.0
            blue_box.primitives = [box_primitive]
            blue_box.primitive_poses = [blue_pose]

            black_box = CollisionObject()
            black_box.id = "object_box_black"
            black_box.header.frame_id = self.planning_frame
            black_box.operation = CollisionObject.ADD
            black_pose = Pose()
            black_pose.position.x = 0.55
            black_pose.position.y = -0.12
            black_pose.position.z = 0.72
            black_pose.orientation.w = 1.0
            black_box.primitives = [box_primitive]
            black_box.primitive_poses = [black_pose]

            request.scene.world.collision_objects.extend([red_box, blue_box, black_box])
            scene_msg.world.collision_objects.extend([red_box, blue_box, black_box])
            if perception_enabled:
                matrix = self.build_box_visualization_matrix([red_box.id, blue_box.id, black_box.id])
                if matrix is not None:
                    request.scene.allowed_collision_matrix = matrix
                    scene_msg.allowed_collision_matrix = matrix

        scene_msg.robot_state.is_diff = True
        self.planning_scene_pub.publish(scene_msg)

        future = self.apply_scene_client.call_async(request)
        try:
            if not self.wait_for_future(future, timeout_sec=2.0):
                raise TimeoutError("apply_planning_scene timeout")
            response = future.result()
            if response is None or not response.success:
                self.get_logger().warning("Failed to apply planning scene.")
                return False
            self.get_logger().info("Collision objects added.")
            return True
        except Exception as e:
            self.get_logger().warning(f"Failed to add collision objects: {e}")
            return False

    def reset_object_pose(self) -> bool:
        service_ready = False
        for _ in range(25):
            if self.entity_client.wait_for_service(timeout_sec=0.5):
                service_ready = True
                break
            time.sleep(0.1)
        if not service_ready:
            # If the service is in the graph but wait_for_service missed it, attempt the call anyway.
            service_names = [name for name, _ in self.get_service_names_and_types()]
            if "/set_entity_state" not in service_names:
                self.get_logger().warning("/set_entity_state service not available")
                return False
            self.get_logger().warning("Service listed but not ready; attempting call anyway.")
        success = True
        for name, position in self.box_slots:
            state = EntityState()
            state.name = name
            state.pose.position.x = position[0]
            state.pose.position.y = position[1]
            state.pose.position.z = position[2]
            state.pose.orientation.w = 1.0
            future = self.entity_client.call_async(SetEntityState.Request(state=state))
            try:
                if not self.wait_for_future(future, timeout_sec=2.0):
                    success = False
                    continue
                res = future.result()
                success = success and res is not None and res.success
            except Exception:
                success = False
        return success

    def handle_model_states(self, msg: ModelStates) -> None:
        targets = {"table", "object_box_red", "object_box_blue", "object_box_black"}
        for name, pose in zip(msg.name, msg.pose):
            if name in targets:
                self.model_poses[name] = pose

    def handle_joint_states(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def wait_for_valid_start_state(self) -> bool:
        timeout_sec = self.get_parameter("startup.wait_for_valid_state_sec").get_parameter_value().double_value
        period = self.get_parameter("startup.state_check_period_sec").get_parameter_value().double_value
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            if self.latest_joint_state is None:
                time.sleep(max(0.05, period))
                continue
            if not self.state_validity_client.wait_for_service(timeout_sec=0.5):
                time.sleep(max(0.05, period))
                continue
            request = GetStateValidity.Request()
            request.group_name = "arm"
            request.robot_state.joint_state = self.latest_joint_state
            future = self.state_validity_client.call_async(request)
            if not self.wait_for_future(future, timeout_sec=1.0):
                time.sleep(max(0.05, period))
                continue
            response = future.result()
            if response is not None and response.valid:
                return True
            time.sleep(max(0.05, period))
        return False

    def sync_scene_from_gazebo(self) -> None:
        if not self.scene_applied:
            return
        if not self.get_parameter("planning.scene_sync_enabled").get_parameter_value().bool_value:
            return
        if not self.apply_scene_client.wait_for_service(timeout_sec=0.2):
            return

        updates: List[CollisionObject] = []
        table_pose = self.model_poses.get("table")
        if table_pose is not None:
            updates.append(self.make_box_collision("table", table_pose, [0.8, 0.6, 0.7]))

        for name in ("object_box_red", "object_box_blue", "object_box_black"):
            pose = self.model_poses.get(name)
            if pose is None:
                continue
            updates.append(self.make_box_collision(name, pose, [0.04, 0.04, 0.04]))

        if not updates:
            return

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = updates
        self.apply_scene_client.call_async(request)

    def make_box_collision(self, object_id: str, pose: Pose, dimensions: List[float]) -> CollisionObject:
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = self.planning_frame
        obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(dimensions)
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        return obj

    def task_loop(self) -> None:
        self.get_logger().info("Task Thread Started. Waiting for system initialization...")
        time.sleep(3.0) 
        
        # 1. 서비스 연결 확인 (노드 생존 여부)
        self.get_logger().info("Checking connection to MoveGroup node via service...")
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("CRITICAL: MoveGroup Service not found! Node might be down.")
        else:
            self.get_logger().info("Service found! Node is visible.")

        # 2. [디버그] 현재 노드가 볼 수 있는 토픽 목록 출력
        self.get_logger().info("DEBUG: Scanning available topics for 'move_group'...")
        topic_names_and_types = self.get_topic_names_and_types()
        found_move_group_topics = []
        for name, types in topic_names_and_types:
            if "move_group" in name:
                found_move_group_topics.append(name)
        
        if found_move_group_topics:
            self.get_logger().info(f"DEBUG: Found move_group topics: {found_move_group_topics}")
        else:
            self.get_logger().error("DEBUG: NO topics containing 'move_group' found! Check remapping or namespace.")

        # 3. 액션 서버 연결 대기
        self.wait_for_action(self.move_group_client, "move_action")
        self.wait_for_action(self.execute_client, "execute_trajectory")
        self.wait_for_action(self.arm_traj_client, "arm_controller/follow_joint_trajectory")
        if self.get_parameter("startup.force_home").get_parameter_value().bool_value:
            if self.force_move_home():
                self.get_logger().info("Force home completed before planning.")
            else:
                self.get_logger().warning("Force home failed; planning may start in collision.")
        
        self.add_collision_objects()

        if not self.wait_for_valid_start_state():
            message = "Start state is still invalid after timeout."
            if self.get_parameter("startup.abort_on_invalid_state").get_parameter_value().bool_value:
                self.get_logger().error(message + " Aborting task.")
                return
            self.get_logger().warning(message + " Continuing anyway.")
        
        if not self.get_parameter("enable_task").get_parameter_value().bool_value:
            self.get_logger().info("Task execution disabled; services only mode.")
            return

        stress_enabled = self.get_parameter("stress_test.enabled").get_parameter_value().bool_value
        iterations = int(self.get_parameter("stress_test.iterations").get_parameter_value().integer_value)
        color_cycle_enabled = self.get_parameter("color_cycle.enabled").get_parameter_value().bool_value
        if color_cycle_enabled:
            cycle_iterations = int(self.get_parameter("color_cycle.iterations").get_parameter_value().integer_value)
            self.run_color_cycle(cycle_iterations)
        elif stress_enabled:
            self.run_stress_test(iterations)
        else:
            self.run_single_iteration(1)

    def wait_for_action(self, client: ActionClient, name: str) -> None:
        self.get_logger().info(f"Waiting for Action server '{name}'...")
        while not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f"Action server '{name}' not available yet. Retrying...")
        self.get_logger().info(f"Action server '{name}' connected!")

    @staticmethod
    def wait_for_future(future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def run_stress_test(self, iterations: int) -> None:
        print_every = int(self.get_parameter("metrics.print_summary_every").get_parameter_value().integer_value)
        sleep_between = self.get_parameter("stress_test.sleep_between_iterations_sec").get_parameter_value().double_value
        for iteration in range(1, iterations + 1):
            self.run_single_iteration(iteration)
            if iteration % print_every == 0:
                summary = self.metrics_logger.summarize()
                self.get_logger().info(
                    f"Progress {iteration}/{iterations} | Success {summary['success_rate']:.1f}% | "
                    f"Plan avg {summary['plan_avg_ms']:.1f} ms | Exec avg {summary['exec_avg_ms']:.1f} ms"
                )
            time.sleep(sleep_between)
        summary = self.metrics_logger.summarize()
        self.get_logger().info(
            f"Final Summary: Success {summary['success_rate']:.1f}% | "
            f"Plan avg {summary['plan_avg_ms']:.1f} ms | Exec avg {summary['exec_avg_ms']:.1f} ms"
        )

    def run_color_cycle(self, iterations: int) -> None:
        sleep_between = self.get_parameter("color_cycle.sleep_between_iterations_sec").get_parameter_value().double_value
        colors = list(self.get_parameter("color_cycle.colors").get_parameter_value().string_array_value)
        if not colors:
            self.get_logger().warning("color_cycle.colors is empty; skipping.")
            return

        for iteration in range(1, iterations + 1):
            self.get_logger().info(f"Color cycle iteration {iteration}/{iterations}: target=red")
            success = self.run_single_iteration(iteration)
            if not success:
                self.get_logger().warning("Color cycle iteration failed; skipping color shuffle.")
                self.recover_after_failure()
                time.sleep(sleep_between)
                continue

            self.move_home()
            self.reset_object_pose()

            if iteration >= iterations:
                time.sleep(sleep_between)
                continue

            if len(colors) >= len(self.box_slots):
                slot_colors = random.sample(colors, k=len(self.box_slots))
            else:
                slot_colors = list(colors)
                slot_colors.extend(random.choices(colors, k=len(self.box_slots) - len(slot_colors)))
            if "red" not in slot_colors:
                self.get_logger().warning("color_cycle.colors does not include red; forcing target to red.")
                slot_colors[0] = "red"

            red_entity = self.respawn_boxes(slot_colors)
            if red_entity is None:
                self.get_logger().warning("Failed to respawn boxes for color cycle.")
            else:
                self.attached_entity = red_entity
                if not self.set_target_color("red", update_entity=False):
                    self.get_logger().warning("Failed to set perception color to red.")
            time.sleep(sleep_between)

    def run_single_iteration(self, iteration_id: int) -> bool:
        if self.get_parameter("perception.enabled").get_parameter_value().bool_value:
            max_retries = int(
                self.get_parameter("recovery.max_retries").get_parameter_value().integer_value
            )
            recovery_enabled = self.get_parameter("recovery.enabled").get_parameter_value().bool_value
            for attempt in range(max_retries + 1):
                if self.wait_for_detected_pose() is None:
                    self.get_logger().warning(
                        "Perception enabled but no detected pose available; skipping motion."
                    )
                    if recovery_enabled and attempt < max_retries:
                        self.recover_after_failure()
                        continue
                    return False
                candidates = self.build_grasp_candidates()
                if not candidates:
                    self.get_logger().warning("Perception enabled but failed to build grasp candidates.")
                    if recovery_enabled and attempt < max_retries:
                        self.recover_after_failure()
                        continue
                    return False
                for index, grasp_target in enumerate(candidates, start=1):
                    stages = [
                        (f"cand{index}_grasp", lambda: self.plan_pose(grasp_target)),
                        (f"cand{index}_close_gripper", self.close_gripper),
                        (f"cand{index}_attach", self.attach_object),
                        (f"cand{index}_pre_place", lambda: self.plan_pose_stage("pre_place_pose")),
                        (f"cand{index}_place", lambda: self.plan_pose_stage("place_pose")),
                        (f"cand{index}_open_gripper", self.open_gripper),
                        (f"cand{index}_detach", self.detach_object),
                    ]

                    if self.run_stage_sequence(iteration_id, stages):
                        return True
                self.get_logger().warning("All grasp candidates failed.")
                if recovery_enabled and attempt < max_retries:
                    self.get_logger().warning("Recovering and retrying perception iteration.")
                    self.recover_after_failure()
                    continue
                return False
        else:
            stages = [
                ("home", self.move_home),
                ("reset_object", self.reset_object_pose),
                ("grasp", lambda: self.plan_pose_stage("grasp_pose")),
                ("close_gripper", self.close_gripper),
                ("attach", self.attach_object),
                ("pre_place", lambda: self.plan_pose_stage("pre_place_pose")),
                ("place", lambda: self.plan_pose_stage("place_pose")),
                ("open_gripper", self.open_gripper),
                ("detach", self.detach_object),
                ("retreat", lambda: self.plan_pose_stage("retreat_pose")),
            ]

        return self.run_stage_sequence(iteration_id, stages)

    def set_target_color(self, color: str, update_entity: bool = True) -> bool:
        color = color.lower()
        hsv_config = self.get_hsv_config(color)
        if hsv_config is None:
            self.get_logger().warning(f"Unsupported target color '{color}'.")
            return False

        if update_entity:
            self.attached_entity = f"object_box_{color}"
        if not self.perception_param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("Perception parameter service not available.")
            return False

        params = []
        for name, values in hsv_config.items():
            param = ParameterMsg()
            param.name = name
            param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=values)
            params.append(param)

        future = self.perception_param_client.call_async(SetParameters.Request(parameters=params))
        if not self.wait_for_future(future, timeout_sec=2.0):
            return False
        response = future.result()
        if response is None:
            return False
        return all(result.successful for result in response.results)

    def respawn_boxes(self, slot_colors: List[str]) -> Optional[str]:
        if len(slot_colors) != len(self.box_slots):
            self.get_logger().warning("slot_colors length mismatch.")
            return None
        if not self.spawn_entity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("/spawn_entity service not available.")
            return None
        if not self.delete_entity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("/delete_entity service not available.")
            return None

        for name, _ in self.box_slots:
            future = self.delete_entity_client.call_async(DeleteEntity.Request(name=name))
            self.wait_for_future(future, timeout_sec=2.0)

        red_entity = None
        for (name, position), color in zip(self.box_slots, slot_colors):
            model_xml = self.box_models.get(color)
            if not model_xml:
                self.get_logger().warning(f"Missing model XML for color '{color}'.")
                return None
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.w = 1.0
            request = SpawnEntity.Request()
            request.name = name
            request.xml = model_xml
            request.initial_pose = pose
            request.reference_frame = "world"
            future = self.spawn_entity_client.call_async(request)
            if not self.wait_for_future(future, timeout_sec=3.0):
                self.get_logger().warning(f"Spawn timed out for {name}.")
                return None
            if color == "red":
                red_entity = name
        return red_entity

    @staticmethod
    def get_hsv_config(color: str) -> Optional[dict]:
        if color == "red":
            return {
                "red_hsv_lower1": [0, 120, 70],
                "red_hsv_upper1": [10, 255, 255],
                "red_hsv_lower2": [170, 120, 70],
                "red_hsv_upper2": [180, 255, 255],
            }
        if color == "blue":
            return {
                "red_hsv_lower1": [100, 120, 50],
                "red_hsv_upper1": [130, 255, 255],
                "red_hsv_lower2": [100, 120, 50],
                "red_hsv_upper2": [130, 255, 255],
            }
        if color == "black":
            return {
                "red_hsv_lower1": [0, 0, 0],
                "red_hsv_upper1": [180, 255, 50],
                "red_hsv_lower2": [0, 0, 0],
                "red_hsv_upper2": [180, 255, 50],
            }
        return None

    def run_stage_sequence(self, iteration_id: int, stages: List[Tuple[str, callable]]) -> bool:
        for name, func in stages:
            ret = self.run_with_retries(name, func)
            if isinstance(ret, tuple) and len(ret) == 4:
                result, plan_time, exec_time, retries = ret
            else:
                result = StageResult.SUCCESS if ret else StageResult.EXEC_FAIL
                plan_time, exec_time, retries = 0.0, 0.0, 0

            self.metrics_logger.log(
                MetricsRow(
                    iteration_id=iteration_id,
                    stage_name=name,
                    plan_success=1 if result == StageResult.SUCCESS else 0,
                    exec_success=1 if result == StageResult.SUCCESS else 0,
                    plan_time_ms=plan_time,
                    exec_time_ms=exec_time,
                    retries_used=retries,
                    fail_reason=result.value if isinstance(result, StageResult) else str(result),
                    timestamp=time.time(),
                )
            )
            self.get_logger().info(
                f"Stage {name}: {result.value} | plan {plan_time:.0f} ms | exec {exec_time:.0f} ms | retries {retries}"
            )
            if result != StageResult.SUCCESS:
                self.get_logger().warning(f"Iteration {iteration_id} stage {name} failed: {result.value}")
                return False
        return True

    def run_with_retries(self, stage_name: str, func) -> Tuple[StageResult, float, float, int]:
        timeout = self.get_parameter("stress_test.per_stage_timeout_sec").get_parameter_value().double_value
        max_retries = int(
            self.get_parameter("stress_test.max_retries_per_stage").get_parameter_value().integer_value
        )
        for attempt in range(max_retries + 1):
            start_time = time.perf_counter()
            ret = func()
            elapsed = time.perf_counter() - start_time
            
            if isinstance(ret, tuple) and len(ret) == 3:
                result, plan_time, exec_time = ret
            elif isinstance(ret, bool): 
                result = StageResult.SUCCESS if ret else StageResult.EXEC_FAIL
                plan_time, exec_time = 0.0, 0.0
            else:
                result, plan_time, exec_time = StageResult.SUCCESS, 0.0, 0.0
            
            if elapsed > timeout:
                result = StageResult.TIMEOUT

            if result == StageResult.SUCCESS:
                return result, plan_time, exec_time, attempt
            self.get_logger().info(f"Retry {attempt + 1}/{max_retries} for stage {stage_name}")
            time.sleep(0.5)
            
        return result, plan_time, exec_time, max_retries

    def move_home(self) -> Tuple[StageResult, float, float]:
        joints = self.get_parameter("home_pose_joint_values").get_parameter_value().double_array_value
        return self.plan_joint_stage("arm", self.arm_joint_names, list(joints))

    def force_move_home(self) -> bool:
        if not self.arm_traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning("arm_controller follow_joint_trajectory action not available.")
            return False
        joints = list(self.get_parameter("startup.safe_joint_values").get_parameter_value().double_array_value)
        if len(joints) != len(self.arm_joint_names):
            self.get_logger().warning("startup.safe_joint_values length mismatch.")
            return False
        duration = float(self.get_parameter("startup.force_home_duration_sec").get_parameter_value().double_value)
        duration = max(0.5, duration)

        traj = JointTrajectory()
        traj.joint_names = list(self.arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        send_future = self.arm_traj_client.send_goal_async(goal)
        if not self.wait_for_future(send_future, timeout_sec=5.0):
            return False
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self.wait_for_future(result_future, timeout_sec=duration + 5.0):
            return False
        result = result_future.result()
        if result is None:
            return False
        if result.result.error_code != 0:
            self.get_logger().warning(f"Force home failed with error {result.result.error_code}")
            return False
        return True

    def plan_pose_stage(self, param_name: str) -> Tuple[StageResult, float, float]:
        pose = self.build_target_pose(param_name)
        return self.plan_pose(pose)

    def plan_pose(self, pose: Pose) -> Tuple[StageResult, float, float]:
        constraints = self.pose_constraints(pose)
        result = self.plan_and_execute("arm", constraints)
        if result[0] == StageResult.PLAN_FAIL:
            self.get_logger().warning(
                f"Plan failed for pose x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
            )
        return result

    def close_gripper(self) -> Tuple[StageResult, float, float]:
        return self.plan_joint_stage("gripper", self.gripper_joint_names, [0.0, 0.0])

    def open_gripper(self) -> Tuple[StageResult, float, float]:
        return self.plan_joint_stage("gripper", self.gripper_joint_names, [0.04, 0.04])

    def attach_object(self) -> Tuple[StageResult, float, float]:
        if self.get_parameter("perception.enabled").get_parameter_value().bool_value:
            target_pose = self.build_target_pose("grasp_pose")
            current_pose = self.get_current_ee_pose()
            if current_pose is None:
                self.get_logger().warning("Attach failed: missing end-effector pose.")
                return StageResult.ATTACH_FAIL, 0.0, 0.0
            threshold = self.get_parameter("perception.attach_distance_threshold").get_parameter_value().double_value
            dx = target_pose.position.x - current_pose.position.x
            dy = target_pose.position.y - current_pose.position.y
            dz = target_pose.position.z - current_pose.position.z
            distance = (dx * dx + dy * dy + dz * dz) ** 0.5
            if distance > threshold:
                self.get_logger().warning(
                    f"Attach failed: end-effector too far from target (dist={distance:.3f} > {threshold:.3f})."
                )
                return StageResult.ATTACH_FAIL, 0.0, 0.0
            entity_pose = self.get_entity_pose(self.attached_entity)
            if entity_pose is not None:
                ex = entity_pose.position.x - current_pose.position.x
                ey = entity_pose.position.y - current_pose.position.y
                ez = entity_pose.position.z - current_pose.position.z
                entity_distance = (ex * ex + ey * ey + ez * ez) ** 0.5
                if entity_distance > threshold:
                    self.get_logger().warning(
                        f"Attach failed: object too far from gripper (dist={entity_distance:.3f} > {threshold:.3f})."
                    )
                    return StageResult.ATTACH_FAIL, 0.0, 0.0
            else:
                self.get_logger().warning("Attach failed: unable to read object pose from Gazebo.")
                return StageResult.ATTACH_FAIL, 0.0, 0.0
        self.attach_active = True
        if self.get_parameter("perception.enabled").get_parameter_value().bool_value:
            self.follow_object()
            if not self.verify_attached():
                self.attach_active = False
                self.get_logger().warning("GRASP_FAIL: attachment verification failed.")
                return StageResult.ATTACH_FAIL, 0.0, 0.0
            self.get_logger().info("GRASP_SUCCESS: object attached to gripper.")
        return StageResult.SUCCESS, 0.0, 0.0

    def detach_object(self) -> Tuple[StageResult, float, float]:
        self.attach_active = False
        return StageResult.SUCCESS, 0.0, 0.0

    def plan_joint_stage(self, group_name: str, joint_names: List[str], positions: List[float]) -> Tuple[StageResult, float, float]:
        constraints = self.joint_constraints(joint_names, positions)
        return self.plan_and_execute(group_name, constraints)

    def plan_and_execute(self, group_name: str, constraints: Constraints) -> Tuple[StageResult, float, float]:
        start = time.perf_counter()
        plan_trajectory = self.plan_motion(group_name, constraints)
        plan_time = (time.perf_counter() - start) * 1000.0
        
        if plan_trajectory is None:
            return StageResult.PLAN_FAIL, plan_time, 0.0

        exec_start = time.perf_counter()
        exec_success = self.execute_trajectory(plan_trajectory)
        exec_time = (time.perf_counter() - exec_start) * 1000.0
        
        if not exec_success:
            return StageResult.EXEC_FAIL, plan_time, exec_time
        return StageResult.SUCCESS, plan_time, exec_time

    def plan_motion(self, group_name: str, constraints: Constraints):
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = group_name
        goal.request.goal_constraints = [constraints]
        goal.request.start_state = RobotState(is_diff=True)
        attempts = int(self.get_parameter("planning.num_attempts").get_parameter_value().integer_value)
        allowed_time = self.get_parameter("planning.allowed_time").get_parameter_value().double_value
        goal.request.num_planning_attempts = max(1, attempts)
        goal.request.allowed_planning_time = max(0.1, allowed_time)
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = self.move_group_client.send_goal_async(goal)
        try:
            goal_timeout = self.get_parameter("planning.goal_timeout_sec").get_parameter_value().double_value
            if not self.wait_for_future(send_future, timeout_sec=goal_timeout):
                raise TimeoutError("move_action goal timeout")
            goal_handle = send_future.result()
        except Exception as e:
            self.get_logger().error(f"Plan Goal rejected or timed out: {e}")
            return None

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Plan Goal rejected")
            return None
        
        result_future = goal_handle.get_result_async()
        try:
            result_timeout = self.get_parameter("planning.result_timeout_sec").get_parameter_value().double_value
            if not self.wait_for_future(result_future, timeout_sec=result_timeout):
                raise TimeoutError("move_action result timeout")
            res = result_future.result()
        except Exception as e:
            self.get_logger().error(f"Plan Result timed out: {e}")
            return None
            
        result = res.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            error_name = self.error_code_to_string(result.error_code.val)
            self.get_logger().error(
                f"Planning failed with error code: {result.error_code.val} ({error_name})"
            )
            return None
        return result.planned_trajectory

    def execute_trajectory(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_client.send_goal_async(goal)
        try:
            if not self.wait_for_future(send_future, timeout_sec=5.0):
                return False
            goal_handle = send_future.result()
        except Exception:
            return False

        if not goal_handle or not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        try:
            if not self.wait_for_future(result_future, timeout_sec=20.0):
                return False
            res = result_future.result()
        except Exception:
            return False
            
        result = res.result
        return result.error_code.val == MoveItErrorCodes.SUCCESS

    def pose_constraints(self, pose: Pose) -> Constraints:
        position = PositionConstraint()
        position.header.frame_id = self.planning_frame
        position.link_name = self.ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        return constraints

    def build_target_pose(self, param_name: str) -> Pose:
        values = self.get_parameter(param_name).get_parameter_value().double_array_value
        default_pose = self.pose_from_xyz_rpy(values)

        if not self.get_parameter("perception.enabled").get_parameter_value().bool_value:
            return default_pose

        detected_pose = self.get_fresh_detected_pose()
        if detected_pose is None:
            return default_pose

        offsets = {
            "pre_grasp_pose": self.get_parameter("perception.pre_grasp_offset").get_parameter_value().double_array_value,
            "grasp_pose": self.get_parameter("perception.grasp_offset").get_parameter_value().double_array_value,
            "lift_pose": self.get_parameter("perception.lift_offset").get_parameter_value().double_array_value,
        }
        if param_name not in offsets:
            return default_pose

        offset = offsets[param_name]
        target_pose = Pose()
        target_pose.position.x = detected_pose.position.x + offset[0]
        target_pose.position.y = detected_pose.position.y + offset[1]
        target_pose.position.z = detected_pose.position.z + offset[2]

        use_detected_orientation = self.get_parameter(
            "perception.use_detected_orientation"
        ).get_parameter_value().bool_value
        if use_detected_orientation:
            target_pose.orientation = detected_pose.orientation
        else:
            target_pose.orientation = default_pose.orientation
        return target_pose

    def build_grasp_candidates(self) -> List[Pose]:
        detected_pose = self.get_fresh_detected_pose()
        if detected_pose is None:
            return []

        default_values = self.get_parameter("grasp_pose").get_parameter_value().double_array_value
        base_roll, base_pitch, base_yaw = default_values[3], default_values[4], default_values[5]
        base_orientation = self.quaternion_from_rpy(base_roll, base_pitch, base_yaw)

        offsets = self.get_parameter("perception.grasp_candidate_offsets").get_parameter_value().double_array_value
        yaw_offsets = self.get_parameter("perception.grasp_candidate_yaws").get_parameter_value().double_array_value
        if len(offsets) % 3 != 0:
            self.get_logger().warning("perception.grasp_candidate_offsets length is not a multiple of 3.")
            return []

        use_detected_orientation = self.get_parameter(
            "perception.use_detected_orientation"
        ).get_parameter_value().bool_value

        candidates: List[Pose] = []
        for idx in range(0, len(offsets), 3):
            dx, dy, dz = offsets[idx], offsets[idx + 1], offsets[idx + 2]
            for yaw_delta in yaw_offsets:
                pose = Pose()
                pose.position.x = detected_pose.position.x + dx
                pose.position.y = detected_pose.position.y + dy
                pose.position.z = detected_pose.position.z + dz
                if use_detected_orientation:
                    pose.orientation = detected_pose.orientation
                else:
                    yaw = base_yaw + yaw_delta
                    pose.orientation = self.quaternion_from_rpy(base_roll, base_pitch, yaw)
                candidates.append(pose)
        return candidates

    def get_fresh_detected_pose(self) -> Optional[Pose]:
        if self.detected_pose is None:
            return None

        timeout_sec = self.get_parameter("perception.timeout_sec").get_parameter_value().double_value
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(self.detected_pose.header.stamp)
        if (now - stamp).nanoseconds > int(timeout_sec * 1e9):
            return None
        pose = self.detected_pose.pose
        if not self.is_pose_valid(pose):
            return None
        return pose

    def get_current_ee_pose(self) -> Optional[Pose]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception:
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def get_entity_pose(self, name: str) -> Optional[Pose]:
        if not self.get_entity_client.wait_for_service(timeout_sec=0.5):
            return None
        request = GetEntityState.Request()
        request.name = name
        future = self.get_entity_client.call_async(request)
        if not self.wait_for_future(future, timeout_sec=1.0):
            return None
        response = future.result()
        if response is None or not response.success:
            return None
        return response.state.pose

    def verify_attached(self) -> bool:
        current_pose = self.get_current_ee_pose()
        if current_pose is None:
            return False
        entity_pose = self.get_entity_pose(self.attached_entity)
        if entity_pose is None:
            return False
        offset = self.get_parameter("ee_to_object_offset").get_parameter_value().double_array_value
        expected_x = current_pose.position.x + offset[0]
        expected_y = current_pose.position.y + offset[1]
        expected_z = current_pose.position.z + offset[2]
        dx = entity_pose.position.x - expected_x
        dy = entity_pose.position.y - expected_y
        dz = entity_pose.position.z - expected_z
        threshold = self.get_parameter("perception.attach_distance_threshold").get_parameter_value().double_value
        return (dx * dx + dy * dy + dz * dz) ** 0.5 <= threshold

    def build_box_visualization_matrix(self, box_ids: List[str]) -> Optional[AllowedCollisionMatrix]:
        existing = self.get_allowed_collision_matrix()
        names = list(existing.entry_names) if existing and existing.entry_names else []
        if not names:
            names = list(self.robot_link_names)

        names.extend([box_id for box_id in box_ids if box_id not in names])
        size = len(names)
        matrix = AllowedCollisionMatrix()
        matrix.entry_names = names
        matrix.entry_values = [AllowedCollisionEntry(enabled=[False] * size) for _ in range(size)]

        index = {name: idx for idx, name in enumerate(names)}
        if existing and existing.entry_names:
            for row_idx, entry in enumerate(existing.entry_values):
                if row_idx >= len(existing.entry_names):
                    break
                row_name = existing.entry_names[row_idx]
                new_row_idx = index.get(row_name)
                if new_row_idx is None:
                    continue
                for col_idx, enabled in enumerate(entry.enabled):
                    if col_idx >= len(existing.entry_names):
                        break
                    col_name = existing.entry_names[col_idx]
                    new_col_idx = index.get(col_name)
                    if new_col_idx is None:
                        continue
                    matrix.entry_values[new_row_idx].enabled[new_col_idx] = enabled
        else:
            for link_a, link_b in self.self_collision_allow_pairs:
                idx_a = index.get(link_a)
                idx_b = index.get(link_b)
                if idx_a is None or idx_b is None:
                    continue
                matrix.entry_values[idx_a].enabled[idx_b] = True
                matrix.entry_values[idx_b].enabled[idx_a] = True

        for box_id in box_ids:
            box_idx = index.get(box_id)
            if box_idx is None:
                continue
            for link_name in self.robot_link_names:
                link_idx = index.get(link_name)
                if link_idx is None:
                    continue
                matrix.entry_values[box_idx].enabled[link_idx] = True
                matrix.entry_values[link_idx].enabled[box_idx] = True
        return matrix

    def get_allowed_collision_matrix(self) -> Optional[AllowedCollisionMatrix]:
        if not self.get_scene_client.wait_for_service(timeout_sec=1.0):
            return None
        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        future = self.get_scene_client.call_async(request)
        if not self.wait_for_future(future, timeout_sec=2.0):
            return None
        response = future.result()
        if response is None:
            return None
        return response.scene.allowed_collision_matrix

    @staticmethod
    def error_code_to_string(code: int) -> str:
        mapping = {
            MoveItErrorCodes.SUCCESS: "SUCCESS",
            MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "PLAN_INVALIDATED_BY_ENV",
            MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
            MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "SENSOR_DATA_FAILED",
            MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
            MoveItErrorCodes.PREEMPTED: "PREEMPTED",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_CONSTRAINTS",
            MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_CONSTRAINTS",
            MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
            MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
        }
        return mapping.get(code, "UNKNOWN")

    def is_pose_valid(self, pose: Pose) -> bool:
        bounds = self.get_parameter("perception.valid_bounds").get_parameter_value().double_array_value
        if len(bounds) != 6:
            return True
        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        if any(value != value for value in (x, y, z)):
            self.get_logger().warning("Perception pose invalid: NaN detected.")
            return False
        if not (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax):
            self.get_logger().warning(
                f"Perception pose out of bounds: x={x:.3f}, y={y:.3f}, z={z:.3f}."
            )
            return False
        return True

    def wait_for_detected_pose(self) -> Optional[Pose]:
        timeout = self.get_parameter("perception.wait_timeout_sec").get_parameter_value().double_value
        deadline = time.monotonic() + max(0.1, timeout)
        while time.monotonic() < deadline:
            pose = self.get_fresh_detected_pose()
            if pose is not None:
                return pose
            time.sleep(0.05)
        return None

    def recover_after_failure(self) -> None:
        self.detach_object()
        self.open_gripper()
        self.move_home()
        self.reset_object_pose()

    def apply_planning_profile(self) -> None:
        profile = self.get_parameter("planning.profile").get_parameter_value().string_value.lower()
        if profile not in ("fast", "stable"):
            self.get_logger().warning(f"Unknown planning.profile '{profile}', using current values.")
            return
        if profile == "fast":
            overrides = {
                "planning.allowed_time": 1.0,
                "planning.goal_timeout_sec": 1.5,
                "planning.result_timeout_sec": 2.0,
                "planning.num_attempts": 1,
            }
        else:
            overrides = {
                "planning.allowed_time": 2.5,
                "planning.goal_timeout_sec": 4.0,
                "planning.result_timeout_sec": 4.0,
                "planning.num_attempts": 1,
            }
        params = [RclpyParameter(name, value=value) for name, value in overrides.items()]
        self.set_parameters(params)
        self.get_logger().info(f"Planning profile applied: {profile}")

    @staticmethod
    def joint_constraints(joint_names: List[str], positions: List[float]) -> Constraints:
        constraints = Constraints()
        for name, pos in zip(joint_names, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = pos
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    @staticmethod
    def pose_from_xyz_rpy(values: List[float]) -> Pose:
        pose = Pose()
        pose.position.x = values[0]
        pose.position.y = values[1]
        pose.position.z = values[2]
        roll, pitch, yaw = values[3], values[4], values[5]
        pose.orientation = PickPlaceTask.quaternion_from_rpy(roll, pitch, yaw)
        return pose

    @staticmethod
    def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        quat = Pose().orientation
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat


def main() -> None:
    rclpy.init()
    node = PickPlaceTask()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
