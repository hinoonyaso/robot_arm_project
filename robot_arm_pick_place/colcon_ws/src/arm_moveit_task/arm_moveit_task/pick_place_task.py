#!/usr/bin/env python3
import csv
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from math import cos, sin
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


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
        self.declare_parameter("pre_grasp_pose", [0.55, 0.0, 0.82, 3.14, 0.0, 0.0])
        self.declare_parameter("grasp_pose", [0.55, 0.0, 0.76, 3.14, 0.0, 0.0])
        self.declare_parameter("lift_pose", [0.55, 0.0, 0.90, 3.14, 0.0, 0.0])
        self.declare_parameter("pre_place_pose", [0.4, -0.25, 0.82, 3.14, 0.0, 0.0])
        self.declare_parameter("place_pose", [0.4, -0.25, 0.76, 3.14, 0.0, 0.0])
        self.declare_parameter("retreat_pose", [0.4, -0.25, 0.90, 3.14, 0.0, 0.0])
        self.declare_parameter("ee_to_object_offset", [0.0, 0.0, 0.08])
        self.declare_parameter("stress_test.enabled", False)
        self.declare_parameter("stress_test.iterations", 100)
        self.declare_parameter("stress_test.per_stage_timeout_sec", 12.0)
        self.declare_parameter("stress_test.max_retries_per_stage", 2)
        self.declare_parameter("stress_test.sleep_between_iterations_sec", 1.0)
        self.declare_parameter("metrics.log_csv", True)
        self.declare_parameter("metrics.csv_path", "/tmp/arm_pick_place_metrics.csv")
        self.declare_parameter("metrics.print_summary_every", 10)

        self.ee_link = "ee_link"
        self.planning_frame = "world"
        self.arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = ["finger_left_joint", "finger_right_joint"]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_group_client = ActionClient(self, MoveGroup, "/move_group")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        self.attach_service = self.create_service(Trigger, "/attach", self.handle_attach)
        self.detach_service = self.create_service(Trigger, "/detach", self.handle_detach)

        self.entity_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.attach_active = False
        self.attached_entity = "object_box"
        self.follow_timer = self.create_timer(1.0 / 30.0, self.follow_object)

        self.metrics_logger = MetricsLogger(
            self,
            self.get_parameter("metrics.csv_path").get_parameter_value().string_value,
            self.get_parameter("metrics.log_csv").get_parameter_value().bool_value,
        )

        self.task_thread = threading.Thread(target=self.task_loop, daemon=True)
        self.task_thread.start()

    def handle_attach(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.attach_active = True
        response.success = True
        response.message = "Attached object follow enabled"
        return response

    def handle_detach(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.attach_active = False
        response.success = True
        response.message = "Detached object follow disabled"
        return response

    def follow_object(self) -> None:
        if not self.attach_active:
            return
        try:
            transform = self.tf_buffer.lookup_transform("world", self.ee_link, rclpy.time.Time())
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

    def add_collision_objects(self) -> None:
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("/apply_planning_scene service not available")
            return

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

        box = CollisionObject()
        box.id = "object_box"
        box.header.frame_id = self.planning_frame
        box.operation = CollisionObject.ADD
        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [0.04, 0.04, 0.04]
        box_pose = Pose()
        box_pose.position.x = 0.55
        box_pose.position.y = 0.0
        box_pose.position.z = 0.72
        box_pose.orientation.w = 1.0
        box.primitives = [box_primitive]
        box.primitive_poses = [box_pose]

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [table, box]

        future = self.apply_scene_client.call_async(request)
        try:
            future.result(timeout=2.0)
            self.get_logger().info("Collision objects added.")
        except Exception as exc:
            self.get_logger().warning(f"Failed to add collision objects: {exc}")

    def reset_object_pose(self) -> bool:
        if not self.entity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("/gazebo/set_entity_state service not available")
            return False
        state = EntityState()
        state.name = self.attached_entity
        state.pose.position.x = 0.55
        state.pose.position.y = 0.0
        state.pose.position.z = 0.72
        state.pose.orientation.w = 1.0

        future = self.entity_client.call_async(SetEntityState.Request(state=state))
        try:
            res = future.result(timeout=2.0)
            return res is not None and res.success
        except Exception:
            return False

    def task_loop(self) -> None:
        self.get_logger().info("Task Thread Started. Waiting for system ready...")
        time.sleep(2.0)
        self.wait_for_action(self.move_group_client, "/move_group")
        self.wait_for_action(self.execute_client, "/execute_trajectory")

        self.add_collision_objects()

        if not self.get_parameter("enable_task").get_parameter_value().bool_value:
            self.get_logger().info("Task execution disabled; services only mode.")
            return

        stress_enabled = self.get_parameter("stress_test.enabled").get_parameter_value().bool_value
        iterations = int(self.get_parameter("stress_test.iterations").get_parameter_value().integer_value)
        if stress_enabled:
            self.run_stress_test(iterations)
        else:
            self.run_single_iteration(1)

    def wait_for_action(self, client: ActionClient, name: str) -> None:
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for Action server {name} to be available...")
        self.get_logger().info(f"Action server {name} connected.")

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

    def run_single_iteration(self, iteration_id: int) -> None:
        stages = [
            ("home", self.move_home),
            ("reset_object", self.reset_object_pose),
            ("pre_grasp", lambda: self.plan_pose_stage("pre_grasp_pose")),
            ("grasp", lambda: self.plan_pose_stage("grasp_pose")),
            ("close_gripper", self.close_gripper),
            ("attach", self.attach_object),
            ("lift", lambda: self.plan_pose_stage("lift_pose")),
            ("pre_place", lambda: self.plan_pose_stage("pre_place_pose")),
            ("place", lambda: self.plan_pose_stage("place_pose")),
            ("open_gripper", self.open_gripper),
            ("detach", self.detach_object),
            ("retreat", lambda: self.plan_pose_stage("retreat_pose")),
        ]

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
            if result != StageResult.SUCCESS:
                self.get_logger().warning(f"Iteration {iteration_id} stage {name} failed: {result.value}")
                break

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

    def plan_pose_stage(self, param_name: str) -> Tuple[StageResult, float, float]:
        values = self.get_parameter(param_name).get_parameter_value().double_array_value
        pose = self.pose_from_xyz_rpy(values)
        constraints = self.pose_constraints(pose)
        return self.plan_and_execute("arm", constraints)

    def close_gripper(self) -> Tuple[StageResult, float, float]:
        return self.plan_joint_stage("gripper", self.gripper_joint_names, [0.0, 0.0])

    def open_gripper(self) -> Tuple[StageResult, float, float]:
        return self.plan_joint_stage("gripper", self.gripper_joint_names, [0.04, 0.04])

    def attach_object(self) -> Tuple[StageResult, float, float]:
        self.attach_active = True
        return StageResult.SUCCESS, 0.0, 0.0

    def detach_object(self) -> Tuple[StageResult, float, float]:
        self.attach_active = False
        return StageResult.SUCCESS, 0.0, 0.0

    def plan_joint_stage(
        self, group_name: str, joint_names: List[str], positions: List[float]
    ) -> Tuple[StageResult, float, float]:
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
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = self.move_group_client.send_goal_async(goal)

        try:
            goal_handle = send_future.result(timeout=10.0)
        except Exception as exc:
            self.get_logger().error(f"Plan Goal rejected or timed out: {exc}")
            return None

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Plan Goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        try:
            res = result_future.result(timeout=10.0)
        except Exception as exc:
            self.get_logger().error(f"Plan Result timed out: {exc}")
            return None

        result = res.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Planning failed with error code: {result.error_code.val}")
            return None
        return result.planned_trajectory

    def execute_trajectory(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_client.send_goal_async(goal)

        try:
            goal_handle = send_future.result(timeout=5.0)
        except Exception:
            return False

        if not goal_handle or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        try:
            res = result_future.result(timeout=20.0)
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
        sphere.dimensions = [0.01]
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.planning_frame
        orientation.link_name = self.ee_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.1
        orientation.absolute_y_axis_tolerance = 0.1
        orientation.absolute_z_axis_tolerance = 0.1
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        return constraints

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
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
            pitch / 2
        ) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
            pitch / 2
        ) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
            pitch / 2
        ) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
            pitch / 2
        ) * sin(yaw / 2)
        quat = Pose().orientation
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat


def main() -> None:
    rclpy.init()
    node = PickPlaceTask()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
