import csv
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from math import cos, sin
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from tf2_ros import Buffer, TransformListener
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander


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
            os.makedirs(directory, exist_ok=True)
        write_header = not self._header_written and not os.path.exists(self._csv_path)
        with open(self._csv_path, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            if write_header:
                writer.writerow([
                    "iteration_id",
                    "stage_name",
                    "plan_success",
                    "exec_success",
                    "plan_time_ms",
                    "exec_time_ms",
                    "retries_used",
                    "fail_reason",
                    "timestamp",
                ])
                self._header_written = True
            writer.writerow([
                row.iteration_id,
                row.stage_name,
                row.plan_success,
                row.exec_success,
                f"{row.plan_time_ms:.2f}",
                f"{row.exec_time_ms:.2f}",
                row.retries_used,
                row.fail_reason,
                f"{row.timestamp:.3f}",
            ])

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
        self.declare_parameter("pre_grasp_pose", [0.55, 0.0, 0.82, 3.14, 0.0, 1.57])
        self.declare_parameter("grasp_pose", [0.55, 0.0, 0.76, 3.14, 0.0, 1.57])
        self.declare_parameter("lift_pose", [0.55, 0.0, 0.90, 3.14, 0.0, 1.57])
        self.declare_parameter("pre_place_pose", [0.4, -0.25, 0.82, 3.14, 0.0, 1.57])
        self.declare_parameter("place_pose", [0.4, -0.25, 0.76, 3.14, 0.0, 1.57])
        self.declare_parameter("retreat_pose", [0.4, -0.25, 0.90, 3.14, 0.0, 1.57])
        self.declare_parameter("ee_to_object_offset", [0.0, 0.0, 0.08])
        self.declare_parameter("stress_test.enabled", False)
        self.declare_parameter("stress_test.iterations", 100)
        self.declare_parameter("stress_test.per_stage_timeout_sec", 12.0)
        self.declare_parameter("stress_test.max_retries_per_stage", 2)
        self.declare_parameter("stress_test.sleep_between_iterations_sec", 1.0)
        self.declare_parameter("metrics.log_csv", True)
        self.declare_parameter("metrics.csv_path", "/tmp/arm_pick_place_metrics.csv")
        self.declare_parameter("metrics.print_summary_every", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")

        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_num_planning_attempts(3)
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)

        self.gripper_group.set_planning_time(2.0)

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
        if not self.entity_client.service_is_ready():
            return
        try:
            transform = self.tf_buffer.lookup_transform("world", "ee_link", rclpy.time.Time())
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
        table = PoseStamped()
        table.header.frame_id = "world"
        table.pose.position.x = 0.5
        table.pose.position.y = 0.0
        table.pose.position.z = 0.35
        table.pose.orientation.w = 1.0

        box = PoseStamped()
        box.header.frame_id = "world"
        box.pose.position.x = 0.55
        box.pose.position.y = 0.0
        box.pose.position.z = 0.72
        box.pose.orientation.w = 1.0

        self.scene.add_box("table", table, size=(0.8, 0.6, 0.7))
        self.scene.add_box("object_box", box, size=(0.04, 0.04, 0.04))

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.done() and future.result() is not None

    def task_loop(self) -> None:
        time.sleep(2.0)
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
            result, plan_time, exec_time, retries = self.run_with_retries(name, func)
            self.metrics_logger.log(
                MetricsRow(
                    iteration_id=iteration_id,
                    stage_name=name,
                    plan_success=1 if result == StageResult.SUCCESS else 0,
                    exec_success=1 if result == StageResult.SUCCESS else 0,
                    plan_time_ms=plan_time,
                    exec_time_ms=exec_time,
                    retries_used=retries,
                    fail_reason=result.value,
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
            result, plan_time, exec_time = func()
            elapsed = time.perf_counter() - start_time
            if elapsed > timeout:
                result = StageResult.TIMEOUT
            if result == StageResult.SUCCESS:
                return result, plan_time, exec_time, attempt
            self.get_logger().info(f"Retry {attempt + 1}/{max_retries} for stage {stage_name}")
        return result, plan_time, exec_time, max_retries

    def move_home(self) -> Tuple[StageResult, float, float]:
        joints = self.get_parameter("home_pose_joint_values").get_parameter_value().double_array_value
        self.arm_group.set_joint_value_target(list(joints))
        return self.plan_and_execute(self.arm_group)

    def plan_pose_stage(self, param_name: str) -> Tuple[StageResult, float, float]:
        values = self.get_parameter(param_name).get_parameter_value().double_array_value
        pose = self.pose_from_xyz_rpy(values)
        self.arm_group.set_pose_target(pose)
        return self.plan_and_execute(self.arm_group)

    def close_gripper(self) -> Tuple[StageResult, float, float]:
        self.gripper_group.set_joint_value_target([0.0, 0.0])
        return self.plan_and_execute(self.gripper_group)

    def open_gripper(self) -> Tuple[StageResult, float, float]:
        self.gripper_group.set_joint_value_target([0.04, 0.04])
        return self.plan_and_execute(self.gripper_group)

    def attach_object(self) -> Tuple[StageResult, float, float]:
        self.attach_active = True
        return StageResult.SUCCESS, 0.0, 0.0

    def detach_object(self) -> Tuple[StageResult, float, float]:
        self.attach_active = False
        return StageResult.SUCCESS, 0.0, 0.0

    def plan_and_execute(self, group: MoveGroupCommander) -> Tuple[StageResult, float, float]:
        start = time.perf_counter()
        plan_result = group.plan()
        plan_time = (time.perf_counter() - start) * 1000.0

        plan_success, plan = self.extract_plan(plan_result)
        if not plan_success:
            group.clear_pose_targets()
            return StageResult.PLAN_FAIL, plan_time, 0.0

        exec_start = time.perf_counter()
        exec_success = group.execute(plan, wait=True)
        exec_time = (time.perf_counter() - exec_start) * 1000.0
        group.stop()
        group.clear_pose_targets()
        if not exec_success:
            return StageResult.EXEC_FAIL, plan_time, exec_time
        return StageResult.SUCCESS, plan_time, exec_time

    @staticmethod
    def extract_plan(plan_result) -> Tuple[bool, Optional[object]]:
        if isinstance(plan_result, tuple) and len(plan_result) >= 2:
            success = bool(plan_result[0])
            plan = plan_result[1]
            return success, plan
        if plan_result:
            return True, plan_result
        return False, None

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
    executor = rclpy.executors.MultiThreadedExecutor()
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
