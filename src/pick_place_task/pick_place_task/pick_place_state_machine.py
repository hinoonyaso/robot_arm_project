import csv
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from moveit2_py.move_group_interface import MoveGroupInterface
from moveit2_py.planning_scene_interface import PlanningSceneInterface


class PickPlaceState(Enum):
    HOME = "HOME"
    PREGRASP = "PREGRASP"
    GRASP = "GRASP"
    LIFT = "LIFT"
    PLACE = "PLACE"
    RELEASE = "RELEASE"


@dataclass
class MetricsRecord:
    timestamp: str
    episode_id: int
    planning_time: float
    execution_time: float
    success: bool
    fail_reason: str


class PickPlaceStateMachine(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_state_machine")

        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("ee_link", "tool0")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("object_frame", "object")
        self.declare_parameter("home_joint_positions", [0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("timeout_s", 8.0)
        self.declare_parameter("metrics_path", "logs/metrics.csv")

        self.planning_group = self.get_parameter("planning_group").get_parameter_value().string_value
        self.ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.object_frame = self.get_parameter("object_frame").get_parameter_value().string_value
        self.home_joint_positions = self.get_parameter("home_joint_positions").get_parameter_value().double_array_value
        self.max_retries = self.get_parameter("max_retries").get_parameter_value().integer_value
        self.timeout_s = self.get_parameter("timeout_s").get_parameter_value().double_value
        self.metrics_path = Path(self.get_parameter("metrics_path").get_parameter_value().string_value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_group = MoveGroupInterface(
            node=self,
            joint_model_group_name=self.planning_group,
        )
        self.scene = PlanningSceneInterface(node=self)

        self.episode_id = 0
        self.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_metrics_file()
        self._apply_static_scene()

    def _init_metrics_file(self) -> None:
        if not self.metrics_path.exists():
            with self.metrics_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow([
                    "timestamp",
                    "episode_id",
                    "planning_time",
                    "execution_time",
                    "success",
                    "fail_reason",
                ])

    def _apply_static_scene(self) -> None:
        self.get_logger().info("Applying planning scene collision objects...")
        self.scene.remove_collision_object("table")
        self.scene.remove_collision_object("obstacle")

        table_pose = PoseStamped()
        table_pose.header.frame_id = self.base_frame
        table_pose.pose.position.x = 0.6
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.02
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box("table", table_pose, size=[1.2, 0.8, 0.04])

        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = self.base_frame
        obstacle_pose.pose.position.x = 0.4
        obstacle_pose.pose.position.y = -0.2
        obstacle_pose.pose.position.z = 0.2
        obstacle_pose.pose.orientation.w = 1.0
        self.scene.add_box("obstacle", obstacle_pose, size=[0.1, 0.1, 0.4])

    def run_episode(self) -> None:
        self.episode_id += 1
        state = PickPlaceState.HOME
        success = True
        fail_reason = ""
        planning_time = 0.0
        execution_time = 0.0

        try:
            for next_state in [
                PickPlaceState.HOME,
                PickPlaceState.PREGRASP,
                PickPlaceState.GRASP,
                PickPlaceState.LIFT,
                PickPlaceState.PLACE,
                PickPlaceState.RELEASE,
                PickPlaceState.HOME,
            ]:
                state = next_state
                self.get_logger().info(f"State: {state.value}")
                plan_time, exec_time = self._execute_state(state)
                planning_time += plan_time
                execution_time += exec_time
        except RuntimeError as exc:
            success = False
            fail_reason = str(exc)
            self.get_logger().error(f"Episode failed: {fail_reason}")
            self._return_home()

        self._record_metrics(
            MetricsRecord(
                timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                episode_id=self.episode_id,
                planning_time=planning_time,
                execution_time=execution_time,
                success=success,
                fail_reason=fail_reason,
            )
        )

    def _execute_state(self, state: PickPlaceState) -> tuple[float, float]:
        if state == PickPlaceState.HOME:
            return self._move_to_joint_goal(self.home_joint_positions)
        if state == PickPlaceState.PREGRASP:
            pose = self._lookup_target_pose(offset_z=0.15)
            return self._plan_execute_pose(pose)
        if state == PickPlaceState.GRASP:
            pose = self._lookup_target_pose(offset_z=0.02)
            plan_time, exec_time = self._cartesian_move(pose)
            self._attach_object()
            return plan_time, exec_time
        if state == PickPlaceState.LIFT:
            pose = self._lookup_target_pose(offset_z=0.25)
            return self._plan_execute_pose(pose)
        if state == PickPlaceState.PLACE:
            pose = self._get_place_pose()
            return self._plan_execute_pose(pose)
        if state == PickPlaceState.RELEASE:
            pose = self._get_place_pose(offset_z=0.05)
            plan_time, exec_time = self._cartesian_move(pose)
            self._detach_object()
            return plan_time, exec_time

        raise RuntimeError(f"Unknown state: {state}")

    def _plan_execute_pose(self, target_pose: PoseStamped) -> tuple[float, float]:
        for attempt in range(self.max_retries):
            plan_start = time.time()
            plan = self.move_group.plan_to_pose(target_pose, self.ee_link)
            plan_time = time.time() - plan_start
            if not plan:
                self.get_logger().warn(f"Plan attempt {attempt + 1} failed")
                continue
            exec_start = time.time()
            executed = self.move_group.execute(plan)
            exec_time = time.time() - exec_start
            if executed:
                return plan_time, exec_time
        raise RuntimeError("planning_failed")

    def _move_to_joint_goal(self, joint_positions) -> tuple[float, float]:
        for attempt in range(self.max_retries):
            plan_start = time.time()
            plan = self.move_group.plan_to_joint_configuration(joint_positions)
            plan_time = time.time() - plan_start
            if not plan:
                self.get_logger().warn(f"Joint plan attempt {attempt + 1} failed")
                continue
            exec_start = time.time()
            executed = self.move_group.execute(plan)
            exec_time = time.time() - exec_start
            if executed:
                return plan_time, exec_time
        raise RuntimeError("joint_planning_failed")

    def _cartesian_move(self, target_pose: PoseStamped) -> tuple[float, float]:
        plan_start = time.time()
        trajectory, fraction = self.move_group.compute_cartesian_path(
            [target_pose],
            max_step=0.01,
            jump_threshold=0.0,
        )
        plan_time = time.time() - plan_start
        if fraction < 0.9:
            raise RuntimeError("cartesian_path_failed")
        exec_start = time.time()
        executed = self.move_group.execute(trajectory)
        exec_time = time.time() - exec_start
        if not executed:
            raise RuntimeError("cartesian_execution_failed")
        return plan_time, exec_time

    def _lookup_target_pose(self, offset_z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = offset_z
        pose.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.object_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.timeout_s),
            )
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z + offset_z
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed, using default pose: {exc}")

        return pose

    def _get_place_pose(self, offset_z: float = 0.1) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.3
        pose.pose.position.z = offset_z
        pose.pose.orientation.w = 1.0
        return pose

    def _attach_object(self) -> None:
        self.scene.attach_box(
            self.ee_link,
            "object",
            PoseStamped(header=self._header()),
            size=[0.04, 0.04, 0.12],
        )

    def _detach_object(self) -> None:
        self.scene.remove_attached_object(self.ee_link, name="object")

    def _return_home(self) -> None:
        try:
            self._move_to_joint_goal(self.home_joint_positions)
        except RuntimeError:
            self.get_logger().warn("Failed to return home")

    def _record_metrics(self, record: MetricsRecord) -> None:
        with self.metrics_path.open("a", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow([
                record.timestamp,
                record.episode_id,
                f"{record.planning_time:.3f}",
                f"{record.execution_time:.3f}",
                str(record.success).lower(),
                record.fail_reason,
            ])

    def _header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.ee_link
        return header


def main() -> None:
    rclpy.init()
    node = PickPlaceStateMachine()
    node.get_logger().info("Pick & Place state machine started")
    node.run_episode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
