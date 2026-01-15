#!/usr/bin/env python3
"""Run multiple fail-aware manipulation trials.

This script supports two modes:
1) --simulate: Generates synthetic events for plotting without ROS2.
2) --ros: Launches ROS2 bringup and logs to the run directory.
"""

import argparse
import json
import random
import subprocess
import time
from pathlib import Path

from parse_metrics import summarize_run

FAILURE_TYPES = ["IK", "COLLISION", "TIMEOUT", "GOAL_NOT_REACHED", "UNKNOWN"]
RECOVERY_ACTIONS = ["Z_OFFSET", "YAW_JITTER", "TIMEOUT_UP", "SPEED_DOWN"]


def write_event(handle, payload):
    handle.write(json.dumps(payload) + "\n")


def simulate_run(run_dir: Path, max_steps: int = 12) -> None:
    events_path = run_dir / "events.jsonl"
    with events_path.open("w", encoding="utf-8") as handle:
        write_event(handle, {"timestamp": time.time(), "event": "state_enter", "state": "PLAN_APPROACH"})
        planning_time = random.randint(80, 260)
        write_event(handle, {"timestamp": time.time(), "event": "plan_success", "planning_time_ms": planning_time})
        if random.random() < 0.75:
            exec_time = random.randint(300, 700)
            write_event(handle, {"timestamp": time.time(), "event": "exec_success", "exec_time_ms": exec_time})
        else:
            failure_type = random.choice(FAILURE_TYPES)
            write_event(handle, {"timestamp": time.time(), "event": "plan_fail", "failure_type": failure_type})
            action = random.choice(RECOVERY_ACTIONS)
            write_event(handle, {"timestamp": time.time(), "event": "recovery_action", "action": action, "retry_count": 1})
            exec_time = random.randint(280, 900)
            write_event(handle, {"timestamp": time.time(), "event": "exec_success", "exec_time_ms": exec_time})
        max_joint_vel = round(random.uniform(1.2, 2.4), 2)
        write_event(handle, {"timestamp": time.time(), "event": "metric", "max_joint_vel": max_joint_vel})
        write_event(handle, {"timestamp": time.time(), "event": "state_enter", "state": "SUCCESS"})


def run_ros_trial(run_dir: Path, launch_cmd: str) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["bash", "-lc", f"{launch_cmd} run_id:={run_dir.name} log_dir:={run_dir.parent}"]
    subprocess.run(cmd, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--out", type=str, default="data/runs")
    parser.add_argument("--simulate", action="store_true", help="Generate synthetic data without ROS")
    parser.add_argument("--launch", type=str, default="ros2 launch fail_aware_bringup sim.launch.py")
    args = parser.parse_args()

    base_dir = Path(args.out)
    base_dir.mkdir(parents=True, exist_ok=True)

    for idx in range(args.runs):
        run_dir = base_dir / time.strftime(f"run_%Y%m%d_%H%M%S_{idx:02d}")
        run_dir.mkdir(parents=True, exist_ok=True)
        if args.simulate:
            simulate_run(run_dir)
        else:
            run_ros_trial(run_dir, args.launch)
        summarize_run(run_dir)
        time.sleep(0.05)


if __name__ == "__main__":
    main()
