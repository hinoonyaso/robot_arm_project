#!/usr/bin/env python3
"""Parse events.jsonl into summary.json for a single run."""

import argparse
import json
from collections import Counter
from pathlib import Path
from statistics import mean, median


def summarize_run(run_dir: Path) -> dict:
    events_path = run_dir / "events.jsonl"
    if not events_path.exists():
        raise FileNotFoundError(f"Missing {events_path}")

    planning_times = []
    failure_types = Counter()
    recovery_actions = Counter()
    exec_success = 0
    exec_total = 0
    max_joint_vel = []

    with events_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            payload = json.loads(line)
            event = payload.get("event")
            if event == "plan_success":
                planning_times.append(float(payload.get("planning_time_ms", 0)))
            elif event == "plan_fail":
                failure_types[payload.get("failure_type", "UNKNOWN")] += 1
            elif event == "recovery_action":
                recovery_actions[payload.get("action", "UNKNOWN")] += 1
            elif event == "exec_success":
                exec_success += 1
                exec_total += 1
            elif event == "exec_fail":
                exec_total += 1
            elif event == "metric" and "max_joint_vel" in payload:
                max_joint_vel.append(float(payload["max_joint_vel"]))

    summary = {
        "planning_time_ms_mean": mean(planning_times) if planning_times else 0,
        "planning_time_ms_median": median(planning_times) if planning_times else 0,
        "failure_type_counts": dict(failure_types),
        "recovery_action_counts": dict(recovery_actions),
        "recovery_success_rate": exec_success / exec_total if exec_total else 0,
        "max_joint_vel": max(max_joint_vel) if max_joint_vel else 0,
        "events": events_path.name,
    }

    summary_path = run_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_dir", type=str, required=True)
    args = parser.parse_args()
    summarize_run(Path(args.run_dir))


if __name__ == "__main__":
    main()
