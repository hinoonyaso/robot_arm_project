#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
from statistics import mean

import matplotlib.pyplot as plt


def load_metrics(path: Path):
    rows = []
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            rows.append({
                "timestamp": row.get("timestamp"),
                "episode_id": int(row.get("episode_id", 0)),
                "planning_time": float(row.get("planning_time", 0.0)),
                "execution_time": float(row.get("execution_time", 0.0)),
                "success": row.get("success", "false").lower() == "true",
                "fail_reason": row.get("fail_reason", ""),
            })
    return rows


def plot_metrics(rows, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)

    episode_ids = [row["episode_id"] for row in rows]
    planning_times = [row["planning_time"] for row in rows]
    execution_times = [row["execution_time"] for row in rows]
    success_flags = [1 if row["success"] else 0 for row in rows]

    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(episode_ids, planning_times, label="Planning Time (s)")
    plt.plot(episode_ids, execution_times, label="Execution Time (s)")
    plt.ylabel("Time (s)")
    plt.legend(loc="upper right")

    plt.subplot(2, 1, 2)
    plt.plot(episode_ids, success_flags, label="Success (1/0)")
    plt.xlabel("Episode")
    plt.ylabel("Success")
    plt.ylim(-0.1, 1.1)
    plt.legend(loc="upper right")

    plt.tight_layout()
    output_path = output_dir / "metrics.png"
    plt.savefig(output_path)
    print(f"Saved: {output_path}")

    success_rate = (sum(success_flags) / len(success_flags)) * 100 if success_flags else 0
    avg_plan = mean(planning_times) if planning_times else 0
    avg_exec = mean(execution_times) if execution_times else 0
    print(f"Success Rate: {success_rate:.1f}%")
    print(f"Avg Planning Time: {avg_plan:.3f}s")
    print(f"Avg Execution Time: {avg_exec:.3f}s")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=Path("logs/metrics.csv"))
    parser.add_argument("--output", type=Path, default=Path("docs/metrics"))
    args = parser.parse_args()

    if not args.input.exists():
        raise SystemExit(f"Input CSV not found: {args.input}")

    rows = load_metrics(args.input)
    plot_metrics(rows, args.output)


if __name__ == "__main__":
    main()
