#!/usr/bin/env python3
import csv
import statistics
import sys
from collections import Counter, defaultdict


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as csvfile:
        reader = csv.DictReader(csvfile)
        return list(reader)


def summarize(rows):
    stage_plan = defaultdict(list)
    stage_exec = defaultdict(list)
    failures = Counter()
    for row in rows:
        stage = row["stage_name"]
        if row["plan_success"] == "1":
            stage_plan[stage].append(float(row["plan_time_ms"]))
        if row["exec_success"] == "1":
            stage_exec[stage].append(float(row["exec_time_ms"]))
        if row["fail_reason"] != "SUCCESS":
            failures[row["fail_reason"]] += 1

    print("Stage Averages (ms)")
    print("stage,plan_avg,exec_avg")
    for stage in sorted(set(stage_plan) | set(stage_exec)):
        plan_avg = statistics.mean(stage_plan[stage]) if stage_plan[stage] else 0.0
        exec_avg = statistics.mean(stage_exec[stage]) if stage_exec[stage] else 0.0
        print(f"{stage},{plan_avg:.2f},{exec_avg:.2f}")

    print("\nFailure Reasons")
    for reason, count in failures.most_common():
        print(f"{reason}: {count}")


def main():
    if len(sys.argv) < 2:
        print("Usage: parse_metrics.py /path/to/metrics.csv")
        sys.exit(1)
    rows = load_rows(sys.argv[1])
    if not rows:
        print("No rows found")
        return
    summarize(rows)


if __name__ == "__main__":
    main()
