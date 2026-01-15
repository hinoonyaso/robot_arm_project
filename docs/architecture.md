# System Architecture

## Overview
The system is split into four layers:
1. **Simulation + Visualization**: Gazebo for physics and RViz2 for MoveIt2 planning and execution visualization.
2. **Motion Planning**: MoveIt2 handles IK, collision checking, trajectory planning, and execution.
3. **Task Manager FSM**: A ROS2 C++ node that sequences pick & place states and applies recovery policies.
4. **Metrics & Automation**: Python scripts log JSONL events and compute summary metrics/plots.

## Data Flow
- Task manager triggers planning/execution and receives success/fail feedback.
- Every state transition and failure/recovery action is logged to `events.jsonl`.
- `parse_metrics.py` aggregates events into `summary.json`.
- `make_plots.py` renders graphs for the portfolio report.
