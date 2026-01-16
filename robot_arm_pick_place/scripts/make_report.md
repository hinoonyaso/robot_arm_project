# Pick & Place Stress Test Report Template

## Summary
- Iterations: {{iterations}}
- Success Rate: {{success_rate}}%
- Avg Planning Time: {{plan_avg_ms}} ms
- Avg Execution Time: {{exec_avg_ms}} ms

## Failure Breakdown (Top 3)
1. {{fail_reason_1}}: {{fail_count_1}}
2. {{fail_reason_2}}: {{fail_count_2}}
3. {{fail_reason_3}}: {{fail_count_3}}

## Stage Timing Table (ms)
| Stage | Plan Avg | Exec Avg |
|------|---------:|---------:|
| home | {{home_plan_avg}} | {{home_exec_avg}} |
| pre_grasp | {{pre_grasp_plan_avg}} | {{pre_grasp_exec_avg}} |
| grasp | {{grasp_plan_avg}} | {{grasp_exec_avg}} |
| lift | {{lift_plan_avg}} | {{lift_exec_avg}} |
| pre_place | {{pre_place_plan_avg}} | {{pre_place_exec_avg}} |
| place | {{place_plan_avg}} | {{place_exec_avg}} |
| retreat | {{retreat_plan_avg}} | {{retreat_exec_avg}} |
