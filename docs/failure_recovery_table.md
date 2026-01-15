# Failure / Recovery Scenarios

| Failure Type | Trigger Method | Detection Signal | Recovery Action | Success Criteria |
| --- | --- | --- | --- | --- |
| IK Failure | Unreachable pose in grasp stage | MoveIt planning fails | Increase Z offset, yaw jitter | Valid plan + exec success |
| Collision | Place obstacle on approach path | Planning scene collision | Replan with higher approach | No collision in plan |
| Timeout | Set planning timeout too low | Planning timeout | Increase timeout, speed down | Planning success |
| Goal Not Reached | Tight goal tolerance | Execution error > threshold | Reduce speed, adjust goal | Pose error within threshold |
| Repeated Failure | N retries exceeded | Retry counter | Fail-safe stop | Robot in safe pose |
