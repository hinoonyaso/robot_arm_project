# State Machine Design

## States
- IDLE
- SPAWN_OR_SELECT_OBJECT
- PLAN_APPROACH
- EXEC_APPROACH
- PLAN_GRASP
- EXEC_GRASP
- ATTACH
- PLAN_PLACE
- EXEC_PLACE
- DETACH
- SUCCESS
- RECOVERY
- FAIL_SAFE

## Key Transitions
- **Planning Fail N times → RECOVERY**
- **Collision detected → RECOVERY**
- **Goal not reached → RECOVERY**
- **Timeout → RECOVERY**
- **Retry limit exceeded → FAIL_SAFE**

## Recovery Policies
- Increase approach height (Z offset)
- Jitter goal yaw
- Increase planning timeout
- Reduce speed/acceleration scale
