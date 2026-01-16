# 실패/복구 전략

| 단계 | 실패 유형 | 감지 방식 | 복구 전략 | 재시도 횟수 | 비고 |
| --- | --- | --- | --- | --- | --- |
| PREGRASP | Planning 실패 | MoveIt2 plan 결과 | 현재 목표 유지, 계획 재시도 | 3 | 3회 실패 시 HOME 복귀 |
| GRASP | Cartesian 경로 실패 | fraction < threshold | approach 거리 감소 후 재시도 | 2 | 실패 시 HOME |
| LIFT | 충돌/경로 실패 | plan 실패/충돌 감지 | lift 거리 감소, 높이 변경 | 2 | 실패 시 HOME |
| PLACE | Planning 실패 | plan 실패 | 목표 위치 offset 조정 후 재시도 | 3 | 실패 시 HOME |
| RELEASE | detach 실패 | planning_scene 응답 실패 | detach 재시도 후 HOME | 2 | 실패 시 HOME |
| 전체 | 타임아웃 | 상태 전이 시간 초과 | HOME 복귀 후 상태 리셋 | 1 | 로그 기록 |
