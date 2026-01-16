---
marp: true
title: ros2_manipulation_pick_place
---
# ros2_manipulation_pick_place

**ROS2 Humble 기반 Pick & Place 자동화 포트폴리오**

- Gazebo + MoveIt2 + RViz2
- 실패/복구 전략 포함
- 성능 지표 자동 수집

---
# 문제 정의

- Gazebo 시뮬레이션에서 로봇팔이 물체를 집어 목표 위치에 놓기
- 충돌 회피, TF 정합, 실패 복구 포함

---
# 시스템 아키텍처

- gazebo_ros + ros2_control
- MoveIt2 planning_scene
- pick_place_task 상태 머신

---
# 상태 머신

HOME → PREGRASP → GRASP → LIFT → PLACE → RELEASE → HOME

- 재시도/타임아웃 정책 포함

---
# 성능 지표

- 성공률, 평균 수행 시간, 평균 플래닝 시간
- logs/metrics.csv 기록
- scripts/plot_metrics.py 로 그래프 생성

---
# 데모 실행

```bash
./scripts/run_demo.sh
```

---
# 다음 단계

- 실제 카메라 기반 객체 인식 연동
- 강화학습 기반 grasp pose 탐색
- 하드웨어 인더스트리얼 로봇 연동
