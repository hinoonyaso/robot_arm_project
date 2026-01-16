# 기술 의사결정 기록 (ADR 요약)

1. **Gazebo Classic + gazebo_ros 사용**
   - ROS2 Humble과의 호환성을 고려.
2. **MoveIt2 기반 플래닝**
   - RViz2와 planning_scene 통합이 용이.
3. **Python(a ment_python) 중심 구현**
   - 포트폴리오 목적상 빠른 프로토타이핑과 가독성 중시.
4. **텍스트 기반 다이어그램/슬라이드**
   - 바이너리 파일 금지 제약에 대응.
5. **메트릭 CSV + Python plot 스크립트**
   - 성능 지표를 재현 가능하게 유지.
