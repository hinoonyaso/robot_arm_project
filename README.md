# Fail-Aware Manipulation: 실패 조건 인지 및 복구 전략을 포함한 Pick & Place 시스템

## 프로젝트 한 줄 요약
MoveIt2 기반 Pick & Place 작업에서 **실패 조건(IK/충돌/타임아웃/목표 미도달)**을 인지하고, FSM이 **재계획/목표 미세조정/속도 저감**으로 복구하는 시스템입니다.

## Step 0 사전 점검 결과
> 이 리포지토리 환경에서는 ROS2/Gazebo/MoveIt2가 설치되어 있지 않습니다.

| 항목 | 결과 | 확인 명령 |
| --- | --- | --- |
| ROS2 | 미설치 | `ros2 --version` |
| Gazebo | 미설치 | `gazebo --version` |
| MoveIt2 | 미설치 | `dpkg -l | grep -i moveit` |

## 시스템 구성
시스템 다이어그램은 `assets/diagrams/system_architecture.svg`에 생성하도록 구성했습니다.

## FSM 설계
상태 머신 다이어그램은 `assets/diagrams/state_machine.svg`에 생성하도록 구성했습니다.

## 실패/복구 시나리오
- 상세 표: [docs/failure_recovery_table.md](docs/failure_recovery_table.md)

## 성능 지표 예시 (샘플 데이터)
그래프는 `scripts/make_plots.py` 실행 시 `assets/plots/`에 생성됩니다.

## 실행 방법
### 1) 설치
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

Gazebo 설치 여부에 따라 아래 중 하나를 선택:
```bash
sudo apt install -y gazebo
# 또는
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

### 2) 워크스페이스 준비/빌드
```bash
mkdir -p colcon_ws/src
cp -r src/fail_aware_task_manager colcon_ws/src/
cp -r src/fail_aware_bringup colcon_ws/src/
cd colcon_ws
colcon build --symlink-install
source install/setup.bash
```

### 3) 실행
```bash
ros2 launch fail_aware_bringup sim.launch.py
```

### 4) 실험/로그
```bash
python3 scripts/run_experiments.py --runs 30 --out data/runs --simulate
python3 scripts/make_plots.py --run_dir data/runs --out assets/plots
```

## 로그 구조
- `data/runs/run_*/events.jsonl`
- `data/runs/run_*/summary.json`

## 문서
- 시스템 구조: [docs/architecture.md](docs/architecture.md)
- 상태 머신: [docs/state_machine.md](docs/state_machine.md)
- 실패/복구 표: [docs/failure_recovery_table.md](docs/failure_recovery_table.md)
- 실험 프로토콜: [docs/experiment_protocol.md](docs/experiment_protocol.md)

## 영상
Gazebo + RViz 데모 영상은 `assets/videos/`에 저장하도록 설계되어 있습니다.
- `assets/videos/demo_success.mp4`
- `assets/videos/demo_failure_recovery.mp4`

> 현재 환경에는 ROS2/Gazebo가 없어 영상이 생성되지 않았습니다.

## PPT
`ppt/Fail-Aware_Manipulation_Portfolio.pptx`는 아래 스크립트로 생성하도록 구성했습니다.
```bash
python3 -m pip install --user python-pptx
python3 scripts/generate_pptx.py
```

## 트러블슈팅
- **MoveIt 플래닝 실패**: planning time 증가, goal tolerance 완화
- **Gazebo 컨트롤러 문제**: `ros2 control list_controllers` 확인 후 활성화
- **TF 오류**: `ros2 run tf2_tools view_frames`로 TF tree 점검

## 배운 점 / 다음 확장
- 실패 조건을 명시적으로 분류하면 복구 정책의 우선순위 설계가 쉬워집니다.
- 다음 확장: 카메라 기반 pose 추정, 실제 하드웨어 적용, AMR 연계
