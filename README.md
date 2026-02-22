# Branch Update: Finish building a sequence
> **Current Branch:** `feature/develop`
> **Last Updated:** 2026.2.23

# Tech Stack
- ROS2 Humble / Python 3.10
- Google Gemini 2.5 (Reasoning)
- Doosan Robotics E0509 (Hardware)
- Docker (Environment)

## Current Status
**음성 인식(STT)** 80% 향상, **제조 공정 One sequence** 완성.
음성을 통한 주문 -> 칵테일 쉐이킹 및 제조 -> 음료 전달
LLM을 통해 VLA로 테이블에 흘려진 물이나 칵테일을 닦아달라고 하면 닦아주는 기능 추가

## 설치 및 실행 (Installation & Usage)
```bash
## 저장소 복제
git clone [https://github.com/scienceking2/robotarm_bartender.git](https://github.com/scienceking2/robotarm_bartender.git)

## 환경 설정 및 실행
cd robotarm_bartender
docker compose up -d --build

## 각 Container 진입

### LLM
docker exec -it llm_container bash
colcon build --symlink-install
source install/local_setup.bash
ros2 run cocktail_brain brain_node

### Base
#### Arduino
docker exec -it base_container bash
colcon build --symlink-install
source install/local_setup.bash
ros2 run doosan_control arduino_gripper_controller
#### Robot Controller
docker exec -it base_container bash
colcon build --symlink-install
source install/local_setup.bash
ros2 run doosan_control main_orchestrator
#### Robot Bringup
docker exec -it base_container bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.38 port:=12345 model:=e0509

### Yolo
docker exec -it yolo_container bash
colcon build --symlink-install
source install/local_setup.bash
ros2 run yolo_service yolo_visualizer

### VLA
docker exec -it vla_container bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch gemini_vla gemini_vla.launch.py
```