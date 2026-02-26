# Branch Update: Finish building a sequence
> **Current Version:** `v1.2.0`
> **Current Branch:** `main`
> **Last Updated:** 2026.02.26

# Tech Stack
- ROS2 Humble / Python 3.10
- Google Gemini 2.5 (Reasoning)
- Doosan Robotics E0509 (Hardware)
- Docker (Environment)

## Current Status (v1.2.0 업데이트 내용)
- **제조 과정 속도 2배 향상**: Movel 전용 리니어 파라미터(VEL_LINEAR_FAST)를 제어 코드 전체에 적용하여 불필요한 딜레이를 제거하고 로봇 팔 직선 이동 속도를 2배 이상 끌어올렸습니다.
- **음료 제조 과정 실증 완료**: 진토닉 및 잭콕 등 실제 칵테일 음료를 펌핑, 쉐이킹, 컵 푸어링하는 전체 제조 공정에 대한 성공적인 실증 테스트를 완료했습니다.

## Previous Updates (v1.1.0)
- **음성 인식(STT) 인식률 50% 향상**: 오프라인 Whisper 모델에서 **Google Web Speech API** 스트리밍으로 전면 교체.
- **연속 청취(무한 듣기) 모드 적용**: ROS 2의 타이머 간섭 버그를 해결하고 백그라운드 스레드 방식을 도입하여 말을 더듬거나 잠시 고민해도 끊기지 않고 끝까지 경청하도록 개선.
- **콘솔 로그 최적화**: C 커널 레벨의 ALSA 마이크 에러 로그 완벽 억제 및 LLM 디버그 스팸 메시지 삭제로 깔끔한 대화창 UI 확보.
- **로봇 제어 안정화**: 제어 스포너 타임아웃 우회, 제조 공정 딜레이 안정화 (Shaking 3.0초, Dispenser 4.0초 대기).
- **제조 공정 One sequence** 완성: 음성 주문 -> 칵테일 쉐이킹 및 제조 -> 음료 전달.
- LLM을 통해 VLM로 테이블에 흘려진 물이나 칵테일을 닦아달라고 하면 닦아주는 기능 추가.

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

### VLM
docker exec -it vlm_container bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch gemini_vlm gemini_vlm.launch.py
```