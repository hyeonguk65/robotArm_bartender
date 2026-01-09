# Branch Update: Integrate LLM with robot arm using ROS communication(WIP)
> **Current Branch:** `feature/robotarm`
> **Last Updated:** 2026.1.9

# Tech Stack
- ROS2 Humble / Python 3.10
- Google Gemini 2.0 Flash Lite (Reasoning)
- Doosan Robotics E0509 (Hardware)
- Docker (Environment)

## Current Status
현재 **LLM 두뇌(Brain)**와 **음성 인식(STT)** 파트는 구현 완료되었으며, **로봇 팔 제어(Controller)**와의 ROS2 토픽 통신 안정화 작업을 진행 중입니다.

## To-Do List
- [x] LLM(Gemini) 프롬프트 엔지니어링 및 JSON 출력 제어
- [x] Docker 컨테이너(llm - base) 간 통신 환경 구축
- [ ] yolo 컨테이너 간 통신 환경 구축
- [ ] 로봇 팔의 '진토닉', '잭콕' 제조 모션 웨이포인트(Waypoint) 최적화
- [ ] 예외 처리 로직 (주문 취소, 인식 불가 등) 강화