# gemini_vlm_Doosan-robotic-arm 실행 및 과정 설명

colcon build --packages-select gemini_vlm  
source install/setup.bash 

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.38 port:=12345 model:=e0509



source install/setup.bash 
ros2 launch gemini_vlm gemini_vlm.launch.py
--------------------------------------------------------------------------------------------------------------

ros2 launch gemini_vlm gemini_vlm.launch.py 실행 시 아래 3개 노드가 동시에 시작됩니다.

  1. gemini_vlm_node (executable: vlm_node)

  - 역할: 카메라 프레임 기반 타겟 인식(VLM)
  - 주요 발행 토픽:
      - target_towel_point (geometry_msgs/PointStamped)
      - target_water_point (geometry_msgs/PointStamped)
      - target_tissue_point (구버전 호환용 legacy 토픽)
  - 주요 구독 토픽:
      - /wipe_sequence_active (std_msgs/Bool)

  2. dsr01/gripper_service_node (executable: gripper_service_node)

  - 역할: 그리퍼 열기/닫기 서비스 제공
  - 네임스페이스: /dsr01
  - 제공 서비스:
      - /dsr01/set_gripper_stroke (std_srvs/SetBool)
      - True = CLOSE, False = OPEN

  3. dsr01/robot_main_controller (executable: robot_control)

  - 역할: 전체 로봇 시퀀스 오케스트레이션
  - 네임스페이스: /dsr01
  - 주요 구독 토픽:
      - /target_towel_point 또는 target_towel_point
      - /target_water_point 또는 target_water_point
      - /target_tissue_point (legacy 호환)
  - 주요 발행 토픽:
      - /wipe_sequence_active (std_msgs/Bool)

  -------------------------------------------------------------------------------------------

  ## 시스템 동작 순서 (End-to-End)

  1. vlm_node가 이미지에서 수건/물 좌표를 추정하여 토픽으로 발행
  2. robot_main_controller가 좌표를 수신하면 시퀀스 시작
  3. 시퀀스 중 /wipe_sequence_active=True를 발행하여 인식 좌표 업데이트를 잠금
  4. 로봇이 수건 집기 -> 물 닦기 -> 수건 원위치 작업 수행
  5. 완료 후 /wipe_sequence_active=False 발행, 다음 사이클 좌표 탐색 재개
