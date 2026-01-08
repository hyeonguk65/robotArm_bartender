# 1. 베이스를 ROS 2 Humble Desktop으로 시작
FROM osrf/ros:humble-desktop

# 기본 쉘 설정
SHELL ["/bin/bash", "-c"]

# 2. 필수 시스템 도구 및 그래픽/통신 의존성 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    usbutils \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# 3. 파이썬 라이브러리 설치 (충돌 방지 최적화)

# 3-1. GPU용 PyTorch 설치 (CUDA 12.1 대응)
# 가장 무거운 패키지이므로 레이어 상단에 배치
RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# 3-2. YOLO 및 관련 라이브러리 설치 
# OpenCV 버전 제한(<4.10) 및 자동 업데이트 방지를 위해 lap 추가
RUN pip3 install --no-cache-dir ultralytics pyrealsense2 "opencv-python<4.10" lap

# 3-3. [가장 중요] NumPy 1.x대로 강제 고정
# 모든 라이브러리 설치 후 마지막에 실행하여 SciPy 호환성 보장
RUN pip3 install --force-reinstall --no-cache-dir "numpy<2.0"

# 4. 작업 디렉토리 설정
WORKDIR /root/robotArm_ws

# 5. 환경 설정 (자동 로드 및 로그 실시간 출력)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/robotArm_ws/install/setup.bash" >> ~/.bashrc

# 파이썬 로그가 버퍼링 없이 즉시 출력되도록 설정
ENV PYTHONUNBUFFERED=1

# 시작 위치 설정
WORKDIR /root/robotArm_ws
