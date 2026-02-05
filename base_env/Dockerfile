# 1. 베이스: ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

# 2. 필수 패키지 + RealSense 저장소 세팅
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release sudo git python3-pip python3-rosdep ros-dev-tools \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list \
    && rm -rf /var/lib/apt/lists/*

# Node.js 20 및 Codex CLI 설치
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get update && apt-get install -y --no-install-recommends nodejs \
    && npm install -g @openai/codex \
    && rm -rf /var/lib/apt/lists/*

# 3. RealSense SDK 및 ROS 2 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-utils librealsense2-dev ros-humble-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

# 4. 환경 설정 미리 등록 (핵심: 접속 시 자동으로 ROS 환경 로드)
# - 기본 ROS 환경 로드
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# - 내 워크스페이스(두산 로봇 패키지 등) 환경 로드
RUN echo "source /root/robotArm_ws/install/setup.bash" >> ~/.bashrc

# 5. Doosan 로봇 패키지 다운로드 (src 폴더 안에 넣기)
WORKDIR /root/robotArm_ws/src
RUN git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git

# 6. 의존성 설치 및 빌드
WORKDIR /root/robotArm_ws
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update && \
    apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

# 7. apt 패키지 설치 (pip 포함)
# 리눅스 패키지 목록 업데이트 후 pip 설치 -> 캐시 삭제(용량 최적화)
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 8. 파이썬 라이브러리 설치
# pyserial 뿐만 아니라 아까 코드에 있던 dynamixel_sdk도 같이 설치해두면 좋습니다!
RUN pip3 install pyserial dynamixel-sdk

# 시작 위치 고정
WORKDIR /root/robotArm_ws
