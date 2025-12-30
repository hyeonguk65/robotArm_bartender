# 1. 베이스: ROS 2 Humble Desktop (Ubuntu 22.04 jammy)
FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

# 2. 필수 패키지 + RealSense 저장소 세팅
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release sudo \
    git \
    python3-pip \
    python3-rosdep \
    ros-dev-tools \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list \
    && rm -rf /var/lib/apt/lists/*

# 3. RealSense SDK 및 ROS 2 패키지 설치
# - DKMS는 호스트 커널 모듈용이라 컨테이너에서는 제거 (형욱님 호스트에서 realsense-viewer OK)
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-utils \
    librealsense2-dev \
    ros-humble-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

# 4. Doosan 로봇 패키지 (드라이버) 미리 받아두기
WORKDIR /root/robotArm_ws/src
RUN git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git

# 5. rosdep 초기화(이미지 안에서 안전하게) + 의존성 설치 + 빌드
WORKDIR /root/robotArm_ws
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update && \
    apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

