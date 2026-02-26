FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# 1. Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    git \
    python3-rosdep \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    "google-generativeai>=0.8.3" \
    "numpy<1.25.0" \
    pillow \
    opencv-python

# 2. Clone Doosan Robot Package (for dsr_msgs2)
WORKDIR /root/robotArm_ws/src
RUN git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git

# 3. Build workspace
WORKDIR /root/robotArm_ws
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update && \
    apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y && \
    colcon build --symlink-install --packages-select dsr_msgs2 && \
    rm -rf /var/lib/apt/lists/*

# 4. Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/robotArm_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
