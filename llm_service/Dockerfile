# 1. 베이스 이미지: ROS2 Humble이 이미 설치된 우분투
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# 2. 시스템 패키지 설치 (한 번에 모아서 설치)
# - portaudio19-dev: 마이크 필수 드라이버
# - alsa-utils, pulseaudio-utils: 리눅스 사운드 유틸리티
# - ffmpeg: Whisper 오디오 변환용
# - mpg123: gTTS 목소리 재생용
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    portaudio19-dev \
    alsa-utils \
    ffmpeg \
    mpg123 \
    libasound2-plugins \
    pulseaudio-utils \
    libportaudio2 \
    && rm -rf /var/lib/apt/lists/*

# 3. 파이썬 라이브러리 설치 (여기에 gTTS 포함!)
# - google-generativeai: 제미나이
# - openai-whisper: 듣기 (STT)
# - gTTS: 말하기 (TTS)
# - sounddevice, pyaudio: 마이크/스피커 제어
RUN pip3 install \
    "google-generativeai>=0.8.3" \
    openai-whisper \
    gTTS \
    SpeechRecognition \
    pyaudio \
    sounddevice \
    "numpy<1.25.0"\
    scipy \
    setuptools==58.2.0

# 4. 워크스페이스 설정 
WORKDIR /root/robotArm_ws

# 5. 소스 코드 복사 
# COPY src/ src/

# 6. 빌드 
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 7. 환경 설정 자동화
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /robotArm_ws/install/setup.bash" >> ~/.bashrc

# [오디오 설정] 파이썬이 호스트의 PulseAudio를 사용하도록 강제
RUN echo 'pcm.!default { type pulse hint.description "Default Audio Device" } ctl.!default { type pulse }' > /etc/asound.conf

# 8. 실행 명령
CMD ["bash"]