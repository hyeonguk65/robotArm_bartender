import sounddevice as sd
import numpy as np
import whisper
import tempfile
import scipy.io.wavfile as wav
import signal
import sys

model = whisper.load_model("small")


def speech_to_text(duration=5):
    device_id = 13  # 마이크(Realtek Audio), MME
    fs = 48000
    channels = 2

    print("🎤 말하세요...")

    audio = sd.rec(
        int(duration * fs), samplerate=fs, channels=1, dtype="float32"
    )
    sd.wait()

    max_volume = np.max(np.abs(audio))
    print(f"🔊 max volume: {max_volume}")

    if max_volume < 0.001:
        print("⚠️ 마이크 입력이 거의 없습니다.")
        return ""

    audio_mono = np.mean(audio, axis=1)

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        wav.write(f.name, fs, audio_mono)
        result = model.transcribe(f.name, language="ko")

    text = result["text"].strip()
    print("📝 인식된 텍스트:", text)
    return text


# 종료 신호가 오면 실행될 함수 (유언장)
def signal_handler(sig, frame):
    print("강제 종료 신호 감지! 마이크를 내려놓습니다...")
    # 여기에 sd.stop() 같은 마이크 정지 코드 추가
    sys.exit(0)


# 신호 등록 (Docker가 끄라고 할 때 signal_handler를 실행해라)
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)
