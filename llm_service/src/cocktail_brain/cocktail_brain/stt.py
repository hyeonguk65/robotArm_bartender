import speech_recognition as sr
import signal
import sys
import ctypes

# ALSA 레벨의 C 에러 메시지를 완벽하게 차단하기 위한 핸들러
ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

try:
    asound = ctypes.cdll.LoadLibrary('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
except OSError:
    pass

def speech_to_text(duration=5):
    recognizer = sr.Recognizer()
    
    # 기본 감도(에너지 임계값) 설정 (목소리가 작더라도 잡아내도록)
    recognizer.energy_threshold = 300
    recognizer.dynamic_energy_threshold = True

    # 마이크 객체 생성
    mic = sr.Microphone()
        
    print("🎤 말하세요...")
    
    with mic as source:
        # 노이즈 분석 시간 최소화 (0.5 -> 0.2초)
        recognizer.adjust_for_ambient_noise(source, duration=0.2)
        
        try:
            # timeout: 아무 말도 안할 때 기다리는 대기 시간 (5초)
            # phrase_time_limit: 제한 없음 (말이 끝날 때까지 다 듣기)
            audio = recognizer.listen(source, timeout=duration, phrase_time_limit=None)
        except sr.WaitTimeoutError:
            print("⚠️ 마이크 입력이 없습니다.")
            return ""

    try:
        # 구글 웹 STT API 사용
        text = recognizer.recognize_google(audio, language="ko-KR")
        return text
    except sr.UnknownValueError:
        print("⚠️ 음성을 인식하지 못했습니다.")
        return ""
    except sr.RequestError as e:
        print(f"⚠️ 구글 API 요청 에러: {e}")
        return ""

# 종료 신호 처리
def signal_handler(sig, frame):
    print("강제 종료 신호 감지! 마이크를 끕니다...")
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)
