import sounddevice as sd

def print_devices():
    print("🎤 현재 연결된 오디오 장치 목록:")
    print(sd.query_devices())

if __name__ == "__main__":
    try:
        print_devices()
        print("\n✅ 오디오 라이브러리가 정상 작동합니다!")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        print("도커가 마이크를 찾지 못했습니다. docker-compose 설정을 확인해야 합니다.")

print_devices()
