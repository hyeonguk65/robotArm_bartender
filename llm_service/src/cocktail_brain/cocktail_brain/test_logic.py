from stt import speech_to_text
from gemini_handler import ask_gemini 

def main():
    # 1. 듣기 (STT)
    user_text = speech_to_text(duration=5)

    if not user_text:
        print("⚠️ 인식된 음성이 없습니다.")
        return

    print(f"\n🙋 사용자 질문: {user_text}")

    # 2. 생각하기 (Gemini)
    answer = ask_gemini(user_text)
    
    print(f"\n🤖 Gemini 답변:\n{answer}")

if __name__ == "__main__":
    main()
