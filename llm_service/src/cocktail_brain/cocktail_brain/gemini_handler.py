import json
import os

import google.generativeai as genai

# 1. 설정 구역 (Gemini API)
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

if not GOOGLE_API_KEY:
    print("⚠️ 경고: GOOGLE_API_KEY 환경변수가 없습니다.")

genai.configure(api_key=GOOGLE_API_KEY)

SYSTEM_INSTRUCTION = "너는 한국어로 친절하게 답하는 AI 비서야."

MODEL_NAME = "gemini-2.0-flash-lite"
model = genai.GenerativeModel(
    model_name=MODEL_NAME, system_instruction=SYSTEM_INSTRUCTION
)


def ask_gemini(user_text: str) -> str:
    """사용자의 텍스트를 받아 Gemini의 답변을 반환하는 함수."""
    try:
        # Gemini에게 질문 던지기
        response = model.generate_content(user_text)

        # 답변 텍스트 추출
        if response.text:
            return response.text.strip()
        else:
            # 답변 거부 시 JSON 반환
            return json.dumps(
                {
                    "reason": "답변하기 곤란한 내용이에요.",  # TTS용
                    "cocktail": None,  # 로봇에게 보낼 데이터
                    "action_code": "error",
                },
                ensure_ascii=False,
            )

    except Exception as e:
        print(f"Gemini 호출 중 에러 발생: {e}")
        error_msg = "죄송해요, 지금은 생각할 수가 없어요."

        if "429" in str(e):
            error_msg = "오늘 대화 에너지를 다 썼어요. 잠시 쉬어야 해요."

        return json.dumps(
            {
                "reason": error_msg,
                "cocktail": None,
                "action_code": "error",
            },
            ensure_ascii=False,
        )
