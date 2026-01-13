import google.generativeai as genai
import os

# API 키 설정
GOOGLE_API_KEY = os.getenv('GOOGLE_API_KEY')

def main():
    genai.configure(api_key=GOOGLE_API_KEY)

    print("============ [내 API 키로 쓸 수 있는 모델 목록] ============")
    try:
        # 모델 리스트 가져오기
        for m in genai.list_models():
            if 'generateContent' in m.supported_generation_methods:
                print(f"- {m.name}")
    except Exception as e:
        print(f"에러 발생: {e}")
    print("==========================================================")

if __name__ == '__main__':
    main()