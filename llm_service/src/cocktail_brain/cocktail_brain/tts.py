from gtts import gTTS
import os

def speak(text):
    try:
        # 1. 구글 번역기 목소리로 음성 파일 생성 (언어: 한국어)
        tts = gTTS(text=text, lang='ko')
        
        # 2. 파일로 저장 (로컬 경로로 수정)
        filename = '/tmp/voice_temp.mp3'
        tts.save(filename)
        
        # 3. 재생 (mpg123 플레이어 사용)
        # -q : 조용히(로그 없이) 재생
        os.system(f'mpg123 -q {filename}')

    except Exception as e:
        print(f"TTS 오류 발생: {e}")