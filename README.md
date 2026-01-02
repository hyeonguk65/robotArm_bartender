# Branch Update: LLM Service & Refactoring
> **Current Branch:** `feature/llm`
> **Last Updated:** 2025.12.31

## 작업 목적 (Objective)
* **구조 개선:** 중구난방이던 파일 디렉토리 구조를 체계적으로 재정리 (Refactoring)
* **기능 강화:** Google Gemini API 연동 로직을 고도화하여 칵테일 추천 정확도 향상
* **보안 강화:** API Key 노출 방지를 위한 환경 변수(`.env`) 설정 적용

## 주요 변경 사항 (Changelog)

### 1. 파일 구조 재배치 (Refactoring)
기존 `STT_gemini` 폴더를 `cocktail_brain` 패키지로 통합 및 이동하였습니다.
* **Before:** `llm_service/src/cocktail_brain/STT_gemini/*.py`
* **After:** `llm_service/src/cocktail_brain/cocktail_brain/*.py`
> *이유: 기능별로 흩어져 있던 모듈을 하나의 'Brain' 패키지로 묶어 관리 효율을 높임.*

### 2. Gemini Handler 업데이트 (`gemini_handler.py`)
* 칵테일 레시피 생성 프롬프트를 구체화 (사용자 기분 -> 레시피 매핑 로직 개선)
* 에러 발생 시 재시도(Retry) 로직 추가

### 3. 보안 설정 (`.gitignore`)
* `.env` 파일을 추적 목록에서 제외하여 API Key 유출 사고 방지

### 4. docker-compose.yml (llm_service 코드 추가)
 services:
  base_control:

        .
        .
        .

 ----- 위는 변동 사항 없음 ----- 
 
  llm_service:
    build: ./llm_service
    container_name: llm_container
    network_mode: host
    privileged: true
    stdin_open: true 
    tty: true
    
    # key 원본 연결
    env_file:
      - .env

    #[수정됨] 경로를 명확하게 고정했습니다.
    environment:
      - PULSE_SERVER=unix:/run/user/1000/pulse/native
      - PULSE_COOKIE=/root/.config/pulse/cookie

    volumes:
      - ./llm_service/src:/root/robotArm_ws/src/user_pkgs
      
      # 1. 오디오 소켓 (기존 유지)
      - /run/user/1000/pulse:/run/user/1000/pulse
      # 2. 인증 쿠키 (기존 유지하되, 환경변수와 경로 일치시킴)
      - ~/.config/pulse/cookie:/root/.config/pulse/cookie      
      # 3. [추가됨] 머신 ID (지문 인식 - 이게 핵심!)
      - /etc/machine-id:/etc/machine-id:ro