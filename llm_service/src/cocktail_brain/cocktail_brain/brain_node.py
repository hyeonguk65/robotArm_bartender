import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# stt.py, tts.py, gemini_handler.py 가져오기
try:
    from . import gemini_handler
    from . import stt
    from . import tts
except ImportError:
    # ROS2 패키지 구조가 아닌 일반 실행일 경우를 대비한 예외처리
    import gemini_handler
    import stt
    import tts

# 바텐더 로봇의 행동 지침 (프롬프트)
BARTENDER_PROMPT = """
[SYSTEM INSTRUCTION]
당신은 로봇 바텐더입니다. 손님의 말을 듣고 상황에 맞는 칵테일을 추천해주세요.
반드시 아래의 JSON 형식으로만 답변해야 하며,
다른 사족(마크다운, 인사말 등)은 절대 붙이지 마세요.

가능한 칵테일: [Gin Tonic, Martini, Whiskey Sour, Orange Juice, Jack & Coke]

출력 형식 예시:
{
  "reason": "힘든 하루를 보내신 것 같아 상큼한 진토닉을 추천해 드립니다.",
  "cocktail": "Gin Tonic",
  "action_code": "make_gin_tonic"
}
"""


class CocktailBrain(Node):
    def __init__(self):
        super().__init__("cocktail_brain_node")

        # 1. 퍼블리셔 (척수): 로봇 팔에게 명령 전달
        self.publisher_ = self.create_publisher(String, "robot_order", 10)

        self.get_logger().info("🍸 칵테일 바텐더 뇌(Brain)가 깨어났습니다!")

        # 2. 주기적 실행 (타이머)
        # STT와 TTS가 시간을 잡아먹으므로 타이머 주기는 좀 넉넉하게 잡거나,
        # 루프가 끝나면 다시 호출되는 방식을 고려해야 하지만,
        # 일단 1.0초로 설정하고 내부 로직이 끝날 때까지
        # 블로킹(대기)되는 구조로 갑니다.
        self.timer = self.create_timer(1.0, self.listen_and_think)

    def listen_and_think(self):
        # --- [Step 1] 듣기 (STT) ---
        # stt.py의 speech_to_text 함수 사용 (기본 5초 듣기)
        # 로그 너무 많으면 지저분하니 생략 가능
        # self.get_logger().info("👂 듣는 중...")

        user_text = stt.speech_to_text(duration=5)

        # 말이 없거나 너무 작으면 패스
        if not user_text:
            return

        self.get_logger().info(f'🙋 손님: "{user_text}"')

        # --- [Step 2] 생각하기 (Gemini) ---
        # gemini_handler는 "친절한 비서" 설정이므로,
        # 질문 앞에 "바텐더 프롬프트"를 붙여서 보냅니다.
        full_query = f"{BARTENDER_PROMPT}\n손님: {user_text}"

        ai_response = gemini_handler.ask_gemini(full_query)

        # JSON 포맷팅 정리 (가끔 ```json ... ``` 이렇게 줄 때가 있음)
        clean_json = ai_response.replace("```json", "").replace("```", "").strip()

        self.get_logger().info(f"🤖 제미나이 생각: {clean_json}")

        # --- [Step 3] 말하기 (TTS) & 명령 내리기 ---
        try:
            # 문자열을 진짜 JSON 객체로 변환
            order_data = json.loads(clean_json)

            reason = order_data.get("reason", "알겠습니다.")
            cocktail = order_data.get("cocktail", "Water")

            # 1. 손님에게 말해주기 (TTS)
            self.get_logger().info(f'🗣️ 로봇: "{reason}"')
            # 여기서 로봇이 말을 합니다!
            tts.speak(reason)

            # 2. 로봇 팔에게 명령 보내기 (Publish)
            msg = String()
            msg.data = clean_json
            self.publisher_.publish(msg)
            self.get_logger().info(f">> 🦾 명령 전송 완료: {cocktail}")

        except json.JSONDecodeError:
            self.get_logger().error("Gemini가 JSON이 아닌 이상한 말을 했습니다.")
            tts.speak("죄송해요, 제가 잠시 딴생각을 했네요. " "다시 말씀해 주시겠어요?")
        except Exception as e:
            self.get_logger().error(f"처리 중 에러 발생: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CocktailBrain()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청 받음")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
