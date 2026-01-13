import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 패키지 임포트 예외처리
try:
    from . import gemini_handler
    from . import stt
    from . import tts
except ImportError:
    import gemini_handler
    import stt
    import tts


BARTENDER_PROMPT = """
[SYSTEM INSTRUCTION]
당신은 로봇 바텐더입니다. 손님의 말을 듣고 JSON으로만 응답하세요.

[행동 지침]
1. **주문 접수**: 손님이 칵테일 이름을 말하거나,
   **발음이 비슷하면(예: '김토니' -> 'Gin Tonic', '잭코' -> 'Jack & Coke')**
   절대 되묻지 말고 **즉시 주문으로 확정**하세요.
   -> `action_code`="make_cocktail", `cocktail`="정확한 영어 메뉴명", `ice_size`="small|medium|large"
2. **얼음 크기**: 주문일 때는 반드시 얼음 크기를 포함하세요.
   - 손님이 말하지 않으면 기본값은 `medium`입니다.
3. **일상 대화**: 주문이 아니라면 친절하게 대답하세요.
   -> `action_code`="chat", `cocktail`=null, `ice_size`=null

가능한 칵테일: [Gin Tonic, Jack & Coke]

[출력 예시]
{"reason": "네, 시원한 진토닉 바로 만들어 드리겠습니다.",
 "cocktail": "Gin Tonic", "ice_size": "medium", "action_code": "make_cocktail"}
{"reason": "안녕하세요! 오늘 기분은 어떠신가요?",
 "cocktail": null, "ice_size": null, "action_code": "chat"}
"""


class CocktailBrain(Node):
    def __init__(self):
        super().__init__("cocktail_brain_node")

        # 1. 로봇 팔 명령 (입)
        self.pub_cocktail = self.create_publisher(String, "/robot_order_cocktail", 10)
        self.pub_ice = self.create_publisher(String, "/robot_order_ice_size", 10)

        # 2. 로봇 상태 수신 (귀)
        self.status_sub = self.create_subscription(
            String, "/robot_status", self.robot_status_callback, 10
        )

        self.get_logger().info("🍸 칵테일 바텐더 뇌(Brain) 가동 - 빠른 응답 모드")

        # [상태 변수]
        # 로봇이 제조 중인가?
        self.waiting_for_robot = False

        self.timer = self.create_timer(1.0, self.listen_and_think)

    def robot_status_callback(self, msg):
        """로봇이 'DONE' 신호를 보내면 실행."""
        if msg.data == "DONE":
            self.get_logger().info("🤖 로봇: 제조 완료")
            tts.speak(
                "칵테일이 완성되었습니다. 맛있게 드세요. " "다음 주문 말씀해주세요."
            )
            self.waiting_for_robot = False

    def listen_and_think(self):
        # 1. 로봇이 일하는 중이면 듣지 않음
        if self.waiting_for_robot:
            return

        # --- [Step 1] 듣기 ---
        user_text = stt.speech_to_text(duration=5)

        # 잡음 처리 (너무 짧으면 무시)
        if not user_text or len(user_text.strip()) < 2:
            return

        self.get_logger().info(f'🙋 손님: "{user_text}"')

        # --- [Step 2] 생각하기 (Context 주입 로직 삭제됨) ---
        full_query = f"{BARTENDER_PROMPT}\n손님: {user_text}"  # noqa: E501

        # 제미나이에게 질문
        ai_response = gemini_handler.ask_gemini(full_query)
        clean_json = ai_response.replace("```json", "").replace("```", "").strip()
        self.get_logger().info(f"🤖 생각: {clean_json}")

        # --- [Step 3] 행동 결정 ---
        try:
            data = json.loads(clean_json)
            reason = data.get("reason", "")
            cocktail = data.get("cocktail", "")
            ice_size = data.get("ice_size", "")
            action = data.get("action_code", "unknown")

            # 1. 안내 멘트 (TTS)
            self.get_logger().info(f'🗣️ 로봇 말: "{reason}"')
            tts.speak(reason)

            # 2. 행동 처리
            if action == "make_cocktail":
                # 되묻기 없이 바로 로봇에게 명령 전송
                self.send_order_to_robot(cocktail, ice_size)
            else:
                # chat 또는 에러 상황 등 -> 아무 행동 안 함 (TTS만 하고 끝)
                pass

        except Exception as e:
            self.get_logger().error(f"처리 오류: {e}")
            tts.speak("죄송해요, 오류가 났어요.")

    def send_order_to_robot(self, cocktail_name, ice_size):
        """로봇에게 최종 명령 전송."""
        if not cocktail_name:
            return
        if not ice_size:
            ice_size = "medium"

        msg_cocktail = String()
        msg_cocktail.data = str(cocktail_name)
        self.pub_cocktail.publish(msg_cocktail)

        msg_ice = String()
        msg_ice.data = str(ice_size)
        self.pub_ice.publish(msg_ice)

        self.get_logger().info(
            f">> 🦾 로봇에게 확정 명령 전송: {cocktail_name} (ice={ice_size})"
        )
        # 로봇 대기 모드 진입
        self.waiting_for_robot = True


def main(args=None):
    rclpy.init(args=args)
    node = CocktailBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
