import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import cv2  # 이미지 저장용
from sensor_msgs.msg import Image  # 메시지 타입
from cv_bridge import CvBridge  # 변환기

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
3. **물 청소 요청**: 손님이 "물 닦아줘", "테이블이 젖었어" 등 물을 닦아달라고 요청하면 청소를 지시하세요.
   -> `action_code`="wipe_water", `cocktail`=null, `ice_size`=null
4. **일상 대화**: 주문이나 부탁이 아니라면 친절하게 대답하세요.
   -> `action_code`="chat", `cocktail`=null, `ice_size`=null

가능한 칵테일: [Gin Tonic, Jack & Coke, Mojito]

[출력 예시]
{"reason": "네, 시원한 진토닉 바로 만들어 드리겠습니다.",
 "cocktail": "Gin Tonic", "ice_size": "medium", "action_code": "make_cocktail"}
{"reason": "네, 테이블의 물기를 바로 닦아드릴게요.",
 "cocktail": null, "ice_size": null, "action_code": "wipe_water"}
{"reason": "안녕하세요! 오늘 기분은 어떠신가요?",
 "cocktail": null, "ice_size": null, "action_code": "chat"}
"""


class CocktailBrain(Node):
    def __init__(self):
        super().__init__("cocktail_brain_node")

        # 1. 로봇 팔 명령 (입)
        self.pub_cocktail = self.create_publisher(String, "/robot_order_cocktail", 10)
        self.pub_ice = self.create_publisher(String, "/robot_order_ice_size", 10)
        # 물 닦기 명령 Pub/Sub
        self.pub_wipe = self.create_publisher(Bool, "/wipe_water_command", 10)
        self.sub_wipe_active = self.create_subscription(
            Bool, "/wipe_sequence_active", self.wipe_active_callback, 10
        )

        # 2. 로봇 상태 수신 (귀)
        self.sub_robot_status = self.create_subscription(
            String, "/robot_status", self.robot_status_callback, 10
        )
        self.sub_cocktail_complete = self.create_subscription(
            Bool, "/cocktail_sequence_complete", self.cocktail_complete_callback, 10
        )
        # [추가] 시각 신경 연결 (YOLO가 보내주는 이미지 구독)
        self.bridge = CvBridge()
        self.latest_image = None  # 가장 최신 장면을 기억할 변수
        self.sub_img = self.create_subscription(
            Image, "/camera/color/image_raw", self.img_callback, 10
        )

        self.get_logger().info("🍸 칵테일 바텐더 뇌(Brain) 가동 - 빠른 응답 모드")

        # [상태 변수]
        # 로봇이 제조 중인가?
        self.waiting_for_robot = False

        self.timer = self.create_timer(1.0, self.listen_and_think)

    # [추가] 이미지가 들어올 때마다 최신 장면으로 업데이트
    def img_callback(self, msg):
        self.latest_image = msg

    def capture_snapshot(self, filename="snapshot.jpg"):
        """현재 로봇의 시야를 파일로 저장"""
        if self.latest_image is None:
            return False
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        cv2.imwrite(filename, cv_img)
        return True

    def wipe_active_callback(self, msg):
        """물 닦기 시퀀스 활성화/비활성화 상태 수신."""
        if not msg.data:
            # False가 들어오면 닦기 완료(또는 초기화)를 의미
            if self.waiting_for_robot:
                self.get_logger().info("💧 로봇: 물 닦기 시퀀스 완료")
                tts.speak("물 닦기를 완료했습니다. 다음 주문이 있으시면 말씀해주세요.")
                self.waiting_for_robot = False

    def robot_status_callback(self, msg):
        """기존 /robot_status 토픽 (참고용)"""
        pass

    def cocktail_complete_callback(self, msg):
        """칵테일 제조 완료 신호 수신."""
        if msg.data and self.waiting_for_robot:
            self.get_logger().info("🤖 로봇: 제조 완료")
            tts.speak(
                "칵테일이 완성되었습니다. 맛있게 드세요. " "다음 주문이 있으시면 말씀해주세요."
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

        # --- [Step 2] 생각하기 (Context ---
        # [수정] 텍스트에 '추천'이나 '뭐' 같은 단어가 있는지 확인
        full_query = f"{BARTENDER_PROMPT}\n손님: {user_text}"
        ai_response = ""

        # [수정] 키워드 대폭 추가 (오인식 대비)
        # 컵퇴, 칵테, 추천, 춰, 줘, 뭐, 어울리는 등등
        keywords = ["추천", "뭐", "어울", "컵퇴", "칵테", "주세요"]

        # 위 키워드 중 하나라도 포함되면 Vision AI 발동
        if any(word in user_text for word in keywords):
            # 이미지가 있다면 저장하고 멀티모달 질문
            if self.latest_image is not None:
                cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
                cv2.imwrite("snapshot.jpg", cv_img)  # 현재 화면 찰칵!

                # 프롬프트 약간 변경 (사진을 참고하라고 지시)
                vision_prompt = (
                    full_query
                    + "\n(참고: 첨부된 손님 사진을 보고 분위기에 맞춰 추천해줘)"
                )
                self.get_logger().info("📸 사진을 보고 고민 중...")
                ai_response = gemini_handler.ask_gemini_vision(
                    vision_prompt, "snapshot.jpg"
                )
            else:
                # 사진이 없으면 그냥 텍스트로 질문
                ai_response = gemini_handler.ask_gemini(full_query)
        else:
            # 일반 대화는 기존 방식대로
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
            elif action == "wipe_water":
                # 물 닦기 명령 전송
                self.send_wipe_command()
            else:
                # chat 또는 에러 상황 등 -> 아무 행동 안 함 (TTS만 하고 끝)
                pass

        except Exception as e:
            self.get_logger().error(f"처리 오류: {e}")
            tts.speak("죄송해요, 오류가 났어요.")

    def send_wipe_command(self):
        """로봇에게 물 닦기 명령 전송."""
        msg_wipe = Bool()
        msg_wipe.data = True
        self.pub_wipe.publish(msg_wipe)
        self.get_logger().info(">> 💧 로봇에게 물 닦기 명령 전송 완료")
        # 로봇 대기 모드 진입
        self.waiting_for_robot = True

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
