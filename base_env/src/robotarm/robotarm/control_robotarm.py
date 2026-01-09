import json
import textwrap
import time

import DR_init
import rclpy
from dsr_msgs2.srv import DrlStart
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

DRL_GRIPPER_BASE = """
g_slaveid = 0
def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid

def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, 'big')
    data += (6).to_bytes(1, 'big')
    data += (address).to_bytes(2, 'big')
    data += (value).to_bytes(2, 'big')
    return modbus_send_make(data)

def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, 'big')
    data += (16).to_bytes(1, 'big')
    data += (startaddress).to_bytes(2, 'big')
    data += (cnt).to_bytes(2, 'big')
    data += (2 * cnt).to_bytes(1, 'big')
    for i in range(cnt):
        data += (valuelist[i]).to_bytes(2, 'big')
    return modbus_send_make(data)

def gripper_move(stroke):
    flange_serial_open(
        baudrate=57600,
        bytesize=DR_EIGHTBITS,
        parity=DR_PARITY_NONE,
        stopbits=DR_STOPBITS_ONE
    )
    modbus_set_slaveid(1)
    flange_serial_write(modbus_fc06(256, 1))
    wait(0.1)
    flange_serial_write(modbus_fc06(275, 400))
    wait(0.1)
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    wait(1.5)
    flange_serial_close()
"""


class RobotBartender(Node):
    def __init__(self):
        super().__init__("robot_bartender_node", namespace="dsr01")
        self.callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            String,
            "/robot_order",
            self.order_callback,
            10,
            callback_group=self.callback_group,
        )

        self.status_publisher = self.create_publisher(String, "/robot_status", 10)

        self.drl_client = self.create_client(
            DrlStart,
            "drl/drl_start",
            callback_group=self.callback_group,
        )

        # 로봇 제어 함수들 (나중에 연결됨)
        self.movej = None
        self.posj = None
        self.set_robot_mode = None
        self.robot_mode_autonomous = None

        self.robot_ready = False
        self.is_busy = False

        # 나중에 티칭해서 값만 바꾸면 됩니다.
        self.LOCATIONS = {
            "HOME": [0, 0, 90, 0, 90, 0],
            "ICE_MACHINE": [-18, 43.5, 65, 0, 71.5, -18],  # 얼음 위치
            # [숙제] 아래 좌표들은 실제 로봇을 움직여서 값을 알아내고 채워넣어야 합니다!
            "GIN_BOTTLE": [10, 10, 90, 0, 90, 0],  # (예시) 진 병 위치
            "WHISKEY_BOTTLE": [20, 20, 90, 0, 90, 0],  # (예시) 잭다니엘 병 위치
            "TONIC_DISPENSER": [30, 30, 90, 0, 90, 0],  # (예시) 토닉워터
            "COKE_DISPENSER": [40, 40, 90, 0, 90, 0],  # (예시) 콜라
            "SERVING_POINT": [0, -40, 90, 0, 90, 0],  # 손님에게 주는 위치
        }

        # [핵심 2] 레시피 북 (RECIPE_BOOK)
        # 칵테일 이름 : [이동할 위치 순서 목록]
        self.RECIPE_BOOK = {
            "Gin Tonic": [
                "ICE_MACHINE",
                "GIN_BOTTLE",
                "TONIC_DISPENSER",
                "SERVING_POINT",
            ],
            "Jack & Coke": [
                "ICE_MACHINE",
                "WHISKEY_BOTTLE",
                "COKE_DISPENSER",
                "SERVING_POINT",
            ],
        }

        self.get_logger().info("🦾 로봇 바텐더 준비 완료 (레시피 북 탑재)")

    def order_callback(self, msg):
        if self.is_busy:
            return

        try:
            clean_json = msg.data.replace("```json", "").replace("```", "").strip()
            order_data = json.loads(clean_json)
            cocktail_name = order_data.get("cocktail", "")

            # 메뉴판에 없는 주문 방어
            if cocktail_name not in self.RECIPE_BOOK:
                self.get_logger().warning(f"🚫 레시피 없는 주문: {cocktail_name}")
                return

            self.get_logger().info(f"🍹 주문 접수: {cocktail_name}")

            # 로봇 연결 체크
            if not self.robot_ready:
                if self.set_robot_mode:
                    self.set_robot_mode(self.robot_mode_autonomous)
                    self.robot_ready = True
                else:
                    return

            self.is_busy = True
            self.make_cocktail(cocktail_name)  # 만능 함수 호출
            self.is_busy = False

            # 완료 신호 전송
            done_msg = String()
            done_msg.data = "DONE"
            self.status_publisher.publish(done_msg)

        except Exception as e:
            self.get_logger().error(f"주문 처리 중 에러: {e}")
            self.is_busy = False

    def gripper_move(self, stroke, settle=2.0):
        """그리퍼 제어 (비동기)."""
        code = textwrap.dedent(DRL_GRIPPER_BASE + f"\n\ngripper_move({int(stroke)})\n")
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = code
        self.drl_client.call_async(req)
        time.sleep(settle)

    def make_cocktail(self, menu_name):
        """레시피 북을 보고 순서대로 움직이는 만능 함수."""
        if self.posj is None:
            return

        # 1. 레시피 가져오기 (예: ["ICE", "GIN", ...])
        recipe_steps = self.RECIPE_BOOK[menu_name]
        self.get_logger().info(f"🎬 {menu_name} 제조 시작! 단계: {recipe_steps}")

        VEL = 30
        ACC = 30

        # 2. 초기화 (홈 이동 & 그리퍼 열기)
        self.movej(self.posj(*self.LOCATIONS["HOME"]), VEL, ACC)
        self.gripper_move(0)

        # 3. 레시피 순서대로 착착 이동
        for step_name in recipe_steps:
            # 좌표 사전에서 좌표 꺼내기
            target_coords = self.LOCATIONS.get(step_name)

            if target_coords:
                self.get_logger().info(f"➡️ 이동 중: {step_name}")

                # 로봇 이동
                self.movej(self.posj(*target_coords), VEL, ACC)
                time.sleep(0.5)  # 이동 후 잠시 안정화

                # [응용] 만약 특정 위치에서 특별한 행동(따르기 등)이 필요하면
                # 여기에 if step_name == "GIN_BOTTLE": self.pour_drink() 등을 추가
                time.sleep(2.0)  # (임시) 작업 시간 시뮬레이션
            else:
                self.get_logger().error(f"❌ 좌표 없음: {step_name}")

        # 4. 마무리 (홈 복귀)
        self.get_logger().info("🏠 홈으로 복귀")
        self.movej(self.posj(*self.LOCATIONS["HOME"]), VEL, ACC)
        self.get_logger().info(f"✨ {menu_name} 완성!")


def main(args=None):
    rclpy.init(args=args)
    node = RobotBartender()

    DR_init.__dsr__id = ""
    DR_init.__dsr__model = "e0509"
    DR_init.__dsr__node = node

    try:
        import DSR_ROBOT2 as dr

        # 함수 연결
        node.movej = dr.movej
        node.posj = dr.posj
        node.set_robot_mode = dr.set_robot_mode
        node.robot_mode_autonomous = dr.ROBOT_MODE_AUTONOMOUS

        # 멀티스레드 (4개)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()

    except Exception as e:
        node.get_logger().error(f"치명적 오류: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
