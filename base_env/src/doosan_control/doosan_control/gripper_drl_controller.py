import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import textwrap
import time

# ... DRL_FUNCTIONS 문자열은 기존과 동일하므로 생략하거나 그대로 두세요 ...
DRL_FUNCTIONS = """
g_slaveid = 0
def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid
def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (6).to_bytes(1, byteorder='big')
    data += (address).to_bytes(2, byteorder='big')
    data += (value).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (16).to_bytes(1, byteorder='big')
    data += (startaddress).to_bytes(2, byteorder='big')
    data += (cnt).to_bytes(2, byteorder='big')
    data += (2 * cnt).to_bytes(1, byteorder='big')
    for i in range(0, cnt):
        data += (valuelist[i]).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def gripper_move(stroke):
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    # DRL 스크립트 내부 wait는 로봇 컨트롤러의 해석을 막을 수 있으므로 
    # 여기서는 명령만 보내고 Python에서 대기하는 것이 안전합니다.
    # wait(1.2)  <-- 제거 추천 (Python 제어권 확보 위해)
"""

class GripperController:
    # [수정 1] dsr_node를 추가로 받습니다.
    def __init__(self, node: Node, dsr_node: Node, namespace: str = "dsr01"):
        self.main_node = node
        self.dsr_node = dsr_node  # 로봇 드라이버 노드 저장
        self.namespace = namespace

        # 데드락 방지: 서비스 전용 노드
        self.client_node = rclpy.create_node(f"{namespace}_gripper_client")
        self.cli = self.client_node.create_client(DrlStart, f"/{namespace}/drl/drl_start")

        # 서비스 연결 대기 시에도 dsr_node를 돌려주는 것이 안전함
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.main_node.get_logger().info("Waiting for Gripper Service...")
            rclpy.spin_once(self.dsr_node, timeout_sec=0.01)

        self.main_node.get_logger().info("✅ Gripper service connected")

    def _send_drl_script(self, code: str, timeout_sec: float = 10.0) -> bool:
        req = DrlStart.Request()
        if hasattr(req, "robot_system"):
            req.robot_system = 0
        req.code = code

        future = self.cli.call_async(req)

        # [수정 2] 핵심: Future가 완료될 때까지 'client_node'와 'dsr_node'를 모두 spin 합니다.
        # 기존 spin_until_future_complete는 메인 루프를 막아서 dsr_node가 멈추는 원인이었습니다.
        start_time = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.client_node, timeout_sec=0.01)
            rclpy.spin_once(self.dsr_node, timeout_sec=0.01) # 로봇 드라이버 살려두기
            
            if time.time() - start_time > timeout_sec:
                self.main_node.get_logger().error("❌ Gripper DRL timeout")
                return False

        try:
            res = future.result()
            if res is None:
                return False
            if hasattr(res, "success"):
                return bool(res.success)
            return True
        except Exception as e:
            self.main_node.get_logger().error(f"❌ Gripper DRL result error: {e}")
            return False

    def initialize(self) -> bool:
        task_code = textwrap.dedent("""
            flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
            modbus_set_slaveid(1)
            flange_serial_write(modbus_fc06(256, 1)) # Torque Enable
            wait(0.1)
            flange_serial_write(modbus_fc06(275, 400)) # Current Set
            wait(0.1)
        """)
        return self._send_drl_script(textwrap.dedent(f"{DRL_FUNCTIONS}\n{task_code}"), timeout_sec=10.0)

    def move(self, stroke: int) -> bool:
        task_code = textwrap.dedent(f"""
            flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
            modbus_set_slaveid(1)
            gripper_move({int(stroke)})
        """)
        return self._send_drl_script(textwrap.dedent(f"{DRL_FUNCTIONS}\n{task_code}"), timeout_sec=10.0)

    def shutdown(self):
        try:
            self.client_node.destroy_node()
        except Exception:
            pass