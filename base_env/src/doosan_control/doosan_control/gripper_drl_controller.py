<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import textwrap
import time

# ========================================================
# DRL Modbus 함수 정의
# ========================================================
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
"""

class GripperController:
<<<<<<< Updated upstream
    def __init__(self, node: Node, dsr_node: Node, namespace: str = "dsr01"):
        self.main_node = node
        self.dsr_node = dsr_node 
        self.namespace = namespace
=======
    def __init__(
        self,
        node: Node,
        dsr_node: Node,
        namespace: str = "dsr01",
        retries: int = 2,
        retry_delay_sec: float = 0.2,
        reinit_on_fail: bool = True,
    ):
        self.main_node = node
        self.dsr_node = dsr_node 
        self.namespace = namespace
        self.retries = retries
        self.retry_delay_sec = retry_delay_sec
        self.reinit_on_fail = reinit_on_fail
>>>>>>> Stashed changes

        self.client_node = rclpy.create_node(f"{namespace}_gripper_client")
        self.cli = self.client_node.create_client(DrlStart, f"/{namespace}/drl/drl_start")

        # 서비스 연결 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.main_node.get_logger().info("Waiting for Gripper Service...")
            rclpy.spin_once(self.dsr_node, timeout_sec=0.01)

        self.main_node.get_logger().info("✅ Gripper service connected")

    def _send_drl_script(self, code: str, timeout_sec: float = 10.0) -> bool:
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = code

        future = self.cli.call_async(req)
        start_time = time.time()

        # 데드락 방지용 스핀
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.client_node, timeout_sec=0.01)
            rclpy.spin_once(self.dsr_node, timeout_sec=0.01)
            
            if time.time() - start_time > timeout_sec:
                self.main_node.get_logger().error("❌ Gripper DRL timeout")
                return False

        try:
            res = future.result()
            return True if res and res.success else False
        except Exception as e:
            self.main_node.get_logger().error(f"❌ Gripper DRL error: {e}")
            return False

<<<<<<< Updated upstream
=======
    def _send_with_retries(self, code: str, timeout_sec: float = 10.0) -> bool:
        last_ok = False
        for attempt in range(self.retries + 1):
            last_ok = self._send_drl_script(code, timeout_sec=timeout_sec)
            if last_ok:
                return True
            if attempt < self.retries:
                self.main_node.get_logger().warn(
                    f"⚠️ Gripper DRL failed (attempt {attempt + 1}/{self.retries + 1}), retrying..."
                )
                time.sleep(self.retry_delay_sec)
        return last_ok

>>>>>>> Stashed changes
    def initialize(self) -> bool:
        task_code = textwrap.dedent("""
            flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
            wait(0.5)
            modbus_set_slaveid(1)
            flange_serial_write(modbus_fc06(256, 1)) # Torque Enable
            wait(0.2)
            flange_serial_write(modbus_fc06(275, 400)) # Current Set
            wait(0.2)
            flange_serial_close()
        """)
<<<<<<< Updated upstream
        return self._send_drl_script(textwrap.dedent(f"{DRL_FUNCTIONS}\n{task_code}"), timeout_sec=10.0)
=======
        return self._send_with_retries(textwrap.dedent(f"{DRL_FUNCTIONS}\n{task_code}"), timeout_sec=10.0)
>>>>>>> Stashed changes

    def move(self, stroke: int) -> bool:
        # 👇👇👇 [절대 방어 코드] 👇👇👇
        # 1. 포트 열기
        # 2. 0.5초 대기 (포트 안정화: Warm-up) <-- 이게 없어서 안 됐을 확률 99%
        # 3. 쓰기
        # 4. 1.5초 대기 (동작 시간)
        # 5. 포트 닫기
        task_code = textwrap.dedent(f"""
            flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
            wait(0.5)
            modbus_set_slaveid(1)
            flange_serial_write(modbus_fc16(282, 2, [{int(stroke)}, 0]))
            wait(1.5) 
            flange_serial_close()
        """)
<<<<<<< Updated upstream
        return self._send_drl_script(textwrap.dedent(f"{DRL_FUNCTIONS}\n{task_code}"), timeout_sec=10.0)
=======
        ok = self._send_with_retries(textwrap.dedent(f"{DRL_FUNCTIONS}\n{task_code}"), timeout_sec=10.0)
        if ok:
            return True
        if self.reinit_on_fail:
            self.main_node.get_logger().warn("⚠️ Gripper move failed, re-initializing and retrying once...")
            if self.initialize():
                return self._send_with_retries(textwrap.dedent(f"{DRL_FUNCTIONS}\n{task_code}"), timeout_sec=10.0)
        return False
>>>>>>> Stashed changes

    def shutdown(self):
        try:
            self.client_node.destroy_node()
        except Exception:
<<<<<<< Updated upstream
            pass
=======
            pass
>>>>>>> Stashed changes
