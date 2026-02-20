"""Build and send DRL scripts that operate the end-effector gripper via Modbus."""

import textwrap
import time

import rclpy
from dsr_msgs2.srv import DrlStart


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


class GripperDrlController:
    """Manage DRL service calls with retries and optional re-initialization."""

    def __init__(
        self,
        node,
        service_name="drl/drl_start",
        callback_group=None,
        retries=5,
        retry_delay_sec=0.5,
        reinit_on_fail=True,
    ):
        """Initialize DRL service client and retry policy settings."""
        self.node = node
        self.cli = node.create_client(DrlStart, service_name, callback_group=callback_group)
        self.retries = retries
        self.retry_delay_sec = retry_delay_sec
        self.reinit_on_fail = reinit_on_fail

    def _send_drl_script(self, code, timeout_sec=10.0):
        """Send DRL code and wait until the robot acknowledges success/failure."""
        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("로봇 DRL 서비스를 찾을 수 없습니다!")
            return False

        req = DrlStart.Request()
        req.robot_system = 0
        req.code = code

        future = self.cli.call_async(req)
        start_time = time.time()
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
            if time.time() - start_time > timeout_sec:
                self.node.get_logger().error("그리퍼 DRL timeout")
                return False

        res = future.result()
        if not res or not res.success:
            self.node.get_logger().error("그리퍼 DRL 실패")
            return False
        return True

    def _initialize(self):
        """Run low-level gripper serial initialization DRL once."""
        return self._send_drl_script(self._build_init_code())

    def _send_with_retries(self, code):
        """Retry DRL execution when transient communication errors occur."""
        last_ok = False
        for attempt in range(self.retries + 1):
            last_ok = self._send_drl_script(code)
            if last_ok:
                return True
            if attempt < self.retries:
                self.node.get_logger().warn(
                    f"그리퍼 DRL 실패, 재시도 {attempt + 1}/{self.retries}"
                )
                time.sleep(self.retry_delay_sec)
        return last_ok

    def initialize(self):
        """Public initializer used by the gripper service node."""
        return self._send_with_retries(self._build_init_code())

    def _build_init_code(self):
        """Create DRL code that configures flange serial + Modbus setup."""
        drl_body = """
def gripper_init():
    res = -1
    for _ in range(3):
        res = flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
        if res == 0:
            break
        wait(0.2)
    if res != 0:
        return

    wait(0.5)
    modbus_set_slaveid(1)
    flange_serial_write(modbus_fc06(256, 1))
    wait(0.3)
    flange_serial_write(modbus_fc06(275, 400))
    wait(0.3)
    flange_serial_close()

gripper_init()
"""
        return textwrap.dedent(f"{DRL_FUNCTIONS}\n{drl_body}")

    def move_stroke(self, stroke, wait_sec=2.5):
        """Move gripper to target stroke value; reinitialize once if needed."""
        drl_body = f"""
def gripper_action():
    res = -1
    for _ in range(3):
        res = flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
        if res == 0:
            break
        wait(0.2)
    if res != 0:
        return

    wait(0.5)
    modbus_set_slaveid(1)
    flange_serial_write(modbus_fc06(256, 1))
    wait(0.3)
    flange_serial_write(modbus_fc06(275, 400))
    wait(0.3)
    flange_serial_write(modbus_fc16(282, 2, [{int(stroke)}, 0]))
    wait(0.1)
    flange_serial_write(modbus_fc16(282, 2, [{int(stroke)}, 0]))
    wait({float(wait_sec)})
    flange_serial_close()

gripper_action()
"""
        code = textwrap.dedent(f"{DRL_FUNCTIONS}\n{drl_body}")
        ok = self._send_with_retries(code)
        if ok:
            return True
        if self.reinit_on_fail:
            self.node.get_logger().warn("그리퍼 재초기화 후 재시도")
            if self._initialize():
                return self._send_with_retries(code)
        return False
