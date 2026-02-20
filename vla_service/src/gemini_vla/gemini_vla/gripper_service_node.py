"""ROS2 service node that translates open/close requests to DRL gripper commands."""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool

from .gripper_drl_controller import GripperDrlController

class GripperServiceNode(Node):
    """Expose `set_gripper_stroke` service for simple boolean gripper control."""

    def __init__(self):
        """Create service endpoint and lazy-initialized DRL controller."""
        super().__init__("gripper_service_node", namespace="dsr01")
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            SetBool, "set_gripper_stroke", self.handle_gripper, callback_group=self.cb_group
        )
        self.controller = GripperDrlController(self, "drl/drl_start", callback_group=self.cb_group)
        self._initialized = False
        self.get_logger().info("그리퍼 서비스 노드 준비 완료 (표준 Modbus 방식)")

    def handle_gripper(self, request, response):
        """Handle SetBool request: True=close(700), False=open(0)."""
        # 닫기(True)일 때 stroke 700, 열기(False)일 때 0
        stroke = 700 if request.data else 0

        if not self._initialized:
            if self.controller.initialize():
                self.get_logger().info("그리퍼 초기화 완료")
                self._initialized = True
            else:
                self.get_logger().warn("그리퍼 초기화 실패")

        ok = self.controller.move_stroke(stroke)
        if not ok:
            response.success = False
            return response

        self.get_logger().info(f"그리퍼 명령 전송: {'CLOSE' if request.data else 'OPEN'} (Value: {stroke})")
        response.success = True
        return response

def main():
    """Run gripper service node with a multi-threaded executor."""
    rclpy.init()
    node = GripperServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
