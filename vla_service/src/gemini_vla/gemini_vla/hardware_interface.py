"""Thin ROS2 service client wrapper for robot motion and gripper commands."""

import time

import rclpy
from dsr_msgs2.srv import MoveLine, SetRobotControl
from std_srvs.srv import SetBool


class RobotHardwareInterface:
    """Provide blocking helper methods over asynchronous robot services."""

    def __init__(
        self,
        node,
        cb_group,
        move_service="motion/move_line",
        gripper_service="set_gripper_stroke",
        control_service="system/set_robot_control",
    ):
        """Create service clients used by the orchestrator node."""
        self.node = node
        self.move_cli = node.create_client(MoveLine, move_service, callback_group=cb_group)
        self.gripper_cli = node.create_client(SetBool, gripper_service, callback_group=cb_group)
        self.control_cli = node.create_client(
            SetRobotControl, control_service, callback_group=cb_group
        )

    def acquire_control(self, timeout_sec=5.0):
        """Request remote control mode before sending robot motion commands."""
        self.node.get_logger().info("로봇 제어권 요청 중...")
        if not self.control_cli.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error("로봇 제어 서비스 응답 없음")
            return False

        req = SetRobotControl.Request()
        req.robot_control = 1  # CONTROL_MODE_REMOTE
        self.control_cli.call_async(req)
        time.sleep(1.0)
        self.node.get_logger().info("제어권 확보 시도 완료")
        return True

    def move_line(self, pos, vel=60.0, radius=0.0, post_delay_sec=0.5):
        """Send a Cartesian linear move and wait for completion."""
        if not self.move_cli.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("이동 서비스 응답 없음")
            return False

        req = MoveLine.Request()
        req.pos = [float(p) for p in pos]
        req.vel, req.acc = [vel, vel], [80.0, 80.0]
        req.radius = float(radius)

        self.node.get_logger().info(f"이동 시도: {pos}")
        future = self.move_cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        res = future.result()
        if res and res.success:
            self.node.get_logger().info("이동 성공")
            if post_delay_sec > 0.0:
                time.sleep(post_delay_sec)
            return True

        self.node.get_logger().error("이동 실패 (로봇 상태 확인 필요)")
        if post_delay_sec > 0.0:
            time.sleep(min(post_delay_sec, 0.2))
        return False

    def set_gripper(self, is_close, retries=5, retry_delay_sec=0.5):
        """Open/close gripper with retry logic on communication failure."""
        if not self.gripper_cli.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("그리퍼 노드 응답 없음")
            return False

        for attempt in range(retries + 1):
            future = self.gripper_cli.call_async(SetBool.Request(data=is_close))
            while rclpy.ok() and not future.done():
                time.sleep(0.1)

            res = future.result()
            if res and res.success:
                self.node.get_logger().info(
                    f"그리퍼 {'닫기' if is_close else '열기'} 완료"
                )
                time.sleep(1.5)
                return True

            if attempt < retries:
                self.node.get_logger().warn(
                    f"그리퍼 동작 실패, 재시도 {attempt + 1}/{retries}"
                )
                time.sleep(retry_delay_sec)

        self.node.get_logger().error("그리퍼 동작 실패")
        return False
