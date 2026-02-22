import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
import DR_init

import cv2
import numpy as np
import time

from . import config
from .action_tasks import ActionTasks
from .hardware_interface import GripperController, RobotController


class Orchestrator(Node):
    def __init__(self, dsr, dsr_node):
        super().__init__("robot_orchestrator")
        self.dsr_node = dsr_node
        self.robot = RobotController(self, dsr)
        self.gripper = GripperController(self, self.dsr_node, namespace=config.ROBOT_ID)
        self.tasks = ActionTasks(self, self.robot, self.gripper)

        # [추가] 칵테일 제조 완료 신호 퍼블리셔 (VLA 등에게 알림)
        self.complete_pub = self.create_publisher(Bool, "/cocktail_sequence_complete", 10)
        # [추가] 물 닦기 중임을 알리는 퍼블리셔
        self.wipe_active_pub = self.create_publisher(Bool, "/wipe_sequence_active", 10)

        # 상태 변수
        self.busy = False
        self.wipe_trigger_received = False
        self.current_cocktail = ""
        
        self.latest_towel_pixel = None
        self.latest_water_pixel = None
        self.accept_after_time_towel = 0.0
        self.accept_after_time_water = 0.0
        
        # 픽셀 -> 로봇 베이스 호모그래피 보정 (4점 캘리브레이션)
        pixel_points = np.array(
            [[233.0, 181.0], [212.0, 387.0], [435.0, 181.0], [436.0, 383.0]],
            dtype=np.float32,
        )
        robot_points = np.array(
            [[110.15, -232.24], [669.86, -171.42], [112.58, 307.34], [629.81, 214.01]],
            dtype=np.float32,
        )
        self.homography = cv2.getPerspectiveTransform(pixel_points, robot_points)

        # 구독
        self.sub_coord = self.create_subscription(
            PointStamped, "/hand_target_point", self.cocktail_target_cb, 10
        )
        # 물/수건 타겟 좌표 (VLA Node)
        self.towel_sub = self.create_subscription(
            PointStamped, "/target_towel_point", self.towel_callback, 10
        )
        self.water_sub = self.create_subscription(
            PointStamped, "/target_water_point", self.water_callback, 10
        )
        # LLM 닦기 명령
        self.wipe_cmd_sub = self.create_subscription(
            Bool, "/wipe_water_command", self.wipe_command_callback, 10
        )
        self.order_sub = self.create_subscription(
            String, "/robot_order_cocktail", self.cocktail_order_cb, 10
        )

        self.get_logger().info("[OK] Orchestrator ready. Waiting for tasks...")
        self.wipe_active_pub.publish(Bool(data=False))

    def cocktail_order_cb(self, msg: String):
        """Receive cocktail name from brain"""
        self.current_cocktail = str(msg.data).strip()
        self.get_logger().info(f"Registered cocktail order: {self.current_cocktail}")

    def wipe_command_callback(self, msg: Bool):
        """LLM으로부터 물 닦기 명령 수신"""
        if msg.data:
            self.get_logger().info("💧 물 닦기 명령 수신! 좌표 검토 후 닦기 시작.")
            self.wipe_trigger_received = True
            self._try_start_wipe_sequence()

    def towel_callback(self, msg: PointStamped):
        """수건 좌표 갱신"""
        if self.busy:
            return
        now_mono = time.monotonic()
        if now_mono <= self.accept_after_time_towel:
            return
        self.latest_towel_pixel = (float(msg.point.x), float(msg.point.y))
        self.accept_after_time_towel = now_mono + 0.3
        self.get_logger().info(f"<< Towel Target: ({int(msg.point.x)}, {int(msg.point.y)})")
        self._try_start_wipe_sequence()

    def water_callback(self, msg: PointStamped):
        """물 좌표 갱신"""
        if self.busy:
            return
        now_mono = time.monotonic()
        if now_mono <= self.accept_after_time_water:
            return
        self.latest_water_pixel = (float(msg.point.x), float(msg.point.y))
        self.get_logger().info(f"<< Water Target: ({int(msg.point.x)}, {int(msg.point.y)})")
        self._try_start_wipe_sequence()

    def _pixel_to_robot(self, px: float, py: float):
        """이미지 픽셀 좌표를 로봇 베이스(mm) 좌표로 변환"""
        pt = np.array([[[px, py]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography)
        return float(transformed[0][0][0]), float(transformed[0][0][1])

    def _try_start_wipe_sequence(self):
        """조건이 충족되면 물 닦기 시퀀스 시작"""
        if self.busy or not self.wipe_trigger_received:
            return
        if self.latest_towel_pixel is None:
            self.get_logger().info("닦기 명령 수신됨 - 수건 좌표 대기 중")
            return
        if self.latest_water_pixel is None:
            self.get_logger().info("닦기 명령 수신됨 - 물 좌표 대기 중")
            return

        self.get_logger().info("=== 물 닦기 시퀀스 시작 ===")
        self.busy = True
        self.wipe_trigger_received = False
        self.wipe_active_pub.publish(Bool(data=True))
        
        try:
            tx, ty = self._pixel_to_robot(*self.latest_towel_pixel)
            wx, wy = self._pixel_to_robot(*self.latest_water_pixel)
            self.get_logger().info(f"변환된 로봇 좌표 - 수건:({tx:.1f}, {ty:.1f}), 물:({wx:.1f}, {wy:.1f})")
            
            # ActionTasks의 wipe 로직 호출
            self.tasks.process_wipe_action(tx, ty, wx, wy)
        except Exception as exc:
            self.get_logger().error(f"Wipe action failed: {exc}")
        finally:
            self.get_logger().info("Wipe done. Returning home and unlocking.")
            self.robot.go_home()
            self.latest_towel_pixel = None
            self.latest_water_pixel = None
            self.accept_after_time_water = time.monotonic() + 0.5
            self.busy = False
            self.wipe_active_pub.publish(Bool(data=False))

    def cocktail_target_cb(self, msg: PointStamped):
        if self.busy:
            return
        self.busy = True

        x = msg.point.x * 1000.0
        y = msg.point.y * 1000.0
        z = msg.point.z * 1000.0
        self.get_logger().info(f"Target received: ({x:.1f}, {y:.1f}, {z:.1f})")

        try:
            # 1. 먼저 타겟을 집습니다 (기존 Vision Pick)
            self.tasks.pick_target((x, y, z))

            # 2. 집은 상태에서 바로 칵테일 제조 시퀀스를 이어서 실행합니다 (이름 전달)
            self.tasks.process_cocktail_action(self.current_cocktail)

        except Exception as exc:
            self.get_logger().error(f"Action failed: {exc}")

        finally:
            self.get_logger().info("Done. Returning home.")
            self.robot.go_home()
            self.busy = False
            self.current_cocktail = ""
            
            # [추가] 칵테일 시퀀스 완료 및 홈 복귀 완료 신호 전송
            msg_complete = Bool()
            msg_complete.data = True
            self.complete_pub.publish(msg_complete)
            self.get_logger().info("[SIGNAL] Cocktail sequence complete signal sent.")


def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node", namespace=config.ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = config.ROBOT_ID
    DR_init.__dsr__model = config.ROBOT_MODEL
    DR_init.__dsr__srv_name_prefix = f"/{config.ROBOT_ID}/"
    DR_init.__dsr__topic_name_prefix = f"/{config.ROBOT_ID}/"
    sys.modules["DR_init"] = DR_init

    if "DSR_ROBOT2" in sys.modules:
        del sys.modules["DSR_ROBOT2"]
    import DSR_ROBOT2 as dsr

    node = Orchestrator(dsr, dsr_node)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            rclpy.spin_once(dsr_node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
