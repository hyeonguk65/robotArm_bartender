import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import DR_init

from doosan_control.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
RX, RY, RZ = 0.0, 180.0, 0.0
GRIPPER_OPEN_VAL = 0
GRIPPER_CLOSE_VAL = 700


class SimpleVisionPick(Node):
    def __init__(self, dsr, dsr_node):
        super().__init__("vision_pick_simple")
        self.dsr_node = dsr_node

        # Use the initialized DSR module with correct prefixes.
        self.movel = dsr.movel
        self.movej = dsr.movej
        self.posj = dsr.posj
        self.get_current_posx = dsr.get_current_posx
        dsr.set_robot_mode(dsr.ROBOT_MODE_AUTONOMOUS)

        self.gripper = GripperController(node=self, dsr_node=self.dsr_node, namespace=ROBOT_ID)
        if not self.gripper.initialize():
            self.get_logger().error("❌ Gripper Initialization Failed!")

        self.hover = 100.0
        self.drop = 0.0

        self.sub_coord = self.create_subscription(
            PointStamped, "/hand_target_point", self.target_cb, 10
        )

        self.received = False
        self.busy = False
        self.get_logger().info("[OK] Simple vision pick ready. Waiting for target...")

    def target_cb(self, msg: PointStamped):
        if self.received or self.busy:
            return
        self.received = True
        self.busy = True

        x = msg.point.x * 1000.0
        y = msg.point.y * 1000.0
        z = msg.point.z * 1000.0
        self.get_logger().info(f"Target received: ({x:.1f}, {y:.1f}, {z:.1f})")

        try:
            # 1) 접근
            self.movel([x, y, z + self.hover, RX, RY, RZ], vel=50, acc=50)
            time.sleep(1.0)
            # 2) 그리퍼 오픈
            self.gripper.move(GRIPPER_OPEN_VAL)
            time.sleep(2.5)
            # 3) 내려감
            self.movel([x, y, z + self.drop, RX, RY, RZ], vel=30, acc=30)
            time.sleep(1.0)
            # 4) 그리퍼 클로즈
            self.gripper.move(GRIPPER_CLOSE_VAL)
            time.sleep(2.5)
            # 5) 들어올림
            self.movel([x, y, z + self.hover, RX, RY, RZ], vel=50, acc=50)
        except Exception as exc:
            self.get_logger().error(f"Move failed: {exc}")
        finally:
            self.get_logger().info("Done. Returning home.")
            self._go_home()
            self.busy = False
            self.received = False

    def _go_home(self):
        try:
            P0 = self.posj(0, 0, 90, 0, 90, 0)
            self.movej(P0, 50, 50)
        except Exception as exc:
            self.get_logger().error(f"Home move failed: {exc}")



def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__srv_name_prefix = f"/{ROBOT_ID}/"
    DR_init.__dsr__topic_name_prefix = f"/{ROBOT_ID}/"
    sys.modules["DR_init"] = DR_init

    if "DSR_ROBOT2" in sys.modules:
        del sys.modules["DSR_ROBOT2"]
    import DSR_ROBOT2 as dsr

    node = SimpleVisionPick(dsr, dsr_node)
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
