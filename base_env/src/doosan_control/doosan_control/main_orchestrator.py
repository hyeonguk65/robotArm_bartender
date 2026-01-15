import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import DR_init

from . import config
from .action_tasks import ActionTasks
from .hardware_interface import GripperController, RobotController


class Orchestrator(Node):
    def __init__(self, dsr, dsr_node):
        super().__init__("robot_orchestrator")
        self.dsr_node = dsr_node
        self.robot = RobotController(self, dsr)
        self.gripper = GripperController(self, dsr_node, namespace=config.ROBOT_ID)
        self.tasks = ActionTasks(self, self.robot, self.gripper)

        self.busy = False
        self.sub_coord = self.create_subscription(
            PointStamped, "/hand_target_point", self.target_cb, 10
        )

        self.get_logger().info("[OK] Orchestrator ready. Waiting for target...")

    def target_cb(self, msg: PointStamped):
        if self.busy:
            return
        self.busy = True

        x = msg.point.x * 1000.0
        y = msg.point.y * 1000.0
        z = msg.point.z * 1000.0
        self.get_logger().info(f"Target received: ({x:.1f}, {y:.1f}, {z:.1f})")

        try:
            self.tasks.pick_target((x, y, z))
            self.tasks.process_cocktail_action()
        except Exception as exc:
            self.get_logger().error(f"Action failed: {exc}")
        finally:
            self.get_logger().info("Done. Returning home.")
            self.robot.go_home()
            self.busy = False


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
