import rclpy
import DR_init
from doosan_control.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = rclpy.create_node("gripper_example_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, wait

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    gripper = None
    try:
        gripper = GripperController(node=node, dsr_node=node, namespace=ROBOT_ID)
        if not gripper.initialize():
            node.get_logger().error("Gripper initialization failed. Exiting.")
            return

        node.get_logger().info("Priming...")
        wait(1)

        node.get_logger().info("CLOSE (700)")
        gripper.move(700)
        wait(2)

        node.get_logger().info("OPEN (0)")
        gripper.move(0)
        wait(2)

        node.get_logger().info("CLOSE (700)")
        gripper.move(700)
        wait(2)


    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        if gripper:
            gripper.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
