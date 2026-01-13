import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MenuStore(Node):
    def __init__(self):
        super().__init__("menu_store")
        self.current_menu = None
        self.sub_menu = self.create_subscription(
            String, "/robot_order_cocktail", self.menu_cb, 10
        )
        self.get_logger().info("[OK] Menu store ready. Waiting for cocktail name...")

    def menu_cb(self, msg: String):
        name = str(msg.data).strip()
        if not name:
            self.get_logger().warn("Empty cocktail name received; ignoring.")
            return
        self.current_menu = name
        self.get_logger().info(f"Menu stored: {name}")


def main(args=None):
    rclpy.init(args=args)
    node = MenuStore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
