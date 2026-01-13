class HandoverManager:
    def __init__(self, node, robot, gripper, holder=None):
        self.node = node
        self.robot = robot
        self.gripper = gripper
        self.holder = holder

    def take_from_holder(self, *args, **kwargs):
        self.node.get_logger().warn("take_from_holder not implemented yet.")

    def place_to_holder(self, *args, **kwargs):
        self.node.get_logger().warn("place_to_holder not implemented yet.")
