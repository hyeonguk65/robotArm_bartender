import time

from . import config
from .gripper_drl_controller import GripperController as DrlGripper


class GripperController:
    def __init__(self, node, dsr_node, namespace: str):
        self.node = node
        self.drl = DrlGripper(node=node, dsr_node=dsr_node, namespace=namespace)
        if not self.drl.initialize():
            self.node.get_logger().error("Gripper init failed.")

    def open(self) -> bool:
        return self.drl.move(config.GRIPPER_OPEN_VAL)

    def close(self) -> bool:
        return self.drl.move(config.GRIPPER_CLOSE_VAL)

    def move(self, value: int) -> bool:
        return self.drl.move(value)

    def shutdown(self) -> None:
        self.drl.shutdown()


class RobotController:
    def __init__(self, node, dsr):
        self.node = node
        self.dsr = dsr
        self.movel = dsr.movel
        self.movej = dsr.movej
        self.posj = dsr.posj
        self.get_current_posx = dsr.get_current_posx
        dsr.set_robot_mode(dsr.ROBOT_MODE_AUTONOMOUS)

    def move_to_xyz(self, x, y, z, vel=config.VEL_FAST, acc=config.ACC_FAST) -> None:
        self.movel([x, y, z, config.RX, config.RY, config.RZ], vel=vel, acc=acc)

    def move_to_pos(self, pos_list, vel=config.VEL_FAST, acc=config.ACC_FAST) -> None:
        self.movel(pos_list, vel=vel, acc=acc)

    def go_home(self) -> None:
        self.movej(self.posj(*config.HOME_JOINTS), vel=config.VEL_FAST, acc=config.ACC_FAST)

    def pause(self, sec: float) -> None:
        time.sleep(sec)
