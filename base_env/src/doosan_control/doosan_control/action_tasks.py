from . import config


class ActionTasks:
    def __init__(self, node, robot, gripper):
        self.node = node
        self.robot = robot
        self.gripper = gripper

    def pick_target(self, xyz) -> None:
        x, y, z = xyz
        # 1) 접근
        self.robot.move_to_xyz(x, y, z + config.HOVER_OFFSET_Z, vel=config.VEL_FAST, acc=config.ACC_FAST)
        self.robot.pause(1.0)
        # 2) 그리퍼 오픈
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)
        # 3) 내려감
        self.robot.move_to_xyz(x, y, z + config.DROP_OFFSET_Z, vel=config.VEL_SLOW, acc=config.ACC_SLOW)
        self.robot.pause(1.0)
        # 4) 그리퍼 클로즈
        self.gripper.close()
        self.robot.pause(config.GRIPPER_WAIT_SEC)
        # 5) 들어올림
        self.robot.move_to_xyz(x, y, z + config.HOVER_OFFSET_Z, vel=config.VEL_FAST, acc=config.ACC_FAST)
