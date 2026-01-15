from . import config


class ActionTasks:
    def __init__(self, node, robot, gripper):
        self.node = node
        self.robot = robot
        self.gripper = gripper

    def _pause(self, sec: float) -> None:
        self.robot.pause(sec)

    def _move_xyz(self, x, y, z, vel, acc, pause_sec=1.0) -> None:
        self.robot.move_to_xyz(x, y, z, vel=vel, acc=acc)
        if pause_sec:
            self._pause(pause_sec)

    def _move_pos(self, pos, vel=None, acc=None, pause_sec=0.5) -> None:
        if vel is None:
            vel = config.VEL_FAST
        if acc is None:
            acc = config.ACC_FAST
        self.robot.move_to_pos(pos, vel=vel, acc=acc)
        if pause_sec:
            self._pause(pause_sec)

    def _grip_open(self, pause_sec=None) -> None:
        self.gripper.open()
        self._pause(config.GRIPPER_WAIT_SEC if pause_sec is None else pause_sec)

    def _grip_close(self, pause_sec=None) -> None:
        self.gripper.close()
        self._pause(config.GRIPPER_WAIT_SEC if pause_sec is None else pause_sec)

    def _grip_move(self, value, pause_sec=None) -> None:
        self.gripper.move(value)
        self._pause(config.GRIPPER_WAIT_SEC if pause_sec is None else pause_sec)

    def pick_target(self, xyz) -> None:
        x, y, z = xyz
        # 1) 접근
        self._move_xyz(x, y, z + config.HOVER_OFFSET_Z, vel=config.VEL_FAST, acc=config.ACC_FAST)
        # 2) 그리퍼 오픈
        self._grip_open()
        # 3) 내려감
        self._move_xyz(x, y, z + config.DROP_OFFSET_Z, vel=config.VEL_SLOW, acc=config.ACC_SLOW)
        # 4) 그리퍼 클로즈
        self._grip_close()
        # 5) 들어올림
        self._move_xyz(x, y, z + config.HOVER_OFFSET_Z, vel=config.VEL_FAST, acc=config.ACC_FAST, pause_sec=0.0)

    def process_cocktail_action(self) -> None:
        """
        얼음 드랍 -> 컵 회수 -> 디스펜서 -> 컵 놓기 시나리오
        """
        self.node.get_logger().info("Starting Cocktail Action Sequence...")

        # 1) 얼음 드랍
        self._move_pos(config.POS_ICE_DROP)
        self._grip_open()

        # 2) 데드락 방지 홈 복귀
        self.robot.go_home()
        self._pause(0.5)

        # 3) 그리퍼2(쉐이커 거치) 접근 -> 컵 집기
        self._move_pos(config.POS_GRIPPER2_HOVER)
        self._move_pos(config.POS_GRIPPER2_LAND)
        self._grip_move(config.GRIPPER_CUP_CLOSE)
        self._move_pos(config.POS_GRIPPER2_HOVER)

        # 4) 디스펜서 동작
        self._move_pos(config.POS_DISPENSER_READY)
        self._move_pos(config.POS_DISPENSER_PUSH, vel=config.VEL_SLOW)
        self._pause(2.0)
        self._move_pos(config.POS_DISPENSER_READY, vel=config.VEL_SLOW)
        self._pause(2.0)

        # 5) 컵을 다시 거치
        self._move_pos(config.POS_GRIPPER2_HOVER)
        self._move_pos(config.POS_GRIPPER2_LAND)
        self._grip_open()
        self._move_pos(config.POS_GRIPPER2_HOVER)

        # 6) 뚜껑 집기
        self._move_pos(config.POS_LID_HOVER)
        self._move_pos(config.POS_LID_PICK)
        self._grip_move(config.GRIPPER_TOP_CLOSE)
        self._move_pos(config.POS_LID_HOVER)

        # 7) 뚜껑-쉐이커 결합
        self._move_pos(config.POS_LID_SHAKER_HOVER)
        self._move_pos(config.POS_LID_SHAKER_LAND)
        self._grip_open()
        self._move_pos(config.POS_GRIPPER2_HOVER)

        # 8) 컵 집기 (쉐이킹 전)
        self._move_pos(config.POS_GRIPPER2_LAND, vel=config.VEL_SLOW)
        self._grip_move(config.GRIPPER_CUP_CLOSE)
        self._move_pos(config.POS_GRIPPER2_HOVER)

        # 9) 쉐이커 뚜껑 다시 장착
        self._move_pos(config.POS_LID_SHAKER_HOVER)
        self._move_pos(config.POS_LID_SHAKER_LAND)
        self._grip_move(config.GRIPPER_TOP_CLOSE)
        self._move_pos(config.POS_LID_SHAKER_HOVER)
        self._move_pos(config.POS_LID_HOVER)
        self._move_pos(config.POS_LID_PICK)
        self._grip_open()
        self._move_pos(config.POS_LID_HOVER)

        # 10) 컵 집기 -> 따르기
        self._move_pos(config.POS_GRIPPER2_HOVER)
        self._move_pos(config.POS_GRIPPER2_LAND)
        self._grip_move(config.GRIPPER_CUP_CLOSE)
        self._move_pos(config.POS_GRIPPER2_HOVER)
        self._move_pos(config.POS_POUR_READY)
        self._move_pos(config.POS_POUR_ACTION)
        self._move_pos(config.POS_POUR_READY)

        # 11) 컵 놓기
        self._move_pos(config.POS_GRIPPER2_HOVER)
        self._move_pos(config.POS_GRIPPER2_LAND)
        self._grip_open()
        self._move_pos(config.POS_GRIPPER2_HOVER)

        # 12) 컵 회수 및 서빙
        self.robot.go_home()
        self._pause(0.5)
        self._move_pos(config.POS_CUP_RECOVERY, vel=config.VEL_SLOW)
        self._grip_move(config.GRIPPER_CUP_CLOSE)
        self._move_pos(config.POS_CUP_RECOVERY_LIFT)
        self._move_pos(config.POS_CUSTOMER_HOVER)
        self._move_pos(config.POS_CUSTOMER_LAND)
        self._grip_open()
        self.robot.go_home()
        self._pause(0.5)
        self.node.get_logger().info("Cocktail Action Sequence Complete.")
