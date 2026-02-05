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

    # [NEW] 특정 값으로 그리퍼 제어 (컵 집기용 260 등)
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
        # [NEW] 쉐이킹을 위한 주기 운동 명령어 가져오기
        self.move_periodic = dsr.move_periodic
        dsr.set_robot_mode(dsr.ROBOT_MODE_AUTONOMOUS)

    def move_to_xyz(self, x, y, z, vel=config.VEL_FAST, acc=config.ACC_FAST) -> None:
        self.movel([x, y, z, config.RX, config.RY, config.RZ], vel=vel, acc=acc)

    # [NEW] 6축 좌표(x,y,z,a,b,c)로 이동하는 함수 추가
    def move_to_pos(self, pos_list, vel=config.VEL_FAST, acc=config.ACC_FAST) -> None:
        """
        pos_list: [x, y, z, rx, ry, rz] 형태의 리스트
        """
        self.movel(pos_list, vel=vel, acc=acc)

    # [NEW] 쉐이킹 동작 함수 추가
    def shake(self, amp, period, repeat, atime, ref=None):
        """
        주기적인 반복 운동(쉐이킹)을 수행합니다.
        :param amp: [x, y, z, rx, ry, rz] 진폭 (mm, deg)
        :param period: [x, y, z, rx, ry, rz] 주기 (sec)
        :param repeat: 반복 횟수
        :param atime: 가속/감속 시간
        """
        # ref가 없으면 기본적으로 Tool 좌표계(DR_TOOL) 또는 Base(DR_BASE) 사용
        # 여기서는 움직임이 직관적인 DR_BASE(로봇 바닥 기준)를 사용하겠습니다.
        if ref is None:
            ref = self.dsr.DR_BASE

        self.node.get_logger().info(
            f"Adding periodic motion: amp={amp}, period={period}"
        )
        self.move_periodic(amp=amp, period=period, atime=atime, repeat=repeat, ref=ref)

    def go_home(self) -> None:
        self.movej(
            self.posj(*config.HOME_JOINTS), vel=config.VEL_FAST, acc=config.ACC_FAST
        )

    def pause(self, sec: float) -> None:
        time.sleep(sec)
