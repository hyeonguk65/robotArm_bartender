import socket

from . import config


def send_signal(
    ip: str, message: str, port: int = 5000, timeout: float = 3.0
) -> None:
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(timeout)
        client_socket.connect((ip, port))
        client_socket.sendall(message.encode("utf-8"))
        client_socket.close()
        print(f"[OK] '{message}' 신호 전송 완료 -> {ip}:{port}")
    except Exception as exc:
        print(f">>> '{message}' 신호 전송 실패: {exc}")


def wait_for_ack(
    bind_ip: str,
    port: int,
    timeout: float,
    expected: str = "opened",
) -> bool:
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((bind_ip, port))
    server_sock.listen(1)
    server_sock.settimeout(timeout)
    try:
        conn, addr = server_sock.accept()
        data = conn.recv(1024).decode("utf-8").strip()
        conn.close()
        if data == expected:
            print(f"[OK] '{expected}' ACK 수신 <- {addr[0]}:{addr[1]}")
            return True
        print(f"[WARN] ACK 수신 실패 (got='{data}')")
        return False
    except socket.timeout:
        print(f"[WARN] ACK 대기 시간 초과 ({timeout}s)")
        return False
    except Exception as exc:
        print(f"[WARN] ACK 수신 에러: {exc}")
        return False
    finally:
        server_sock.close()


class ActionTasks:
    def __init__(self, node, robot, gripper):
        self.node = node
        self.robot = robot
        self.gripper = gripper

    def pick_target(self, xyz) -> None:
        x, y, z = xyz
        # 1) 접근
        self.robot.move_to_xyz(
            x, y, z + config.HOVER_OFFSET_Z, vel=config.VEL_FAST, acc=config.ACC_FAST
        )
        self.robot.pause(1.0)
        # 2) 그리퍼 오픈
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)
        # 3) 내려감
        self.robot.move_to_xyz(
            x, y, z + config.DROP_OFFSET_Z, vel=config.VEL_SLOW, acc=config.ACC_SLOW
        )
        self.robot.pause(1.0)
        # 4) 그리퍼 클로즈
        self.gripper.close()
        self.robot.pause(config.GRIPPER_WAIT_SEC)
        # 5) 들어올림
        self.robot.move_to_xyz(
            x, y, z + config.HOVER_OFFSET_Z, vel=config.VEL_FAST, acc=config.ACC_FAST
        )

    def process_cocktail_action(self) -> None:
        """
        얼음 드랍 -> 컵 회수 -> 디스펜서 -> 컵 놓기 시나리오
        """
        self.node.get_logger().info("Starting Cocktail Action Sequence...")

        # 기존 시나리오 동작은 보존을 위해 주석 처리
        # ------------------------------------------------------------
        # # 1. 얼음 드랍 위치로 이동
        # self.robot.move_to_pos(config.POS_ICE_DROP)
        # self.robot.pause(0.5)
        #
        # # 2. 그리퍼 열기 (얼음 낙하)
        # self.gripper.open()
        # self.robot.pause(config.GRIPPER_WAIT_SEC)
        #
        # # 3. 데드락 방지 홈 복귀
        # self.robot.go_home()
        # self.robot.pause(0.5)
        #
        # # 그리퍼2 상공
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)
        #
        # # 그리퍼2 착륙
        # self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        # self.robot.pause(0.5)
        #
        # # 그리퍼 닫기(260)
        # self.gripper.move(config.GRIPPER_CUP_CLOSE)
        # self.robot.pause(config.GRIPPER_WAIT_SEC)
        #
        # # 그리퍼2 상공
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)
        # ------------------------------------------------------------

        # # 7. 디스펜서 앞으로 이동
        # self.robot.move_to_pos(config.POS_DISPENSER_READY)
        # self.robot.pause(0.5)

        # # 8. 디스펜서로 전진 (Push)
        # self.robot.move_to_pos(config.POS_DISPENSER_PUSH, vel=config.VEL_SLOW)
        # self.robot.pause(2.0)  # 음료 나오는 시간 대기 (필요시 config로 이동)

        # # 9. 디스펜서 후진 (Pull - 다시 Ready 위치로)
        # self.robot.move_to_pos(config.POS_DISPENSER_READY, vel=config.VEL_SLOW)
        # self.robot.pause(2.0)

        # # 11. 그리퍼2(놓는 곳) 상공 이동
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # # 12. 착륙 (Land)
        # self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        # self.robot.pause(0.5)

        # # 13. 그리퍼 열기 (컵 놓기)
        # self.gripper.open()
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 11. 그리퍼2(놓는 곳) 상공 이동
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # # 뚜껑 상공 이동
        # self.robot.move_to_pos(config.POS_LID_HOVER)
        # self.robot.pause(0.5)

        # # 뚜껑 집기
        # self.robot.move_to_pos(config.POS_LID_PICK)
        # self.robot.pause(0.5)

        # # 그리퍼 닫기(550)
        # self.gripper.move(config.GRIPPER_TOP_CLOSE)
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 뚜껑 상공 이동
        # self.robot.move_to_pos(config.POS_LID_HOVER)
        # self.robot.pause(0.5)

        # # 뚜껑-쉐이커 상공 이동
        # self.robot.move_to_pos(config.POS_LID_SHAKER_HOVER)
        # self.robot.pause(0.5)

        # # 뚜껑-쉐이커 착륙
        # self.robot.move_to_pos(config.POS_LID_SHAKER_LAND)
        # self.robot.pause(0.5)

        # # 2. 그리퍼 열기
        # self.gripper.open()
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 11. 그리퍼2(놓는 곳) 상공 이동
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # # 12. 착륙 (Land)
        # self.robot.move_to_pos(config.POS_GRIPPER2_LAND, vel=config.VEL_SLOW)
        # self.robot.pause(0.5)

        # # 5. 그리퍼 닫기 (컵 집기 - 값 260)
        # self.gripper.move(config.GRIPPER_CUP_CLOSE)
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 11. 그리퍼2(놓는 곳) 상공 이동
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # [NEW] 테스트용 간단 시퀀스 (11~13)
        # 11. 그리퍼2(놓는 곳) 상공 이동
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.5)

        # 12. 착륙 (Land)
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.5)

        # 도착 신호 전송 (윈도우 센서 동작 시작)
        send_signal(config.WINDOWS_ARRIVAL_IP, "arrived", config.WINDOWS_ARRIVAL_PORT)

        # 13. 그리퍼 열기 (컵 놓기)
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 다시 그리퍼2 착륙 좌표로 이동 후 그리퍼 잡기
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.5)
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.5)
        self.gripper.close()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 작업 완료 신호 전송 (윈도우에서 센서 무관하게 그리퍼2 열기)
        send_signal(config.WINDOWS_ARRIVAL_IP, "finished", config.WINDOWS_ARRIVAL_PORT)

        # 윈도우에서 완전 오픈 ACK 기다림
        wait_for_ack(
            config.DOCKER_ACK_BIND_IP,
            config.DOCKER_ACK_PORT,
            config.DOCKER_ACK_TIMEOUT,
        )

        # 초기 위치 복귀
        self.robot.go_home()
        self.robot.pause(0.5)

        # # 뚜껑-쉐이커 상공 이동
        # self.robot.move_to_pos(config.POS_LID_SHAKER_HOVER)
        # self.robot.pause(0.5)

        # # 뚜껑-쉐이커 착륙
        # self.robot.move_to_pos(config.POS_LID_SHAKER_LAND)
        # self.robot.pause(0.5)

        # # 그리퍼 닫기(550)
        # self.gripper.move(config.GRIPPER_TOP_CLOSE)
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 뚜껑-쉐이커 상공 이동
        # self.robot.move_to_pos(config.POS_LID_SHAKER_HOVER)
        # self.robot.pause(0.5)

        # # 뚜껑 상공 이동
        # self.robot.move_to_pos(config.POS_LID_HOVER)
        # self.robot.pause(0.5)

        # # 뚜껑 놓기
        # self.robot.move_to_pos(config.POS_LID_PICK)
        # self.robot.pause(0.5)

        # # 2. 그리퍼 열기
        # self.gripper.open()
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 뚜껑 상공 이동
        # self.robot.move_to_pos(config.POS_LID_HOVER)
        # self.robot.pause(0.5)

        # # 그리퍼2 상공
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # # 그리퍼2 착륙
        # self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        # self.robot.pause(0.5)

        # # 5. 그리퍼 닫기 (컵 집기 - 값 260)
        # self.gripper.move(config.GRIPPER_CUP_CLOSE)
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 그리퍼2 상공
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # # 손님 컵 앞
        # self.robot.move_to_pos(config.POS_POUR_READY)
        # self.robot.pause(0.5)

        # # 손님 컵에 붓기
        # self.robot.move_to_pos(config.POS_POUR_ACTION)
        # self.robot.pause(0.5)

        # # 손님 컵 부은 후 back
        # self.robot.move_to_pos(config.POS_POUR_READY)
        # self.robot.pause(0.5)

        # # 그리퍼2 상공
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # # 그리퍼2 착륙
        # self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        # self.robot.pause(0.5)

        # # 2. 그리퍼 열기
        # self.gripper.open()
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 그리퍼2 상공
        # self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        # self.robot.pause(0.5)

        # # 홈 경유
        # self.robot.go_home()
        # self.robot.pause(0.5)

        # # 4. 컵 회수 위치로 이동 (Approaching Cup)
        # self.robot.move_to_pos(config.POS_CUP_RECOVERY, vel=config.VEL_SLOW)
        # self.robot.pause(0.5)

        # # 5. 그리퍼 닫기 (컵 집기 - 값 260)
        # self.gripper.move(config.GRIPPER_CUP_CLOSE)
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # # 6. 컵 상승
        # self.robot.move_to_pos(config.POS_CUP_RECOVERY_LIFT)
        # self.robot.pause(0.5)

        # # 손님 위치 상공
        # self.robot.move_to_pos(config.POS_CUSTOMER_HOVER)
        # self.robot.pause(0.5)

        # # 손님 위치 착륙
        # self.robot.move_to_pos(config.POS_CUSTOMER_LAND)

        # self.robot.pause(0.5)
        # # 그리퍼 열기
        # self.gripper.open()
        # self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 홈 복귀
        self.robot.go_home()
        self.robot.pause(0.5)

        self.node.get_logger().info("Cocktail Action Sequence Complete.")
