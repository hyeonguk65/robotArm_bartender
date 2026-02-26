import socket

from . import config



def send_signal(ip: str, message: str, port: int = 5000, timeout: float = 3.0) -> None:
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

        # 홈
        self.robot.go_home()
        self.robot.pause(0.2)

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

    def process_cocktail_action(self, menu_name: str = "") -> None:
        """
        얼음 드랍 -> 컵 회수 -> 디스펜서 -> 컵 놓기 시나리오
        """
        self.node.get_logger().info("Starting Cocktail Action Sequence...")

        # 1. 얼음 드랍 위치로 이동
        self.robot.move_to_pos(config.POS_ICE_DROP)
        self.robot.pause(0.2)

        # 얼음 드랍 다운
        self.robot.move_to_pos(config.POS_ICE_DROP_DOWN)
        self.robot.pause(0.2)

        # 그리퍼 열기 (얼음 낙하)
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 얼음 드랍 위치로 이동
        self.robot.move_to_pos(config.POS_ICE_DROP)
        self.robot.pause(0.2)

        # 3. 데드락 방지 홈 복귀
        self.robot.go_home()
        self.robot.pause(0.2)

        # 그리퍼2 상공
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 그리퍼2 착륙
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.2)

        # 그리퍼 닫기(260)
        self.gripper.move(config.GRIPPER_CUP_CLOSE)
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 그리퍼2 상공
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 7. 디스펜서로 이동
        self.robot.move_to_pos(config.POS_DISPENSER_READY)
        self.robot.pause(0.2)

        # 8. 디스펜서로 전진 (Push)
        self.robot.move_to_pos(config.POS_DISPENSER_PUSH)
        self.robot.pause(4.0)
        
        name_lower = menu_name.lower()
        if "gin" in name_lower or "jin" in name_lower or "진" in name_lower:
            # 진토닉 분기
            self.node.get_logger().info(f"Dispensing Gin Tonic (menu: {menu_name})")
            self.robot.move_to_pos(config.POS_DISPENSER_JINTONIC_PUSH)
            self.robot.pause(4.0)
        elif "jack" in name_lower or "잭" in name_lower:
            # 잭콕 분기
            self.node.get_logger().info(f"Dispensing Jack Coke (menu: {menu_name})")
            self.robot.move_to_pos(config.POS_DISPENSER_JACKDANIELS_PUSH)
            self.robot.pause(4.0)
        else:
            # 기본
            self.node.get_logger().info(f"Dispensing Default (menu: {menu_name})")
            self.robot.pause(4.0)  # 음료 나오는 시간 대기 (필요시 config로 이동)

        # 9. 디스펜서 후진 (Pull)
        self.robot.move_to_pos(config.POS_DISPENSER_READY)
        self.robot.pause(0.2)

        # 11. 그리퍼2(놓는 곳) 상공 이동
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 12. 착륙 (Land)
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.2)

        # 13. 그리퍼 열기 (컵 놓기)
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 11. 그리퍼2(놓는 곳) 상공 이동
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 뚜껑 상공 이동
        self.robot.move_to_pos(config.POS_LID_HOVER)
        self.robot.pause(0.2)

        # 뚜껑 착륙
        self.robot.move_to_pos(config.POS_LID_PICK)
        self.robot.pause(0.2)

        # 그리퍼 닫기(510)
        self.gripper.move(config.GRIPPER_TOP_CLOSE)
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 뚜껑 상공 이동
        self.robot.move_to_pos(config.POS_LID_HOVER)
        self.robot.pause(0.2)

        # 뚜껑-쉐이커 상공 이동
        self.robot.move_to_pos(config.POS_LID_SHAKER_HOVER)
        self.robot.pause(0.2)

        # 뚜껑-쉐이커 착륙
        self.robot.move_to_pos(config.POS_LID_SHAKER_LAND)
        self.robot.pause(0.2)

        # 2. 그리퍼 열기
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 11. 그리퍼2(놓는 곳) 상공 이동
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 12. 착륙 (Land)
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.2)

        # 5. 그리퍼 닫기
        self.gripper.move(config.GRIPPER_CUP_CLOSE)
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 11. 그리퍼2(놓는 곳) 상공 이동
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 쉐이킹 동작 구현
        self.node.get_logger().info(">>> Shaking Start! (Mode: Vertical & Fast)")

        # 1) 쉐이킹 위치 이동
        self.robot.move_to_pos(config.POS_SHAKE_READY)
        self.robot.pause(1.5)

        # 2) 쉐이킹 실행
        self.robot.shake(
            amp=config.SHAKE_AMP,
            period=config.SHAKE_PERIOD,
            repeat=config.SHAKE_REPEAT,
            atime=config.SHAKE_ATIME,
        )

        self.node.get_logger().info(">>> Shaking in progress...")

        # 3) 대기 시간 수정
        self.robot.pause(3.0)

        self.node.get_logger().info(">>> Shaking Done.")

        # 쉐이킹 직후 안정화 대기 (짧게 유지)
        self.robot.pause(1.0)

        # 11. 그리퍼2(놓는 곳) 상공 이동
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 12. 착륙 (Land)
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.2)

        # 그리퍼 열기
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 도착 신호 전송 (센서 동작 시작)
        send_signal(config.LINUX_ARRIVAL_IP, "arrived", config.LINUX_ARRIVAL_PORT)

        # 다시 그리퍼2 상공 좌표로 이동
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 뚜껑-쉐이커 상공 이동
        self.robot.move_to_pos(config.POS_LID_SHAKER_HOVER)
        self.robot.pause(0.2)

        # 뚜껑-쉐이커 착륙
        self.robot.move_to_pos(config.POS_LID_SHAKER_LAND)
        self.robot.pause(0.2)

        # 그리퍼 닫기(550)
        self.gripper.move(config.GRIPPER_TOP_CLOSE)
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 뚜껑-쉐이커 상공 이동
        self.robot.move_to_pos(config.POS_LID_SHAKER_HOVER)
        self.robot.pause(0.2)

        # 작업 완료 신호 전송 (센서 무관하게 그리퍼2 열기)
        send_signal(config.LINUX_ARRIVAL_IP, "finished", config.LINUX_ARRIVAL_PORT)

        # 완전 오픈 ACK 기다림
        wait_for_ack(
            config.DOCKER_ACK_BIND_IP,
            config.DOCKER_ACK_PORT,
            config.DOCKER_ACK_TIMEOUT,
        )

        # 뚜껑 상공 이동
        self.robot.move_to_pos(config.POS_LID_HOVER)
        self.robot.pause(0.2)

        # 뚜껑 놓기
        self.robot.move_to_pos(config.POS_LID_DROP)
        self.robot.pause(0.2)

        # 2. 그리퍼 열기
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 뚜껑 상공 이동
        self.robot.move_to_pos(config.POS_LID_HOVER)
        self.robot.pause(0.2)

        # 그리퍼2 상공
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 그리퍼2 착륙
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.2)

        # 5. 그리퍼 닫기 (컵 집기 - 값 160)
        self.gripper.move(config.GRIPPER_CUP_CLOSE)
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 그리퍼2 상공
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 손님 컵 앞 대기
        self.robot.move_to_pos(config.POS_POUR_STANDBY)
        self.robot.pause(0.2)

        # 손님 컵 앞
        self.robot.move_to_pos(config.POS_POUR_READY)
        self.robot.pause(0.2)

        # 손님 컵에 붓기 (이 부분만 느리게!)
        self.robot.move_to_pos(config.POS_POUR_ACTION, vel=config.VEL_SLOW, acc=config.ACC_SLOW)
        self.robot.pause(1.0) # 붓는 동안 잠시 대기

        # 손님 컵 부은 후 back
        self.robot.move_to_pos(config.POS_POUR_READY)
        self.robot.pause(0.2)

        # 손님 컵 앞 대기
        self.robot.move_to_pos(config.POS_POUR_STANDBY)
        self.robot.pause(0.2)

        # 그리퍼2 상공
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 그리퍼2 착륙
        self.robot.move_to_pos(config.POS_GRIPPER2_LAND)
        self.robot.pause(0.2)

        # 2. 그리퍼 열기
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 그리퍼2 상공
        self.robot.move_to_pos(config.POS_GRIPPER2_HOVER)
        self.robot.pause(0.2)

        # 홈 경유
        self.robot.go_home()
        self.robot.pause(0.2)

        # 컵 회수 위치로 이동 (Approaching Cup)
        self.robot.move_to_pos(config.POS_CUP_RECOVERY)
        self.robot.pause(0.2)

        # 컵 회수 위치 DOWN
        self.robot.move_to_pos(config.POS_CUP_RECOVERY_DOWN)
        self.robot.pause(0.2)

        # 5. 그리퍼 닫기
        self.gripper.move(config.GRIPPER_CUSTOMER_CLOSE)
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 6. 컵 상승
        self.robot.move_to_pos(config.POS_CUP_RECOVERY_LIFT)
        self.robot.pause(0.2)

        # 손님 위치 상공
        self.robot.move_to_pos(config.POS_CUSTOMER_HOVER)
        self.robot.pause(0.2)

        # 손님 위치 착륙
        self.robot.move_to_pos(config.POS_CUSTOMER_LAND)
        self.robot.pause(0.2)

        # 그리퍼 열기
        self.gripper.open()
        self.robot.pause(config.GRIPPER_WAIT_SEC)

        # 손님 위치 상공
        self.robot.move_to_pos(config.POS_CUSTOMER_HOVER)
        self.robot.pause(0.2)

        # 홈 복귀
        self.robot.go_home()
        self.robot.pause(0.2)

        self.node.get_logger().info("Cocktail Action Sequence Complete.")

    def process_wipe_action(self, tx, ty, wx, wy) -> bool:
        """
        수건 집기 -> 지정된 물 위치 닦기 -> 수건 원위치 시나리오
        tx, ty: 수건 픽셀 좌표를 로봇 좌표로 변환한 값
        wx, wy: 물 픽셀 좌표를 로봇 좌표로 변환한 값
        """
        self.node.get_logger().info("Starting Wipe Action Sequence...")
        
        # 기본 닦기 동작 파라미터들
        wipe_line_stroke_mm = 50.0
        wipe_circle_radius_mm = 40.0
        wipe_repeats = 2

        # 1. 수건 집기 (_pick_towel)
        self.node.get_logger().info("1. 수건 집으러 이동 (Pick Towel)")
        self.robot.go_home()
        self.robot.pause(1.0)
        
        # 미리 그리퍼 열기 (완전 개방 0)
        self.gripper.move(0)
        self.robot.pause(1.5)
        
        # 수건 상공
        self.robot.move_to_pos([tx, ty, 250.0, 0.0, 180.0, 0.0], vel=60, acc=80)
        self.robot.pause(1.0)
        
        # 수건 착륙 (하강)
        self.robot.move_to_pos([tx, ty, 140.0, 0.0, 180.0, 0.0], vel=30, acc=80)
        self.robot.pause(1.0)
        
        # 그리퍼 닫기 (수건 꽉 잡기 - 700)
        self.gripper.move(700)
        self.robot.pause(1.5)
        
        # 들어올리기
        self.robot.move_to_pos([tx, ty, 350.0, 0.0, 180.0, 0.0], vel=60, acc=80)
        self.robot.pause(1.0)

        # 2. 물 닦기 (_wipe_water)
        self.node.get_logger().info("2. 물 닦으러 이동 (Wipe Water)")
        # 물 상공
        self.robot.move_to_pos([wx, wy, 250.0, 0.0, 180.0, 0.0], vel=60, acc=80)
        self.robot.pause(1.0)
        
        # 물 위치 하강 (조금 더 강하게 누를 수 있음)
        z_wipe = 140.0
        self.robot.move_to_pos([wx, wy, z_wipe, 0.0, 180.0, 0.0], vel=25, acc=80)
        self.robot.pause(1.0)

        half_stroke = wipe_line_stroke_mm / 2.0
        self.node.get_logger().info(f"직선 닦기 시도 (반폭: {half_stroke:.1f}mm)")
        for _ in range(wipe_repeats):
            # Y축 왕복 (반경 15를 주어 부드럽게 넘어가도록 함)
            self.robot.move_to_pos([wx, wy - half_stroke, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)
            self.robot.move_to_pos([wx, wy + half_stroke, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)
            # X축 왕복
            self.robot.move_to_pos([wx - half_stroke, wy, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)
            self.robot.move_to_pos([wx + half_stroke, wy, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)

        self.node.get_logger().info(f"스파이럴(원형) 닦기 시도 (반지름: {wipe_circle_radius_mm:.1f}mm)")
        r = wipe_circle_radius_mm
        self.robot.move_to_pos([wx + r, wy, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)
        self.robot.move_to_pos([wx, wy + r, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)
        self.robot.move_to_pos([wx - r, wy, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)
        self.robot.move_to_pos([wx, wy - r, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80, radius=15.0)
        # 중심 복귀
        self.robot.move_to_pos([wx, wy, z_wipe, 0.0, 180.0, 0.0], vel=40, acc=80)
        self.robot.pause(0.2)

        # 닦기 완료 후 상승
        self.robot.move_to_pos([wx, wy, 250.0, 0.0, 180.0, 0.0], vel=60, acc=80)
        self.robot.pause(1.0)
        
        # 3. 수건 원래 위치에 두기 (_place_towel_back)
        self.node.get_logger().info("3. 수건 원위치 복귀 (Place Towel Back)")
        # 수건 상공으로 이동
        self.robot.move_to_pos([tx, ty, 250.0, 0.0, 180.0, 0.0], vel=60, acc=80)
        self.robot.pause(1.0)
        
        # 하강
        self.robot.move_to_pos([tx, ty, 140.0, 0.0, 180.0, 0.0], vel=30, acc=80)
        self.robot.pause(1.0)
        
        # 그리퍼 열기 (수건 놓기 - 0)
        self.gripper.move(0)
        self.robot.pause(1.5)
        
        # 상승
        self.robot.move_to_pos([tx, ty, 350.0, 0.0, 180.0, 0.0], vel=60, acc=80)
        self.robot.pause(1.0)
        
        self.robot.go_home()
        self.robot.pause(0.2)

        self.node.get_logger().info("Wipe Action Sequence Complete.")
        return True
