import rclpy
import DR_init
import time
import textwrap
from dsr_msgs2.srv import DrlStart


# DRL GRIPPER BASE SCRIPT (WORKING VERSION)
# - includes init loop + recv_check
DRL_GRIPPER_BASE = """
g_slaveid = 0
flag = 0

def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid

def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, 'big')
    data += (6).to_bytes(1, 'big')
    data += (address).to_bytes(2, 'big')
    data += (value).to_bytes(2, 'big')
    return modbus_send_make(data)

def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, 'big')
    data += (16).to_bytes(1, 'big')
    data += (startaddress).to_bytes(2, 'big')
    data += (cnt).to_bytes(2, 'big')
    data += (2 * cnt).to_bytes(1, 'big')
    for i in range(cnt):
        data += (valuelist[i]).to_bytes(2, 'big')
    return modbus_send_make(data)

def recv_check():
    size, val = flange_serial_read(0.1)
    return size > 0, val

def gripper_move(stroke):
    # move command (assumes serial already opened + gripper initialized)
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    wait(1.0)

# --- init loop: keep trying until gripper responds ---
while True:
    flange_serial_open(
        baudrate=57600,
        bytesize=DR_EIGHTBITS,
        parity=DR_PARITY_NONE,
        stopbits=DR_STOPBITS_ONE,
    )
    modbus_set_slaveid(1)

    # enable / init
    flange_serial_write(modbus_fc06(256, 1))
    flag, _ = recv_check()

    # param set (example: speed/force etc. depending on device)
    flange_serial_write(modbus_fc06(275, 400))
    flag, _ = recv_check()

    if flag:
        break

    flange_serial_close()
"""


def main(args=None):
    rclpy.init(args=args)

    # Robot basic setup
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = rclpy.create_node("robot_executor", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej,
        move_periodic,
        posj,
        set_robot_mode,
        ROBOT_MODE_AUTONOMOUS,
    )

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 속도 설정
    VEL = 15
    ACC = 15

    def pause(t):
        time.sleep(t)

    # DRL client (create once)
    cli = node.create_client(DrlStart, f"/{ROBOT_ID}/drl/drl_start")
    while not cli.wait_for_service(timeout_sec=1.0):
        print("⏳ Waiting for DRL service...")

    # Gripper control function (robust)
    # - sends full DRL script (init + move)
    # - waits for service response
    def gripper_move(stroke: int, settle: float = 0.3):
        code = textwrap.dedent(
            DRL_GRIPPER_BASE + f"\n\ngripper_move({int(stroke)})\n"
        )
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = code

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.result() is None:
            print("❌ DRL gripper call failed: no response")
        else:
            if not future.result().success:
                print("❌ DRL gripper call returned success=False")
            else:
                print(f"✅ Gripper command sent: stroke={stroke}")

        time.sleep(settle)

    # Gin Tonic Sequence
    P0 = posj(0, 0, 90, 0, 90, 0)

    print("🏠 Home")
    gripper_move(0, settle=0.5)
    pause(1.0)
    movej(P0, VEL, ACC)
    pause(1.0)

    # 얼음컵 만들기 코드는 삭제 후 vision제어로 변경
    print("🧊 얼음컵 만들기")
    movej([-18, 43.5, 65, 0, 71.5, -18], VEL, ACC)
    pause(1.0)  # 얼음 위치로 이동

    movej([-26, 9, 92, 0, 79, -26], VEL, ACC)
    pause(1.0)  # 얼음->컵 경유 지점

    movej([-32, -2.5, 117, 0, 66, -32], VEL, ACC)
    pause(1.0)  # 얼음 투입

    # 하부 gripper 제작 후 정확한 좌표 다시 지정해야함.
    print("🥃 shaker로 이동")
    movej([1.5, -14, 115.5, 0, 78.5, 1.5], VEL, ACC)
    pause(1.0)  # 얼음컵->shaker 경유 지점1

    movej([70, -11, 113, 0, 78, 70], VEL, ACC)
    pause(1.0)  # 얼음컵->shaker 경유 지점2

    movej([88, 23.5, 75, 0, 81.5, 88], VEL, ACC)
    pause(1.0)  # 얼음컵->shaker 경유 지점3

    movej([90, 50, 116, -70.5, 94.5, 77], VEL, ACC)
    pause(2.0)  # shaker_body 위치

    print("🤏 shaker body 잡기")
    gripper_move(260, settle=0.5)
    # shaker_body 잡기 (확실하게 grab하기 위해 pause_timer 넉넉하게 설정)
    pause(3.0)

    movej([90, 40, 118.5, -71.5, 97, 70.5], VEL, ACC)
    pause(1.0)  # shaker 들기

    print("🥤 디스펜서 위치로 이동")
    movej([7, 53, 113, -171, 76.5, 88], VEL, ACC)
    pause(1.0)  # 음료1번 앞 위치 ex)콜라 앞.

    movej([5, 63, 82.5, -172, 55.5, 85.5], VEL, ACC)
    pause(2.0)  # 디스펜서 push로 음료 추출 (pause_timer 조정할 것)

    movej([7, 53, 113, -171, 76.5, 88], VEL, ACC)
    pause(1.0)  # 음료 추출 완료

    movej([90, 50, 116, -70.5, 94.5, 77], VEL, ACC)
    # shaker_home 위치로 이동(이후 하부 gripper가 shaker 잡아야함)
    pause(1.0)

    print("🧢 뚜껑 닫기")
    gripper_move(0, settle=0.3)
    pause(1.0)

    movej([90, 21, 118.5, -74.5, 102.5, 51], VEL, ACC)
    pause(1.0)  # shaker_home->뚜껑 경유 지점

    movej([129.5, 64, 92.5, -33, 110, 77.5], VEL, ACC)
    pause(1.0)  # 뚜껑_home 위치
    gripper_move(260, settle=0.3)
    pause(2.0)  # 뚜껑 잡기

    movej([117.5, 21.5, 112, -60.5, 117.5, 52.5], VEL, ACC)
    pause(1.0)  # 뚜껑->shaker 경유 지점

    movej([93, 26, 118.5, -71, 103, 57], VEL, ACC)
    pause(1.0)  # 뚜껑 : shaker 위에 위치

    movej([93, 37.5, 118, -69, 99, 67.5], VEL, ACC)
    pause(2.0)  # 뚜껑 닫기
    gripper_move(0, settle=0.3)
    pause(1.0)  # 뚜껑 닫은 후 gripper release

    movej([90, 50, 116, -70.5, 94.5, 77], VEL, ACC)
    pause(1.0)  # shaker에 결합되어 있는 뚜껑 위치
    gripper_move(260, settle=0.3)
    pause(2.0)  # 뚜껑 grab

    print("🍸 Shaking")
    movej([0, 0, 90, -30, 90, 0], VEL, ACC)
    pause(1.0)  # shaking 동작을 위한 위치

    # 안전: 흔들기 전에 한 번 더 잡기
    gripper_move(260, settle=0.3)
    pause(1.0)

    move_periodic(
        [30, 30, 30, 10, 0, 10],
        [3, 3, 3, 3, 3, 3],
        3,
        6
    )
    pause(1.0)  # shaking

    print("🧢 뚜껑 열기")
    movej([90, 50, 116, -70.5, 94.5, 77], VEL, ACC)
    pause(1.0)  # shaker_home 위치
    gripper_move(0, settle=0.3)
    pause(2.0)

    movej([93, 37.5, 118, -69, 99, 67.5], VEL, ACC)
    pause(1.0)  # shaker 결합되어 있는 뚜껑 위치
    gripper_move(260, settle=0.3)
    pause(2.0)  # 뚜껑 잡기

    movej([93, 21.5, 117.5, -72, 104.5, 52], VEL, ACC)
    pause(1.0)  # 뚜껑 해제

    print("🧢 뚜껑 제자리에 놓기")
    movej([129.5, 64, 92.5, -33, 110, 77.5], VEL, ACC)
    pause(1.0)  # 뚜껑_home 위치
    gripper_move(0, settle=0.3)
    pause(2.0)  # 뚜껑 내려놓기

    movej([117.5, 21.5, 112, -60.5, 117.5, 52.5], VEL, ACC)
    pause(1.0)  # 뚜껑->shaker 경유 지점

    movej([90, 50, 116, -70.5, 94.5, 77], VEL, ACC)
    pause(2.0)  # shaker_body 위치

    print("🤏 shaker body 잡기")
    gripper_move(260, settle=0.5)
    # shaker_body 잡기 (확실하게 grab하기 위해 pause_timer 넉넉하게 설정)
    pause(3.0)

    # 컵 위로 이동 후 기울여서 따라야함.
    print("🏁 Done")
    movej(P0, VEL, ACC)
    pause(1.0)
    gripper_move(0, settle=0.3)
    pause(1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
