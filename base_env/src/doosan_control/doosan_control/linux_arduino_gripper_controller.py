import serial
import time
import socket
import threading
from dynamixel_sdk import *
import sys  # sys 모듈 추가 (종료 처리를 위해)

# =====================================================================
# 1. 설정 변수 (이건 함수 밖, 맨 위에 있어도 됩니다)
# =====================================================================
ARDUINO_PORT = "/dev/ttyACM1"
DXL_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
DXL_ID = 1
PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 512
ADDR_GOAL_POSITION = 564

OPEN_POS = 0
CLOSE_POS = 230

MY_IP = "0.0.0.0"
MY_PORT = 5000

DOCKER_IP = "127.0.0.1"
DOCKER_ACK_PORT = 5001

FORCE_OPEN_DWELL_SEC = 2.0

# 전역 변수 (쓰레드와 공유하기 위해)
robot_arrived = False
force_open = False
pending_ack = False


# =====================================================================
# 2. 소켓 서버 쓰레드 (함수 정의는 main 밖에 있어도 됩니다)
# =====================================================================
def socket_server_thread():
    global robot_arrived, force_open, pending_ack

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((MY_IP, MY_PORT))
    server_sock.listen(1)

    print(f"[*] 서버 가동: {MY_PORT}번 포트 대기 중...")

    while True:
        try:
            conn, addr = server_sock.accept()
            data = conn.recv(1024).decode("utf-8").strip()

            if data == "arrived":
                print(f"\n[신호] 도커로부터 'arrived' 수신 -> 닫기 대기")
                robot_arrived = True

            elif data == "finished":
                print(f"\n[신호] 도커로부터 'finished' 수신 -> 강제 열기 요청")
                force_open = True
                pending_ack = True

            conn.close()
        except Exception as e:
            print(f"소켓 에러: {e}")
            time.sleep(1)


# =====================================================================
# 3. 메인 함수 (실제 실행 로직을 여기에 다 넣습니다!)
# =====================================================================
def main(args=None):
    global robot_arrived, force_open, pending_ack  # 전역 변수 사용 선언

    # 1) 쓰레드 시작
    threading.Thread(target=socket_server_thread, daemon=True).start()

    # 2) 다이내믹셀 초기화
    portHandler = PortHandler(DXL_PORT)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        print("오류: 다이내믹셀 통신 실패. 포트와 ID를 확인하세요.")
        return  # 함수 종료

    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 3)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
    print("그리퍼 초기화 완료 (Torque ON).")

    # 3) 아두이노 초기화
    try:
        ser_arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=1)
        print("아두이노 센서 연결 성공.")
    except Exception as e:
        print(f"아두이노 연결 실패: {e}")
        return  # 함수 종료

    # 4) 메인 루프
    current_state = "OPEN"
    print("시스템 가동... 도커 신호를 기다리는 중입니다.")

    try:
        while True:
            # finished 신호 처리
            if force_open:
                packetHandler.write4ByteTxRx(
                    portHandler, DXL_ID, ADDR_GOAL_POSITION, OPEN_POS
                )
                current_state = "OPEN"
                robot_arrived = False
                force_open = False
                print("[작동] finished 신호로 그리퍼 강제 오픈 완료")

                if pending_ack:
                    time.sleep(FORCE_OPEN_DWELL_SEC)
                    try:
                        ack = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        ack.settimeout(3)
                        ack.connect((DOCKER_IP, DOCKER_ACK_PORT))
                        ack.sendall(b"opened")
                        ack.close()
                        print("[ACK] opened 전송 완료")
                    except Exception as e:
                        print(f"[ACK] 전송 실패: {e}")
                    pending_ack = False

            # 아두이노 센서 값 읽기
            if ser_arduino.readable():
                # 데이터가 깨져서 들어올 때를 대비해 try-except 추가
                try:
                    line = ser_arduino.readline().decode("utf-8").strip()
                    if line.isdigit():
                        distance = int(line)

                        if 0 < distance < 130:
                            if current_state == "OPEN" and robot_arrived:
                                print(f"\n[작동] 거리 감지({distance}mm) -> 닫기 실행")
                                packetHandler.write4ByteTxRx(
                                    portHandler, DXL_ID, ADDR_GOAL_POSITION, CLOSE_POS
                                )
                                current_state = "CLOSE"

                        elif distance > 150:
                            if current_state == "CLOSE":
                                print(f"\n[이탈] 거리 멀어짐 -> 열기 및 신호 리셋")
                                packetHandler.write4ByteTxRx(
                                    portHandler, DXL_ID, ADDR_GOAL_POSITION, OPEN_POS
                                )
                                current_state = "OPEN"
                                robot_arrived = (
                                    False  # 이탈 시 도착 정보 초기화 (선택 사항)
                                )
                except Exception:
                    pass

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[중단] 프로그램을 종료합니다.")
        packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
        ser_arduino.close()
        portHandler.closePort()


# ROS 2가 이 파일을 실행할 때 main()을 호출하게 됩니다.
if __name__ == "__main__":
    main()
