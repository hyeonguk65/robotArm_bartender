import serial
import time
import socket
import threading
from dynamixel_sdk import *

# =====================================================================
# 1. 설정 구간
# =====================================================================
ARDUINO_PORT = 'COM5'
DXL_PORT     = 'COM4'
BAUDRATE     = 57600
DXL_ID       = 1
PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE  = 512
ADDR_GOAL_POSITION  = 564

OPEN_POS  = 0
CLOSE_POS = 740

MY_IP   = "0.0.0.0"
MY_PORT = 5000

# 도커 ACK 수신 서버 정보
DOCKER_IP = "192.168.0.9"   # <- 도커 IP로 수정
DOCKER_ACK_PORT = 5001

robot_arrived = False
force_open = False
pending_ack = False

# 강제 오픈 후 유지 시간 (초)
FORCE_OPEN_DWELL_SEC = 2.0

# =====================================================================
# 2. 소켓 서버 쓰레드 (도커 신호 수신용)
# =====================================================================

def socket_server_thread():
    global robot_arrived, force_open, pending_ack

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((MY_IP, MY_PORT))
    server_sock.listen(1)

    print(f"[*] 윈도우 서버 가동: {MY_PORT}번 포트 대기 중...")

    while True:
        try:
            conn, addr = server_sock.accept()
            data = conn.recv(1024).decode('utf-8').strip()

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


threading.Thread(target=socket_server_thread, daemon=True).start()

# =====================================================================
# 3. 장치 초기화 (Dynamixel & Arduino)
# =====================================================================
portHandler = PortHandler(DXL_PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("오류: 다이내믹셀 통신 실패. 포트와 ID를 확인하세요."); quit()

packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 3)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
print("그리퍼 초기화 완료 (Torque ON).")

try:
    ser_arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=1)
    print("아두이노 센서 연결 성공.")
except Exception as e:
    print(f"아두이노 연결 실패: {e}"); quit()

# =====================================================================
# 4. 메인 제어 루프
# =====================================================================
current_state = "OPEN"
print("시스템 가동... 도커 신호를 기다리는 중입니다.")

try:
    while True:
        # finished 신호 처리: 센서 무관 즉시 오픈
        if force_open:
            packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, OPEN_POS)
            current_state = "OPEN"
            robot_arrived = False
            force_open = False
            print("[작동] finished 신호로 그리퍼 강제 오픈 완료")

            # 오픈 완료 후 도커에 ACK 전송
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

        if ser_arduino.readable():
            line = ser_arduino.readline().decode('utf-8').strip()
            if line.isdigit():
                distance = int(line)

                if 0 < distance < 130:
                    if current_state == "OPEN" and robot_arrived:
                        print(f"\n[작동] 거리 감지({distance}mm) -> 닫기 실행")
                        packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, CLOSE_POS)
                        current_state = "CLOSE"

                elif distance > 150:
                    if current_state == "CLOSE":
                        print(f"\n[이탈] 거리 멀어짐 -> 열기 및 신호 리셋")
                        packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, OPEN_POS)
                        current_state = "OPEN"
                        robot_arrived = False

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n[중단] 프로그램을 종료합니다. 토크를 해제합니다.")
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    ser_arduino.close()
    portHandler.closePort()
