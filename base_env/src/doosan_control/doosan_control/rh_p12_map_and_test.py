import serial
import time
from dynamixel_sdk import *

# ================= 1. 설정 구간 (Linux 기준 수정) =================
ARDUINO_PORT = "/dev/ttyACM1"  # 보통 아두이노는 ACM 번호를 가집니다.
DXL_PORT = "/dev/ttyUSB0"  # U2D2는 보통 USB 번호를 가집니다.
BAUDRATE = 57600
DXL_ID = 1
PROTOCOL_VERSION = 2.0

# 그리퍼 주소값 (성공 코드 기준)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 512
ADDR_GOAL_POSITION = 564

# 동작 범위
OPEN_POS = 0
CLOSE_POS = 740  # 성공 코드의 740 사용 (안전하게 1150 대신 740으로 시작)
# =====================================================================

# 1. 다이내믹셀(U2D2) 통신 준비
portHandler = PortHandler(DXL_PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("오류: U2D2 포트를 열 수 없습니다.")
    quit()
if not portHandler.setBaudRate(BAUDRATE):
    print("오류: 다이내믹셀 통신 속도 설정 실패.")
    quit()

# --- [중요] 그리퍼 초기 설정 (성공 코드의 핵심 부분) ---
# 1단계: 토크 끄기 (설정 변경을 위해)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
# 2단계: 위치 제어 모드(3)로 설정
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 3)
# 3단계: 토크 켜기
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
print("그리퍼 초기 설정 완료! 힘이 들어갔습니다.")

# 2. 아두이노 연결
try:
    ser_arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=1)
    print("아두이노 센서 연결 성공!")
except Exception as e:
    print(f"오류: 아두이노 포트를 열 수 없습니다. ({e})")
    quit()
    
# 3. 실시간 제어 루프
print("시스템 가동... 센서에 손을 대보세요!")
current_state = "OPEN"  # 중복 명령 방지용 변수

try:
    while True:
        if ser_arduino.readable():
            line = ser_arduino.readline().decode("utf-8").strip()

            if line.isdigit():
                distance = int(line)
                # print(f"거리: {distance} mm", end='\r')

                # 로직: 50mm(5cm)보다 가까우면 닫기
                if 0 < distance < 130:
                    if current_state == "OPEN":
                        print(f"\n[감지] 거리: {distance}mm -> 그리퍼 닫기")
                        packetHandler.write4ByteTxRx(
                            portHandler, DXL_ID, ADDR_GOAL_POSITION, CLOSE_POS
                        )
                        current_state = "CLOSE"

                # 로직: 150mm(15cm)보다 멀어지면 열기
                elif distance > 150:
                    if current_state == "CLOSE":
                        print(f"\n[이탈] 거리: {distance}mm -> 그리퍼 열기")
                        packetHandler.write4ByteTxRx(
                            portHandler, DXL_ID, ADDR_GOAL_POSITION, OPEN_POS
                        )
                        current_state = "OPEN"

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n중단됨. 토크를 해제합니다.")
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    ser_arduino.close()
    portHandler.closePort()
