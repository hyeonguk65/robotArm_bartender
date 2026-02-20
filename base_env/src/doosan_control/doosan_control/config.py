ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
RX, RY, RZ = 0.0, 180.0, 0.0

# 그리퍼 값 설정
GRIPPER_OPEN_VAL = 0
GRIPPER_CLOSE_VAL = 700
GRIPPER_CUP_CLOSE = 160  # 쉐이커 집기 값 (160)
GRIPPER_TOP_CLOSE = 510  # 쉐이커 뚜껑 값 (510)
GRIPPER_CUSTOMER_CLOSE = 180  # 손님 컵 값 (180)

GRIPPER_WAIT_SEC = 2.5  # 동작 사이 안정화 대기 시간

# 속도/가속도 설정
VEL_FAST = 100
ACC_FAST = 100
VEL_SLOW = 50
ACC_SLOW = 50

# [NEW] 바텐더 로봇 동작 좌표 (x, y, z, `a`, b, c)
# --- [1] 얼음 드랍 ---
POS_ICE_DROP = [252.0, -185.0, 300.0, 0.0, 180.0, 0.0]
POS_ICE_DROP_DOWN = [252.0, -185.0, 270.0, 0.0, 180.0, 0.0]

# --- [2] 그리퍼2 (쉐이커 거치대) 관련 ---
POS_GRIPPER2_HOVER = [295.0, 347.0, 260.0, 93.06, -90.0, 90.0]
POS_GRIPPER2_LAND = [301.0, 347.0, 115.0, 88.61, -90.0, 90.0]

# --- [3] 디스펜서 관련 ---
POS_DISPENSER_READY = [550.0, 50.0, 225.0, 176.72, -90.0, 90.0]
POS_DISPENSER_PUSH = [630.0, 50.0, 225.0, 176.72, -90.0, 90.0]

# --- [5] 뚜껑 (Lid) 관련 ---
POS_LID_HOVER = [155.0, 265.0, 400.0, 0.0, -180.0, 0.0]
POS_LID_PICK = [155.0, 265.0, 190.0, 0.0, -180.0, 0.0]
POS_LID_DROP = [155.0, 265.0, 210.0, 0.0, -180.0, 0.0]

# --- [6] 뚜껑-쉐이커 결합 관련 ---
POS_LID_SHAKER_HOVER = [303.0, 230.0, 450.0, 0.0, -180.0, 0.0]
POS_LID_SHAKER_LAND = [303.0, 230.0, 342.0, 0.0, -180.0, 0.0]

# --- [7] 푸어링 (따르기) 관련 ---
POS_POUR_STANDBY = [268.0, 70.0, 260.0, 93.94, -90.0, 90.0]
POS_POUR_READY = [393.0, -27.0, 180.0, 51.82, -90.0, 90.0]
POS_POUR_ACTION = [293.0, -160.0, 278.0, 65.0, 173.0, 116.0]

# 쉐이킹을 수행할 안전한 허공 위치 (로봇이 자유롭게 흔들 수 있는 공간)
POS_SHAKE_READY = [395.0, 65.0, 450.0, 0.0, -180.0, 0.0]

# Amp (진폭): [x, y, z, rx, ry, rz]
# Z(상하): 40mm로 줄여서 모터 부하 감소
# Ry(손목): 8도로 줄여서 안정성 확보
SHAKE_AMP = [0, 0, 40, 0, 8, 0]

# Period (주기): 한 번 움직이는 시간 (초)
# 1.2초로 늘려 부드러운 왕복 보장
SHAKE_PERIOD = [0, 0, 1.2, 0, 1.2, 0]

# Repeat: 반복 횟수
# 7초 대기 시간에 맞추기 위해 7회로 설정
SHAKE_REPEAT = 7

# Atime: 가속 시간
# 0.8초로 연장하여 연속 동작 시 충격 방지
SHAKE_ATIME = 0.8

# --- [8] 최종 컵 회수 및 서빙 ---
POS_CUP_RECOVERY = [254.0, -50.0, 140.0, 88.53, -90.0, 90.0]
POS_CUP_RECOVERY_DOWN = [258.0, -65.0, 70.0, 88.53, -90.0, 90.0]
POS_CUP_RECOVERY_LIFT = [254.0, -65.0, 300.0, 88.53, -90.0, 90.0]

POS_CUSTOMER_HOVER = [583.0, 167.0, 300.0, 174.74, -90.0, 90.0]
POS_CUSTOMER_LAND = [583.0, 167.0, 60.0, 174.74, -90.0, 90.0]
POS_CUSTOMER_DOWN = [583.0, 109.0, 60.0, 49.58, -90.0, 90.0]

LINUX_ARRIVAL_IP = "127.0.0.1"
DOCKER_ACK_BIND_IP = "0.0.0.0"
LINUX_ARRIVAL_PORT = 5000
DOCKER_ACK_PORT = 5001
DOCKER_ACK_TIMEOUT = 10.0

HOME_JOINTS = (0, 0, 90, 0, 90, 0)
HOVER_OFFSET_Z = 120.0
DROP_OFFSET_Z = 0
