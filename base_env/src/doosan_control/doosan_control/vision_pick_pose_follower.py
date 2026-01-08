import sys
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

class DoosanFollowTarget(Node):
    def __init__(self):
        super().__init__("doosan_follow_target")
        
        # --- DR_init 및 로봇 연결 (기존과 동일) ---
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        sys.modules['DR_init'] = DR_init
        sys.modules['dsr_common2.DR_init'] = DR_init
        if 'DSR_ROBOT2' in sys.modules:
            del sys.modules['DSR_ROBOT2']

        try:
            from DSR_ROBOT2 import get_current_posx, movel, wait, movej
            self.movel = movel
            self.get_logger().info("✅ Robot Connected Successfully!")
        except Exception as e:
            self.get_logger().error(f"❌ Critical Error during DSR import: {e}")
            raise e

        # ---- Parameters ----
        self.declare_parameter("vel", 30.0)
        self.declare_parameter("acc", 60.0)
        self.declare_parameter("deadband_mm", 5.0)

        self.vel = float(self.get_parameter("vel").value)
        self.acc = float(self.get_parameter("acc").value)
        self.deadband = float(self.get_parameter("deadband_mm").value)
        
        self.prev_target = None
        
        # [핵심 변경 1] 상태 관리 플래그
        # True: 비전 좌표를 따라다님 / False: 도착해서 작업 중이니 비전 무시함
        self.tracking_mode = True 
        self.is_busy = False

        # ---- Subscriber (Input 1: 비전 좌표) ----
        self.sub_coord = self.create_subscription(
            PointStamped,
            "/hand_target_point",
            self.target_cb,
            10
        )

        # [핵심 변경 2] Subscriber (Input 2: 작업 완료 신호)
        # 픽킹 작업이 다 끝나면 누군가 "done"을 보내줘야 다시 추적을 시작함
        self.sub_resume = self.create_subscription(
            String,
            "/robot/resume",
            self.resume_cb,
            10
        )

        # ---- Publisher (Output) ----
        self.pub_status = self.create_publisher(String, "/robot/status", 10)

        self.get_logger().info("[OK] Ready. Waiting for target...")

    # 작업 재개 신호를 받는 콜백 함수
    def resume_cb(self, msg: String):
        if msg.data == "done":
            self.get_logger().info("[:recycle:] Resume Tracking Mode!")
            self.tracking_mode = True
            self.prev_target = None # 이전 좌표 기억 리셋 (새로운 마음으로 시작)
            
    def target_cb(self, msg: PointStamped):

        self.get_logger().info(f">>> Callback Triggered! Mode: {self.tracking_mode}, Busy: {self.is_busy}")
        
        if not self.tracking_mode or self.is_busy:
            return

        # [수정] 역행렬 및 변환 연산 삭제 -> 비전 좌표(m)를 직접 로봇 좌표로 사용
        # 1. 입력받은 미터 단위 좌표를 바로 mm로 변환
        x = msg.point.x * 1000.0
        y = msg.point.y * 1000.0
        z = msg.point.z * 1000.0

        if self.prev_target is not None:
            dx = abs(x - self.prev_target[0])
            dy = abs(y - self.prev_target[1])
            dz = abs(z - self.prev_target[2])
            if max(dx, dy, dz) < self.deadband:
                return
        
        self.prev_target = (x, y, z)
        self.move_robot(x, y, z)

    def move_robot(self, x, y, z):
        self.is_busy = True
        self.get_logger().info(f"[START] Move to {x:.1f}, {y:.1f}, {z:.1f}")

        try:
            self.movel([x, y, z, 0, 180, 0], vel=self.vel, acc=self.acc)
            self.get_logger().info(f"[DONE] Arrived at target.")

            # [핵심 변경 3] 도착했으니 일단 멈춤 (Lock)
            # 이제 "/robot/resume" 토픽이 올 때까지 손이 움직여도 로봇은 가만히 있음
            self.tracking_mode = False 
            
            # 도착 신호 보냄
            msg = String()
            msg.data = "arrived"
            self.pub_status.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Motion Error: {str(e)}")
        
        finally:
            self.is_busy = False 

def main(args=None):
    rclpy.init(args=args)
    
    # 1. dsr_node 생성 및 전역 설정
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    try:
        # 필요한 함수와 상수를 추가로 임포트합니다.
        from DSR_ROBOT2 import (
            movel, movej, posj, 
            set_robot_mode, ROBOT_MODE_AUTONOMOUS
        )
    except ImportError as e:
        print(f"DSR Library Import Failed: {e}")
        rclpy.shutdown()
        exit(1)

    # 2. 로봇을 자동 모드로 설정 (중요!)
    # 이 코드가 있어야 로봇이 스크립트의 명령을 받아들입니다.
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 3. 비전 추적 노드 생성
    node = DoosanFollowTarget()

    # 4. 초기화: 홈 위치로 이동하여 자세 정렬
    # P0는 예제 스크립트에서 사용한 안정적인 홈 자세입니다.
    P0 = posj(0, 0, 90, 0, 90, 0)
    node.get_logger().info("--- [초기화] 홈 위치로 이동 중... ---")
    movej(P0, vel=50, acc=50) # 홈 이동은 관절 이동(movej)이 가장 안전합니다.
    node.get_logger().info("--- [초기화] 완료. 이제 좌표를 기다립니다. ---")

    # 5. 메인 루프 (Spin)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            rclpy.spin_once(dsr_node, timeout_sec=0.001)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()