import sys
import rclpy
import time  # 시간 지연을 위해 추가
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import DR_init
import numpy as np

# 수정된 컨트롤러 임포트
from doosan_control.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
# 로봇의 End-Effector 회전값 (상황에 맞게 조절)
RX, RY, RZ = 0.0, 180.0, 0.0 

GRIPPER_OPEN_VAL = 0
GRIPPER_CLOSE_VAL = 700

class DoosanPickAndPlace(Node):
    IDLE=0; MOVE_ABOVE=1; GRIP_OPEN=2; MOVE_DOWN=3; GRIP_CLOSE=4; MOVE_UP=5; GO_HOME=6; DONE=7

    # [수정 1] 초기화 시 dsr_node를 받습니다.
    def __init__(self, dsr, dsr_node_obj):
        super().__init__("doosan_pick_and_place")

        self.dsr = dsr
        self.dsr_node_obj = dsr_node_obj # 저장

        from DSR_ROBOT2 import movel, movej, posj, get_current_posx, set_robot_mode, ROBOT_MODE_AUTONOMOUS
        self.movel = movel
        self.movej = movej
        self.posj = posj
        self.get_current_posx = get_current_posx
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        self.get_logger().info("✅ Robot Connected Successfully!")

        # [수정 2] GripperController에 dsr_node_obj를 함께 전달합니다.
        self.gripper = GripperController(node=self, dsr_node=self.dsr_node_obj, namespace=ROBOT_ID)
        if not self.gripper.initialize():
            self.get_logger().error("❌ Gripper Initialization Failed!")

        self.goal = None
        self.prev_target = None
        self.deadband = 5.0
        self.state = self.IDLE
        self.sent = False

        self.hover = 100.0
        self.lift = 150.0
        self.tol = 6.0
        self.v_fast, self.a_fast = 50, 60
        self.v_slow, self.a_slow = 30, 30

        self.sub_coord = self.create_subscription(PointStamped, "/hand_target_point", self.target_cb, 10)
        self.pub_resume = self.create_publisher(String, "/robot/resume", 10)

        self.timer = self.create_timer(0.02, self.step_loop)

        # 초기 위치 이동
        P0 = self.posj(0,0,90,0,90,0)
        self.movej(P0, vel=50, acc=50)

        self.get_logger().info("[OK] FSM Ready. Waiting for target...")

    def target_cb(self, msg: PointStamped):
        if self.state != self.IDLE:
            return

        x = msg.point.x * 1000.0
        y = msg.point.y * 1000.0
        z = msg.point.z * 1000.0

        if self.prev_target is not None:
            if max(abs(x-self.prev_target[0]), abs(y-self.prev_target[1]), abs(z-self.prev_target[2])) < self.deadband:
                return

        self.prev_target = (x,y,z)
        self.goal = (x,y,z)
        self.state = self.MOVE_ABOVE
        self.sent = False
        self.get_logger().info(f"🎯 goal set: ({x:.1f}, {y:.1f}, {z:.1f})")

    def tcp_xyz(self):
        try:
            cur, _ = self.get_current_posx()
            return float(cur[0]), float(cur[1]), float(cur[2])
        except Exception:
            return None

    def reached(self, tx, ty, tz, tol=None):
        tol = self.tol if tol is None else tol
        cur = self.tcp_xyz()
        if cur is None: return False
        cx, cy, cz = cur
        return max(abs(cx-tx), abs(cy-ty), abs(cz-tz)) <= tol

    def reset(self):
        self.goal = None
        # self.prev_target = None
        self.state = self.IDLE
        self.sent = False

    def step_loop(self):
        if self.goal is None:
            return

        x, y, z = self.goal

        # 1. 접근 위치 이동
        if self.state == self.MOVE_ABOVE:
            tx, ty, tz = x, y, z + self.hover
            if not self.sent:
                self.get_logger().info("STEP MOVE_ABOVE")
                self.movel([tx, ty, tz, RX, RY, RZ], vel=self.v_fast, acc=self.a_fast)
                self.sent = True
            if self.reached(tx, ty, tz, tol=10.0):
                self.state = self.GRIP_OPEN
                self.sent = False

        # 2. 그리퍼 벌리기
        elif self.state == self.GRIP_OPEN:
            if not self.sent:
                self.get_logger().info("STEP GRIP_OPEN")
                ok = self.gripper.move(GRIPPER_OPEN_VAL)
                self.get_logger().info(f"GRIP_OPEN request sent: {ok}")
                
                # [중요] 명령 전송 후 물리적으로 벌어질 시간 대기 (1.5초)
                time.sleep(1.5) 
                
                self.sent = True
                self.state = self.MOVE_DOWN
                self.sent = False

        # 3. 내려가기 (잡으러)
        elif self.state == self.MOVE_DOWN:
            tx, ty, tz = x, y, z
            if not self.sent:
                self.get_logger().info("STEP MOVE_DOWN")
                self.movel([tx, ty, tz, RX, RY, RZ], vel=self.v_slow, acc=self.a_slow)
                self.sent = True
            if self.reached(tx, ty, tz, tol=8.0):
                self.state = self.GRIP_CLOSE
                self.sent = False

        # 4. 그리퍼 닫기
        elif self.state == self.GRIP_CLOSE:
            if not self.sent:
                self.get_logger().info("STEP GRIP_CLOSE")
                ok = self.gripper.move(GRIPPER_CLOSE_VAL)
                self.get_logger().info(f"GRIP_CLOSE request sent: {ok}")

                # [중요] 명령 전송 후 물리적으로 닫힐 시간 대기 (1.5초)
                time.sleep(1.5)

                self.state = self.MOVE_UP
                self.sent = False

        # 5. 들어올리기
        elif self.state == self.MOVE_UP:
            tx, ty, tz = x, y, z + self.lift
            if not self.sent:
                self.get_logger().info("STEP MOVE_UP")
                self.movel([tx, ty, tz, RX, RY, RZ], vel=self.v_fast, acc=self.a_fast)
                self.sent = True
            if self.reached(tx, ty, tz, tol=12.0):
                self.state = self.GO_HOME
                self.sent = False

        # 6. 홈 복귀
        elif self.state == self.GO_HOME:
            if not self.sent:
                self.get_logger().info("STEP GO_HOME")
                P_HOME = self.posj(0,0,90,0,90,0)
                self.movej(P_HOME, vel=30, acc=30)
                self.sent = True
            
            # 여기서 posj 비교 로직을 넣거나, 시간으로 대충 처리 후 DONE
            # 일단 메시지 보냄
            self.state = self.DONE
            self.sent = False

        # 7. 완료 및 리셋
        elif self.state == self.DONE:
            self.get_logger().info("✅ DONE publish /robot/resume=done")
            m = String()
            m.data = "done"
            self.pub_resume.publish(m)
            self.reset()


def main(args=None):
    rclpy.init(args=args)

    # 1. dsr_node 생성
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__srv_name_prefix = f"/{ROBOT_ID}/"
    DR_init.__dsr__topic_name_prefix = f"/{ROBOT_ID}/"
    sys.modules["DR_init"] = DR_init

    import DSR_ROBOT2 as dsr
    
    # [수정 3] dsr_node 객체를 메인 로직에 전달
    node = DoosanPickAndPlace(dsr, dsr_node)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            rclpy.spin_once(dsr_node, timeout_sec=0.001)
            # node.gripper.client_node 스핀은 이제 gripper 내부에서 처리하므로 여기서 필수 아님 
            # 하지만 안전상 놔둬도 무방. 다만 _send_drl_script 내부의 spin이 핵심임.
            rclpy.spin_once(node.gripper.client_node, timeout_sec=0.001)
            
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.gripper.shutdown()
        except Exception:
            pass
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()