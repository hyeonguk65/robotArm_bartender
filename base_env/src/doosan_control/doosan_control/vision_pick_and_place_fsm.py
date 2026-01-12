import json
import sys
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import DR_init
import numpy as np

# 위에서 수정한 컨트롤러 임포트
from doosan_control.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
RX, RY, RZ = 0.0, 180.0, 0.0 

GRIPPER_OPEN_VAL = 0
GRIPPER_CLOSE_VAL = 700

class DoosanPickAndPlace(Node):
    IDLE=0; MOVE_ABOVE=1; GRIP_OPEN=2; MOVE_DOWN=3; GRIP_CLOSE=4; MOVE_UP=5; GO_HOME=6; DONE=7

    def __init__(self, dsr, dsr_node_obj):
        super().__init__("doosan_pick_and_place")

        self.dsr = dsr
        self.dsr_node_obj = dsr_node_obj 

        from DSR_ROBOT2 import movel, movej, posj, get_current_posx, set_robot_mode, ROBOT_MODE_AUTONOMOUS
        self.movel = movel
        self.movej = movej
        self.posj = posj
        self.get_current_posx = get_current_posx
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        self.get_logger().info("✅ Robot Connected Successfully!")

        self.gripper = GripperController(node=self, dsr_node=self.dsr_node_obj, namespace=ROBOT_ID)
        if not self.gripper.initialize():
            self.get_logger().error("❌ Gripper Initialization Failed!")

        self.goal = None
        self.prev_target = None
        self.deadband = 5.0
        self.state = self.IDLE
        self.sent = False
        self.order_active = False
        self.cocktail = None
        self.ice_size = None

        self.hover = 100.0
        self.lift = 150.0
        self.tol = 10.0 
        self.v_fast, self.a_fast = 50, 60
        self.v_slow, self.a_slow = 30, 30
        self.grip_fail_count = 0
        self.grip_max_fail = 3
        self.last_grip_attempt = 0.0
        self.grip_retry_delay = 0.5
        self.last_tcp = None
        self.still_count = 0
        self.still_eps = 1.0
        self.still_need = 25

        self.sub_coord = self.create_subscription(PointStamped, "/hand_target_point", self.target_cb, 10)
        self.pub_resume = self.create_publisher(String, "/llm_command", 10)
        self.sub_order = self.create_subscription(String, "/robot_order", self.order_cb, 10)

        self.timer = self.create_timer(0.02, self.step_loop)

        # 초기 위치 이동
        P0 = self.posj(0,0,90,0,90,0)
        self.movej(P0, vel=50, acc=50)

        self.get_logger().info("[OK] FSM Ready. Waiting for target...")

    def target_cb(self, msg: PointStamped):
        if self.state != self.IDLE or not self.order_active:
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
        self.last_tcp = None
        self.still_count = 0
        self.get_logger().info(f"🎯 goal set: ({x:.1f}, {y:.1f}, {z:.1f})")

    def order_cb(self, msg: String):
        if self.order_active or self.state != self.IDLE:
            self.get_logger().warn("Order received while busy; ignoring.")
            return

        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f"Invalid order payload: {exc}")
            return

        cocktail = data.get("cocktail")
        ice_size = data.get("ice_size", "medium")
        if not cocktail:
            self.get_logger().warn("Order missing cocktail; ignoring.")
            return

        ice_size = str(ice_size).strip().lower()
        if ice_size not in {"small", "medium", "large"}:
            self.get_logger().warn(f"Unknown ice_size '{ice_size}', defaulting to medium.")
            ice_size = "medium"

        self.cocktail = cocktail
        self.ice_size = ice_size
        self.order_active = True

        # YOLO에게 목표 라벨 전달
        cmd = String()
        cmd.data = ice_size
        self.pub_resume.publish(cmd)
        self.get_logger().info(f"📨 Order received: {cocktail} (ice={ice_size})")

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
        self.state = self.IDLE
        self.sent = False
        self.order_active = False
        self.cocktail = None
        self.ice_size = None
        self.grip_fail_count = 0
        self.last_grip_attempt = 0.0
        self.last_tcp = None
        self.still_count = 0
        self.get_logger().info("System Reset. Waiting for NEW target.")

    def is_still(self):
        cur = self.tcp_xyz()
        if cur is None:
            self.still_count = 0
            return False

        if self.last_tcp is None:
            self.last_tcp = cur
            self.still_count = 0
            return False

        dx = max(abs(cur[0]-self.last_tcp[0]), abs(cur[1]-self.last_tcp[1]), abs(cur[2]-self.last_tcp[2]))
        self.last_tcp = cur

        if dx <= self.still_eps:
            self.still_count += 1
        else:
            self.still_count = 0

        return self.still_count >= self.still_need

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
            if self.reached(tx, ty, tz, tol=20.0) and self.is_still():
                self.state = self.GRIP_OPEN
                self.sent = False
                self.last_tcp = None
                self.still_count = 0

        # 2. 그리퍼 벌리기
        elif self.state == self.GRIP_OPEN:
            if time.time() - self.last_grip_attempt < self.grip_retry_delay:
                return
            self.last_grip_attempt = time.time()
            self.get_logger().info("STEP GRIP_OPEN")
            # DRL 내부: open -> wait(0.5) -> write -> wait(1.5) -> close
            ok = self.gripper.move(GRIPPER_OPEN_VAL)
            self.get_logger().info(f"GRIP_OPEN request sent: {ok}")
            if not ok:
                self.grip_fail_count += 1
                self.get_logger().warn(f"GRIP_OPEN failed ({self.grip_fail_count}/{self.grip_max_fail})")
                if self.grip_fail_count >= self.grip_max_fail:
                    self.get_logger().error("GRIP_OPEN failed too many times. Resetting.")
                    self.reset()
                return
            self.grip_fail_count = 0
            time.sleep(2.5) # Python 대기
            self.state = self.MOVE_DOWN
            self.sent = False

        # 3. 내려가기
        elif self.state == self.MOVE_DOWN:
            tx, ty, tz = x, y, z
            if not self.sent:
                self.get_logger().info("STEP MOVE_DOWN")
                self.movel([tx, ty, tz, RX, RY, RZ], vel=self.v_slow, acc=self.a_slow)
                self.sent = True
            if self.reached(tx, ty, tz, tol=10.0) and self.is_still():
                self.state = self.GRIP_CLOSE
                self.sent = False
                self.last_tcp = None
                self.still_count = 0

        # 4. 그리퍼 닫기
        elif self.state == self.GRIP_CLOSE:
            if time.time() - self.last_grip_attempt < self.grip_retry_delay:
                return
            self.last_grip_attempt = time.time()
            self.get_logger().info("STEP GRIP_CLOSE")
            # DRL 내부: open -> wait(0.5) -> write -> wait(1.5) -> close
            ok = self.gripper.move(GRIPPER_CLOSE_VAL)
            self.get_logger().info(f"GRIP_CLOSE request sent: {ok}")
            if not ok:
                self.grip_fail_count += 1
                self.get_logger().warn(f"GRIP_CLOSE failed ({self.grip_fail_count}/{self.grip_max_fail})")
                if self.grip_fail_count >= self.grip_max_fail:
                    self.get_logger().error("GRIP_CLOSE failed too many times. Resetting.")
                    self.reset()
                return
            self.grip_fail_count = 0
            time.sleep(2.5) # Python 대기
            self.state = self.MOVE_UP
            self.sent = False

        # 5. 들어올리기
        elif self.state == self.MOVE_UP:
            tx, ty, tz = x, y, z + self.lift
            if not self.sent:
                self.get_logger().info("STEP MOVE_UP")
                self.movel([tx, ty, tz, RX, RY, RZ], vel=self.v_fast, acc=self.a_fast)
                self.sent = True
            if self.reached(tx, ty, tz, tol=20.0) and self.is_still():
                self.state = self.GO_HOME
                self.sent = False
                self.last_tcp = None
                self.still_count = 0

        # 6. 홈 복귀
        elif self.state == self.GO_HOME:
            if not self.sent:
                self.get_logger().info("STEP GO_HOME")
                P_HOME = self.posj(0,0,90,0,90,0)
                self.movej(P_HOME, vel=50, acc=50)
                self.sent = True
            
            self.state = self.DONE
            self.sent = False

        # 7. 완료 및 리셋
        elif self.state == self.DONE:
            self.get_logger().info("✅ DONE - Sending 'done'")
            m = String()
            m.data = "done"
            self.pub_resume.publish(m)
            self.reset()


def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__srv_name_prefix = f"/{ROBOT_ID}/"
    DR_init.__dsr__topic_name_prefix = f"/{ROBOT_ID}/"
    sys.modules["DR_init"] = DR_init

    import DSR_ROBOT2 as dsr
    
    node = DoosanPickAndPlace(dsr, dsr_node)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            rclpy.spin_once(dsr_node, timeout_sec=0.001)
            
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
