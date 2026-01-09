import sys
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
RX, RY, RZ = 0.0, 180.0, 0.0 

class DoosanPickAndPlace(Node):
    # [상태 정의] 딱 필요한 단계만 남김
    IDLE = 0            # 대기 중
    MOVE_ABOVE = 1      # 좌표로 이동 (Hover)
    REQ_GRIP_OPEN = 2   # 도착했으니 그리퍼 열라고 명령
    WAIT_GRIP_OPEN = 3  # 그리퍼가 다 열릴 때까지 대기
    DONE = 99           # 끝 (멈춤)

    def __init__(self, dsr):
        super().__init__("doosan_pick_and_place")
        
        from DSR_ROBOT2 import movel, movej, posj, get_current_posx, set_robot_mode, ROBOT_MODE_AUTONOMOUS
        self.movel = movel; self.movej = movej; self.posj = posj
        self.get_current_posx = get_current_posx
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        self.goal = None
        self.prev_target = None
        self.state = self.IDLE
        self.sent = False
        self.gripper_finished = False 
        
        self.hover = 100.0 # 물체 위 10cm에서 멈춤
        self.v_fast, self.a_fast = 50, 60

        # 통신 설정
        self.sub_coord = self.create_subscription(PointStamped, "/hand_target_point", self.target_cb, 10)
        self.pub_gripper = self.create_publisher(String, "/gripper/command", 10)
        self.sub_gripper_res = self.create_subscription(String, "/gripper/result", self.gripper_res_cb, 10)

        # 0.02초마다 상태 체크 (Loop)
        self.timer = self.create_timer(0.02, self.step_loop)

        # 시작 시 초기 위치로 이동
        self.get_logger().info("🚀 로봇 초기화 중... 홈 위치로 이동합니다.")
        P0 = self.posj(0,0,90,0,90,0)
        self.movej(P0, vel=50, acc=50)
        self.get_logger().info("✅ 준비 완료! 비전 좌표를 기다립니다.")

    def gripper_res_cb(self, msg: String):
        """그리퍼 노드에서 'done'이 오면 실행됨"""
        if msg.data == "done":
            self.get_logger().info("📩 [응답 수신] 그리퍼가 동작을 완료했답니다.")
            self.gripper_finished = True

    def target_cb(self, msg: PointStamped):
        """비전에서 좌표가 오면 실행됨"""
        if self.state != self.IDLE: return # 이미 일하고 있으면 무시

        x, y, z = msg.point.x * 1000.0, msg.point.y * 1000.0, msg.point.z * 1000.0
        
        # 중복 좌표 방지
        if self.prev_target and max([abs(a-b) for a,b in zip((x,y,z), self.prev_target)]) < 5.0: return

        self.prev_target = (x,y,z); self.goal = (x,y,z)
        self.state = self.MOVE_ABOVE
        self.sent = False
        self.get_logger().info(f"🎯 [비전 수신] 목표 좌표 설정: {self.goal}")
        self.get_logger().info("➡️ 상태 변경: 이동 시작 (MOVE_ABOVE)")

    def reached(self, target, tol=5.0):
        """현재 위치가 목표와 가까운지 확인"""
        try:
            cur, _ = self.get_current_posx()
            cur_xyz = tuple(cur[:3])
            diff = max([abs(c-t) for c,t in zip(cur_xyz, target)])
            return diff <= tol
        except: return False

    def step_loop(self):
        if self.goal is None: return
        x, y, z = self.goal

        # 1. 목표 지점 상공(Hover)으로 이동
        if self.state == self.MOVE_ABOVE:
            target_pos = (x, y, z + self.hover) # Z축 + 100mm 위
            
            if not self.sent:
                self.get_logger().info(f"🛫 이동 중... 목표: {target_pos}")
                self.movel([target_pos[0], target_pos[1], target_pos[2], RX, RY, RZ], vel=self.v_fast, acc=self.a_fast)
                self.sent = True
            
            # 도착했는지 확인
            if self.reached(target_pos, tol=10.0):
                self.get_logger().info("📍 도착 완료! 그리퍼에게 신호를 보냅니다.")
                self.state = self.REQ_GRIP_OPEN

        # 2. 도착했으니 그리퍼 열라고 명령 ('open')
        elif self.state == self.REQ_GRIP_OPEN:
            self.get_logger().info("📤 [명령 전송] 그리퍼 열어! (Open)")
            self.pub_gripper.publish(String(data="open"))
            
            self.gripper_finished = False # 응답 기다리기 위해 초기화
            self.state = self.WAIT_GRIP_OPEN 

        # 3. 그리퍼가 다 열렸다는 응답('done') 기다리기
        # elif self.state == self.WAIT_GRIP_OPEN:
        #     if self.gripper_finished: # 콜백함수에서 True로 바뀜
        #         self.get_logger().info("✅ 그리퍼 열기 완료 확인됨.")
        #         self.state = self.DONE

        # # 4. 끝 (여기서 멈춤)
        # elif self.state == self.DONE:
        #     self.get_logger().info("🎉 [미션 성공] 이동 후 그리퍼 열기까지 완료되었습니다.")
        #     self.goal = None # 목표 초기화 (다음 비전 좌표 대기)
        #     self.state = self.IDLE # 다시 처음 상태로 대기

def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node_main", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    import DSR_ROBOT2 as dsr
    node = DoosanPickAndPlace(dsr)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            rclpy.spin_once(dsr_node, timeout_sec=0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()