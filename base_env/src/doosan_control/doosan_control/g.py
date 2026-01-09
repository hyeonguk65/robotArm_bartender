import rclpy
import time
import DR_init
from rclpy.node import Node
from std_msgs.msg import String

# 형욱 님이 사용하는 두산 그리퍼 컨트롤러 라이브러리 임포트
from doosan_control.gripper_drl_controller import GripperController
from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS

# 로봇 ID 설정 (형욱님 코드 기준)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

class GripperServiceNode(Node):
    def __init__(self):
        super().__init__('gripper_service_node')

        # 1. ROS2 통신 설정 (기존과 동일)
        self.sub_cmd = self.create_subscription(String, '/gripper/command', self.cmd_callback, 10)
        self.pub_res = self.create_publisher(String, '/gripper/result', 10)

        # 2. 그리퍼 컨트롤러 초기화 (형욱님 코드 반영)
        try:
            # GripperController 객체 생성 (현재 노드를 넘겨줌)
            self.gripper = GripperController(node=self, dsr_node=self, namespace=ROBOT_ID)
            
            # 그리퍼 초기화 시도
            if not self.gripper.initialize():
                self.get_logger().error("❌ 그리퍼 초기화 실패! (케이블 연결 등을 확인하세요)")
            else:
                self.get_logger().info("✅ [그리퍼 노드] 초기화 성공! 명령 대기 중...")
                
                # 안전을 위해 자율 모드 설정 (형욱님 코드 반영)
                set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        except Exception as e:
            self.get_logger().error(f"그리퍼 설정 중 에러 발생: {e}")

    def cmd_callback(self, msg: String):
        command = msg.data.lower().strip()
        
        # 3. 로봇 제어 노드에서 'open' 명령이 오면
        if command == "open":
            print(f"\n>> 📡 [명령 수신] Open 요청")
            
            # 형욱 님이 알려준 방식: 0 이면 열기
            print(">> 🔓 그리퍼 여는 중... (move(0))")
            self.gripper.move(0) 
            
            # 물리적으로 열릴 시간 1초 대기 (안전빵)
            time.sleep(1.0)

            # 완료 신호 전송
            self.send_result("done")

        # 4. 'close' 명령이 오면
        # elif command == "close":
        #     print(f"\n>> 📡 [명령 수신] Close 요청")

        #     # 형욱 님이 알려준 방식: 700 이면 닫기
        #     print(">> 🔒 그리퍼 닫는 중... (move(700))")
        #     self.gripper.move(700)
            
        #     # 꽉 잡을 시간 1초 대기
        #     time.sleep(1.0)
            
        #     self.send_result("done")

    def send_result(self, status):
        msg = String()
        msg.data = status
        self.pub_res.publish(msg)
        self.get_logger().info(f"📤 [완료 보고] 로봇에게 '{status}' 전송함.")

def main(args=None):
    rclpy.init(args=args)

    # 두산 로봇 라이브러리 초기화 (형욱님 코드의 main 부분 반영)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    # 노드 생성 및 실행
    node = GripperServiceNode()
    
    # 두산 라이브러리에 노드 등록
    DR_init.__dsr__node = node

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 그리퍼 안전 종료
        if hasattr(node, 'gripper') and node.gripper:
            node.gripper.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
