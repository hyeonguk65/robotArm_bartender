import rclpy
from dsr_msgs2.srv import DrlStart

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("check_power")
    cli = node.create_client(DrlStart, "/dsr01/drl/drl_start")
    
    # 로봇의 툴(Tool) 디지털 출력 1번과 2번을 강제로 켭니다.
    # (매뉴얼 7.1.9 set_tool_digital_output 참조)
    drl_code = "set_tool_digital_output(1, 1)\nset_tool_digital_output(2, 1)"
    
    req = DrlStart.Request()
    req.robot_system = 0
    req.code = drl_code
    
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result().success:
        print("✅ 툴 전원/신호 ON 명령 전송 완료. 그리퍼 LED를 확인하세요.")
    else:
        print("❌ 명령 전송 실패")

if __name__ == "__main__":
    main()