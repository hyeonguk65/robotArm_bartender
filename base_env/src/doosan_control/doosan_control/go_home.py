import rclpy
import DR_init
import sys
import time # 대기 시간을 위해 추가

def main(args=None):
    rclpy.init(args=args)

    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    VEL = 50
    ACC = 50
   
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = rclpy.create_node('example_py', namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    from DSR_ROBOT2 import(
        movej, movel, posj, posx,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS
    )
   
    # 로봇 모드 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
   
    # 1. 홈 위치 (기존 좌표)
    P0 = posj(0, 0, 90, 0, 90, 0)
    
    # 2. 추가된 위치 (앞으로 살짝 숙이는 자세)
    # 각도 정보: [J1, J2, J3, J4, J5, J6]
    P1 = posj(0, 25, 65, 0, 90, 0)

    print("2번 추가 좌표(P1)로 이동합니다...")
    movej(P1, VEL, ACC)

    print("1번 홈 위치로 이동합니다...")
    movej(P0, VEL, ACC)
    
    # 이동 완료 후 1초 대기
    time.sleep(1.0)

    

    print("Example complete")
    rclpy.shutdown()

if __name__ == '__main__':
    main()