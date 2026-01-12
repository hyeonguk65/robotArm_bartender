# 110.120.1.38
# ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.38 port:=12345 model:=e0509

import rclpy
import DR_init
import sys

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
   
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
   
    P0 = posj(0,0,90,0,90,0)
    print("홈 위치로 이동합니다")

    movej(P0, VEL, ACC)

    print("Example complete")
    rclpy.shutdown()


if __name__ == '__main__':
    main()