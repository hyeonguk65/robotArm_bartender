import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraVerifyNode(Node):
    def __init__(self):
        super().__init__('camera_verify_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Waiting for /camera/color/image_raw...')

    def listener_callback(self, msg):
        self.get_logger().info('Received an image!')
        # 간단하게 이미지 크기만 출력하여 확인
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channels = cv_image.shape
        self.get_logger().info(f'Image size: {width}x{height}')
        # 첫 번째 이미지를 받으면 종료
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = CameraVerifyNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Success: Image received and verified.')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
