import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

from sensor_msgs.msg import Image  # 이미지를 통신으로 보내기 위한 편지봉투
from cv_bridge import CvBridge  # OpenCV 이미지를 ROS 메시지로 변환하는 번역기


class YoloRobotBaseVisualizer(Node):
    def __init__(self):
        super().__init__("yolo_robot_base_visualizer")

        # 0) 설정값 (필요시 조절)
        self.conf_th = 0.5
        self.device = 0

        # ROI depth 안정화 설정
        self.roi_radius = 2  # 2 -> 5x5, 3 -> 7x7
        self.min_valid_depth_samples = 6  # ROI에서 유효 depth 최소 개수

        # Lock 전에 평균낼 프레임 수 (30FPS 기준)
        self.lock_window_frames = 10  # 약 0.33초
        self.log_interval_sec = 0.5  # 로그 출력 주기

        # 1) 통신 설정
        self.publisher_ = self.create_publisher(PointStamped, "/hand_target_point", 10)
        # [추가] 이미지 원본을 다른 노드들에게 방송할 퍼블리셔 생성
        self.img_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.cv_bridge = CvBridge()
        self.command_sub = self.create_subscription(
            String, "/robot_order_ice_size", self.command_callback, 10
        )
        # 2) 상태 변수
        self.target_label = None
        self.is_locked = False
        self.locked_xyz = None
        self.locked_label = None
        self.lock_publish_count = 0
        self.lock_publish_max = 1

        # Lock 전에 좌표 모으기 버퍼 (테스트/평균)
        self.xyz_buffer = deque(maxlen=self.lock_window_frames)
        self.last_log_time = time.time()

        # 3) YOLO + RealSense
        self.model = YOLO(
            "/root/robotArm_ws/src/user_pkgs/yolo_service/yolo_service/best.pt"
        )

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.align = rs.align(rs.stream.color)
        profile = self.pipeline.start(config)

        self.intr = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        # 컬러 자동 노출 끄고 고정
        try:
            rgb_sensor = profile.get_device().query_sensors()[1]
            rgb_sensor.set_option(rs.option.enable_auto_exposure, 0)
            rgb_sensor.set_option(rs.option.exposure, 90)
            self.get_logger().info("Camera Exposure Fixed to 90")
        except Exception as e:
            self.get_logger().warn(f"Exposure option not applied: {e}")

        # 사용자 제공 변환행렬
        self.T_base_cam = np.array(
            [
                [-0.017, 0.885, -0.465, 0.848],
                [0.999, 0.002, -0.032, 0.064],
                [-0.027, -0.466, -0.885, 1.06],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        self.is_running = True
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30FPS
        self.get_logger().info("YOLO + ROI-median depth + lock-average node started.")

    # LLM(얼음 크기) 명령 수신
    def command_callback(self, msg: String):
        data = msg.data.strip().lower()
        if not data:
            return
        self.target_label = data
        self.is_locked = False
        self.locked_xyz = None
        self.locked_label = None
        self.lock_publish_count = 0
        self.xyz_buffer.clear()
        self.get_logger().info(f"New Command Received: {self.target_label}")

    # ROI median depth
    def get_roi_median_depth(self, depth_frame, u: int, v: int, w: int, h: int):
        """
        중심 (u,v) 주변 ROI에서 depth 샘플을 모아 median을 반환.
        유효 샘플이 부족하면 (None) 반환.
        """
        depths = []
        r = self.roi_radius

        # ROI가 이미지 밖으로 나가지 않도록 clamp
        u0 = max(0, u - r)
        u1 = min(w - 1, u + r)
        v0 = max(0, v - r)
        v1 = min(h - 1, v + r)

        for vv in range(v0, v1 + 1):
            for uu in range(u0, u1 + 1):
                d = depth_frame.get_distance(uu, vv)
                if d > 0:
                    depths.append(d)

        if len(depths) < self.min_valid_depth_samples:
            return None

        return float(np.median(depths))

    # 좌표 publish
    def publish_point(self, xyz):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x, msg.point.y, msg.point.z = (
            float(xyz[0]),
            float(xyz[1]),
            float(xyz[2]),
        )
        self.publisher_.publish(msg)

    # 메인 루프
    def process_frame(self):
        if not self.is_running:
            return

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]
        display_img = color_image.copy()

        # [추가] 방금 얻은 따끈한 이미지를 방송(Publish)
        if (
            self.img_pub.get_subscription_count() > 0
        ):  # 듣는 사람이 있을 때만 보내면 효율적
            self.img_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            )

        # 1) lock 상태면: 화면은 계속 갱신 + locked_xyz만 publish
        if self.is_locked and self.locked_xyz is not None:
            if self.lock_publish_count < self.lock_publish_max:
                self.publish_point(self.locked_xyz)
                self.lock_publish_count += 1
            cv2.putText(
                display_img,
                f"LOCKED: {self.locked_label}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
            )
            # locked xyz도 화면에 표시
            lx, ly, lz = self.locked_xyz
            cv2.putText(
                display_img,
                f"XYZ(mm): {lx*1000:.1f}, {ly*1000:.1f}, {lz*1000:.1f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            if self.lock_publish_count == self.lock_publish_max:
                self.get_logger().info("Lock publish limit reached (1). Resetting.")
                self.target_label = None
                self.is_locked = False
                self.locked_xyz = None
                self.locked_label = None
                self.xyz_buffer.clear()

            cv2.imshow("Robot-Centric Detection", display_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stop_node()
            return

        # 2) lock이 아니면: YOLO track 수행
        results = self.model.track(
            source=color_image,
            persist=True,
            conf=self.conf_th,
            device=self.device,
            verbose=False,
        )

        if len(results) == 0:
            cv2.imshow("Robot-Centric Detection", display_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stop_node()
            return

        r = results[0]
        display_img = r.plot(conf=False, line_width=2)

        # 타겟 라벨이 없으면 화면만 보여주고 끝
        if self.target_label is None or r.boxes is None:
            cv2.imshow("Robot-Centric Detection", display_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stop_node()
            return

        # 3) 타겟 라벨 중 "confidence 최고" 1개 선택
        best = None  # (conf, u, v, label_name)
        for box in r.boxes:
            cls_idx = int(box.cls[0])
            label_name = str(self.model.names[cls_idx]).strip().lower()
            if label_name != self.target_label:
                continue

            conf = float(box.conf[0]) if box.conf is not None else 0.0
            b = box.xyxy[0].cpu().numpy()
            u = int((b[0] + b[2]) / 2)
            v = int((b[1] + b[3]) / 2)

            if (best is None) or (conf > best[0]):
                best = (conf, u, v, label_name)

        # 타겟이 없으면 버퍼 비우고 화면만
        if best is None:
            self.xyz_buffer.clear()
            cv2.imshow("Robot-Centric Detection", display_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stop_node()
            return

        conf, u, v, label_name = best

        # 4) ROI median depth로 안정화 (핵심)
        depth_val = self.get_roi_median_depth(depth_frame, u, v, w, h)
        if depth_val is None:
            # 유효 depth가 부족하면 버퍼 누적 안 함
            now = time.time()
            if now - self.last_log_time > self.log_interval_sec:
                self.get_logger().warn("[DEPTH] Not enough valid samples in ROI.")
                self.last_log_time = now

            cv2.imshow("Robot-Centric Detection", display_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stop_node()
            return

        # 5) (u,v)+depth -> camera 3D -> base 3D
        p_cam3 = rs.rs2_deproject_pixel_to_point(self.intr, [u, v], depth_val)
        p_cam = np.array([p_cam3[0], p_cam3[1], p_cam3[2], 1.0], dtype=np.float64)
        p_base = (self.T_base_cam @ p_cam)[:3]  # (x,y,z)

        # 6) lock 전에 몇 프레임 누적 + 로그 출력
        self.xyz_buffer.append(p_base)

        # 화면에 현재 후보 좌표 표시
        cv2.circle(display_img, (u, v), 4, (0, 255, 255), -1)
        cv2.putText(
            display_img,
            f"Target: {label_name} conf={conf:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        # 주기적으로 mean/std 로그
        now = time.time()
        if (
            now - self.last_log_time > self.log_interval_sec
            and len(self.xyz_buffer) >= 3
        ):
            arr = np.array(self.xyz_buffer)
            mean_xyz = np.mean(arr, axis=0)
            std_xyz = np.std(arr, axis=0)
            self.get_logger().info(
                f"[XYZ LOG] n={len(self.xyz_buffer)} | "
                f"mean(mm)=({mean_xyz[0]*1000:.1f}, {mean_xyz[1]*1000:.1f}, {mean_xyz[2]*1000:.1f}) | "
                f"std(mm)=({std_xyz[0]*1000:.2f}, {std_xyz[1]*1000:.2f}, {std_xyz[2]*1000:.2f})"
            )
            self.last_log_time = now

        # 7) 버퍼가 충분히 차면 "평균 좌표로 lock"
        if len(self.xyz_buffer) >= self.lock_window_frames:
            arr = np.array(self.xyz_buffer)
            mean_xyz = np.mean(arr, axis=0)
            std_xyz = np.std(arr, axis=0)

            self.locked_xyz = mean_xyz
            self.is_locked = True
            self.locked_label = label_name
            self.lock_publish_count = 0

            self.get_logger().info(
                f"[LOCK] {label_name} locked with mean(mm)=({mean_xyz[0]*1000:.1f}, {mean_xyz[1]*1000:.1f}, {mean_xyz[2]*1000:.1f}) "
                f"std(mm)=({std_xyz[0]*1000:.2f}, {std_xyz[1]*1000:.2f}, {std_xyz[2]*1000:.2f}) "
                f"roi={(2*self.roi_radius+1)}x{(2*self.roi_radius+1)}"
            )

        cv2.imshow("Robot-Centric Detection", display_img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.stop_node()

    # 종료
    def stop_node(self):
        if not self.is_running:
            return

        self.get_logger().info("Shutting down...")
        self.is_running = False
        try:
            self.pipeline.stop()
        except Exception:
            pass

        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)

        rclpy.shutdown()
        try:
            sys.exit(0)
        except SystemExit:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloRobotBaseVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_node()
        except Exception:
            pass
        node.destroy_node()


if __name__ == "__main__":
    main()
