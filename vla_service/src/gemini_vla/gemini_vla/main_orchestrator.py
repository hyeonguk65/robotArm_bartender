"""Robot-side orchestrator: pick towel, wipe water, then place towel back."""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile
import numpy as np
import cv2

from .hardware_interface import RobotHardwareInterface


class MainOrchestrator(Node):
    """Coordinate perception targets with deterministic robot motion sequence."""

    def __init__(self):
        """Create subscriptions, publishers, calibration, and startup states."""
        super().__init__("robot_main_controller", namespace="dsr01")
        self.cb_group = ReentrantCallbackGroup()
        self.hw = RobotHardwareInterface(self, self.cb_group)

        self.accept_after_time_towel = 0.0
        self.accept_after_time_water = 0.0
        qos = QoSProfile(depth=1)
        # Legacy topic name is kept for compatibility with older VLA node versions.
        self.towel_subscription_legacy = self.create_subscription(
            PointStamped,
            "/target_tissue_point",
            self.towel_callback,
            qos,
            callback_group=self.cb_group,
        )
        self.towel_subscription = self.create_subscription(
            PointStamped,
            "/target_towel_point",
            self.towel_callback,
            qos,
            callback_group=self.cb_group,
        )
        self.towel_subscription_rel = self.create_subscription(
            PointStamped,
            "target_towel_point",
            self.towel_callback,
            qos,
            callback_group=self.cb_group,
        )
        self.water_subscription = self.create_subscription(
            PointStamped,
            "/target_water_point",
            self.water_callback,
            qos,
            callback_group=self.cb_group,
        )
        self.water_subscription_rel = self.create_subscription(
            PointStamped,
            "target_water_point",
            self.water_callback,
            qos,
            callback_group=self.cb_group,
        )
        self.sequence_state_pub = self.create_publisher(Bool, "/wipe_sequence_active", 10)
        # [추가] 칵테일 제조 완료 신호 구독
        self.cocktail_complete_sub = self.create_subscription(
            Bool, '/cocktail_sequence_complete', self.cocktail_complete_callback, 10
        )

        self.is_working = False
        self.move_delay_sec = 1.0
        self.grip_delay_sec = 1.5
        self.latest_towel_pixel = None
        self.latest_water_pixel = None
        self.init_gripper_open_done = False

        self.wipe_line_stroke_mm = 50.0
        self.wipe_circle_radius_mm = 40.0
        self.wipe_repeats = 2
        self.down_retry_count = 2
        self.down_retry_pause_sec = 0.35

        # Pixel -> robot (base XY) homography from 4-point calibration
        pixel_points = np.array(
            [
                [233.0, 181.0],  # left-bottom
                [212.0, 387.0],  # left-top
                [435.0, 181.0],  # right-bottom
                [436.0, 383.0],  # right-top
            ],
            dtype=np.float32,
        )
        robot_points = np.array(
            [
                [110.15, -232.24],  # left-bottom
                [669.86, -171.42],  # left-top
                [112.58, 307.34],  # right-bottom
                [629.81, 214.01],  # right-top
            ],
            dtype=np.float32,
        )
        self.vla_enabled = False  # [추가] 초기 상태는 비활성
        self.homography = cv2.getPerspectiveTransform(pixel_points, robot_points)
        # [삭제] __init__ 단계에서 제어권을 가져오는 것을 cocktail_complete_callback으로 이동
        # self.hw.acquire_control()
        self._publish_sequence_active(False)
        # Do not call blocking service waits in __init__ before executor starts.
        self.init_gripper_timer = None # [수정] 타이머 나중에 생성
        self.get_logger().info("robot_main_controller started - Waiting for cocktail completion...")

    def cocktail_complete_callback(self, msg):
        """Enable VLA control and acquire hardware when cocktail sequence is finished."""
        if msg.data and not self.vla_enabled:
            self.get_logger().info("칵테일 완료 수신: VLA 로봇 제어권 확보 및 초기화 시작")
            self.vla_enabled = True
            # 이제서야 제어권을 가져오고 그리퍼를 초기화합니다.
            self.hw.acquire_control()
            if self.init_gripper_timer is None:
                self.init_gripper_timer = self.create_timer(1.0, self._init_gripper_open_once)

    def _init_gripper_open_once(self):
        """Open gripper once at startup after executor/service becomes available."""
        if self.init_gripper_open_done or self.is_working or not self.vla_enabled:
            return
        if self.hw.set_gripper(False):
            self.init_gripper_open_done = True
            self.get_logger().info("초기 그리퍼 열기 완료")
            # 한번 열었으면 타이머 중지
            if self.init_gripper_timer:
                self.init_gripper_timer.cancel()

    def towel_callback(self, msg):
        """Accept first towel point for current cycle and trigger sequence check."""
        # Do not re-search while a sequence is running.
        if self.is_working:
            return
        # Lock towel coordinate after first valid estimate, and unlock only
        # after the full sequence (pick -> wipe -> place back) is completed.
        if self.latest_towel_pixel is not None:
            return
        now_mono = time.monotonic()
        if now_mono <= self.accept_after_time_towel:
            return
        self.latest_towel_pixel = (float(msg.point.x), float(msg.point.y))
        self.accept_after_time_towel = now_mono + 0.3
        self.get_logger().info(
            f"<< towel Target Received: ({int(msg.point.x)}, {int(msg.point.y)})"
        )
        self._try_start_sequence()

    def water_callback(self, msg):
        """Update latest water point and trigger sequence check."""
        now_mono = time.monotonic()
        if now_mono <= self.accept_after_time_water:
            return
        if self.is_working:
            return
        self.latest_water_pixel = (float(msg.point.x), float(msg.point.y))
        self.get_logger().info(
            f"<< water Target Received: ({int(msg.point.x)}, {int(msg.point.y)})"
        )
        self._try_start_sequence()

    def _try_start_sequence(self):
        """Start full robot sequence only when both towel/water targets exist."""
        if self.is_working:
            return
        if self.latest_towel_pixel is None:
            return
        if self.latest_water_pixel is None:
            self.get_logger().info("수건 좌표 수신됨, 물 좌표 대기 중")
            return

        self.is_working = True
        self._publish_sequence_active(True)
        try:
            tx, ty = self._pixel_to_robot(*self.latest_towel_pixel)
            wx, wy = self._pixel_to_robot(*self.latest_water_pixel)
            self.get_logger().info("시퀀스 시작: 수건 집기 -> 닦기 -> 원위치 복귀")
            if not self._pick_towel(tx, ty):
                return
            if not self._wipe_water(wx, wy):
                return
            if not self._place_towel_back(tx, ty):
                return
            self.get_logger().info("시퀀스 완료: 좌표 재탐색 모드로 전환")
            self.latest_towel_pixel = None
            self.latest_water_pixel = None
            # self.vla_enabled = False # [수정] 무한 반복 탐색을 위해 주석 처리
        finally:
            self.accept_after_time_water = time.monotonic() + 0.5
            self.is_working = False
            self._publish_sequence_active(False)

    def _pick_towel(self, tx, ty):
        """Move above towel, descend, close gripper, and lift."""
        if not self.hw.move_line([350.0, 0.0, 350.0, 0.0, 180.0, 0.0]):  # Home
            self.get_logger().error("Home 이동 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        if not self.hw.set_gripper(False):  # Open before approach
            self.get_logger().error("그리퍼 열기 실패, 시퀀스 중단")
            return False
        time.sleep(self.grip_delay_sec)
        if not self.hw.move_line([tx, ty, 250.0, 0.0, 180.0, 0.0]):  # Target Top
            self.get_logger().error("타겟 상부 이동 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        if not self._move_down_with_retry(
            [tx, ty, 140.0, 0.0, 180.0, 0.0], vel=30.0, context="수건 집기 하강"
        ):  # Down
            self.get_logger().error("하강 이동 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        if not self.hw.set_gripper(True):  # Close after down
            self.get_logger().error("그리퍼 닫기 실패, 시퀀스 중단")
            return False
        time.sleep(self.grip_delay_sec)
        if not self.hw.move_line([tx, ty, 350.0, 0.0, 180.0, 0.0]):  # Up
            self.get_logger().error("상승 이동 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        return True

    def _wipe_water(self, wx, wy):
        """Execute line + spiral wipe patterns around the detected water point."""
        if not self.hw.move_line([wx, wy, 250.0, 0.0, 180.0, 0.0]):  # Water Top
            self.get_logger().error("물 상부 이동 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        if not self._move_down_with_retry(
            [wx, wy, 140.0, 0.0, 180.0, 0.0], vel=25.0, context="물 위치 하강"
        ):  # Down
            self.get_logger().error("물 위치 하강 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)

        z_wipe = 140.0

        # 1) 가로/세로 왕복 닦기: 실패 시 스트로크를 줄여 재시도
        line_patterns = [
            self.wipe_line_stroke_mm / 2.0,
            self.wipe_line_stroke_mm * 0.35,
            self.wipe_line_stroke_mm * 0.2,
        ]
        line_wipe_ok = False
        for half_stroke in line_patterns:
            self.get_logger().info(f"직선 닦기 시도 (반폭: {half_stroke:.1f}mm)")
            local_ok = True
            for _ in range(self.wipe_repeats):
                # Y축 왕복
                if not self.hw.move_line(
                    [wx, wy - half_stroke, z_wipe, 0.0, 180.0, 0.0], vel=40.0, radius=15.0, post_delay_sec=0.0
                ):
                    local_ok = False
                    break
                if not self.hw.move_line(
                    [wx, wy + half_stroke, z_wipe, 0.0, 180.0, 0.0], vel=40.0, radius=15.0, post_delay_sec=0.0
                ):
                    local_ok = False
                    break
                # X축 왕복
                if not self.hw.move_line(
                    [wx - half_stroke, wy, z_wipe, 0.0, 180.0, 0.0], vel=40.0, radius=15.0, post_delay_sec=0.0
                ):
                    local_ok = False
                    break
                if not self.hw.move_line(
                    [wx + half_stroke, wy, z_wipe, 0.0, 180.0, 0.0], vel=40.0, radius=15.0, post_delay_sec=0.0
                ):
                    local_ok = False
                    break
            if local_ok:
                line_wipe_ok = True
                break
            self.get_logger().warn("직선 닦기 실패, 더 작은 스트로크로 재시도")

        if not line_wipe_ok:
            self.get_logger().error("직선 닦기 최종 실패")
            return False

        # 2) 스파이럴 아웃 닦기: 중심에서 시작해 회전하며 둘레까지 확장
        circle_radii = [
            self.wipe_circle_radius_mm,
            self.wipe_circle_radius_mm * 0.7,
            self.wipe_circle_radius_mm * 0.4,
        ]
        circle_wipe_ok = False
        for radius in circle_radii:
            self.get_logger().info(f"원형 닦기 시도 (반지름: {radius:.1f}mm)")
            local_ok = True

            if local_ok:
                for _ in range(self.wipe_repeats):
                    spiral_angles = np.linspace(0.0, 4.0 * np.pi, num=64, endpoint=True)
                    for ang in spiral_angles:
                        ratio = ang / (4.0 * np.pi)
                        rr = radius * ratio
                        cx = wx + rr * np.cos(ang)
                        cy = wy + rr * np.sin(ang)
                        if not self.hw.move_line(
                            [cx, cy, z_wipe, 0.0, 180.0, 0.0], vel=40.0, radius=12.0, post_delay_sec=0.0
                        ):
                            local_ok = False
                            break
                    if not local_ok:
                        break
                    # Reset to center for the next outward spiral repeat.
                    if not self.hw.move_line([wx, wy, z_wipe, 0.0, 180.0, 0.0], vel=30.0):
                        local_ok = False
                        break

            if local_ok:
                # Keep end pose at center for the following phase.
                if not self.hw.move_line([wx, wy, z_wipe, 0.0, 180.0, 0.0], vel=30.0):
                    local_ok = False

            if local_ok:
                circle_wipe_ok = True
                break
            self.get_logger().warn("원형 닦기 실패, 더 작은 반지름으로 재시도")

        if not circle_wipe_ok:
            self.get_logger().error("원형 닦기 최종 실패")
            return False

        if not self.hw.move_line([wx, wy, 250.0, 0.0, 180.0, 0.0]):  # Up
            self.get_logger().error("상승 이동 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        return True

    def _place_towel_back(self, tx, ty):
        """Return towel to original position and go back to home pose."""
        if not self.hw.move_line([tx, ty, 250.0, 0.0, 180.0, 0.0]):  # Towel Top
            self.get_logger().error("수건 원위치 상부 이동 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        if not self._move_down_with_retry(
            [tx, ty, 140.0, 0.0, 180.0, 0.0], vel=25.0, context="수건 원위치 하강"
        ):  # Down
            self.get_logger().error("수건 원위치 하강 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        if not self.hw.set_gripper(False):  # Release towel
            self.get_logger().error("그리퍼 열기 실패, 시퀀스 중단")
            return False
        time.sleep(self.grip_delay_sec)
        if not self.hw.move_line([tx, ty, 350.0, 0.0, 180.0, 0.0]):  # Up
            self.get_logger().error("수건 릴리즈 후 상승 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        if not self.hw.move_line([350.0, 0.0, 350.0, 0.0, 180.0, 0.0]):  # Home
            self.get_logger().error("최종 홈 복귀 실패, 시퀀스 중단")
            return False
        time.sleep(self.move_delay_sec)
        return True

    def _pixel_to_robot(self, px, py):
        """Map camera pixel coordinates to robot base XY using homography."""
        point = np.array([[px, py, 1.0]], dtype=np.float32).T
        mapped = self.homography @ point
        if mapped[2, 0] == 0:
            return 0.0, 0.0
        x = mapped[0, 0] / mapped[2, 0]
        y = mapped[1, 0] / mapped[2, 0]
        return float(x), float(y)

    def _move_down_with_retry(self, pos, vel, context):
        """Retry downward approach moves to tolerate temporary planning failures."""
        for attempt in range(self.down_retry_count + 1):
            if self.hw.move_line(pos, vel=vel):
                return True
            if attempt < self.down_retry_count:
                self.get_logger().warn(
                    f"{context} 실패, 재시도 {attempt + 1}/{self.down_retry_count}"
                )
                time.sleep(self.down_retry_pause_sec)
        return False

    def _publish_sequence_active(self, is_active):
        """Broadcast whether robot is currently executing wipe sequence."""
        msg = Bool()
        msg.data = bool(is_active)
        self.sequence_state_pub.publish(msg)


def main(args=None):
    """Entry point for multi-threaded ROS2 orchestrator node."""
    rclpy.init(args=args)
    node = MainOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
