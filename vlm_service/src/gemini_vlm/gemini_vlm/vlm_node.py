"""Vision-Language perception node for towel/water target detection and publishing."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import google.generativeai as genai
import PIL.Image
import json
import os
import logging
import time
from collections import deque

# 로그 정리
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
logging.getLogger('google').setLevel(logging.ERROR)

class GeminiVLMNode(Node):
    """Capture camera frames, infer targets, and publish stable coordinates."""

    def __init__(self):
        """Initialize ROS I/O, Gemini model, camera stream, and tracking state."""
        super().__init__('gemini_vlm_node')
        self.towel_pub = self.create_publisher(PointStamped, 'target_towel_point', 10)
        # Legacy topic is kept for compatibility with older orchestrator versions.
        self.tissue_pub = self.create_publisher(PointStamped, 'target_tissue_point', 10)
        self.water_pub = self.create_publisher(PointStamped, 'target_water_point', 10)
        self.sequence_active_sub = self.create_subscription(
            Bool, '/wipe_sequence_active', self.sequence_active_callback, 10
        )
        # [추가] 칵테일 제조 완료 신호 구독
        self.cocktail_complete_sub = self.create_subscription(
            Bool, '/cocktail_sequence_complete', self.cocktail_complete_callback, 10
        )
        
        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            self.get_logger().error("GOOGLE_API_KEY environment variable is not set!")
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-2.5-flash')

        self.bridge = CvBridge()
        self.latest_image = None
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.roi_x1, self.roi_y1, self.roi_x2, self.roi_y2 = 150, 120, 490, 400
        # Image tone defaults (no extra glare compensation).
        self.brightness_alpha = 0.88
        self.brightness_beta = -12
        self.max_history = 10
        self.towel_required_samples = 10
        self.water_required_samples = 10
        self.towel_history = deque(maxlen=self.max_history)
        self.water_history = deque(maxlen=self.max_history)
        self.last_towel_avg = None
        self.last_water_avg = None
        self.reset_thresh_towel_px = 120.0
        self.reset_thresh_water_px = 80.0
        self.vlm_enabled = False  # 칵테일 로봇 제조 완료 시점까지 대기
        self.sequence_active = False
        self.prev_sequence_active = False
        self.towel_locked = False
        self.water_locked = False
        self.lock_republish_period_sec = 1.0
        self.last_towel_republish_mono = 0.0
        self.last_water_republish_mono = 0.0
        # Water detection stabilization for glare-heavy scenes.
        self.water_hold_sec = 12.0
        self.last_water_valid_xy = None
        self.last_water_valid_mono = 0.0
        self.water_gate_x_min_ratio = 0.20
        self.water_gate_x_max_ratio = 1.00
        self.water_gate_y_min_ratio = 0.05
        self.water_gate_y_max_ratio = 0.98
        self.water_track_mode = "global"
        self.water_local_fail_count = 0
        self.water_local_fail_limit = 6
        self.water_towel_exclusion_px = 90.0
        self.water_refine_window_px = 125
        self.water_refine_max_shift_px = 65.0
        self.water_vlm_vote_count = 3
        self.water_vlm_consensus_px = 70.0
        # Camera auto-exposure settles in the first seconds.
        self.startup_stabilize_sec = 3.0
        self.start_mono = time.monotonic()

        self.get_logger().info('=== Gemini Node: Ready for Towel+Water Detection ===')
        cv2.namedWindow("VLM View")
        cv2.setMouseCallback("VLM View", self.on_mouse)
        self.timer = self.create_timer(0.50, self.timer_callback)

    def image_callback(self, msg):
        """Receive image from topic and update latest_image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def on_mouse(self, event, x, y, flags, param):
        """Debug helper: print clicked pixel coordinates from OpenCV window."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"픽셀 클릭 좌표: ({x}, {y})")

    def cocktail_complete_callback(self, msg):
        """칵테일 시퀀스 완료 신호 수신 시 VLM 활성화."""
        if msg.data and not self.vlm_enabled:
            self.get_logger().info("칵테일 제조 완료: 물/수건 스캔 시작")
            self.vlm_enabled = True

    def sequence_active_callback(self, msg):
        """Lock/unlock coordinate search according to robot sequence state."""
        self.sequence_active = bool(msg.data)
        if self.sequence_active and not self.prev_sequence_active:
            self.get_logger().info("시퀀스 시작 신호 수신: 좌표 전송 잠금 유지")
        if not self.sequence_active and self.prev_sequence_active:
            # Full sequence finished -> unlock for next cycle.
            self.get_logger().info("시퀀스 종료 신호 수신: 다음 탐색 허용")
            # [수정] self.vlm_enabled = False 제거
            self.towel_locked = False
            self.water_locked = False
            self.towel_history.clear()
            self.water_history.clear()
            self.water_track_mode = "global"
            self.water_local_fail_count = 0
        self.prev_sequence_active = self.sequence_active

    def timer_callback(self):
        """Main loop: acquire frame from subscription, run inference/tracking, draw UI, publish points."""
        try:
            if self.latest_image is None:
                self.get_logger().info("Waiting for image from topic...")
                return

            img_np = self.latest_image.copy()
            img_np = cv2.convertScaleAbs(
                img_np, alpha=self.brightness_alpha, beta=self.brightness_beta
            )
            roi_img = img_np[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2].copy()
            
            rgb_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
            pil_img = PIL.Image.fromarray(rgb_img)

            if (time.monotonic() - self.start_mono) < self.startup_stabilize_sec:
                cv2.rectangle(
                    img_np,
                    (self.roi_x1, self.roi_y1),
                    (self.roi_x2, self.roi_y2),
                    (255, 255, 0),
                    1,
                )
                cv2.putText(
                    img_np,
                    "Camera stabilizing...",
                    (self.roi_x1 + 8, self.roi_y1 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.imshow("VLM View", img_np)
                cv2.waitKey(1)
                return
            
            towel_xy, water_xy = None, None
            if not self.sequence_active:
                if self.vlm_enabled:
                    if not self.towel_locked:
                        towel_xy, water_xy = self._infer_targets(pil_img)
                        if towel_xy:
                            self._update_history_and_publish(
                                towel_xy,
                                self.towel_history,
                                "towel",
                                self.towel_pub,
                            )
                    else:
                        water_xy = self._infer_water_with_mode(pil_img, roi_img)
                else:
                    cv2.putText(
                        img_np,
                        "Waiting for cocktail completion...",
                        (self.roi_x1 + 8, self.roi_y1 + 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (200, 200, 200),
                        1,
                        cv2.LINE_AA,
                    )

                water_xy = self._reject_water_near_towel(water_xy, towel_xy)
                water_xy = self._normalize_water_candidate(water_xy)
                if water_xy and not self.water_locked:
                    self._update_history_and_publish(
                        water_xy,
                        self.water_history,
                        "water",
                        self.water_pub,
                    )
                self._republish_locked_targets_if_needed()
            else:
                self.get_logger().info("시퀀스 진행 중: 좌표 탐색 일시정지")

            # UI: ROI와 타겟 표시
            cv2.rectangle(
                img_np,
                (self.roi_x1, self.roi_y1),
                (self.roi_x2, self.roi_y2),
                (255, 255, 0),
                1,
            )
            if towel_xy:
                cv2.circle(img_np, (int(towel_xy[0]), int(towel_xy[1])), 5, (0, 0, 255), -1)
            if water_xy:
                cv2.circle(img_np, (int(water_xy[0]), int(water_xy[1])), 5, (255, 0, 0), -1)
            if len(self.towel_history) >= 2:
                towel_pts = np.array(
                    [[int(x), int(y)] for x, y in self.towel_history], dtype=np.int32
                ).reshape((-1, 1, 2))
                cv2.polylines(img_np, [towel_pts], False, (0, 128, 255), 2)
            if len(self.water_history) >= 2:
                water_pts = np.array(
                    [[int(x), int(y)] for x, y in self.water_history], dtype=np.int32
                ).reshape((-1, 1, 2))
                cv2.polylines(img_np, [water_pts], False, (255, 255, 0), 2)
            if self.last_towel_avg:
                cv2.circle(
                    img_np,
                    (int(self.last_towel_avg[0]), int(self.last_towel_avg[1])),
                    6,
                    (0, 255, 0),
                    -1,
                )
            if self.last_water_avg:
                cv2.circle(
                    img_np,
                    (int(self.last_water_avg[0]), int(self.last_water_avg[1])),
                    6,
                    (0, 255, 255),
                    -1,
                )
            cv2.imshow("VLM View", img_np)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def _infer_targets(self, pil_img):
        """Infer towel and water together using a single Gemini prompt."""
        prompt = (
            "Return JSON only: "
            "{\"towel\": {\"x\": 0-1000, \"y\": 0-1000} or null, "
            "\"water\": {\"x\": 0-1000, \"y\": 0-1000} or null} "
            "for the center of the folded towel and the spilled water. "
            "For water, use the center of the entire connected wet region (not an edge, not a highlight). "
            "If a puddle has an irregular shape, pick the most interior center point of that full puddle."
        )
        response = self.model.generate_content([prompt, pil_img])
        data = self._extract_json_dict(response.text)
        if data is None:
            return None, None
        roi_w, roi_h = self.roi_x2 - self.roi_x1, self.roi_y2 - self.roi_y1

        def _map_xy(obj):
            """Convert ROI-normalized [0..1000] output to full image pixels."""
            if not obj or obj.get('x') is None or obj.get('y') is None:
                return None
            raw_x = (float(obj['x']) / 1000.0) * roi_w + self.roi_x1
            raw_y = (float(obj['y']) / 1000.0) * roi_h + self.roi_y1
            return raw_x, raw_y

        towel_xy = _map_xy(data.get("towel")) or _map_xy(data.get("tissue"))
        water_xy = _map_xy(data.get("water"))
        return towel_xy, water_xy

    def _infer_water_only(self, pil_img):
        """Infer only water center, with multiple prompt fallbacks."""
        towel_note = ""
        if self.last_towel_avg is not None:
            roi_w = float(self.roi_x2 - self.roi_x1)
            roi_h = float(self.roi_y2 - self.roi_y1)
            tx = int(max(0.0, min(1.0, (self.last_towel_avg[0] - self.roi_x1) / roi_w)) * 1000.0)
            ty = int(max(0.0, min(1.0, (self.last_towel_avg[1] - self.roi_y1) / roi_h)) * 1000.0)
            towel_note = f" Folded towel is near ({tx}, {ty}) in ROI scale; do not return towel."
        prompt = (
            "Return JSON only: "
            "{\"water\": {\"x\": 0-1000, \"y\": 0-1000} or null} "
            "for the center of spilled water puddle/wet stain on the table. "
            "Use the center of the full wet spread area, not the rim and not the brightest reflection. "
            "Ignore glossy white reflections, glare, and robot body. "
            "If uncertain or no spilled liquid is visible, return null."
            + towel_note
        )
        water_xy = self._run_water_prompt(pil_img, prompt)
        if water_xy is not None:
            return water_xy

        # Retry on cropped ROI where water most often appears (center-right table area).
        water_xy = self._run_water_prompt_cropped(
            pil_img,
            crop_x_min=0.35,
            crop_x_max=0.98,
            crop_y_min=0.15,
            crop_y_max=0.95,
            prompt=prompt,
        )
        if water_xy is not None:
            return water_xy

        fallback_prompt = (
            "Return JSON only: "
            "{\"water\": {\"x\": 0-1000, \"y\": 0-1000} or null} "
            "Find the most likely wet region on the table surface, even if faint. "
            "Return the interior center of the whole wet region, not boundary pixels. "
            "Do not return bright specular reflections, towel, or robot parts. "
            "If uncertain, return null."
        )
        water_xy = self._run_water_prompt(pil_img, fallback_prompt)
        if water_xy is not None:
            return water_xy
        return self._run_water_prompt_cropped(
            pil_img,
            crop_x_min=0.28,
            crop_x_max=1.00,
            crop_y_min=0.08,
            crop_y_max=0.98,
            prompt=fallback_prompt,
        )

    def _run_water_prompt_local_crop(self, pil_img):
        """Run water prompt in a local crop around last valid water coordinate."""
        if self.last_water_valid_xy is None:
            return None
        roi_w = float(self.roi_x2 - self.roi_x1)
        roi_h = float(self.roi_y2 - self.roi_y1)
        x_norm = (self.last_water_valid_xy[0] - self.roi_x1) / roi_w
        y_norm = (self.last_water_valid_xy[1] - self.roi_y1) / roi_h
        half_w = 0.18
        half_h = 0.18
        return self._run_water_prompt_cropped(
            pil_img,
            crop_x_min=max(0.0, x_norm - half_w),
            crop_x_max=min(1.0, x_norm + half_w),
            crop_y_min=max(0.0, y_norm - half_h),
            crop_y_max=min(1.0, y_norm + half_h),
            prompt=(
                "Return JSON only: "
                "{\"water\": {\"x\": 0-1000, \"y\": 0-1000} or null} "
                "Find the same water puddle in this local crop and return the center of its full spread. "
                "Do not return edge/rim points. Ignore towel texture and bright glare. "
                "If uncertain, return null."
            ),
        )

    def _infer_water_local_hint(self, pil_img):
        """Track water near previous point using local crop + hinted global prompt."""
        if self.last_water_valid_xy is None:
            return None
        local_crop_xy = self._run_water_prompt_local_crop(pil_img)
        if local_crop_xy is not None:
            return local_crop_xy
        roi_w = float(self.roi_x2 - self.roi_x1)
        roi_h = float(self.roi_y2 - self.roi_y1)
        x_norm = (self.last_water_valid_xy[0] - self.roi_x1) / roi_w
        y_norm = (self.last_water_valid_xy[1] - self.roi_y1) / roi_h
        x_hint = int(max(0.0, min(1.0, x_norm)) * 1000.0)
        y_hint = int(max(0.0, min(1.0, y_norm)) * 1000.0)
        prompt = (
            "Return JSON only: "
            "{\"water\": {\"x\": 0-1000, \"y\": 0-1000} or null} "
            f"Track the same spilled water near previous location ({x_hint}, {y_hint}) in ROI scale. "
            "Search nearby area first. Return the interior center of the full puddle region. "
            "Ignore bright glare, reflections, and towel texture. "
            "If uncertain, return null."
        )
        return self._run_water_prompt(pil_img, prompt)

    def _infer_water_with_mode(self, pil_img, roi_img):
        """Use local tracking first, then fallback to global search when needed."""
        if self.water_track_mode == "local":
            water_xy = self._infer_water_local_hint(pil_img)
            if water_xy is not None:
                water_xy = self._refine_water_candidate_near_hint(roi_img, water_xy)
                self.water_local_fail_count = 0
                return water_xy
            self.water_local_fail_count += 1
            if self.water_local_fail_count >= self.water_local_fail_limit:
                self.water_track_mode = "global"
                self.water_local_fail_count = 0
                self.get_logger().info("water tracking: local fail, fallback to global")

        water_xy = self._infer_water_only(pil_img)
        if water_xy is None:
            water_xy = self._infer_water_fallback_cv(roi_img)
        else:
            water_xy = self._refine_water_candidate_near_hint(roi_img, water_xy)

        if water_xy is not None:
            if self.water_track_mode != "local":
                self.get_logger().info("water tracking: global hit, switch to local")
            self.water_track_mode = "local"
            self.water_local_fail_count = 0
        return water_xy

    def _refine_water_candidate_near_hint(self, roi_img_bgr, water_xy):
        """Refine water point to interior of connected wet-region component."""
        if water_xy is None:
            return None
        roi_h, roi_w = roi_img_bgr.shape[:2]
        cx = float(water_xy[0]) - self.roi_x1
        cy = float(water_xy[1]) - self.roi_y1
        w = int(self.water_refine_window_px)
        half = max(20, w // 2)
        x0 = int(max(0, min(roi_w - 1, cx - half)))
        x1 = int(max(1, min(roi_w, cx + half)))
        y0 = int(max(0, min(roi_h - 1, cy - half)))
        y1 = int(max(1, min(roi_h, cy + half)))
        if x1 <= x0 + 2 or y1 <= y0 + 2:
            return water_xy

        patch = roi_img_bgr[y0:y1, x0:x1]
        gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        diff_dark = cv2.subtract(blur, gray)

        # Build a wet-region mask and use its centroid (spread center), not edge points.
        nz = diff_dark[diff_dark > 0]
        base_thr = 5.0 if nz.size == 0 else max(4.0, float(np.percentile(nz, 68)))
        _, mask_dark = cv2.threshold(diff_dark, base_thr, 255, cv2.THRESH_BINARY)

        lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB)
        l_ch = lab[:, :, 0]
        l_blur = cv2.GaussianBlur(l_ch, (7, 7), 0)
        l_diff = cv2.subtract(l_blur, l_ch)
        _, mask_l = cv2.threshold(l_diff, max(3.0, base_thr - 1.0), 255, cv2.THRESH_BINARY)

        mask = cv2.bitwise_or(mask_dark, mask_l)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

        hint_px = int(max(0, min((x1 - x0) - 1, cx - x0)))
        hint_py = int(max(0, min((y1 - y0) - 1, cy - y0)))

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num_labels <= 1:
            return water_xy

        chosen = -1
        chosen_score = -1e9
        for i in range(1, num_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < 20:
                continue
            cx_i, cy_i = float(centroids[i][0]), float(centroids[i][1])
            d = float(np.hypot(cx_i - hint_px, cy_i - hint_py))
            contains_hint = labels[hint_py, hint_px] == i
            score = float(area) - 1.15 * d + (500.0 if contains_hint else 0.0)
            if score > chosen_score:
                chosen_score = score
                chosen = i

        if chosen < 0:
            return water_xy

        # Use the most interior point of the chosen wet region to represent "center".
        comp_mask = (labels == chosen).astype(np.uint8) * 255
        dist_map = cv2.distanceTransform(comp_mask, cv2.DIST_L2, 5)
        _, max_val, _, max_loc = cv2.minMaxLoc(dist_map)
        if max_val > 0.0:
            ref_cx, ref_cy = float(max_loc[0]), float(max_loc[1])
        else:
            ref_cx, ref_cy = float(centroids[chosen][0]), float(centroids[chosen][1])

        best_xy = (ref_cx + x0 + self.roi_x1, ref_cy + y0 + self.roi_y1)
        shift = float(np.hypot(best_xy[0] - water_xy[0], best_xy[1] - water_xy[1]))
        if shift > self.water_refine_max_shift_px:
            return water_xy

        blend = 0.88
        refined = (
            blend * best_xy[0] + (1.0 - blend) * float(water_xy[0]),
            blend * best_xy[1] + (1.0 - blend) * float(water_xy[1]),
        )
        return refined

    def _infer_water_fallback_cv(self, roi_img_bgr):
        """Fallback CV detector when VLM water inference is unreliable."""
        # CV fallback: detect likely wet patch by local darkness/texture change on bright table.
        gray = cv2.cvtColor(roi_img_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        diff_dark = cv2.subtract(blur, gray)
        _, mask_dark = cv2.threshold(diff_dark, 6, 255, cv2.THRESH_BINARY)

        # Secondary cue: subtle edges around wet boundary.
        edges = cv2.Canny(gray, 35, 95)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
        mask = cv2.bitwise_or(mask_dark, edges)

        # Remove tiny noise and connect nearby pixels.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        roi_h, roi_w = gray.shape[:2]
        min_area = max(45, int(0.0008 * roi_w * roi_h))
        max_area = int(0.25 * roi_w * roi_h)
        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if not (min_area <= area <= max_area):
                continue
            m = cv2.moments(c)
            if m["m00"] == 0:
                continue
            cx = float(m["m10"] / m["m00"]) + self.roi_x1
            cy = float(m["m01"] / m["m00"]) + self.roi_y1
            if not self._is_in_water_gate((cx, cy)):
                continue
            candidates.append(c)
        if not candidates:
            return None

        # Prefer candidates near the recent valid water coordinate if available.
        best = None
        best_score = -1e9
        for c in candidates:
            area = cv2.contourArea(c)
            m = cv2.moments(c)
            if m["m00"] == 0:
                continue
            cx = float(m["m10"] / m["m00"]) + self.roi_x1
            cy = float(m["m01"] / m["m00"]) + self.roi_y1
            score = area
            if self.last_water_valid_xy is not None:
                dx = cx - self.last_water_valid_xy[0]
                dy = cy - self.last_water_valid_xy[1]
                score -= 0.6 * np.hypot(dx, dy)
            if score > best_score:
                best_score = score
                best = (cx, cy)
        return best

    def _run_water_prompt_cropped(
        self,
        pil_img,
        crop_x_min,
        crop_x_max,
        crop_y_min,
        crop_y_max,
        prompt,
    ):
        """Run prompt on a normalized crop and map result back to full image."""
        arr = np.array(pil_img)
        h, w = arr.shape[:2]
        x0 = int(max(0.0, min(1.0, crop_x_min)) * w)
        x1 = int(max(0.0, min(1.0, crop_x_max)) * w)
        y0 = int(max(0.0, min(1.0, crop_y_min)) * h)
        y1 = int(max(0.0, min(1.0, crop_y_max)) * h)
        if x1 <= x0 or y1 <= y0:
            return None
        crop = arr[y0:y1, x0:x1]
        if crop.size == 0:
            return None
        crop_img = PIL.Image.fromarray(crop)
        candidates = []
        for _ in range(self.water_vlm_vote_count):
            response = self.model.generate_content([prompt, crop_img])
            data = self._extract_json_dict(response.text)
            if data is None:
                continue
            obj = data.get("water")
            if not obj or obj.get("x") is None or obj.get("y") is None:
                continue
            # Map crop-normalized coordinate back to full ROI pixel.
            cx = (float(obj["x"]) / 1000.0) * (x1 - x0) + x0
            cy = (float(obj["y"]) / 1000.0) * (y1 - y0) + y0
            raw_x = float(cx) + self.roi_x1
            raw_y = float(cy) + self.roi_y1
            candidates.append((raw_x, raw_y))
        return self._consensus_water_xy(candidates)

    def _run_water_prompt(self, pil_img, prompt):
        """Run water prompt on full ROI and aggregate vote candidates."""
        candidates = []
        roi_w, roi_h = self.roi_x2 - self.roi_x1, self.roi_y2 - self.roi_y1
        for _ in range(self.water_vlm_vote_count):
            response = self.model.generate_content([prompt, pil_img])
            data = self._extract_json_dict(response.text)
            if data is None:
                continue
            obj = data.get("water")
            if not obj or obj.get("x") is None or obj.get("y") is None:
                continue
            raw_x = (float(obj["x"]) / 1000.0) * roi_w + self.roi_x1
            raw_y = (float(obj["y"]) / 1000.0) * roi_h + self.roi_y1
            candidates.append((raw_x, raw_y))
        return self._consensus_water_xy(candidates)

    def _consensus_water_xy(self, candidates):
        """Return robust median candidate when model votes agree enough."""
        if not candidates:
            return None
        valid = [xy for xy in candidates if self._is_in_water_gate(xy)]
        if not valid:
            return None
        if len(valid) == 1:
            return valid[0]
        pts = np.array(valid, dtype=np.float32)
        median_xy = np.median(pts, axis=0)
        dists = np.linalg.norm(pts - median_xy, axis=1)
        if float(np.max(dists)) > self.water_vlm_consensus_px:
            self.get_logger().info("water VLM vote disagreed; reject candidate as uncertain")
            return None
        return float(median_xy[0]), float(median_xy[1])

    def _extract_json_dict(self, text):
        """Extract first JSON object block from model response text."""
        if not text:
            return None
        start, end = text.find("{"), text.rfind("}") + 1
        if start == -1 or end <= start:
            return None
        try:
            obj = json.loads(text[start:end])
            return obj if isinstance(obj, dict) else None
        except Exception:
            return None

    def _update_history_and_publish(self, xy, history, label, publisher):
        """Filter outliers, lock stable coordinates, and publish final point."""
        raw_x, raw_y = xy

        if history:
            median_x_tmp = float(np.median([c[0] for c in history]))
            median_y_tmp = float(np.median([c[1] for c in history]))
            reset_thresh = (
                self.reset_thresh_towel_px if label == "towel" else self.reset_thresh_water_px
            )
            dist = np.sqrt((raw_x - median_x_tmp) ** 2 + (raw_y - median_y_tmp) ** 2)
            if dist > reset_thresh:
                if label == "water":
                    self.get_logger().info(
                        f"water outlier rejected (dist={dist:.1f}px > {reset_thresh:.1f}px)"
                    )
                    return
                history.clear()

        history.append((raw_x, raw_y))

        required_samples = (
            self.towel_required_samples if label == "towel" else self.water_required_samples
        )
        if len(history) >= required_samples:
            avg_x = float(np.median([c[0] for c in history]))
            avg_y = float(np.median([c[1] for c in history]))
            if label == "towel":
                self.last_towel_avg = (avg_x, avg_y)
                self.towel_locked = True
                # Publish to both new and legacy topics.
                legacy_msg = PointStamped()
                legacy_msg.header.stamp = self.get_clock().now().to_msg()
                legacy_msg.point.x, legacy_msg.point.y = float(avg_x), float(avg_y)
                self.tissue_pub.publish(legacy_msg)
            else:
                self.last_water_avg = (avg_x, avg_y)
                self.water_locked = True

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x, msg.point.y = float(avg_x), float(avg_y)
            publisher.publish(msg)
            self.get_logger().info(
                f'>> {label} Target Sent: ({int(avg_x)}, {int(avg_y)}) [lock=True]'
            )
            history.clear()
        else:
            self.get_logger().info(f"{label} Sampling... {len(history)}/{required_samples}")

    def _republish_locked_targets_if_needed(self):
        """Re-publish locked targets periodically to avoid missed subscribers."""
        now_mono = time.monotonic()
        # Towel re-send is disabled after first lock to avoid re-search noise.
        # It will be unlocked only when a full robot sequence is completed.

        if self.water_locked and self.last_water_avg is not None:
            if now_mono - self.last_water_republish_mono >= self.lock_republish_period_sec:
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x, msg.point.y = float(self.last_water_avg[0]), float(self.last_water_avg[1])
                self.water_pub.publish(msg)
                self.last_water_republish_mono = now_mono
                self.get_logger().info(
                    f">> water Target ReSent: ({int(self.last_water_avg[0])}, {int(self.last_water_avg[1])})"
                )

    def _reject_water_near_towel(self, water_xy, towel_xy=None):
        """Reject water candidates that are too close to towel coordinate."""
        if water_xy is None:
            return None
        towel_ref = towel_xy if towel_xy is not None else self.last_towel_avg
        if towel_ref is None:
            return water_xy
        dx = float(water_xy[0]) - float(towel_ref[0])
        dy = float(water_xy[1]) - float(towel_ref[1])
        dist = float(np.hypot(dx, dy))
        if dist < self.water_towel_exclusion_px:
            self.get_logger().info(
                f"water candidate rejected near towel (dist={dist:.1f}px < {self.water_towel_exclusion_px:.1f}px)"
            )
            return None
        return water_xy

    def _normalize_water_candidate(self, water_xy):
        """Apply gate/hold logic so water target remains stable across frames."""
        now_mono = time.monotonic()
        if water_xy is not None and self._is_in_water_gate(water_xy):
            self.last_water_valid_xy = (float(water_xy[0]), float(water_xy[1]))
            self.last_water_valid_mono = now_mono
            return water_xy

        if self.last_water_valid_xy is not None:
            if (now_mono - self.last_water_valid_mono) <= self.water_hold_sec:
                self.get_logger().info("water hold: using last valid water coordinate")
                return self.last_water_valid_xy
        return None

    def _is_in_water_gate(self, xy):
        """Check whether a coordinate stays within configured water search region."""
        x, y = float(xy[0]), float(xy[1])
        roi_w = float(self.roi_x2 - self.roi_x1)
        roi_h = float(self.roi_y2 - self.roi_y1)
        x_min = self.roi_x1 + self.water_gate_x_min_ratio * roi_w
        x_max = self.roi_x1 + self.water_gate_x_max_ratio * roi_w
        y_min = self.roi_y1 + self.water_gate_y_min_ratio * roi_h
        y_max = self.roi_y1 + self.water_gate_y_max_ratio * roi_h
        return x_min <= x <= x_max and y_min <= y <= y_max

    def _suppress_specular_glare(self, bgr_img):
        """Reduce bright low-saturation glare pixels in HSV space."""
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        # Typical reflection: low saturation + very high value.
        glare_mask = (s < self.specular_s_max) & (v > self.specular_v_min)
        v = np.where(glare_mask, self.specular_v_target, v).astype(np.uint8)
        hsv_fixed = cv2.merge([h, s, v])
        return cv2.cvtColor(hsv_fixed, cv2.COLOR_HSV2BGR)

    def _draw_ring_marker(self, img, xy, color, text):
        """Draw a ring-style target marker with short label."""
        x, y = int(xy[0]), int(xy[1])
        cv2.circle(img, (x, y), 7, color, 2)
        cv2.circle(img, (x, y), 2, color, -1)
        cv2.line(img, (x - 10, y), (x - 4, y), color, 1)
        cv2.line(img, (x + 4, y), (x + 10, y), color, 1)
        cv2.line(img, (x, y - 10), (x, y - 4), color, 1)
        cv2.line(img, (x, y + 4), (x, y + 10), color, 1)
        cv2.putText(img, text, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    def _draw_fade_trail(self, img, history, color):
        """Draw faded trajectory line from historical points."""
        points = [(int(x), int(y)) for x, y in history]
        for i in range(1, len(points)):
            alpha = i / float(len(points))
            thickness = max(1, int(1 + 2 * alpha))
            seg_color = (
                int(color[0] * alpha),
                int(color[1] * alpha),
                int(color[2] * alpha),
            )
            cv2.line(img, points[i - 1], points[i], seg_color, thickness, cv2.LINE_AA)

    def _draw_direction_arrows(self, img, history, color):
        """Overlay directional arrows along historical movement path."""
        points = [(int(x), int(y)) for x, y in history]
        if len(points) < 3:
            return
        step = max(2, len(points) // 3)
        for i in range(step, len(points), step):
            p1 = points[i - 1]
            p2 = points[i]
            cv2.arrowedLine(img, p1, p2, color, 1, cv2.LINE_AA, tipLength=0.4)

    def _draw_status_hud(self, img):
        """Render compact debug HUD with lock and sequence state."""
        hud_x, hud_y = 10, 10
        hud_w, hud_h = 240, 78
        cv2.rectangle(img, (hud_x, hud_y), (hud_x + hud_w, hud_y + hud_h), (24, 24, 24), -1)
        cv2.rectangle(img, (hud_x, hud_y), (hud_x + hud_w, hud_y + hud_h), (90, 90, 90), 1)
        seq_text = "ACTIVE" if self.sequence_active else "IDLE"
        cv2.putText(
            img,
            f"SEQ: {seq_text}",
            (hud_x + 10, hud_y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (180, 255, 180) if self.sequence_active else (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            f"Towel lock: {self.towel_locked} ({len(self.towel_history)}/{self.max_history})",
            (hud_x + 10, hud_y + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            f"Water lock: {self.water_locked} ({len(self.water_history)}/{self.max_history})",
            (hud_x + 10, hud_y + 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 220, 0),
            1,
            cv2.LINE_AA,
        )

def main(args=None):
    """Entry point for Gemini VLM ROS2 node."""
    rclpy.init(args=args)
    node = GeminiVLMNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()
