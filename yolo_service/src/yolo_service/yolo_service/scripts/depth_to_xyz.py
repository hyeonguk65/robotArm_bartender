import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

# 1. 모델 및 카메라 설정
model = YOLO("best.pt")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

align = rs.align(rs.stream.color)
profile = pipeline.start(config)

# 카메라의 내부 파라미터(내장된 렌즈 정보) 가져오기
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

try:
    print("--- [STEP 3] 3D 좌표(X, Y, Z) 변환 및 검증 시작 ---")
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        display_img = img.copy() 

        # YOLO 추론
        results = model.predict(source=img, conf=0.8, device=0, verbose=False)
        r = results[0]

        for box in r.boxes:
            # 중심점 계산
            b = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, b)
            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)

            # 🟢 핵심: 중심점의 Depth(거리) 값 가져오기 (단위: m)
            depth_val = depth_frame.get_distance(u, v)

            if depth_val > 0:
                # 🔵 핵심: 2D 픽셀(u,v) + 거리(depth) -> 3D 공간 좌표(x,y,z) 변환
                # 결과값은 카메라 렌즈 중심으로부터의 거리 (단위: m)
                point = rs.rs2_deproject_pixel_to_point(intr, [u, v], depth_val)
                X, Y, Z = point[0], point[1], point[2]

                # 콘솔 출력 (로봇 제어에 쓰일 실제 데이터)
                print(f"[3D POS] X:{X:.3f}, Y:{Y:.3f}, Z:{Z:.3f} (meters)")

                # 시각화
                cv2.circle(display_img, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(display_img, f"XYZ: {X:.2f}, {Y:.2f}, {Z:.2f}", (u + 10, v + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

        cv2.imshow("RealSense 3D Detection", display_img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()