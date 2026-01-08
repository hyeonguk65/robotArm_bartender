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
pipeline.start(config)

try:
    print("--- [STEP 2] 중심점 추출 및 좌표 검증 시작 ---")
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        if not color_frame:
            continue

        # 이미지 변환 및 복사본 생성 (시각화용)
        img = np.asanyarray(color_frame.get_data())
        display_img = img.copy() 

        # 2. YOLOv8 추론 (Confidence 0.5 이상 1차 필터링)
        results = model.predict(source=img, conf=0.5, device=0, verbose=False)
        r = results[0]

        # 3. 객체별 루프 (다중 객체 처리)
        for box in r.boxes:
            conf = float(box.conf[0])
            
            # ① Confidence 0.8 기준 신뢰 객체 필터링
            if conf < 0.8:
                continue

            # Bounding Box 좌표 추출
            b = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = map(int, b)

            # ② Bounding Box 중심점(u, v) 계산
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # ③ 중심 좌표 콘솔 출력
            print(f"[CENTER] u={u}, v={v}, conf={conf:.2f}")

            # ④ 시각화 (Overlay)
            # 파란색 박스 (BGR: 255, 0, 0)
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # 중심점 표시 (빨간색 점, BGR: 0, 0, 255)
            cv2.circle(display_img, (u, v), 5, (0, 0, 255), -1)
            
            # 좌표 텍스트 표시 (흰색, BGR: 255, 255, 255)
            cv2.putText(display_img, f"({u}, {v})", (u + 10, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Confidence 표시
            cv2.putText(display_img, f"Conf: {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 5. 최종 화면 출력
        cv2.imshow("Ice Cube Center Detection", display_img)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()