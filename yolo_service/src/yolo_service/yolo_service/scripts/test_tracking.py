import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

# 1. 모델 및 리얼센스 기본 설정 (Color 스트림만 사용)
model = YOLO("best.pt")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

try:
    print("--- [Only STEP 2] 객체 추적(ID) 및 중심점(u, v) 검증 테스트 시작 ---")
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # 이미지 변환
        img = np.asanyarray(color_frame.get_data())
        display_img = img.copy() 
        h, w = display_img.shape[:2]

        # 2. YOLOv8 추적 (persist=True를 통해 프레임 간 ID를 유지)
        # tracker="botsort.yaml" 또는 "bytetrack.yaml" 설정 가능 (기본 botsort)
        results = model.track(source=img, persist=True, conf=0.5, device=0, verbose=False)
        
        # 탐지 결과 및 ID 존재 여부 확인
        if len(results) > 0 and results[0].boxes.id is not None:
            r = results[0]
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < 0.8: # 신뢰도 0.8 미만 제외
                    continue

                # 고유 ID 및 Bounding Box 추출
                obj_id = int(box.id[0])
                b = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, b)

                # [중심점 추출 로직]
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # --- 시각화 ---
                # 1. 바운딩 박스 (파란색)
                cv2.rectangle(display_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
                # 2. 중심점 (빨간색 점)
                cv2.circle(display_img, (u, v), 5, (0, 0, 255), -1)
                
                # 3. 상단 라벨 (ID와 신뢰도)
                label = f"ID: {obj_id} | {conf:.2f}"
                cv2.putText(display_img, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 4. 하단 좌표 (픽셀 좌표 u, v)
                coord_text = f"Center: ({u}, {v})"
                cv2.putText(display_img, coord_text, (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # 로그 출력
                print(f"Tracking ID: {obj_id} | Center: u={u}, v={v}")

        # 최종 화면 출력
        cv2.imshow("Step 2: Tracking & Center Test", display_img)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()