import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

model = YOLO("best.pt")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

align = rs.align(rs.stream.color)
pipeline.start(config)

try:
    print("--- 실시간 추론 및 안정성 테스트 시작 ---")
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())

        results = model.predict(source=img, conf=0.5, device=0, verbose=False)

        # 보통 results는 1개이므로 첫 결과만 사용
        r = results[0]
        annotated_img = r.plot(conf=False, line_width=2)

        h, w = annotated_img.shape[:2]

        for box in r.boxes:
            conf = float(box.conf[0])
            if conf < 0.8:
                continue

            b = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = map(int, b)

            # 기본 라벨은 박스 위쪽에 있으니, 숫자는 아래쪽에 배치
            ty = y2 + 18
            if ty > h - 5:     # 화면 밖이면
                ty = y2 - 8    # 박스 안쪽으로 올림
            if ty < 15:
                ty = y1 + 20   # 혹시 위로도 너무 붙으면 보정

            cv2.putText(
                annotated_img,
                f"{conf:.2f}",
                (x1, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow("Inference Test", annotated_img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
