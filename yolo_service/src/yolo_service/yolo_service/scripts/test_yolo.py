from ultralytics import YOLO
import torch

# 1. GPU 체크
print(f"GPU 사용 가능 여부: {torch.cuda.is_available()}")

# 2. 모델 로드 (가장 가벼운 버전인 nano 모델)
model = YOLO('yolov8n.pt')

# 3. 테스트 이미지 추론 (버스 사진)
results = model('https://ultralytics.com/images/bus.jpg')

# 4. 결과 출력
for result in results:
    print(result.boxes) # 찾은 물체들의 좌표와 정보를 출력