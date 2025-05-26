"""

학습 방법

yolo detect train model=yolov8n.pt data=datasets/coco-person-yolo/dataset.yaml epochs=50 imgsz=640 batch=16

"""


from ultralytics import YOLO
import cv2

# 사전학습된 nano 모델 로드 (필요 시 yolov8s.pt, yolov8m.pt 등으로 교체)
model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(0)  # 또는 GStreamer/UDP 스트림
while True:
    ret, frame = cap.read()
    if not ret: break

    # person 클래스만 필터링 (COCO person 클래스 = 0)
    results = model(frame, classes=[0])

    # 박스 그리기
    for r in results:
        for box, conf in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy()):
            if conf < 0.3: continue
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"{conf:.2f}", (x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    cv2.imshow("Human Detection", frame)
    if cv2.waitKey(1) & 0xFF == 27: break

cap.release()
cv2.destroyAllWindows()
