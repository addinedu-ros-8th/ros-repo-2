import cv2
import numpy as np
import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def get_roi(frame):
    h, w, _ = frame.shape
    x1, y1 = w // 2 - 100, h // 2 - 100
    x2, y2 = w // 2 + 100, h // 2 + 100
    return x1, y1, x2, y2

while True:
    data, addr = sock.recvfrom(65536)
    img_np = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)  # BGR 디코딩
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # BGR → RGB

    x1, y1, x2, y2 = get_roi(frame_rgb)
    roi = frame_rgb[y1:y2, x1:x2]

    # 빨강, 파랑 판별 (RGB 기준)
    red_mask = (roi[:, :, 0] > 150) & (roi[:, :, 1] < 100) & (roi[:, :, 2] < 100)
    blue_mask = (roi[:, :, 0] < 100) & (roi[:, :, 1] < 100) & (roi[:, :, 2] > 150)

    red_ratio = np.sum(red_mask) / red_mask.size
    blue_ratio = np.sum(blue_mask) / blue_mask.size

    box_color = (0, 255, 0)  # 초록 박스
    if red_ratio > 0.15:
        box_color = (0, 0, 255)  # 빨강 박스 (BGR)
    elif blue_ratio > 0.15:
        box_color = (255, 0, 0)  # 파랑 박스 (BGR)

    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
    cv2.putText(frame, f"Red: {red_ratio:.2f} Blue: {blue_ratio:.2f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("UDP RGB Color Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
