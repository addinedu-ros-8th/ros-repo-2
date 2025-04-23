import cv2
import numpy as np
import socket
import time

# UDP 송신 세팅 (핑키 IP)
TURN_UDP_IP = "192.168.4.1"  # 예: "192.168.0.49"
TURN_UDP_PORT = 5005
turn_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# UDP 스트림 수신 세팅 (노트북 IP)
STREAM_UDP_IP = "192.168.0.59"
STREAM_UDP_PORT = 5005  # 핑키의 카메라 송출 포트

stream_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
stream_sock.bind((STREAM_UDP_IP, STREAM_UDP_PORT))

def get_roi(frame):
    h, w, _ = frame.shape
    x1, y1 = w // 2 - 100, h // 2 - 100
    x2, y2 = w // 2 + 100, h // 2 + 100
    return x1, y1, x2, y2

last_sent_time = 0

while True:
    data, addr = stream_sock.recvfrom(65536)
    img_np = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    x1, y1, x2, y2 = get_roi(frame_rgb)
    roi = frame_rgb[y1:y2, x1:x2]

    red_mask = (roi[:, :, 0] > 150) & (roi[:, :, 1] < 100) & (roi[:, :, 2] < 100)
    blue_mask = (roi[:, :, 0] < 100) & (roi[:, :, 1] < 100) & (roi[:, :, 2] > 150)

    red_ratio = np.sum(red_mask) / red_mask.size
    blue_ratio = np.sum(blue_mask) / blue_mask.size

    box_color = (0, 255, 0)
    current_time = time.time()

    if red_ratio > 0.15 and current_time - last_sent_time > 3:
        box_color = (0, 0, 255)
        turn_sock.sendto(b"TURN", (TURN_UDP_IP, TURN_UDP_PORT))
        print("🔴 TURN command sent!")
        last_sent_time = current_time

    elif blue_ratio > 0.15 and current_time - last_sent_time > 3:
        box_color = (255, 0, 0)
        turn_sock.sendto(b"TURN", (TURN_UDP_IP, TURN_UDP_PORT))
        print("🔵 TURN command sent!")
        last_sent_time = current_time

    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
    cv2.putText(frame, f"Red: {red_ratio:.2f} Blue: {blue_ratio:.2f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("Color Detection with TURN Command", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
