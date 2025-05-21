import cv2
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 8887

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# cap = cv2.VideoCapture("/dev/msmarker")
cap = cv2.VideoCapture(2)


# 카메라 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

if not cap.isOpened():
    print("[ERROR] 카메라 열기 실패")
    exit(1)

print("[SEND] UDP 전송 시작")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # JPEG 압축 품질 낮추기 (품질 50 → 더 낮추면 크기 줄어듦)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
    result, buffer = cv2.imencode(".jpg", frame, encode_param)

    if result:
        try:
            sock.sendto(buffer.tobytes(), (UDP_IP, UDP_PORT))
        except OSError as e:
            print("[UDP ERROR]", e)
            continue

    cv2.imshow("Sending", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
sock.close()
cv2.destroyAllWindows()
