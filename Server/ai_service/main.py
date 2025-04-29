import socket
import threading
import cv2
import numpy as np
import json
import time
import easyocr
from cv2 import aruco

# 설정
UDP_IP = "0.0.0.0"        # 본인 IP (수신 대기)
UDP_PORT = 5000           # AI Camera가 보내는 포트
TCP_SERVER_IP = "192.168.0.100"   # Main Service IP
TCP_SERVER_PORT = 6000

# UDP 소켓 생성
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind((UDP_IP, UDP_PORT))

# TCP 소켓 생성
tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))


def receive_udp_frames():
    print(f"UDP 수신 시작: {UDP_IP}:{UDP_PORT}")
    while True:
        try:
            data, addr = udp_sock.recvfrom(65536)
            npdata = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)

            if frame is not None:
                process_frame(frame)
        except Exception as e:
            print(f"[UDP Error] {e}")


def process_frame(frame):
    results = []

    # 1. 번호판 인식 (여기서는 가짜 결과, 나중에 EasyOCR 붙이자)
    # 예시: 번호판이 "1234"라고 가정
    fake_license_plate = "1234"
    results.append({
        "type": "license_plate",
        "data": fake_license_plate
    })

    # 2. ArUco 마커 인식
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for marker_id, corner in zip(ids.flatten(), corners):
                results.append({
                    "type": "aruco_marker",
                    "data": str(marker_id)
                })

    except Exception as e:
        print(f"[ArUco Error] {e}")

    # 결과를 TCP로 전송
    for result in results:
        send_result_to_main(result)


def send_result_to_main(result_dict):
    try:
        message = json.dumps(result_dict).encode()
        tcp_sock.sendall(message + b'\n')  # \n 기준으로 메세지 구분
        print(f"[TCP] Sent: {result_dict}")
    except Exception as e:
        print(f"[TCP Send Error] {e}")


if __name__ == "__main__":
    udp_thread = threading.Thread(target=receive_udp_frames, daemon=True)
    udp_thread.start()

    print("AI Service 실행 중...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("종료 중...")
        udp_sock.close()
        tcp_sock.close()
