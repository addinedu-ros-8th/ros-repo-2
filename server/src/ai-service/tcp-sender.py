import socket
import json

MAIN_SERVER_IP = "192.168.4.2"
MAIN_SERVER_PORT = 9999

def send_result(packet):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((MAIN_SERVER_IP, MAIN_SERVER_PORT))

        message = json.dumps(packet) + "\n"  # JSON 직렬화 + 종료 구분자
        sock.sendall(message.encode("utf-8"))

        sock.close()
    except Exception as e:
        print("[TCP ERROR]", e)
