import socket
import json
from handler import handle_event
from assign_command import monitor_robot_status_and_dispatch
import threading



HOST = '0.0.0.0'
PORT = 9999
BUFFER_SIZE = 4096

def start_tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)

    print(f"[TCP SERVER] Listening on {HOST}:{PORT}")

    while True:
        conn, addr = server.accept()
        print(f"[TCP] Connection from {addr}")

        try:
            data = conn.recv(BUFFER_SIZE).decode("utf-8").strip()
            if not data:
                continue

            try:
                packet = json.loads(data)
            except json.JSONDecodeError:
                print("[ERROR] Invalid JSON received")
                continue

            print(f"[TCP] Received packet: {packet}")
            handle_event(packet)

        except Exception as e:
            print(f"[ERROR] {e}")
        finally:
            conn.close()

if __name__ == "__main__":
    # ✅ 로봇 상태 감시 및 명령 전송 루프 실행 (백그라운드 스레드)
    t = threading.Thread(target=monitor_robot_status_and_dispatch, daemon=True)
    t.start()

    # ✅ TCP 수신 서버 시작
    start_tcp_server()
