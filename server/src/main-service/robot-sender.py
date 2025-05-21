import socket
import json

# 로봇별 IP와 PORT 등록 (필요시 변경 가능)
ROBOT_CONFIG = {
    "robot1": {"ip": "192.168.0.69", "port": 9999},
    "robot2": {"ip": "192.168.0.68", "port": 9999},
}

def send_packet_tcp(ip, port, packet):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ip, port))
            s.sendall(json.dumps(packet).encode("utf-8"))
            print(f"{packet}")
    except Exception as e:
        print(f"[ERROR] Failed to send to {ip}:{port} → {e}")

def dispatch_task(packet):
    robot_id = packet.get("robot_id")
    config = ROBOT_CONFIG.get(robot_id)

    if config:
        send_packet_tcp(config["ip"], config["port"], packet)
    else:
        print(f"[ERROR] Unknown robot_id: {robot_id}")


