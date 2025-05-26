
import socket
import struct
import pickle

SERVER_IP = "192.168.0.69"
POSE_PORT = 10002

def dummy_callback(x, y):
    print(f"[Callback] x = {x:.2f}, y = {y:.2f}")

def receive_pose(pose_callback):
    print("[Client] Trying to connect...")
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((SERVER_IP, POSE_PORT))
        print("[Client] Connected to server")
    except Exception as e:
        print(f"[Client] Connection failed: {e}")
        return

    data = b""
    payload_size = struct.calcsize("Q")
    print(f"[Client] Payload size: {payload_size}")

    while True:
        try:
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    print("[Client] Disconnected by server")
                    return
                data += packet

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                packet = client_socket.recv(4096)
                if not packet:
                    print("[Client] Disconnected during data reception")
                    return
                data += packet

            pose_data = data[:msg_size]
            data = data[msg_size:]

            pose = pickle.loads(pose_data)

            if pose:
                x = pose.get('position_x', 0.0)
                y = pose.get('position_y', 0.0)
                print(f"[Client] Pose received: x={x:.2f}, y={y:.2f}")
                pose_callback(x, y)
            else:
                print("[Client] Empty pose data")

        except Exception as e:
            print(f"[Client] Error: {e}")
            break

if __name__ == "__main__":
    receive_pose(dummy_callback)
