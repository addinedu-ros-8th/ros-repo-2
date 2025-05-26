import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import struct
import pickle
import threading

POSE_PORT = 10002
SERVER_IP = "0.0.0.0"

class PoseClientNode(Node):
    def __init__(self):
        super().__init__('pose_client_node')
        self.get_logger().info("Pose stream node started")

        self.current_pose = None
        self.lock = threading.Lock()

        self.create_subscription(PoseStamped, "/tracked_pose", self.pose_callback, 10)

        self.server_thread = threading.Thread(target=self.start_pose_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def pose_callback(self, msg: PoseStamped):
        with self.lock:
            self.current_pose = {
                'position_x': msg.pose.position.x,
                'position_y': msg.pose.position.y,
            }

    def start_pose_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(("0.0.0.0", POSE_PORT))
        server_socket.listen(1)
        self.get_logger().info(f"Listening for pose clients on {SERVER_IP}:{POSE_PORT}")

        while rclpy.ok():
            conn, addr = server_socket.accept()
            self.get_logger().info(f"Pose client connected from {addr}")
            self.handle_pose_client(conn)

    def handle_pose_client(self, conn):
        try:
            while rclpy.ok():
                with self.lock:
                    pose = self.current_pose

                if pose is not None:
                    data = pickle.dumps(pose)
                    conn.sendall(struct.pack("Q", len(data)) + data)

                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # 10Hz
        except Exception as e:
            self.get_logger().error(f"Pose stream error: {e}")
        finally:
            conn.close()
            self.get_logger().info("Pose client disconnected")

def main(args=None):
    rclpy.init(args=args)
    node = PoseClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
