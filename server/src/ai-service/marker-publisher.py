import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'marker_pose', 10)

        # UDP 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 8887))

        # 카메라 보정값
        data = np.load("camera_calibration_data.npz")
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        self.marker_length = 0.08
        self.marker_3d = np.array([
            [-0.04, 0.04, 0], [0.04, 0.04, 0],
            [0.04, -0.04, 0], [-0.04, -0.04, 0]
        ], dtype=np.float32)

        self.timer = self.create_timer(0.1, self.detect_marker)

    def detect_marker(self):
        try:
            data, _ = self.sock.recvfrom(65536)
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None: return

            corners, ids, _ = self.detector.detectMarkers(frame)

            if ids is not None:
                for i, corner in enumerate(corners):
                    success, rvec, tvec = cv2.solvePnP(
                        self.marker_3d, corner[0].astype(np.float32),
                        self.camera_matrix, self.dist_coeffs)
                    if success:
                        pose = PoseStamped()
                        pose.header.frame_id = f"aruco_{ids[i][0]}"
                        pose.pose.position.x = float(tvec[0])
                        pose.pose.position.y = float(tvec[1])
                        pose.pose.position.z = float(tvec[2])
                        pose.pose.orientation.x = float(rvec[0])
                        pose.pose.orientation.y = float(rvec[1])
                        pose.pose.orientation.z = float(rvec[2])
                        pose.pose.orientation.w = 1.0  # 추후 quaternion으로 변경 가능
                        self.publisher_.publish(pose)

        except Exception as e:
            self.get_logger().warn(f"UDP Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
