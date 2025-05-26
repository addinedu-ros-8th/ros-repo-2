# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import cv2
# import numpy as np
# import time
# import socket
# import os
# from picamera2 import Picamera2

# class ArucoDrive(Node):
#     def __init__(self):
#         super().__init__('aruco_drive_node')
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # 포크 제어
#         self.fork_ip = "192.168.4.18"
#         self.fork_port = 8888

#         self.threshold_z = 0.100
#         self.offset_z = 0.03
#         self.boost_speed = 0.4
#         self.cruise_speed = 0.2
#         self.boost_duration = 0.5
#         self.drive_start = None
#         self.Kp_align = 2.0
#         self.deadband_x = 0.002
#         self.min_turn_speed = 0.2
#         self.reverse_goal_z = 0.45
#         self.deadband_z = 0.02

#         self.K = np.load('/home/pinky/dev_ws/aruco/calibration_matrix.npy')
#         self.D = np.load('/home/pinky/dev_ws/aruco/distortion_coefficients.npy')
#         self.marker_length = 0.05

#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
#         self.params = cv2.aruco.DetectorParameters_create()
#         self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

#         self.picam2 = None
#         self.get_logger().info("✅ ArucoDrive 노드 초기화 완료. /start_aruco_drive 대기 중...")

#     def init_camera(self):
#         self.picam2 = Picamera2()
#         cfg = self.picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)})
#         self.picam2.configure(cfg)
#         self.picam2.start()

#     def close_camera(self):
#         if self.picam2:
#             self.picam2.close()
#             self.picam2 = None

#     def send_fork_up(self, duration_ms: int):
#         try:
#             with socket.create_connection((self.fork_ip, self.fork_port), timeout=5) as s:
#                 s.sendall(f"UP {duration_ms}\n".encode())
#                 time.sleep(duration_ms / 1000)
#         except Exception as e:
#             self.get_logger().error(f"포크 상승 실패: {e}")

#     def run(self):
#         self.init_camera()
#         try:
#             self.get_logger().info(f'▶ 전진 시작')
#             while rclpy.ok():
#                 frame = self.picam2.capture_array()
#                 gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                 corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
#                 if ids is None:
#                     time.sleep(0.1)
#                     continue

#                 rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.K, self.D)
#                 x, _, z_raw = tvecs[0][0]
#                 z_corr = z_raw - self.offset_z

#                 if self.drive_start is None:
#                     self.drive_start = time.time()
#                 elapsed = time.time() - self.drive_start
#                 speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

#                 if z_corr <= self.threshold_z:
#                     self.cmd_pub.publish(Twist())
#                     self.get_logger().info(f"✔ 정지 (z_corr={z_corr:.3f}m)")
#                     break

#                 twist = Twist()
#                 twist.linear.x = speed
#                 if abs(x) > self.deadband_x:
#                     raw_ang = self.Kp_align * x
#                     twist.angular.z = np.sign(raw_ang) * max(abs(raw_ang), self.min_turn_speed)
#                 self.cmd_pub.publish(twist)
#                 time.sleep(0.05)

#             self.get_logger().info("🔼 포크 1000ms 상승")
#             self.send_fork_up(1000)

#             self.get_logger().info(f"▶ 후진 시작")
#             self.drive_start = None
#             while rclpy.ok():
#                 frame = self.picam2.capture_array()
#                 gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                 corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
#                 if ids is None:
#                     time.sleep(0.1)
#                     continue

#                 rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.K, self.D)
#                 x, _, z_raw = tvecs[0][0]
#                 z_corr = z_raw - self.offset_z

#                 if self.drive_start is None:
#                     self.drive_start = time.time()
#                 elapsed = time.time() - self.drive_start
#                 speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

#                 if z_corr >= self.reverse_goal_z - self.deadband_z:
#                     self.cmd_pub.publish(Twist())
#                     self.get_logger().info(f"✔ 후진 정지 (z_corr={z_corr:.3f}m)")
#                     break

#                 twist = Twist()
#                 twist.linear.x = -speed
#                 if abs(x) > self.deadband_x:
#                     raw_ang = self.Kp_align * x
#                     twist.angular.z = np.sign(raw_ang) * max(abs(raw_ang), self.min_turn_speed)
#                 self.cmd_pub.publish(twist)
#                 time.sleep(0.05)

#         finally:
#             self.cmd_pub.publish(Twist())
#             self.close_camera()
#             self.get_logger().info("🏁 ArucoDrive 종료")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArucoDrive()
#     node.run()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
import socket
from picamera2 import Picamera2
from std_msgs.msg import Bool

class ArucoDrive(Node):
    def __init__(self):
        super().__init__('aruco_drive_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ar_done_pub = self.create_publisher(Bool, '/aruco_drive_done', 10)

        # 포크 제어 설정
        self.fork_ip = "192.168.4.3"
        self.fork_port = 8888

        # 파라미터들
        self.threshold_z = 0.15
        self.offset_z = 0.03
        self.boost_speed = 0.4
        self.cruise_speed = 0.2
        self.boost_duration = 0.5
        self.drive_start = None
        self.Kp_align = 2.0
        self.deadband_x = 0.001
        self.min_turn_speed = 0.2
        self.reverse_goal_z = 0.45
        self.deadband_z = 0.02

        # 카메라 보정 행렬 로드
        self.K = np.load('/home/pinky/dev_ws/aruco/calibration_matrix.npy')
        self.D = np.load('/home/pinky/dev_ws/aruco/distortion_coefficients.npy')
        self.marker_length = 0.05

        # Aruco 딕셔너리 및 파라미터 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.picam2 = None
        self.active = False

        # 시작 신호 구독 (Bool 메시지)
        self.start_sub = self.create_subscription(Bool, '/aruco_drive_start', self.start_callback, 10)

        self.get_logger().info("✅ ArucoDrive 노드 초기화 완료, 대기 중...")

    def init_camera(self):
        self.picam2 = Picamera2()
        cfg = self.picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)})
        self.picam2.configure(cfg)
        self.picam2.start()
        self.get_logger().info("📷 카메라 ON")

    def close_camera(self):
        if self.picam2:
            self.picam2.close()
            self.picam2 = None
            self.get_logger().info("📷 카메라 OFF")

    def send_fork_up(self, duration_ms: int):
        try:
            with socket.create_connection((self.fork_ip, self.fork_port), timeout=5) as s:
                s.sendall(f"UP {duration_ms}\n".encode())
                time.sleep(duration_ms / 1000)
        except Exception as e:
            self.get_logger().error(f"포크 상승 실패: {e}")

    def send_fork_down(self, duration_ms: int):
        try:
            with socket.create_connection((self.fork_ip, self.fork_port), timeout=5) as s:
                s.sendall(f"DOWN {duration_ms}\n".encode())
                time.sleep(duration_ms / 1000)
        except Exception as e:
            self.get_logger().error(f"포크 하강 실패: {e}")


    def start_callback(self, msg):
        if msg.data and not self.active:
            self.get_logger().info("▶ ArucoDrive 시작 신호 수신")
            self.active = True
            if self.picam2 is None:
                self.init_camera()
            try:
                self.run_drive()
            except Exception as e:
                self.get_logger().error(f"ArucoDrive 동작 중 예외 발생: {e}")
            if self.picam2 is not None:
                self.close_camera()
            self.active = False
            self.get_logger().info("🏁 ArucoDrive 완료")
        elif not msg.data and self.active:
            self.get_logger().info("▶ ArucoDrive 종료 신호 수신 (옵션 처리)")

    def run_drive(self):
        self.get_logger().info(f'▶ 전진 시작')
        self.drive_start = None
        while rclpy.ok():
            frame = self.picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
            if ids is None:
                time.sleep(0.1)
                continue

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.K, self.D)
            x, _, z_raw = tvecs[0][0]
            z_corr = z_raw - self.offset_z

            if self.drive_start is None:
                self.drive_start = time.time()
            elapsed = time.time() - self.drive_start
            speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

            if z_corr <= self.threshold_z:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"✔ 정지 (z_corr={z_corr:.3f}m)")

                self.get_logger().info("🔽 포크 3500ms 하강")
                self.send_fork_down(3000)  # NEW METHOD
                time.sleep(1.0)  # 안정화 대기

                break

            twist = Twist()
            twist.linear.x = speed
            if abs(x) > self.deadband_x:
                raw_ang = self.Kp_align * x
                twist.angular.z = np.sign(raw_ang) * max(abs(raw_ang), self.min_turn_speed)
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        # self.get_logger().info("🔼 포크 1000ms 상승")
        # self.send_fork_up(1000)

        self.get_logger().info(f"▶ 후진 시작")
        self.drive_start = None
        while rclpy.ok():
            frame = self.picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
            if ids is None:
                time.sleep(0.1)
                continue

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.K, self.D)
            x, _, z_raw = tvecs[0][0]
            z_corr = z_raw - self.offset_z

            if self.drive_start is None:
                self.drive_start = time.time()
            elapsed = time.time() - self.drive_start
            speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

            if z_corr >= self.reverse_goal_z - self.deadband_z:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"✔ 후진 정지 (z_corr={z_corr:.3f}m)")
                break

            twist = Twist()
            twist.linear.x = -speed
            if abs(x) > self.deadband_x:
                raw_ang = self.Kp_align * x
                twist.angular.z = np.sign(raw_ang) * max(abs(raw_ang), self.min_turn_speed)
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_pub.publish(Twist())  # 정지 명령
        self.get_logger().info("🏁 ArucoDrive 종료")
        self.ar_done_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
