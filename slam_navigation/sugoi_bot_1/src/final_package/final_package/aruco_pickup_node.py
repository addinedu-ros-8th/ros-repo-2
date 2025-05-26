
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import cv2
# import numpy as np
# import time
# from std_msgs.msg import Bool
# import socket
# from picamera2 import Picamera2


# class ArucoPickup(Node):
#     def __init__(self):
#         super().__init__('aruco_pickup_node')
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.aruco_pickup_sub = self.create_subscription(Bool, '/aruco_pickup', self.aruco_callback, 10)

#         self.fork_ip = "192.168.4.3"
#         self.fork_port = 8888

#         self.threshold_z = 0.35
#         self.reverse_goal_z = 0.50
#         self.initial_stop_z = 0.50
#         self.offset_z = 0.03
#         self.boost_speed = 0.4
#         self.cruise_speed = 0.2
#         self.boost_duration = 0.5
#         self.Kp_align = 2.0
#         self.deadband_x = 0.001
#         self.min_turn_speed = 0.2
#         self.deadband_z = 0.02
#         self.drive_start = None

#         self.K = np.load('/home/pinky/dev_ws/aruco/calibration_matrix.npy')
#         self.D = np.load('/home/pinky/dev_ws/aruco/distortion_coefficients.npy')
#         self.marker_length = 0.05

#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
#         self.params = cv2.aruco.DetectorParameters_create()
#         self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

#         print("✅ ArucoPickup 노드 초기화 완료")

#     def init_camera(self):
#         self.picam2 = Picamera2()
#         cfg = self.picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)})
#         self.picam2.configure(cfg)
#         self.picam2.start()

#     def close_camera(self):
#         if hasattr(self, 'picam2'):
#             self.picam2.close()

#     def aruco_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("📦 Aruco Pickup 명령 수신됨 - 시퀀스 시작")
#             self.run_drive_sequence()

#     def send_fork_up(self, duration_ms: int):
#         try:
#             with socket.create_connection((self.fork_ip, self.fork_port), timeout=5) as s:
#                 s.sendall(f"UP {duration_ms}\n".encode())
#                 time.sleep(duration_ms / 1000)
#         except Exception as e:
#             print(f"❌ 포크 상승 실패: {e}")

#     def run_drive_sequence(self):
#         self.init_camera()
#         try:
#             self.get_logger().info("▶ Forward: stop at z_corr ≤ 0.50m")
#             self.drive_start = None
#             while rclpy.ok():
#                 pose = self.get_marker_pose()
#                 if pose is None:
#                     time.sleep(0.1)
#                     continue
#                 x, _, z_raw = pose
#                 z_corr = z_raw - self.offset_z

#                 if self.drive_start is None:
#                     self.drive_start = time.time()
#                 elapsed = time.time() - self.drive_start
#                 speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

#                 if z_corr <= self.initial_stop_z:
#                     self.cmd_pub.publish(Twist())
#                     self.get_logger().info(f"✔ Initial stop at z_corr={z_corr:.3f}m")
#                     break

#                 twist = Twist()
#                 twist.linear.x = speed
#                 twist.angular.z = self.Kp_align * x if abs(x) > self.deadband_x else 0.0
#                 self.cmd_pub.publish(twist)
#                 time.sleep(0.05)

#             self.get_logger().info("🔼 포크 800ms 상승")
#             self.send_fork_up(800)
#             time.sleep(1)

#             self.get_logger().info("▶ Forward: stop at z_corr ≤ 0.35m")
#             self.drive_towards_target(target_z=self.threshold_z, forward=True)

#             self.cmd_pub.publish(Twist())
#             self.get_logger().info("🔼 포크 추가 1000ms 상승")
#             self.send_fork_up(1500)
#             time.sleep(1)

#             self.get_logger().info("▶ Reverse: back until z_corr ≥ 0.50m")
#             self.drive_towards_target(target_z=self.reverse_goal_z, forward=False)

#             self.cmd_pub.publish(Twist())
#             self.get_logger().info("🏁 ArucoPickup 시퀀스 완료")
#         finally:
#             self.cmd_pub.publish(Twist())
#             self.close_camera()

#     def drive_towards_target(self, target_z, forward=True):
#         self.drive_start = None
#         while rclpy.ok():
#             pose = self.get_marker_pose()
#             if pose is None:
#                 time.sleep(0.1)
#                 continue
#             x, _, z_raw = pose
#             z_corr = z_raw - self.offset_z

#             if self.drive_start is None:
#                 self.drive_start = time.time()
#             elapsed = time.time() - self.drive_start
#             speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

#             if forward and z_corr <= target_z:
#                 self.cmd_pub.publish(Twist())
#                 self.get_logger().info(f"🛑 전진 정지 (z_corr={z_corr:.3f}m)")
#                 break
#             elif not forward and z_corr >= target_z - self.deadband_z:
#                 self.cmd_pub.publish(Twist())
#                 self.get_logger().info(f"🛑 후진 정지 (z_corr={z_corr:.3f}m)")
#                 break

#             twist = Twist()
#             twist.linear.x = speed if forward else -speed
#             twist.angular.z = self.Kp_align * x if abs(x) > self.deadband_x else 0.0
#             self.cmd_pub.publish(twist)
#             time.sleep(0.05)

#     def get_marker_pose(self):
#         frame = self.picam2.capture_array()
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
#         if ids is None:
#             return None
#         rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
#             corners, self.marker_length, self.K, self.D
#         )
#         return tvecs[0][0]  # x, y, z


# def main():
#     rclpy.init()
#     node = ArucoPickup()
#     node.run_drive_sequence()
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
from std_msgs.msg import Bool
import socket
from picamera2 import Picamera2


class ArucoPickup(Node):
    def __init__(self):
        super().__init__('aruco_pickup_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.aruco_pickup_sub = self.create_subscription(Bool, '/aruco_pickup', self.aruco_callback, 10)
        self.done_pub = self.create_publisher(Bool, '/aruco_done', 10)

        self.fork_ip = "192.168.4.3"
        self.fork_port = 8888

        self.threshold_z = 0.36
        self.reverse_goal_z = 0.50
        self.initial_stop_z = 0.50
        self.offset_z = 0.03
        self.boost_speed = 0.4
        self.cruise_speed = 0.2
        self.boost_duration = 0.5
        self.Kp_align = 2.0
        self.deadband_x = 0.001
        self.min_turn_speed = 0.2
        self.deadband_z = 0.02
        self.drive_start = None

        self.K = np.load('/home/pinky/dev_ws/aruco/calibration_matrix.npy')
        self.D = np.load('/home/pinky/dev_ws/aruco/distortion_coefficients.npy')
        self.marker_length = 0.05

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.picam2 = None
        self.active = False  # 실행 중 플래그

        self.get_logger().info("✅ ArucoPickup 노드 초기화 완료")

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

    def aruco_callback(self, msg: Bool):
        if msg.data and not self.active:
            self.active = True
            self.get_logger().info("📦 Aruco Pickup 명령 수신됨 - 시퀀스 시작")
            try:
                self.run_drive_sequence()
            except Exception as e:
                self.get_logger().error(f"시퀀스 중 오류 발생: {e}")
            finally:
                self.close_camera()
                self.active = False
                self.get_logger().info("🏁 ArucoPickup 시퀀스 종료")

    def send_fork_up(self, duration_ms: int):
        try:
            with socket.create_connection((self.fork_ip, self.fork_port), timeout=5) as s:
                s.sendall(f"UP {duration_ms}\n".encode())
                time.sleep(duration_ms / 1000)
        except Exception as e:
            self.get_logger().error(f"❌ 포크 상승 실패: {e}")

    def run_drive_sequence(self):
        self.init_camera()
        self.get_logger().info("▶ Forward: stop at z_corr ≤ 0.50m")
        self.drive_start = None
        while rclpy.ok():
            pose = self.get_marker_pose()
            if pose is None:
                time.sleep(0.1)
                continue
            x, _, z_raw = pose
            z_corr = z_raw - self.offset_z

            if self.drive_start is None:
                self.drive_start = time.time()
            elapsed = time.time() - self.drive_start
            speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

            if z_corr <= self.initial_stop_z:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"✔ Initial stop at z_corr={z_corr:.3f}m")
                break

            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = self.Kp_align * x if abs(x) > self.deadband_x else 0.0
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.get_logger().info("🔼 포크 800ms 상승")
        self.send_fork_up(800)
        time.sleep(1)

        self.get_logger().info("▶ Forward: stop at z_corr ≤ 0.35m")
        self.drive_towards_target(target_z=self.threshold_z, forward=True)

        self.cmd_pub.publish(Twist())
        self.get_logger().info("🔼 포크 추가 7000ms 상승")
        self.send_fork_up(6000)
        time.sleep(1)

        self.get_logger().info("▶ Reverse: back until z_corr ≥ 0.50m")
        self.drive_towards_target(target_z=self.reverse_goal_z, forward=False)

        self.cmd_pub.publish(Twist())
        self.get_logger().info("🏁 ArucoPickup 시퀀스 완료")

        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        self.get_logger().info("📩 Aruco 픽업 완료 신호(/aruco_done) 발행됨")

    def drive_towards_target(self, target_z, forward=True):
        self.drive_start = None
        while rclpy.ok():
            pose = self.get_marker_pose()
            if pose is None:
                time.sleep(0.1)
                continue
            x, _, z_raw = pose
            z_corr = z_raw - self.offset_z

            if self.drive_start is None:
                self.drive_start = time.time()
            elapsed = time.time() - self.drive_start
            speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

            if forward and z_corr <= target_z:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"🛑 전진 정지 (z_corr={z_corr:.3f}m)")
                break
            elif not forward and z_corr >= target_z - self.deadband_z:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"🛑 후진 정지 (z_corr={z_corr:.3f}m)")
                break

            twist = Twist()
            twist.linear.x = speed if forward else -speed
            twist.angular.z = self.Kp_align * x if abs(x) > self.deadband_x else 0.0
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

    def get_marker_pose(self):
        if self.picam2 is None:
            return None
        frame = self.picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        if ids is None:
            return None
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.K, self.D
        )
        return tvecs[0][0]  # x, y, z


def main():
    rclpy.init()
    node = ArucoPickup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
