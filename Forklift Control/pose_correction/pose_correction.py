#주빈이형 핑키
#motor_ratio = 1.05

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
from picamera2 import Picamera2

class ArucoDrive(Node):
    def __init__(self):
        super().__init__('aruco_drive')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 거리 임계치 & Z 오프셋
        self.threshold_z = 0.099  # 목표 정지 거리 (m)
        self.offset_z    = 0.03   # 보정값 (m)

        # 부스트/크루즈 속도 & 시간
        self.boost_speed    = 0.4    # 초기 토크 극복용 (m/s)
        self.cruise_speed   = 0.2    # 유지 속도 (m/s)
        self.boost_duration = 0.5    # 부스트 지속시간 (s)
        self.drive_start    = None

        # 횡방향 정렬 파라미터
        self.Kp_align      = 2.0     # 비례 이득 (rad/m)
        self.deadband_x    = 0.002   # ±0.002m 내에서는 직진
        self.min_turn_speed = 0.2    # 최소 각속도 (rad/s)

        # 카메라 보정 & 마커 세팅
        self.K = np.load('/home/pinky/dev_ws/calibration_matrix.npy')
        self.D = np.load('/home/pinky/dev_ws/distortion_coefficients.npy')
        self.marker_length = 0.05

        # Picamera2 초기화
        self.picam2 = Picamera2()
        cfg = self.picam2.create_preview_configuration(
            main={"format": 'RGB888', "size": (640, 480)}
        )
        self.picam2.configure(cfg)
        self.picam2.start()

        # ArUco 세팅
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_SUBPIX
        self.params.cornerRefinementWinSize       = 5
        self.params.cornerRefinementMaxIterations = 30

    def run(self):
        self.get_logger().info(
            f'▶ 시작: z_corr > {self.threshold_z*100:.1f}cm까지 전진, '
            f'x 오차±{self.deadband_x*100:.1f}cm 내 직진, 벗어나면 회전+직진'
        )
        try:
            while rclpy.ok():
                frame = self.picam2.capture_array()
                gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.params
                )
                if ids is None:
                    self.get_logger().info('→ 마커 미검출, 대기중...')
                    time.sleep(0.1)
                    continue

                # 첫 마커만 사용
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.K, self.D
                )
                x, _, z_raw = tvecs[0][0]
                z_corr = z_raw - self.offset_z

                # 주행 시작 시점 기록
                if self.drive_start is None:
                    self.drive_start = time.time()

                # 속도 모드 결정
                elapsed = time.time() - self.drive_start
                speed = self.boost_speed if elapsed < self.boost_duration else self.cruise_speed

                twist = Twist()

                # 목표 거리 도달?
                if z_corr <= self.threshold_z:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(
                        f"✔ 도달 x={x:.3f}m, z_corr={z_corr:.3f}m → 정지"
                    )
                    break

                # X 오차에 따른 조향 + 직진
                twist.linear.x = speed

                if abs(x) > self.deadband_x:
                    raw_ang = self.Kp_align * x
                    # 최소 속도로 클램핑
                    ang = np.sign(raw_ang) * max(abs(raw_ang), self.min_turn_speed)
                    twist.angular.z = ang

                    action = '우회전+직진' if raw_ang < 0 else '좌회전+직진'
                    self.get_logger().info(
                        f"[{action}] x={x:.3f}m → raw_ang={raw_ang:.3f}, clamped ang={ang:.3f}"
                    )
                else:
                    twist.angular.z = 0.0
                    self.get_logger().info(f"[직진] x={x:.3f}m, z_corr={z_corr:.3f}m")

                self.cmd_pub.publish(twist)
                time.sleep(0.05)

        finally:
            # 안전 정지
            self.cmd_pub.publish(Twist())
            self.picam2.close()
            self.get_logger().info('▶ ArucoDrive 종료')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDrive()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
