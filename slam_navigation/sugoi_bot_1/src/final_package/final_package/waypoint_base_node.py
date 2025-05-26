import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import Int32, Bool
import time

class WaypointBaseNode(Node):
    def __init__(self, name, waypoint_id, waypoint_x, waypoint_y, ar_target_x, ar_target_x_backward):
        super().__init__(name)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.esp32_trigger_pub = self.create_publisher(Bool, '/esp32_start', 10)
        self.aruco_pickup_pub = self.create_publisher(Bool, '/aruco_pickup', 10)
        self.ar_done_sub = self.create_subscription(Bool, '/aruco_drive_done',self.aruco_drive_done_callback ,10)
        self.ar_tag_subscription = self.create_subscription(PoseStamped, '/robot1/pose', self.ar_tag_callback, 10)
        self.pose_subscription = self.create_subscription(PoseStamped, '/tracked_pose', self.pose_callback, 10)
        self.command_sub = self.create_subscription(Int32, '/send_waypoint', self.command_callback, 10)
        
        # ✅ aruco_done 구독 추가 (픽업 완료 신호 수신)
        self.aruco_done_sub = self.create_subscription(Bool, '/aruco_done', self.aruco_done_callback, 10)

        self.current_pose = None
        self.ar_tag_pose = None

        self.waypoint_id = waypoint_id
        self.waypoint_x = waypoint_x
        self.waypoint_y = waypoint_y
        self.ar_target_x = ar_target_x
        self.ar_target_x_backward = ar_target_x_backward

        self.threshold = 0.3
        self.ar_threshold = 0.1

        self.initial_goal_pose = None
        self.arrived_at_waypoint = False
        self.correcting = False
        self.came_from_backward = False

    def ar_tag_callback(self, msg):
        self.ar_tag_pose = msg

    def command_callback(self, msg):
        if msg.data != self.waypoint_id:
            return

        self.arrived_at_waypoint = False
        self.correcting = False
        self.orientation_corrected = False

        if self.current_pose is None:
            self.get_logger().warn("현재 pose 정보를 수신하지 못했습니다.")
            return

        ar_x = self.ar_tag_pose.pose.position.x if self.ar_tag_pose else 0.0

        if self.waypoint_id == 4:
            self.get_logger().info("waypoint id = 4번 입니다")
            self.rotate_to_forward()
            self.came_from_backward = False
        elif self.waypoint_id == 0:
            self.get_logger().info("초기 위치 0번으로 돌아갑니다")
            self.rotate_to_backward()
            self.came_from_backward = True
        elif ar_x > 0.23:
            self.get_logger().info("4번에서 1~3번으로 가는중입니다")
            self.rotate_to_backward()
            self.came_from_backward = True
        else:
            self.get_logger().info("0번에서 1~3번 가는 중입니다")
            self.rotate_to_forward()
            self.came_from_backward = False

        time.sleep(0.5)
        self.initial_goal_pose = Pose()
        self.initial_goal_pose.position.x = self.waypoint_x
        self.initial_goal_pose.position.y = self.waypoint_y
        self.send_waypoint(self.waypoint_x, self.waypoint_y)

    def send_waypoint(self, target_x, target_y=0.0, target_z=0.0):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        pose_msg.pose.position.z = target_z
        pose_msg.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info("가야할 waypoint를 보냈습니다")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        ar_x = self.ar_tag_pose.pose.position.x if self.ar_tag_pose else 0.0

        if self.waypoint_id == 4:
            if self.initial_goal_pose and not self.arrived_at_waypoint:
                dx = abs(self.current_pose.position.x - self.initial_goal_pose.position.x)
                dy = abs(self.current_pose.position.y - self.initial_goal_pose.position.y)
                if dx < self.threshold and dy < self.threshold:
                    self.get_logger().info("4번 waypoint는 오로지 waypoint로만 주행합니다")
                    time.sleep(0.5)
                    self.arrived_at_waypoint = True

                    # 🚩 아루코 픽업 실행 신호만 보냄 (완료 신호는 콜백에서 보냄)
                    msg = Bool()
                    msg.data = True
                    self.aruco_pickup_pub.publish(msg)
                    self.get_logger().info("📦 aruco_pickup 실행 - 완료 대기 중")
            return

        if self.correcting:
            return

        if self.initial_goal_pose and not self.arrived_at_waypoint:
            dx = abs(self.current_pose.position.x - self.initial_goal_pose.position.x)
            dy = abs(self.current_pose.position.y - self.initial_goal_pose.position.y)
            if dx < self.threshold and dy < self.threshold:
                self.get_logger().info("✔ waypoint 도착")
                self.arrived_at_waypoint = True
                self.correcting = True
                self.perform_correction()
                self.create_timer(0.7, self._delayed_correction_callback)

    def aruco_done_callback(self, msg):
        # Aruco 픽업 완료 메시지 수신 시 wp4_done 신호 발행
        if msg.data:
            self.get_logger().info("✅ Aruco 픽업 시퀀스 완료 감지됨 - wp4_done 발행")

            wp_done_pub = self.create_publisher(Bool, '/wp4_done', 10)
            done_msg = Bool()
            done_msg.data = True
            wp_done_pub.publish(done_msg)
            self.get_logger().info("📩 wp4_done 신호 전송 완료")

    def _delayed_correction_callback(self):
        if self.correcting:
            self.perform_correction()

    def perform_correction(self):
        self.get_logger().info("💡 perform_correction() 실행됨")

        if self.ar_tag_pose is None:
            self.get_logger().warn("AR 태그 위치 없음. 보정 불가.")
            return

        robot_x = self.ar_tag_pose.pose.position.x
        robot_y = self.ar_tag_pose.pose.position.y

        diff_x = (self.ar_target_x_backward - robot_x) if self.came_from_backward else (robot_x - self.ar_target_x)
        diff_y = robot_y - 0.24

        if abs(diff_x) > self.ar_threshold:
            if not hasattr(self, 'orientation_corrected') or not self.orientation_corrected:
                self.orientation_corrected = True
                orientation_z, orientation_w = self._compute_orientation_from_y_error(diff_y)
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position = self.current_pose.position
                pose_msg.pose.orientation.z = orientation_z
                pose_msg.pose.orientation.w = orientation_w
                self.pose_publisher.publish(pose_msg)
                self.get_logger().info("좌우회전 각도로 틀었습니다")
                time.sleep(0.5)
            else:
                twist = Twist()
                twist.linear.x = 0.1
                self.cmd_vel_pub.publish(twist)
        else:
            self.get_logger().info("📍 보정 완료 - 정지")
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.5)
            self._publish_orientation(0.72, 0.69, "최종 왼쪽 90도 회전 (전진)")

            msg = Bool()
            msg.data = True
            self.esp32_trigger_pub.publish(msg)
            self.get_logger().info("🚀 ESP32 시퀀스 노드에 시작 신호 보냄")

            self.correcting = False
            self.orientation_corrected = False

    def _compute_orientation_from_y_error(self, diff_y):
        if self.came_from_backward:
            if abs(diff_y) < 0.01:
                return 1.0, 0.0
            elif diff_y > 0:
                return -0.99, 0.01
            else:
                return 0.99, 0.01
        else:
            if abs(diff_y) < 0.01:
                return 0.0, 1.0
            elif diff_y > 0:
                return 0.01, 0.99
            else:
                return -0.01, 0.99

    def rotate_to_forward(self, target_z=-0.15, target_w=1.0):
        self._publish_orientation(target_z, target_w, "전진방향으로 회전")

    def rotate_to_backward(self, target_z=-0.99, target_w=0.01):
        self._publish_orientation(target_z, target_w, "후진으로 회전")

    def _publish_orientation(self, z, w, label):
        if self.current_pose is None:
            self.get_logger().warn("현재 pose 없음, 회전 중단")
            return
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = self.current_pose.position
        pose_msg.pose.orientation.z = z
        pose_msg.pose.orientation.w = w
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"✅ {label} 방향 회전 완료")
    def aruco_drive_done_callback(self, msg):
        if msg.data:
            self.rotate_to_backward()
            time.sleep(0.5)

            # # ✅ 초기화 좌표 설정
            # self.initial_goal_pose = Pose()
            # self.initial_goal_pose.position.x = 0.0
            # self.initial_goal_pose.position.y = 0.0

            # self.arrived_at_waypoint = False
            # self.correcting = False

            # self.send_waypoint(0.0, 0.0)
            # self.get_logger().info("🚗 초기 위치 (0.0, 0.0) 복귀 명령 전송 완료")
