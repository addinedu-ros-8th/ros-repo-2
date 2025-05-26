#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
import json
import subprocess
import time

class MultiNavByInput(Node):
    def __init__(self):
        super().__init__('multi_nav_by_input')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_index = 0
        self.goals = []
        self.waiting_for_continue = False

        self.named_goals = {
            1: [
                {'label': 'Go to P1 (Step 1)', 'position': {'x': 0.60025, 'y': 0.03700}, 'orientation': {'z': -0.71127, 'w': 0.70292}},
                {'label': 'Go to P1 (Step 2)', 'position': {'x': 0.42784, 'y': -1.10289}, 'orientation': {'z': 0.01248, 'w': 0.99992}},
                {'label': 'Pallet P1',         'position': {'x': 0.95169, 'y': -1.10289}, 'orientation': {'z': 0.72235, 'w': 0.69152}}
            ],
            2: [{'label': 'Pallet P2', 'position': {'x': 1.27662, 'y': -1.10289}, 'orientation': {'z': 0.72235, 'w': 0.69152}}],
            3: [{'label': 'Pallet P3', 'position': {'x': 1.57056, 'y': -1.10289}, 'orientation': {'z': 0.72235, 'w': 0.69152}}],
            4: [{'label': 'Car #1', 'position': {'x': 1.95511, 'y': -0.95301}, 'orientation': {'z': 0.00044, 'w': 0.99990}}],
            5: [{'label': 'Car #2', 'position': {'x': 1.88511, 'y': 0.04972}, 'orientation': {'z': -0.00036, 'w': 0.99999}}],
            6: [{'label': 'Pallet P4', 'position': {'x': 0.97976, 'y': 0.22762}, 'orientation': {'z': -0.72112, 'w': 0.69281}}],
            7: [{'label': 'Pallet P5', 'position': {'x': 1.22662, 'y': 0.22761}, 'orientation': {'z': -0.72112, 'w': 0.69281}}],
            8: [{'label': 'Pallet P6', 'position': {'x': 1.52056, 'y': 0.22762}, 'orientation': {'z': -0.72112, 'w': 0.69281}}],
            9: [
                {'label': 'Go to the home (Step 1)', 'position': {'x': 1.53723, 'y': 0.24254}, 'orientation': {'z': 0.99977, 'w': 0.02126}},
                {'label': 'Go to the home (Step 2)', 'position': {'x': 0.32749, 'y': 0.24254}, 'orientation': {'z': 0.99977, 'w': 0.02126}},
                {'label': 'Pallet P4, P5, P6 Line Home', 'position': {'x': -0.09097, 'y': -0.06232}, 'orientation': {'z': -0.02076, 'w': 0.99978}}
            ],
            10: [
                {'label': 'Go to the home (Step 1)', 'position': {'x': 1.51814, 'y': -1.10289}, 'orientation': {'z': 0.99993, 'w': 0.01165}},
                {'label': 'Go to the home (Step 2)', 'position': {'x': 0.34893, 'y': -0.94988}, 'orientation': {'z': 0.65811, 'w': 0.75291}},
                {'label': 'Go to the home (Step 3)', 'position': {'x': 0.32749, 'y': 0.24254}, 'orientation': {'z': 0.99977, 'w': 0.02126}},
                {'label': 'Pallet P1, P2, P3 Line Home', 'position': {'x': -0.09097, 'y': -0.07632}, 'orientation': {'z': -0.02076, 'w': 0.99978}}
            ]
        }

        self.sub_continue = self.create_subscription(Bool, '/nav_continue', self.continue_callback, 10)
        self.task_sub = self.create_subscription(String, '/task_start', self.task_callback, 10)
        self.continue_pub = self.create_publisher(Bool, '/nav_continue', 10)
        self._client.wait_for_server()

    def task_callback(self, msg):
        try:
            task = json.loads(msg.data)
            src = task.get("src", "")
            dst = task.get("dst", "")
            self.get_logger().info(f"📦 Task Received: src={src}, dst={dst}")

            plan = []
            if "P4" in src:
                plan.append(6)
            if "dock" in dst or "Car" in dst:
                plan.append(5)
            plan.append(9)

            self.goals = [step for n in plan for step in self.named_goals.get(n, [])]
            self.goal_index = 0
            self.send_next_goal()
        except Exception as e:
            self.get_logger().error(f"❌ task_callback 예외: {e}")

    def send_next_goal(self):
        if self.goal_index >= len(self.goals):
            self.get_logger().info("🎉 모든 위치 도착 완료!")
            rclpy.shutdown()
            return

        goal_data = self.goals[self.goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_data['position']['x']
        goal_msg.pose.pose.position.y = goal_data['position']['y']
        goal_msg.pose.pose.orientation.z = goal_data['orientation']['z']
        goal_msg.pose.pose.orientation.w = goal_data['orientation']['w']

        self.get_logger().info(f"🚗 {goal_data['label']} 이동 중 ({self.goal_index + 1}/{len(self.goals)})")
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected.")
            rclpy.shutdown()
            return
        self.get_logger().info("✅ Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        label = self.goals[self.goal_index]['label']
        self.get_logger().info(f"🎯 {label} 도착 완료!")

        if label == 'Pallet P4':
            self.get_logger().info("🚀 Pallet P4 도착 → ESP32 + ArUco 실행")
            try:
                subprocess.Popen([
                    "ssh", "pinky@192.168.4.1", "bash", "-ic",
                    "/home/pinky/run_esp32.sh"
                ])
            except Exception as e:
                self.get_logger().error(f"❌ ESP32 실행 실패: {e}")

        elif label == 'Car #2':
            self.get_logger().info("🚗 Car #2 도착 → 내려놓기 실행")
            try:
                subprocess.Popen([
                    "ssh", "pinky@192.168.4.1", "bash", "-ic",
                    "/home/pinky/run_aruco_down.sh"
                ])
            except Exception as e:
                self.get_logger().error(f"❌ 내려놓기 실행 실패: {e}")

        elif label == 'Go to the home (Step 2)':
            self.get_logger().info("🚩 Home Step 2 도착 → 자동 /nav_continue 발행 준비")
            self.waiting_for_continue = True  # 먼저 대기 상태로 전환
            time.sleep(0.3)  # 약간의 딜레이 후 발행 시작
            msg = Bool()
            msg.data = True
            for _ in range(5):
                self.continue_pub.publish(msg)
                self.get_logger().info("✅ /nav_continue → True 발행됨")
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.3)

        self.get_logger().info("⏸ 다음 이동을 위해 /nav_continue (std_msgs/Bool) 토픽 대기 중...")
        self.waiting_for_continue = True

    def continue_callback(self, msg):
        if self.waiting_for_continue and msg.data:
            self.waiting_for_continue = False
            self.get_logger().info("▶️ 이동 재개 신호 수신됨 → 다음 목표로 이동")
            self.goal_index += 1
            self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    navigator = MultiNavByInput()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()