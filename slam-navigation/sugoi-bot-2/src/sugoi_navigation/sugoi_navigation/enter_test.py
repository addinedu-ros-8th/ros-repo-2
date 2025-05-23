#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class MultiNavByInput(Node):
    def __init__(self):
        super().__init__('multi_nav_by_input')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.named_goals = {
            1: [
                {'label': 'Go to P1 (Step 1)', 'position': {'x': 0.60025, 'y': 0.03700}, 'orientation': {'z': -0.71127, 'w': 0.70292}},
                {'label': 'Go to P1 (Step 2)', 'position': {'x': 0.42784, 'y': -1.10289}, 'orientation': {'z': 0.01248, 'w': 0.99992}},
                {'label': 'Pallet P1',         'position': {'x': 0.95169, 'y': -1.10289}, 'orientation': {'z': 0.72235, 'w': 0.69152}}
            ],
            2: [
                {'label': 'Pallet P2', 'position': {'x': 1.27662, 'y': -1.10289}, 'orientation': {'z': 0.72235, 'w': 0.69152}}
            ],
            3: [
                {'label': 'Pallet P3', 'position': {'x': 1.57056, 'y': -1.10289}, 'orientation': {'z': 0.72235, 'w': 0.69152}}
            ],
            4: [
                {'label': 'Car #1', 'position': {'x': 1.95511, 'y': -0.95301}, 'orientation': {'z': 0.00044, 'w': 0.99990}}
            ],
            5: [
                {'label': 'Car #2', 'position': {'x': 1.90511, 'y': 0.00972}, 'orientation': {'z': -0.00036, 'w': 0.99999}}
            ],
            6: [
                {'label': 'Pallet P4', 'position': {'x': 0.93976, 'y': 0.22762}, 'orientation': {'z': -0.72112, 'w': 0.69281}}
            ],
            7: [
                {'label': 'Pallet P5', 'position': {'x': 1.22662, 'y': 0.22761}, 'orientation': {'z': -0.72112, 'w': 0.69281}}
            ],
            8: [
                {'label': 'Pallet P6', 'position': {'x': 1.52056, 'y': 0.22762}, 'orientation': {'z': -0.72112, 'w': 0.69281}}
            ],
            9: [
                {'label': 'Go to the home (Step 1)', 'position': {'x': 1.53723, 'y': 0.24254}, 'orientation': {'z': 0.99977, 'w': 0.02126}},
                {'label': 'Go to the home (Step 2)', 'position': {'x': 0.32749, 'y': 0.24254}, 'orientation': {'z': 0.99977, 'w': 0.02126}},
                {'label': 'Pallet P4, P5, P6 Line Home', 'position': {'x': -0.17097, 'y': -0.035032}, 'orientation': {'z': -0.02076, 'w': 0.99978}}
            ],
            10: [
                {'label': 'Go to the home (Step 1)', 'position': {'x': 1.51814, 'y': -1.10289}, 'orientation': {'z': 0.99993, 'w': 0.01165}},
                {'label': 'Go to the home (Step 2)', 'position': {'x': 0.34893, 'y': -0.94988}, 'orientation': {'z': 0.65811, 'w': 0.75291}},
                {'label': 'Go to the home (Step 3)', 'position': {'x': 0.32749, 'y': 0.24254}, 'orientation': {'z': 0.99977, 'w': 0.02126}},
                {'label': 'Pallet P1, P2, P3 Line Home', 'position': {'x': -0.09097, 'y': -0.07632}, 'orientation': {'z': -0.02076, 'w': 0.99978}}
            ],
        }

        self._client.wait_for_server()
        self.request_goal_sequence()

    def request_goal_sequence(self):
        print("\n📍 이동할 번호들을 **공백으로 구분해서** 입력하세요 (예: 1 3 6):")
        for num, g_list in self.named_goals.items():
            print(f"{num}: {g_list[-1]['label']}")

        input_str = input("▶ 선택 번호들: ")
        try:
            numbers = list(map(int, input_str.strip().split()))
            self.goals = [step for n in numbers if n in self.named_goals for step in self.named_goals[n]]
        except Exception:
            self.get_logger().error("❌ 입력 형식이 잘못되었습니다.")
            rclpy.shutdown()
            return

        if not self.goals:
            self.get_logger().warn("⚠️ 선택된 목표가 없습니다.")
            rclpy.shutdown()
            return

        self.goal_index = 0
        self.send_next_goal()

    def send_next_goal(self):
        if self.goal_index >= len(self.goals):
            self.get_logger().info("🎉 모든 선택된 위치 도착 완료!")
            rclpy.shutdown()
            return

        goal_data = self.goals[self.goal_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal_data['position']['x']
        goal_msg.pose.pose.position.y = goal_data['position']['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = goal_data['orientation']['z']
        goal_msg.pose.pose.orientation.w = goal_data['orientation']['w']

        self.get_logger().info(f"🚗 {goal_data['label']} 이동 중 ({self.goal_index + 1}/{len(self.goals)})")
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        label = self.goals[self.goal_index]['label']
        self.get_logger().info(f"🎯 {label} 도착 완료!\n")

        # ✅ 모든 위치에서 Enter 키 입력 대기
        input(f"⏸ {label} 도착 ✅ → 다음으로 진행하려면 [Enter] 를 누르세요...")

        self.goal_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    navigator = MultiNavByInput()
    rclpy.spin(navigator)


if __name__ == '__main__':
    main()
