#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
import json
import time  # 시간 대기용

class TaskToWaypointNode(Node):
    def __init__(self):
        super().__init__('task_to_waypoint_node')

        # 퍼블리셔: waypoint 번호 발행
        self.wp_pub = self.create_publisher(Int32, '/send_waypoint', 10)

        # 서브스크라이버: waypoint 4번 도착 알림
        self.sub_done = self.create_subscription(Bool, '/wp4_done', self.wp4_done_callback, 10)
    
        # 서브스크라이버: task 서버에서 보내는 goal 정보 수신
        self.task_sub = self.create_subscription(String, '/current_task', self.task_callback, 10)
        self.wp4_arrived = False

        self.pending_task = None  # 현재 처리 중인 task 저장용
        self.get_logger().info("🚀 Task → Waypoint 노드 시작됨")

    def task_callback(self, msg: String):
        try:
            task = json.loads(msg.data)
            self.pending_task = task    
            self.wp4_arrived = False

            self.get_logger().info(
                f"📦 수신된 task: {task['task_type']} / {task['barcode']}, {task['src']} → {task['dst']}"
            )


            # dock1: waypoint 4번으로 우선 이동
            wp_msg = Int32()
            wp_msg.data = 4
            self.wp_pub.publish(wp_msg)
            self.get_logger().info("➡️ 이동: waypoint 4번 (dock1)")

        except json.JSONDecodeError:
            self.get_logger().error("❌ JSON 파싱 실패: /current_task 메시지가 올바르지 않음.")

    def wp4_done_callback(self, msg: Bool):
        if not msg.data or self.wp4_arrived:
            return
        
        self.wp4_arrived = True

        if msg.data and self.pending_task:
            time.sleep(1.0)
            dst = self.pending_task['dst']

            dst_map = {
                'P1-upper': 1,
                'P2-upper': 2,
                'P3-upper': 3,
                'P1-lower': 1,
                'P2-lower': 2,
                'P3-lower': 3,
            }

            if dst in dst_map:
                wp_msg = Int32()
                wp_msg.data = dst_map[dst]
                self.wp_pub.publish(wp_msg)
                self.get_logger().info(f"➡️ 이동: 최종 waypoint {dst_map[dst]}번 ({dst})")
            else:
                self.get_logger().warn(f"❌ 알 수 없는 목적지: {dst}")

            # task 완료 처리
            self.pending_task = None

    def wait_for_task(self):
        # task가 없으면 기다림
        while not self.pending_task:
            time.sleep(1)  # 1초 간격으로 대기

def main(args=None):
    rclpy.init(args=args)
    node = TaskToWaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
