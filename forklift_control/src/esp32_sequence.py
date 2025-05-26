import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socket
import time
DUR = {1: 800, 2: 2600, 3: 23700}
class ESP32SequenceNode(Node):
    def __init__(self):
        super().__init__('esp32_sequence_node')
        self.declare_parameter('case', 3)
        self.declare_parameter('ip', '192.168.4.3')
        self.declare_parameter('port', 8888)
        self.case = self.get_parameter('case').get_parameter_value().integer_value
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.aruco_trigger_pub = self.create_publisher(Bool, '/aruco_drive_start', 10)
        self.get_logger().info(":large_green_circle: ESP32 시퀀스 노드 대기 중...")
        self.subscriber = self.create_subscription(
            Bool,
            '/esp32_start',
            self.trigger_callback,
            10
        )

    def send_and_wait(self, msg, expect, wait_ms=None):
        max_retries = 5
        retry_delay = 2  # 초
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f":satellite_antenna: ESP32에 연결 시도 중 ({attempt+1}/{max_retries}): {self.ip}:{self.port}")
                with socket.create_connection((self.ip, self.port), timeout=5) as s:
                    self.get_logger().info(f":white_check_mark: 연결 성공! 메시지 전송: {msg}")
                    s.sendall((msg + "\n").encode())
                    timeout_sec = (wait_ms / 1000 + 2) if wait_ms else 10
                    s.settimeout(timeout_sec)
                    start_time = time.time()
                    while True:
                        try:
                            data = s.recv(1024).decode().strip()
                            elapsed = round(time.time() - start_time, 2)
                            self.get_logger().info(f":incoming_envelope: [{elapsed}s 수신됨]: {repr(data)}")
                            if expect in data:
                                self.get_logger().info(f":dart: 기대한 응답 수신됨: '{expect}'")
                                return  # 성공 후 종료
                        except socket.timeout:
                            self.get_logger().error(f":alarm_clock: 수신 타임아웃: {timeout_sec}초 내에 응답 없음")
                            return  # 수신 실패
            except socket.timeout:
                self.get_logger().error(":x: ESP32 연결 타임아웃 - IP 또는 포트를 확인하세요.")
            except ConnectionRefusedError:
                self.get_logger().error(":no_entry_sign: 연결 거부됨 - ESP32에 서버가 실행되고 있는지 확인하세요.")
            except Exception as e:
                self.get_logger().error(f":warning: ESP32 통신 중 알 수 없는 예외 발생: {type(e).__name__}: {e}")
            self.get_logger().info(f":hourglass_flowing_sand: {retry_delay}초 후 재시도 예정...")
            time.sleep(retry_delay)
        self.get_logger().error(":x: 최대 재시도 횟수 초과. ESP32와의 연결에 실패했습니다.")
        
    def trigger_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info(":rotating_light: ESP32 시퀀스 시작 명령 수신!")
            self.execute_sequence()
    def execute_sequence(self):
        self.send_and_wait("FORCE_DOWN", ":white_check_mark: FORCE_DOWN DONE", wait_ms=1000)
        
        if self.case != 3:
            duration = DUR[self.case]
            self.send_and_wait(f"UP {duration}", ":white_check_mark: UP COMPLETE", wait_ms=duration)
        # 이제 /start_aruco_drive 메시지 publish
        msg = Bool()
        msg.data = True
        self.aruco_trigger_pub.publish(msg)
        self.get_logger().info(":outbox_tray: ArucoDrive 시작 신호 발행 완료 (/start_aruco_drive)")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SequenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()