from final_package.waypoint_base_node import WaypointBaseNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = WaypointBaseNode(
        name='wp3_node',
        waypoint_id=3,
        waypoint_x=1.3,
        waypoint_y=-0.02,
        ar_target_x=0.0,
        ar_target_x_backward=0.2
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
