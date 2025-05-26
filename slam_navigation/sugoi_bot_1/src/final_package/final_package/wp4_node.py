from final_package.waypoint_base_node import WaypointBaseNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = WaypointBaseNode(
        name='wp4_node',
        waypoint_id=4,
        waypoint_x=1.85,
        waypoint_y=0.10,
        ar_target_x=0.47,
        ar_target_x_backward=0.47
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
