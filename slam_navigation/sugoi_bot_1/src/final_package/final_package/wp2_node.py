from final_package.waypoint_base_node import WaypointBaseNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = WaypointBaseNode(
        name='wp2_node',
        waypoint_id=2,
        waypoint_x=0.98,
        waypoint_y=-0.05,
        ar_target_x=-0.28,
        ar_target_x_backward=-0.1
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
