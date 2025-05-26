from final_package.waypoint_base_node import WaypointBaseNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = WaypointBaseNode(
        name='wp0_node',
        waypoint_id=0,
        waypoint_x=0.19,
        waypoint_y=0.03,
        ar_target_x=-0.91,
        ar_target_x_backward=-0.7
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
