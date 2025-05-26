from final_package.waypoint_base_node import WaypointBaseNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = WaypointBaseNode(
        name='wp1_node',
        waypoint_id=1,
        waypoint_x=0.7,
        waypoint_y=-0.04,
        ar_target_x=-0.655,
        ar_target_x_backward=-0.42
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
