# launch/waypoint_group.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='final_package',
        executable='task_to_waypoint_node',
        name='task_to_waypoint_node',
        output='screen'
        ),
        Node(
            package='final_package',
            executable='wp0_node',
            name='wp0_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp1_node',
            name='wp1_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp2_node',
            name='wp2_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp3_node',
            name='wp3_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp4_node',
            name='wp4_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp5_node',
            name='wp5_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp6_node',
            name='wp6_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp7_node',
            name='wp7_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='wp8_node',
            name='wp8_node',
            output='screen'
        ),
        Node(
            package='final_package',
            executable='esp32_sequence_node',
            name='esp32_node',
            output='screen',
            parameters=[
                {'case': 3},
                {'ip': '192.168.4.3'},
                {'port': 8888},
            ]
        ),
        Node(
        package='final_package',
        executable='aruco_drive_node',
        name='aruco_drive_node',
        output='screen'
        ),
        Node(
        package='final_package',
        executable='pose_client_node',
        name='pose_client_node',
        output='screen'
        ),
        Node(
        package='final_package',
        executable='aruco_pickup_node',
        name='aruco_pickup_node',
        output='screen'
        )
    ])