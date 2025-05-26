from setuptools import find_packages, setup

package_name = 'final_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wp_all.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinky',
    maintainer_email='pinky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_waypoint_node = final_package.send_waypoint_node:main',
            'waypoint_base_node = final_package.waypoint_base_node:main',
            'wp0_node = final_package.wp0_node:main',   
            'wp1_node = final_package.wp1_node:main',       
            'wp2_node = final_package.wp2_node:main',   
            'wp3_node = final_package.wp3_node:main',   
            'wp4_node = final_package.wp4_node:main',
            'wp5_node = final_package.wp5_node:main',    
            'wp6_node = final_package.wp6_node:main',  
            'wp7_node = final_package.wp7_node:main',  
            'wp8_node = final_package.wp8_node:main',  
            'esp32_sequence_node = final_package.esp32_sequence_node:main',
            'aruco_drive_node = final_package.aruco_drive_node:main',
            'task_to_waypoint_node = final_package.task_to_waypoint_node:main',
            'pose_client_node = final_package.pose_client_node:main',
            'aruco_pickup_node = final_package.aruco_pickup_node:main',
        ],
    },
)