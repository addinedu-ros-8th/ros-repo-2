from setuptools import find_packages, setup

package_name = 'sugoi_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jubin',
    maintainer_email='jubineduros@gmail.com',
    description='SUGOI SLAM&Navigation Multi waypoint navigator using nav2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_nav_to_pose = sugoi_navigation.multi_nav_to_pose:main'
        ],
    },
)
