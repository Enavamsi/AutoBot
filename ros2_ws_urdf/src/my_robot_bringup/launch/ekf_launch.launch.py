from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        # Static TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            # explicitly defining --frame-id and --child-frame-id prevents ROS 2 version bugs
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                       '--roll', '0', '--pitch', '0', '--yaw', '0', 
                       '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
        ),

        # Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['--x', '0', '--y', '0', '--z', '0.15', 
                       '--roll', '0', '--pitch', '0', '--yaw', '0', 
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
        ),

        # EKF node: fuses /odom + /imu/data -> /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
    ])
