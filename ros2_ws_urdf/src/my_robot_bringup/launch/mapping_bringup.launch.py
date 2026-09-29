import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Get Package Directories ---
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    my_robot_bringup_dir = get_package_share_directory('my_robot_bringup')
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    my_robot_description_dir = get_package_share_directory('my_robot_description')

    # --- 1. SLAM Toolbox ---
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        )
    )

    # --- 2. ESP32 Hardware Driver ---
    esp32_driver_cmd = Node(
        package='my_robot_hardware',
        executable='esp32_driver',
        name='esp32_driver',
        output='screen',
        parameters=[{'port': '/tmp/esp32_base'}]
    )

    # --- 3. EKF Node ---
    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_bringup_dir, 'launch', 'ekf_launch.launch.py')
        )
    )

    # --- 4. Joint State Publisher ---
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # --- 5. RPLidar A1 ---
    sllidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/tmp/rplidar',  # Or '/dev/ttyUSB0' if not using udev rules
            'frame_id': 'lidar_link'
        }.items()
    )

    # --- 6. Robot Description / TF Display ---
    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_description_dir, 'launch', 'display.launch.py')
        )
    )

    # --- Return the Launch Description ---
    return LaunchDescription([
        esp32_driver_cmd,
        joint_state_publisher_cmd,
        robot_description_cmd,
        sllidar_cmd,
        ekf_cmd,
        slam_toolbox_cmd
    ])