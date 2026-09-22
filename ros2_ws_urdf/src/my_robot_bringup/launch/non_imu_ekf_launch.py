from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ekf_config = os.path.join(
        get_package_share_directory("my_robot_bringup"),
        "config",
        "ekf_non_imu.yaml",
    )

    return LaunchDescription([

        # Publish base_footprint -> base_link only if your URDF does NOT publish it.
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="base_footprint_to_base_link",
        #     arguments=[
        #         "0", "0", "0",
        #         "0", "0", "0",
        #         "base_footprint",
        #         "base_link"
        #     ],
        #     output="screen"
        # ),

        # EKF
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config],
        ),

    ])