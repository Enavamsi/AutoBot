from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("my_robot_description"),
                "urdf",
                "my_robot.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        ####################################
        ## Robot State Publisher
        ####################################

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
                "publish_frequency": 50.0,
            }],
        ),

        ####################################
        ## Joint State Publisher
        ####################################

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
        ),

        ####################################
        ## RViz
        ####################################

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("my_robot_description"),
                    "rviz",
                    "urdf_config.rviz",
                ]),
            ],
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
        ),
    ])