from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value="red_standard_robot1",
            description="Robot namespace in Gazebo",
        ),

        Node(
            package="behavior_manager",
            executable="gimbal_scan_server",
            name="gimbal_scan_server",
            namespace=robot_name,
            output="screen",

            parameters=[
                {"use_sim_time": True},
            ],

            remappings=[
                ("serial/gimbal_joint_state", "joint_states"),
            ],
        ),
    ])