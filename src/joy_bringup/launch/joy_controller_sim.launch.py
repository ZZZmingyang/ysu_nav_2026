import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name")

    joy_launch_file = os.path.join(
        get_package_share_directory("joy_bringup"),
        "launch",
        "joy.launch.py",
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="red_standard_robot1",
        description="Gazebo robot namespace",
    )

    joy_controller_node = Node(
        package="joy_bringup",
        executable="joy_bringup",
        name="joy_node_cpp",
        namespace=robot_name,
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
        remappings=[
            ("joy", "/joy"),
            ("serial/gimbal_joint_state", "joint_states"),
        ],
    )

    return LaunchDescription([
        declare_robot_name,

        # joy_linux:
        # /dev/input/jsX -> /joy
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_file),
        ),

        # /joy -> controller -> robot namespace chassis and gimbal commands
        joy_controller_node,
    ])