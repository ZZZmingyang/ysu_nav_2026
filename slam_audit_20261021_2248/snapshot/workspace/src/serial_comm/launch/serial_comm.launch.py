import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('serial_comm'),
        'config',
        'serial_comm.yaml',
    )

    return LaunchDescription([
        Node(
            package='serial_comm',
            executable='serial_comm',
            name='serial_node_cpp',
            output='screen',
            parameters=[config_file],
        ),
    ])
