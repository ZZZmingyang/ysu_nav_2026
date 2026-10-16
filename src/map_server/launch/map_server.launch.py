import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    bringup_dir = get_package_share_directory('pb2025_nav_bringup')

    default_map_file = os.path.join(
        bringup_dir,
        'map',
        'staging',
        'my_map.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': LaunchConfiguration('map')}
        ]
    )

    manager_mapper_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        map_server_node,
        manager_mapper_node
    ])
