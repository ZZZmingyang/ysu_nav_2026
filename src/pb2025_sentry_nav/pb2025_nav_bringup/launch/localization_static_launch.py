# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""Static-map variant of localization_launch.py.

Keeps Point-LIO, map_server and lifecycle management; replaces small_gicp
with one static map -> odom broadcaster. The parent applies the namespace
and TF remappings, as in the original launch. Zero offsets require initial
alignment with the saved map; odometry drift is not globally corrected.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_z = LaunchConfiguration("map_to_odom_z")
    map_to_odom_roll = LaunchConfiguration("map_to_odom_roll")
    map_to_odom_pitch = LaunchConfiguration("map_to_odom_pitch")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")

    lifecycle_nodes = ["map_server"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "reality", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_map_to_odom_x_cmd = DeclareLaunchArgument(
        "map_to_odom_x",
        default_value="0.0",
        description="Pose of odom in map: x (metres)",
    )

    declare_map_to_odom_y_cmd = DeclareLaunchArgument(
        "map_to_odom_y",
        default_value="0.0",
        description="Pose of odom in map: y (metres)",
    )

    declare_map_to_odom_z_cmd = DeclareLaunchArgument(
        "map_to_odom_z",
        default_value="0.0",
        description="Pose of odom in map: z (metres)",
    )

    declare_map_to_odom_roll_cmd = DeclareLaunchArgument(
        "map_to_odom_roll",
        default_value="0.0",
        description="Pose of odom in map: roll (radians)",
    )

    declare_map_to_odom_pitch_cmd = DeclareLaunchArgument(
        "map_to_odom_pitch",
        default_value="0.0",
        description="Pose of odom in map: pitch (radians)",
    )

    declare_map_to_odom_yaw_cmd = DeclareLaunchArgument(
        "map_to_odom_yaw",
        default_value="0.0",
        description="Pose of odom in map: yaw (radians)",
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"use_sim_time": use_sim_time},
            {"prior_pcd.enable": False},
            {"pcd_save.pcd_save_en": False},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file},
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # Same static publisher pattern as slam_launch.py; no SLAM is started here.
    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x",
            map_to_odom_x,
            "--y",
            map_to_odom_y,
            "--z",
            map_to_odom_z,
            "--roll",
            map_to_odom_roll,
            "--pitch",
            map_to_odom_pitch,
            "--yaw",
            map_to_odom_yaw,
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_map_to_odom_x_cmd)
    ld.add_action(declare_map_to_odom_y_cmd)
    ld.add_action(declare_map_to_odom_z_cmd)
    ld.add_action(declare_map_to_odom_roll_cmd)
    ld.add_action(declare_map_to_odom_pitch_cmd)
    ld.add_action(declare_map_to_odom_yaw_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_transform_node)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
