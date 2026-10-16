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


"""Static-map variant of rm_navigation_reality_launch.py.

Preserves the original entry-point structure and navigation nodes,
uses localization_static_launch.py instead of small_gicp, and reads
the saved grid map without a prior PCD. Static offsets default to zero.
Robot serial joint feedback is still started separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    container_name = LaunchConfiguration("container_name")
    log_level = LaunchConfiguration("log_level")
    use_joy = LaunchConfiguration("use_joy")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_z = LaunchConfiguration("map_to_odom_z")
    map_to_odom_roll = LaunchConfiguration("map_to_odom_roll")
    map_to_odom_pitch = LaunchConfiguration("map_to_odom_pitch")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Select world: 'rmul_2024' or 'rmuc_2024' (map file share the same name as the this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "reality", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
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
        default_value="True",
        description="Whether to use composed bringup",
        choices=["True", "False"],
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="Name of the navigation component container",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_use_joy_cmd = DeclareLaunchArgument(
        "use_joy",
        default_value="True",
        description="Whether to start joystick teleoperation",
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

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    # Create our own temporary YAML files that include substitutions

    # Preserve bringup_launch.py's namespace substitution in the shared YAML.
    navigation_params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ""},
        condition=LaunchConfigurationEquals("namespace", ""),
    )
    navigation_params_file = ReplaceString(
        source_file=navigation_params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=navigation_params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            # Keep the robot model joint-state configuration separate from Nav2.
            "params_file": os.path.join(
                get_package_share_directory("robot_description"),
                "params",
                "robot_description.yaml",
            ),
        }.items(),
    )

    start_livox_ros_driver2_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        namespace=namespace,
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    # Keep bringup_launch.py's namespace/container/navigation wiring here so the
    # original launch files remain unchanged and only these three variants are needed.
    bringup_cmd = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                condition=IfCondition(use_composition),
                name=container_name,
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[
                    configured_params,
                    {"autostart": autostart, "use_sim_time": use_sim_time},
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.dirname(__file__), "localization_static_launch.py"
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": navigation_params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": container_name,
                    "log_level": log_level,
                    "map_to_odom_x": map_to_odom_x,
                    "map_to_odom_y": map_to_odom_y,
                    "map_to_odom_z": map_to_odom_z,
                    "map_to_odom_roll": map_to_odom_roll,
                    "map_to_odom_pitch": map_to_odom_pitch,
                    "map_to_odom_yaw": map_to_odom_yaw,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": navigation_params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": container_name,
                    "log_level": log_level,
                }.items(),
            ),
        ]
    )

    joy_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "joy_teleop_launch.py")),
        condition=IfCondition(use_joy),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "joy_config_file": navigation_params_file,
        }.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_joy_cmd)
    ld.add_action(declare_map_to_odom_x_cmd)
    ld.add_action(declare_map_to_odom_y_cmd)
    ld.add_action(declare_map_to_odom_z_cmd)
    ld.add_action(declare_map_to_odom_roll_cmd)
    ld.add_action(declare_map_to_odom_pitch_cmd)
    ld.add_action(declare_map_to_odom_yaw_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(GroupAction([start_robot_state_publisher_cmd]))
    ld.add_action(start_livox_ros_driver2_node)
    ld.add_action(bringup_cmd)
    ld.add_action(GroupAction([joy_teleop_cmd]))
    ld.add_action(GroupAction([rviz_cmd]))

    return ld
