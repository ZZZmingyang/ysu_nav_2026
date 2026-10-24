# SLAM录制入口：在本文件中读取话题清单，直接启动官方ros2 bag record。

from datetime import datetime
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 1. 查找功能包安装后的话题清单和QoS配置。
    topics_file = os.path.join(
        get_package_share_directory('nav_bag_tools'),
        'config',
        'slam_topics.yaml',
    )
    qos_file = os.path.join(
        get_package_share_directory('nav_bag_tools'),
        'config',
        'slam_qos.yaml',
    )
    # 本工程使用--symlink-install：realpath沿安装链接找到src中的launch文件。
    # 向上两级就是源码功能包，录制数据统一放在其recordings目录下。
    source_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    recordings_dir = os.path.join(source_dir, 'recordings')

    return LaunchDescription([
        # 2. 声明可从终端传入的参数，例如：output_dir:=/data/recordings。
        # 不传output_dir时，默认保存到src/nav_bag_tools/recordings。
        DeclareLaunchArgument(
            'output_dir',
            default_value=recordings_dir,
            description='bag父目录；默认是源码功能包下的recordings目录',
        ),
        DeclareLaunchArgument(
            'bag_name',
            default_value='',
            description='新bag目录名；留空时自动使用时间戳命名',
        ),
        DeclareLaunchArgument(
            'topics_file',
            default_value=topics_file,
            description='录制话题清单文件',
        ),
        DeclareLaunchArgument(
            'qos_file',
            default_value=qos_file,
            description='rosbag2的QoS覆盖文件',
        ),
        DeclareLaunchArgument(
            'include_debug',
            default_value='false',
            description='是否追加调试话题：true或false',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间；实车通常为false',
        ),
        DeclareLaunchArgument(
            'max_bag_size',
            default_value='2147483648',
            description='单卷最大字节数，默认2 GiB；0关闭按大小分卷',
        ),
        DeclareLaunchArgument(
            'max_bag_duration',
            default_value='300',
            description='单卷最长秒数，默认300秒；不是录制总时长',
        ),
        DeclareLaunchArgument(
            'max_cache_size',
            default_value='104857600',
            description='每个缓存的字节数，默认100 MiB',
        ),

        # 3. 参数声明完成后，调用下面的start_record函数。
        # OpaqueFunction让函数在启动时执行，此时才能读到终端传入的实际值。
        OpaqueFunction(function=start_record),
    ])


def start_record(context):
    # 4. 读取实际启动参数。perform(context)把LaunchConfiguration转成字符串。
    topics_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('topics_file').perform(context)))
    qos_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('qos_file').perform(context)))
    output_dir = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('output_dir').perform(context)))
    bag_name = LaunchConfiguration('bag_name').perform(context)
    include_debug = LaunchConfiguration('include_debug').perform(context).lower()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower()
    max_bag_size = int(LaunchConfiguration('max_bag_size').perform(context))
    max_bag_duration = int(LaunchConfiguration('max_bag_duration').perform(context))
    max_cache_size = int(LaunchConfiguration('max_cache_size').perform(context))

    if include_debug not in ('true', 'false') or use_sim_time not in ('true', 'false'):
        raise ValueError('include_debug和use_sim_time必须为true或false')
    if min(max_bag_size, max_bag_duration, max_cache_size) < 0:
        raise ValueError('分卷大小、分卷时长和缓存大小不能为负数')
    if 0 < max_bag_size <= 86016:
        raise ValueError('分卷大小必须为0，或者大于86016字节')
    if not os.path.isfile(qos_file):
        raise ValueError(f'QoS文件不存在：{qos_file}')

    # 5. 从YAML读取record列表；需要调试数据时，再追加debug列表。
    with open(topics_file, encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict) or not isinstance(profile.get('record'), list):
        raise ValueError('话题配置中的record必须是列表')
    topics = list(profile['record'])
    if include_debug == 'true':
        if not isinstance(profile.get('debug'), list):
            raise ValueError('话题配置中的debug必须是列表')
        topics.extend(profile['debug'])
    if not topics:
        raise ValueError('话题清单不能为空')
    for topic in topics:
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise ValueError('话题必须填写以/开头的完整名称')
        if any(character.isspace() for character in topic):
            raise ValueError(f'话题名称不能含空白字符：{topic!r}')
    topics = list(dict.fromkeys(topics))  # 去重，保持YAML中的先后顺序。

    # 6. 自动命名，并检查同名目录。只创建父目录，bag目录由官方录制器创建。
    if not bag_name:
        bag_name = datetime.now().strftime('slam_%Y%m%d_%H%M%S_%f')
    if bag_name in ('.', '..') or os.path.basename(bag_name) != bag_name:
        raise ValueError('bag_name只能是新目录名，不能是多级路径')
    bag_path = os.path.join(output_dir, bag_name)
    if os.path.exists(bag_path):
        raise ValueError(f'输出目录已经存在，请换一个bag_name：{bag_path}')
    os.makedirs(output_dir, exist_ok=True)

    # 7. 组装官方命令。每个列表元素是一个命令参数，含空格的路径也能正确传入。
    command = [
        'ros2', 'bag', 'record',
        '--storage', 'sqlite3',
        '--output', bag_path,
        '--qos-profile-overrides-path', qos_file,
        '--include-unpublished-topics',
        '--max-bag-size', str(max_bag_size),
        '--max-bag-duration', str(max_bag_duration),
        '--max-cache-size', str(max_cache_size),
    ]
    if use_sim_time == 'true':
        command.append('--use-sim-time')
    command.extend(topics)
    print(f'录制{len(topics)}路话题，保存到：{bag_path}', flush=True)

    # 官方ros2 bag是命令行程序，因此用ExecuteProcess启动。
    return [ExecuteProcess(
        cmd=command,
        output='screen',
        log_cmd=True,
        # Ctrl+C后给官方录制器留出写盘、关闭数据库的时间。
        sigterm_timeout='30',
        sigkill_timeout='30',
        # 录制进程结束后，整个launch一同退出。
        on_exit=[Shutdown(reason='rosbag2录制进程已退出')],
    )]
