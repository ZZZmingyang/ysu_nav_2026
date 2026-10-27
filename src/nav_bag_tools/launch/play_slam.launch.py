# SLAM回放入口：在本文件中选择话题，直接启动官方ros2 bag play。
# 此launch只启动播放器；RViz、机器人模型和SLAM算法需要另外启动。

import math
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
    return LaunchDescription([
        # 2. 声明回放参数。bag_path必须指向含metadata.yaml的bag目录。
        DeclareLaunchArgument(
            'bag_path',
            description='要回放的bag目录，必须指定',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='view',
            choices=['view', 'reprocess'],
            description='view查看历史数据；reprocess仅播放算法重算需要的输入',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='播放倍率，例如1.0为原速，0.5为半速',
        ),
        DeclareLaunchArgument(
            'clock_hz',
            default_value='100.0',
            description='播放器发布/clock的频率，单位Hz',
        ),
        DeclareLaunchArgument(
            'start_paused',
            default_value='false',
            description='是否暂停启动；暂停后通过resume服务恢复播放',
        ),
        DeclareLaunchArgument(
            'include_debug',
            default_value='false',
            description='是否追加调试话题，仅view模式可用',
        ),
        DeclareLaunchArgument(
            'topics_file',
            default_value=topics_file,
            description='回放话题清单文件',
        ),
        DeclareLaunchArgument(
            'qos_file',
            default_value=qos_file,
            description='rosbag2的QoS覆盖文件',
        ),

        # 3. 参数声明完成后，调用下面的start_play函数。
        # OpaqueFunction让函数在启动时执行，此时才能读到终端传入的实际值。
        OpaqueFunction(function=start_play),
    ])


def start_play(context):
    # 4. 读取实际启动参数。perform(context)把LaunchConfiguration转成字符串。
    topics_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('topics_file').perform(context)))
    qos_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('qos_file').perform(context)))
    bag_path = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('bag_path').perform(context)))
    mode = LaunchConfiguration('mode').perform(context)
    rate = float(LaunchConfiguration('rate').perform(context))
    clock_hz = float(LaunchConfiguration('clock_hz').perform(context))
    start_paused = LaunchConfiguration('start_paused').perform(context).lower()
    include_debug = LaunchConfiguration('include_debug').perform(context).lower()

    if start_paused not in ('true', 'false') or include_debug not in ('true', 'false'):
        raise ValueError('start_paused和include_debug必须为true或false')
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('播放倍率rate必须是有限正数')
    if not math.isfinite(clock_hz) or clock_hz <= 0:
        raise ValueError('时钟频率clock_hz必须是有限正数')
    if mode == 'reprocess' and include_debug == 'true':
        raise ValueError('重算模式不能混入调试输出，include_debug仅供view模式使用')
    if not os.path.isfile(qos_file):
        raise ValueError(f'QoS文件不存在：{qos_file}')
    if not os.path.isfile(os.path.join(bag_path, 'metadata.yaml')):
        raise ValueError(f'bag_path必须是含metadata.yaml的bag目录：{bag_path}')

    # 5. view读取record列表；reprocess只读取原始输入，避免混入旧算法结果。
    group = 'record' if mode == 'view' else 'reprocess'
    with open(topics_file, encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict) or not isinstance(profile.get(group), list):
        raise ValueError(f'话题配置中的{group}必须是列表')
    topics = list(profile[group])
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
    topics = list(dict.fromkeys(topics))

    # 6. 组装官方命令。算法和RViz应与播放器使用同一个独立的回放ROS域。
    command = [
        'ros2', 'bag', 'play', bag_path,
        '--rate', str(rate),
        '--clock', str(clock_hz),
        '--disable-keyboard-controls',
        '--qos-profile-overrides-path', qos_file,
    ]
    if start_paused == 'true':
        command.append('--start-paused')
    command.extend(['--topics', *topics])
    print(f'回放模式：{mode}，选择{len(topics)}路话题，bag：{bag_path}', flush=True)

    # 直接启动官方播放器；本launch不启动SLAM算法、模型或硬件驱动。
    return [ExecuteProcess(
        cmd=command,
        output='screen',
        log_cmd=True,
        sigterm_timeout='30',
        sigkill_timeout='30',
        # 播完或手动结束后，整个launch一同退出。
        on_exit=[Shutdown(reason='rosbag2回放进程已退出')],
    )]
