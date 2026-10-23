# robot_description

基于 XMacro/SDF 的机器人描述功能包，用于生成 URDF、发布关节状态和 TF，并提供 RViz 可视化入口。

本包基于 [SMBU PolarBear Team 的 pb2025_robot_description](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description) 修改，保留上游来源与致谢。

## 1. Overview

本项目使用 [xmacro](https://github.com/gezp/xmacro) 组合机器人结构和传感器模型，再使用 [sdformat_tools](https://github.com/gezp/sdformat_tools) 将生成的 SDF 转换为 URDF。

描述包和入口采用通用名称；当前入口中的机械结构与安装参数仍对应当前机器人，可按车型更换。

```text
robot_description/
├── launch/robot_description_launch.py
├── params/robot_description.yaml
├── resource/
│   ├── xmacro/
│   │   ├── reality_robot.sdf.xmacro
│   │   └── simulation_robot.sdf.xmacro
│   └── models/
│       ├── mid360/
│       ├── industrial_camera/
│       └── rplidar_a2/
└── rviz/visualize_robot.rviz
```

- [reality_robot](./resource/xmacro/reality_robot.sdf.xmacro)：实车描述入口。Mid360 固连于 `gimbal_yaw_odom`，工业相机固连于 `gimbal_pitch`，使用当前实车的高度和安装参数。
- [simulation_robot](./resource/xmacro/simulation_robot.sdf.xmacro)：仿真描述入口。RPLidar A2 和 Mid360 固连于 `chassis`，工业相机固连于 `gimbal_pitch`，保留现有仿真配置及控制插件。

两份入口都通过 `model://ysu2026_sentry_robot/ysu2026_sentry_robot.def.xmacro` 调用 `rmoss_gz_resources` 中的具体机械结构宏。该模型基于上游示例修改，包含当前车型的底盘、轮子、云台和装甲板，因此保留车型名称。入口中的 `<model name>` 与被调用的宏名可以不同。

换车时可修改入口的模型 include、宏调用和传感器安装参数，也可通过 `robot_xmacro_file` 加载其他位置的描述文件，通过 `params_file` 加载对应的关节状态配置。

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Workspace and Dependencies

```bash
sudo apt install git-lfs
pip install vcstool2
```

以下命令在当前导航工作区执行。本包位于工作区的 `src/robot_description`，上游仓库链接仅用于说明来源。

```bash
cd /home/tungchiahui/UserFolder/MySource/ysu2026_rm_sentry/ysu2026_rm_sentry_nav
source /opt/ros/humble/setup.bash
```

如需补齐本包的源码依赖，可从工作区执行；已有依赖无需重复导入：

```bash
vcs import src --recursive < src/robot_description/dependencies.repos
```

```bash
pip install xmacro
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install
```

### 2.4 Running

#### Option1: 在 RViz 中可视化机器人

实车入口：

```bash
source install/setup.bash
ros2 launch robot_description robot_description_launch.py \
  robot_name:=reality_robot \
  use_sim_time:=false \
  use_rviz:=true
```

仿真入口：

```bash
ros2 launch robot_description robot_description_launch.py \
  robot_name:=simulation_robot \
  use_sim_time:=true \
  use_rviz:=true
```

`robot_name` 默认仍为 `simulation_robot`。上述命令启动描述发布器和 RViz；Gazebo 实体由 `rmu_gazebo_simulator` 的启动流程生成。

`pb2025_nav_bringup` 的实车描述启动入口默认选择 `reality_robot`，`rmu_gazebo_simulator` 则加载 `simulation_robot`。

#### Option2: Python API

通过 Python API，在 launch file 中解析 XMacro 文件，生成 URDF 和 SDF 文件 (Recommend)：

> [!TIP]
>
> [robot_state_publisher](https://github.com/ros/robot_state_publisher) 需要传入 urdf 格式的机器人描述文件
>
> Gazebo 仿真器 spawn robot 时，需要传入 sdf / urdf 格式的机器人描述文件

感谢前辈的开源工具 [xmacro](https://github.com/gezp/xmacro) 和 [sdformat_tools](https://github.com/gezp/sdformat_tools) ，这里简述 XMacro 转 URDF 和 SDF 的示例，用于在 launch file 中生成 URDF 和 SDF 文件。

```python
from xmacro.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator

xmacro = XMLMacro4sdf()
xmacro.set_xml_file(robot_xmacro_path)

# Generate SDF from xmacro
xmacro.generate()
robot_xml = xmacro.to_string()

# Generate URDF from SDF
urdf_generator = UrdfGenerator()
urdf_generator.parse_from_sdf_string(robot_xml)
robot_urdf_xml = urdf_generator.to_string()
```

#### Option3: 命令行

通过命令行直接转换输出 SDF 文件（Not Recommend）:

```bash
source install/setup.bash

xmacro4sdf src/robot_description/resource/xmacro/simulation_robot.sdf.xmacro > /tmp/simulation_robot.sdf
```

## 3. Subscribed Topics

None.

## 4. Published Topics

- `robot_description (std_msgs/msg/String)`

    机器人描述文件（字符串形式）。

- `joint_states (sensor_msgs/msg/JointState)`

    如果命令行中未给出 URDF，则此节点将侦听 `robot_description` 话题以获取要发布的 URDF。一旦收到一次，该节点将开始将关节状态发布到 `joint_states` 话题。

- `any_topic (sensor_msgs/msg/JointState)`

    如果 `sources_list` 参数不为空（请参阅下面的参数），则将订阅此参数中的每个命名话题以进行联合状态更新。不要将默认的 `joint_states` 话题添加到此列表中，因为它最终会陷入无限循环。

- `tf, tf_static (tf2_msgs/msg/TFMessage)`

    机器人关节坐标系信息。

## 5. Launch Arguments

- `use_sim_time` (bool, default: False)

    是否使用仿真时间。

- `robot_name` (str, default: "simulation_robot")

    机器人 XMacro 描述文件的**名字（无需后缀）**，可选 `reality_robot` 或 `simulation_robot`。描述文件应位于 `package://robot_description/resource/xmacro` 目录下。

- `robot_xmacro_file` (str, default: "[simulation_robot.sdf.xmacro](./resource/xmacro/simulation_robot.sdf.xmacro)")

    机器人 XMacro 描述文件的**绝对路径**。本参数的优先级高于 `robot_name`，即若设置了 `robot_xmacro_file`，则 `robot_name` 参数无效。若未设置 `robot_xmacro_file`，则使用 `robot_name` 参数并自动补全路径作为 `robot_xmacro_file` 的值。

- `params_file` (str, default: [robot_description.yaml](./params/robot_description.yaml))

    节点参数配置路径。默认关节状态来源为 `serial/gimbal_joint_state`；换用其他数据源时，可提供另一份 YAML 配置。

- `rviz_config_file` (str, default: [visualize_robot.rviz](./rviz/visualize_robot.rviz))

    RViz 配置文件路径。

- `use_rviz` (bool, default: True)

    是否启动 RViz 可视化界面。

- `use_respawn` (bool, default: False)

    是否在节点退出时尝试重启节点。

- `log_level` (str, default: "info")

    日志级别。
