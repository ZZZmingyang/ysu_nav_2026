# 实车 SLAM 数据保存调查（2026-10-21）

调查地点：`ysu.local` → 容器 `ysu_rm_ros_humble`；工作区 `/home/ysu/robomaster/ysu2026_rm_sentry`。采集时间约 **22:48–22:58，Asia/Hong_Kong（UTC+8）**。本报告以实时 graph、端点 QoS、进程实参、参数服务、逐路消息抽样和当前源码相互核对。

**启动命令正确，已从运行进程核实与你给出的两条命令完全一致。** 实车 `slam:=True`、`use_sim_time:=False`、`use_joy:=False`、`use_robot_state_pub:=True`、`use_rviz:=False` 都已生效。两个终端还继承了 `ROS_DOMAIN_ID=6`；仅复制 launch 命令到一个没有相同 ROS 环境的新终端，并不能保证跨进程发现一致。

**本次没有修改工程代码、没有重启节点、没有改参数、没有启动导航目标或发布运动命令，没有实现 nav_bag_tools。已保存的是调查证据、参数/源码/配置快照和地图/SLAM state；没有录制完整 rosbag，更没有倒推出启动前或调查前的原始传感器历史。** Git status 前后完全相同。探针只在临时目录运行；高带宽点云在实车容器内逐路订阅，每路约 2 秒，没有在远端叠加 hz/echo/RViz。

当前最重要的四个发现：

- 原始 Point-LIO 输入是 **`/livox/lidar`（CustomMsg）和 `/livox/imu`**。当前不是使用 `/serial/imu`；也不是使用驱动同时发布的 PointCloud2 副本。
- **串口云台反馈没有进入 TF。** `/serial/gimbal_joint_state` 约 1000 Hz、yaw 样本约 −0.60055 rad，但没有 subscriber。实际 `/joint_states` 是默认值发布器的 10 Hz 全零关节，`source_list=[]`。这关系到实车外参/底盘位姿的物理正确性，不能把“TF 连通”当作“云台补偿正确”。本次保留原状。
- 主链各级当前有新消息，`/obstacle_scan` 约 20 Hz、`/map` 约 2 Hz；但 **`/odometry.child_frame_id=gimbal_yaw`，不是 base_footprint**。其 twist 有较大尖峰，详见后文，不能笼统判定“所有字段正常”。
- `map→odom` 是**静态单位变换**，不是 slam_toolbox 的优化输出；当前 `use_scan_matching=false`、`do_loop_closing=false`、`transform_publish_period=0.0`。重跑时必须保留这些实际设定，不能套用一个默认 slam_toolbox 配置。

## 环境、节点与证据入口

| 项目 | 本次实测 |
|---|---|
| ROS_DISTRO | humble |
| RMW_IMPLEMENTATION | 环境变量未显式设置；进程实际加载 `librmw_fastrtps_cpp.so`，探针有效实现 `rmw_fastrtps_cpp` |
| ROS_DOMAIN_ID | 6 |
| DDS 其他环境 | `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`，`ROS_LOCALHOST_ONLY=0` |
| workspace | `/home/ysu/robomaster/ysu2026_rm_sentry`；overlay 为其中 `install/` |
| Git HEAD | `1e84e2e731aa9f455f2702097de4acccf9135dfa` |
| Git dirty | 是。原有 RViz 文件、4 个已跟踪 pyc 修改；另有 frames 输出、`map/`、test.pgm/test.yaml 未跟踪。本次前后状态一致，完整 diff 已归档。 |
| tf2 | `0.25.24-1jammy.20260915.203746` |
| tf2_ros | `0.25.24-1jammy.20260915.204410` |
| tf2_sensor_msgs | `0.25.23-1jammy.20260907.225802` |
| navigation2 / nav2_bringup | `1.1.20-1jammy.20260908.015443` / `1.1.20-1jammy.20260908.021125` |
| slam_toolbox | `2.6.10-1jammy.20260908.014627` |
| rosbag2 | `0.15.17-1jammy.20260908.100944`；cpp/py/transport/default storage plugins 均为 0.15.17，完整 Debian build 版本见 TSV |
| rclpy | `3.3.22-1jammy.20260907.213431` |

`ros2 node list` 和 `ros2 topic list -t` 的实际输出保存在 [graph_cli.txt](evidence/graph_cli.txt)。全部 topic 的 publisher/subscriber、GID、完整 QoS、节点连接、服务和参数原始响应在 [graph_parameters.json](evidence/graph_parameters.json)。逐路抽样在 [sequential_samples.json](evidence/sequential_samples.json)，可直接检查首末 Header、单调性、频率、点字段、范围和消息样例。整张图的 QoS/订阅者清单见 [topic_inventory.md](topic_inventory.md)。

| 职责 | 实际节点 |
|---|---|
| MID360 驱动 | `/livox_ros_driver2` |
| Point-LIO | `/point_lio` |
| 坐标/消息接口 | `/loam_interface` |
| 扫描与机器人里程计 | `/sensor_scan_generation` |
| 近场/扩展地形 | `/terrain_analysis`、`/terrain_analysis_ext` |
| 地形转二维扫描 | `/pointcloud_to_laserscan` |
| 二维建图 | `/slam_toolbox`（sync_slam_toolbox_node） |
| 模型与关节 | `/robot_state_publisher`、`/joint_state_publisher` |
| 串口 | `/serial_node_cpp`，来自 serial_comm |
| map→odom | `/static_transform_publisher_map2odom` |
| 导航辅助 TF | `/fake_vel_transform`，发布 gimbal_yaw→gimbal_yaw_fake |
| 容器/生命周期/地图保存 | `/nav2_container`、`/lifecycle_manager_navigation`、`/lifecycle_manager_slam`、`/map_saver` |
| 已运行的 Nav2 消费端 | `/controller_server`、`/local_costmap/local_costmap`、`/global_costmap/global_costmap`、`/planner_server`、`/smoother_server`、`/behavior_server`、`/bt_navigator`、`/waypoint_follower`、`/velocity_smoother` |
| 其他 graph 实体 | `/launch_ros_22760`、两个 bt_navigator action 辅助节点、8 个 transform_listener_impl_*；完整名称见原始 node list。它们不是额外传感器。 |

未发现正在运行的 AMCL、small_gicp_relocalization 或加载先验地图的 map_server；`/map_saver` 是保存端，不是先验地图发布端。

### SLAM 实际数据链

以下都是本次实际完整 topic 名称。节点用括号标明；图中没有画出的参数事件和日志端点见附录。

```text
MID360
  ├─ (/livox_ros_driver2) ─ /livox/lidar [CustomMsg, ≈20 Hz] ─┐
  └─ (/livox_ros_driver2) ─ /livox/imu   [Imu, ≈200 Hz] ─────┤
                                                         ▼
                                                   (/point_lio)
                         ┌─ /aft_mapped_to_init [camera_init→body, ≈642 Hz]
                         └─ /cloud_registered  [camera_init, ≈20 Hz]
                                                         ▼
                                                 (/loam_interface)
                         ┌─ /lidar_odometry [odom→front_mid360, ≈648 Hz]
                         └─ /registered_scan [odom, ≈20 Hz]
                              ├──────────────────────────────────────────┐
                              ▼                                          ▼
                    (/sensor_scan_generation)                   (/terrain_analysis)
                     ├─ /sensor_scan [front_mid360, ≈19 Hz]       └─ /terrain_map [odom, ≈20 Hz]
                     │   （当前无消费者）                                  │
                     ├─ /odometry [odom→gimbal_yaw, ≈19 Hz]                 │
                     └─ /tf [odom→base_footprint, ≈19 Hz]                   │
                                                                         ▼
            /registered_scan + /lidar_odometry + /terrain_map ─ (/terrain_analysis_ext)
                                                                         │
                                            /terrain_map_ext [odom, ≈20 Hz]
                                                                         ▼
                                          (/pointcloud_to_laserscan)
                                  cloud_in:=terrain_map_ext，scan:=obstacle_scan
                                                                         │
                                   /obstacle_scan [base_footprint, ≈20 Hz]
                                                                         ▼
                                              (/slam_toolbox)
                                           /map [map, ≈2 Hz]
```

`terrain_analysis` 同时读 `/lidar_odometry` 和 `/registered_scan`；`terrain_analysis_ext` 同时读这两者及 `/terrain_map`，不是只拿近场地形做单输入滤波。Nav2 local_costmap 消费 `/terrain_map`，global_costmap 消费 `/terrain_map_ext`，两者也消费 `/map`。当前二维扫描不是由 `/sensor_scan` 产生的。

云台/模型支路的真实状态：

```text
/serial_node_cpp ─ /serial/gimbal_joint_state [≈1000 Hz] ─ （无消费者）
                └ /serial/imu                [≈1000 Hz] ─ （无消费者）

/joint_state_publisher [source_list=[], 默认关节值]
  └ /joint_states [≈10 Hz；云台 yaw/pitch=0]
       └ /robot_state_publisher ─ /tf + /tf_static
```

**TF 实测与发布者：** namespace 实际为空；launch 把绝对 `/tf`、`/tf_static` remap 到相对 `tf`、`tf_static`，所以最终仍是 `/tf`、`/tf_static`，没有 `xxx/tf`。为避免只凭节点名字推断，已用本机 C++ 订阅收到的 `publisher_gid` 与 graph 端点 GID 匹配，证据见 [tf_publisher_authority.txt](evidence/tf_publisher_authority.txt)。

```text
map
 └─ odom                         /static_transform_publisher_map2odom；/tf_static；单位变换
     └─ base_footprint            /sensor_scan_generation；/tf；约19–20 Hz
         └─ chassis              /robot_state_publisher；/tf_static；z=0.0625
             └─ gimbal_yaw_odom  /robot_state_publisher；/tf_static；z=0.21046
                 └─ gimbal_pitch_odom  同上；/tf_static；z=0.09641
                     └─ gimbal_yaw     /robot_state_publisher；/tf；约10 Hz；z=-0.09641
                         ├─ front_mid360 /robot_state_publisher；/tf_static
                         ├─ gimbal_pitch /robot_state_publisher；/tf；约10 Hz；z=0.09641
                         └─ gimbal_yaw_fake /fake_vel_transform；/tf；约50 Hz
```

动态 gimbal_yaw/gimbal_pitch 边来自模型与 `/joint_states`，但该 JointState 当前是默认零值，并非串口测量。Point-LIO 和 slam_toolbox 在 graph 中都有 `/tf` publisher 端点（slam_toolbox 有两个），然而抽样没有收到它们的 TF：分别与 `publish.tf_send_en=false`、`transform_publish_period=0.0` 一致。不能因 graph 列出这些 publisher 就把 `odom→base_footprint` 或 `map→odom` 归给它们。

`front_mid360` **安装在 gimbal_yaw 下，不在 gimbal_pitch 下**。有效 URDF 中 `gimbal_yaw→front_mid360` 的 xyz=`[-0.00299,0.10469,0.09558] m`，rpy=`[π/4,0,π]`。按当前全零关节合成，`base_footprint→front_mid360` xyz=`[-0.00299,0.10469,0.36854] m`。已在实际 `/obstacle_scan` 时间戳上成功查询 map/front_mid360、odom/base_footprint、base_footprint/front_mid360 等变换，见 [tf_temporal_odometry_check.json](evidence/tf_temporal_odometry_check.json)。

另有两点不能忽略：`/livox/imu.header.frame_id=livox_frame`，当前 TF 中不存在这个 frame；Point-LIO 读取 IMU 数值并使用自己的 LiDAR/IMU 外参，所以当前链仍在运行，但通用回放工具不能把它静默当作 front_mid360。Point-LIO 的 `camera_init`/`body` 是消息中的内部坐标；当前 TF 发送被关掉，loam_interface 通过消息 pose 和启动时获取的模型外参显式变换，不依赖一条现成的 camera_init→odom TF。

### 必须录制

这里严格按“从原始输入重算当前算法”定义；模型、参数和初始条件另外归档，不把算法输出伪装成不可替代的输入。

| Topic | Type | Producer | Consumer | Rate | QoS | 用途 | 为什么必须 |
|---|---|---|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | `/livox_ros_driver2` | `/point_lio` | 约20 Hz | pub R/V → sub BE/V | 原始 LiDAR | 当前 `preprocess.lidar_type=1` 实际订阅此类型；Header、timebase、逐点 offset_time、tag/line/reflectivity 都要保留，不能用已配准点云代替。 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | `/livox_ros_driver2` | `/point_lio` | 约200 Hz | pub R/V → sub BE/V | 原始 IMU | `mapping.imu_en=true`；`use_imu_as_input=false` 不等于不用 IMU。Point-LIO 的时间推进、初始化和估计仍依赖它。 |

以上两个是当前纯 LIO 重跑不可替代的传感器输入。**重跑整个下游链还必须提供机器人模型、静态变换及关节状态。** 对本次“默认零关节”现场，可用已归档 URDF 和本次运行参数重建相同 JSP/RSP；因此 `/joint_states` 和 `/tf_static` 不是在这个前提下数学上不可替代的原始测量，但实际通用录制 profile 应把它们收进去，避免遗漏启动关节状态和模型版本。如果将来接通串口 source_list，则串口 JointState 或等价有效 `/joint_states` 就上升为必须输入，不能沿用本次的“默认值可重建”结论。

### 建议录制

R=RELIABLE，BE=BEST_EFFORT，V=VOLATILE，TL=TRANSIENT_LOCAL；表中先写 publisher，再写 subscriber。完整逐端点列表见话题附录，未以源码猜测补全 UNKNOWN 的 history/depth。

| Topic | Type | Producer | Consumer | Rate | QoS | 用途 | 为什么建议 |
|---|---|---|---|---|---|---|---|
| `/serial/gimbal_joint_state` | `sensor_msgs/msg/JointState` | `/serial_node_cpp` | 当前无 | ≈1000 Hz | R/V | 实测机械角 | 与有效 JointState/TF 对比，保存当前断链证据；以后接通模型链时成为必要原始输入。 |
| `/serial/imu` | `sensor_msgs/msg/Imu` | `/serial_node_cpp` | 当前无 | ≈1000 Hz | R/V | MCU 姿态旁证 | 当前仅有 orientation，角速度/加速度为0；不能拿来替代 MID360 IMU。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `/joint_state_publisher` | `/robot_state_publisher` | ≈10 Hz | R/V→BE/V | 当时实际进入模型的关节 | 当前全零，与串口角不同；避免重跑时无意间“修好”链而改变历史。 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `/robot_state_publisher`、`/static_transform_publisher_map2odom` | TF listeners、`/bt_navigator` | 静态锁存 | R/TL→R/TL | 安装外参、map/odom关系 | 晚加入录制器也必须取得两个 writer 的完整快照；不以持续 Hz 为标准。 |
| `/tf` | `tf2_msgs/msg/TFMessage` | RSP、sensor_scan_generation、fake_vel_transform 实际发消息；其他空端点见前文 | TF listeners、`/bt_navigator` | 合计≈79 Hz；边各自频率不同 | R/V→R/V | 当时坐标链 | 捕获各边、时间戳、重复发布/断流，供原样复盘；全链重算时须隔离这些已录算法输出。 |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | `/point_lio` | `/loam_interface` | ≈642 Hz | R/V→R/V | LIO 原生位姿 | 判断错误先发生在 LIO 还是外参转换；当前高频，不能按20 Hz预算。 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | `/point_lio` | `/loam_interface` | ≈20 Hz | R/V→R/V | LIO 配准输出 | 点云变形、退化、内参/时序对比；约481点样本。 |
| `/lidar_odometry` | `nav_msgs/msg/Odometry` | `/loam_interface` | sensor_scan_generation、terrain_analysis、terrain_analysis_ext | ≈648 Hz | R/V→BE/V（sensor）及R/V（terrain） | odom中的LiDAR位姿 | 定位 loam_interface 外参/初始对齐问题。 |
| `/registered_scan` | `sensor_msgs/msg/PointCloud2` | `/loam_interface` | 上述三个节点 | ≈20 Hz | R/V→BE/V（sensor）及R/V（terrain） | odom点云 | 在不重跑 LIO 时单独重算地形；约521点样本。 |
| `/odometry` | `nav_msgs/msg/Odometry` | `/sensor_scan_generation` | fake_vel_transform、controller_server、bt_navigator | ≈19 Hz | R/V→R/V（fake）及BE/V（Nav2） | gimbal_yaw位姿/速度 | 核查 child frame、TF和速度尖峰。 |
| `/terrain_map` | `sensor_msgs/msg/PointCloud2` | `/terrain_analysis` | terrain_analysis_ext、local_costmap | ≈20 Hz | R/V→R/V（ext）及BE/V（costmap） | 近场地形 | 分清地面分割与下游问题；intensity表示相对地面高度，不是反射率。 |
| `/terrain_map_ext` | `sensor_msgs/msg/PointCloud2` | `/terrain_analysis_ext` | pointcloud_to_laserscan、global_costmap | ≈20 Hz | R/V→BE/V | 二维转换直接输入 | 直接判断断点在地形输出还是 scan 转换。 |
| `/obstacle_scan` | `sensor_msgs/msg/LaserScan` | `/pointcloud_to_laserscan` | `/slam_toolbox` | ≈20 Hz | BE/V→BE/V | SLAM实际扫描 | 可仅重跑二维算法；保存Inf/量程/角度/header。 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `/slam_toolbox` | local/global_costmap、slam_toolbox内部地图订阅 | ≈2 Hz | R/TL→R/TL（costmap），R/V（内部） | 基线地图 | 比较修改前后的地图；地图消息不等价于 pose graph。 |
| `/rosout`、`/parameter_events`、`/diagnostics` | `rcl_interfaces/msg/Log`、`rcl_interfaces/msg/ParameterEvent`、`diagnostic_msgs/msg/DiagnosticArray` | 对应节点/生命周期管理器 | 本次无业务日志订阅 | 事件型；diagnostics≈2 Hz | R/V | 故障与配置变化 | 参数 dump 仅是时刻快照；事件和日志帮助定位发生时间，静默期间0条不等于断流。 |

**字段核查结果：**

- CustomMsg 样本 9984 点，逐点 offset_time 为 0–49,381,283 ns；实际20 Hz，不能按 `preprocess.scan_rate=10` 推断录制频率。有零点及多种 tag；`finite_xyz=9984` 只说明没有NaN，不代表9984个有效回波，预处理还做tag/距离过滤。
- MID360 IMU Header持续更新，frame为 `livox_frame`；加速度模长约0.98（驱动原样设备值），当前 `mapping.acc_norm=1.0`，Point-LIO 内部有重力单位缩放。不要在录制/回放层擅自乘9.81或补姿态；orientation/covariance也不能当成经过完整融合标定的输出。
- 串口 JointState含4个name与4个position，velocity/effort为空合法，frame_id为空合法；但两个 `_odom_joint` 在模型中是fixed。串口 stamp 是接收回调 `now()`，不是 MCU 硬件采样时间。
- `/cloud_registered`、`/registered_scan`、`/sensor_scan` 的XYZ样本均有限，分别约481/521/503点；`terrain_map`/`terrain_map_ext` 样本901/708点，intensity约0–0.50/0–0.64，符合源码使用相对高度的语义。
- LaserScan样本720束，193个有限距离、527个Inf、0个NaN；有效距离约0.547–7.931m，配置范围0.3–10m，Header约落后接收时刻0.056s；`time_increment=0`，`scan_time≈0.05`。Inf与 `use_inf=true` 一致，不应清洗成0。
- Odometry的位姿四元数范数正常；LIO与接口里程计的twist/协方差主要为0，源码未给出完整协方差意义。`/odometry` 的twist虽然有限，但另一个5秒探针实测线速度分量峰值约 `[32.61,22.20,29.28] m/s`，角速度分量峰值约 `[15.51,5.67,11.76] rad/s`。源码按**回调到达时的 steady_clock 间隔**差分，而非Header时间差，回调聚集可放大速度；这是解释线索，未做运动真值验证，不据此断言真实速度或独占根因。
- 地图样本63×21、分辨率0.05m，51自由/6占据/1266未知；4秒内8条map有更新Header但内容哈希相同。当前静止/门限未触发也可能这样，不能把2Hz地图发布当作持续接受新扫描或完整场景建图的证明。

### 按需录制

| Topic | Type | 本次状态与用途 |
|---|---|---|
| `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | ≈20 Hz，front_mid360；驱动xfer_format=4并发生成，约10080点/262080字节每帧，估计约5.2MB/s payload。保留x/y/z/intensity/tag/line/timestamp，但当前Point-LIO不订阅它；录它不能替代当前CustomMsg输入。 |
| `/sensor_scan` | `sensor_msgs/msg/PointCloud2` | ≈19Hz，front_mid360，当前无消费者；排查传感器坐标转换/可视化时有用，当前二维主链不依赖它。 |
| `/cloud_registered_body`、`/cloud_effected`、`/Laser_map` | `sensor_msgs/msg/PointCloud2` | 图上有publisher，各2秒抽样0条；body发布开关关闭，其他输出受实现路径控制。不能认为已存到有效debug点云。 |
| `/path` | `nav_msgs/msg/Path` | 图上有publisher，3秒0条；`publish.path_en=false`。 |
| `/slam_toolbox/graph_visualization`、`/slam_toolbox/scan_visualization`、`/slam_toolbox/update` | `visualization_msgs/msg/MarkerArray`、`sensor_msgs/msg/LaserScan`、`visualization_msgs/msg/InteractiveMarkerUpdate` | 运行graph存在；未额外订阅验证频率，按调试需求收。不能代替可恢复的序列化state。 |
| `/pose`、`/map_metadata`、`/robot_description` | `geometry_msgs/msg/PoseWithCovarianceStamped`、`nav_msgs/msg/MapMetaData`、`std_msgs/msg/String` | graph存在，未单独抽样；模型已直接从运行参数完整归档。 |
| `/joy`、`/map_clearing`、`/cloud_clearing` | `sensor_msgs/msg/Joy`、`std_msgs/msg/Float32`、`std_msgs/msg/Float32` | 只有subscriber、当前0个publisher；没有把“topic在图上”写成“有消息”。源码中的按钮/清理命令会改变地形缓存，未来一旦有输入，若要精确重跑地形就必须记录；Float32没有Header，要保留bag时间和事件日志。 |
| `/cmd_vel`、`/cmd_gimbal_joint`、`/cmd_spin`、`/goal_pose`、Nav2的plan/costmap等 | 按graph类型 | 面向整车行为故障时补充，非纯SLAM传感器输入；命令应与反馈分开。没有完整回放验证前，回放进程应与真实串口控制隔离。 |

### 必须额外归档但不属于 rosbag

**本次已归档：**

1. **26个实际节点的全量参数快照**：[parameters/](parameters/)。从 `list_parameters/get_parameters` 得到当前有效值，已生成逐节点ROS参数YAML；未设置的null参数另列 [unset_parameters.json](parameters/unset_parameters.json)，避免生成不能直接加载的null值。少数无参数服务的TF监听辅助节点/nav2容器以及两个BT辅助节点未响应，不影响所列关键节点参数已取全的结论。
2. **当前进程真正读取的 launch 临时文件、源码、实际install/share配置、消息定义、模型来源、已有地图/PCD**：[source_config_assets.tar.gz](evidence/source_config_assets.tar.gz)（约202MB压缩包）。内部 manifest 有原始/解析路径、大小及SHA256。便于阅读的源码/配置副本已解到 [snapshot/](snapshot/)。压缩包还含历史/仿真PCD，它们被保全不代表当前被加载。
3. **运行中完整 robot_description**：[robot_description.urdf](assets/robot_description.urdf)，是参数服务读到的成品，而非只保存一个生成脚本。模型生成来源为 `robot_description/resource/xmacro/reality_robot.sdf.xmacro`，经 xmacro→SDF→sdformat_tools URDF。视觉网格与外部rmoss资源没有全部打包；运行TF可直接由已保存URDF重建，完整离线渲染还需原资源包。
4. **Git HEAD、原有dirty状态和binary diff、运行进程实参/环境、进程可执行文件哈希及已加载本地库路径**：[system_git_versions.json](evidence/system_git_versions.json)、[git_diff.patch](evidence/git_diff.patch)、[package_versions.tsv](evidence/package_versions.tsv)。自编译包没有仅靠Debian版本即可识别的二进制版本，所以同时保存当前源码、package.xml和进程/库信息；这不是完整容器镜像或可重装的.deb集合。
5. **当前ROS日志与前后Git状态**：[logs_and_git_diff.tar.gz](evidence/logs_and_git_diff.tar.gz)。日志只包含本次读取到的当前进程日志，不宣称是整台工控机的全部历史日志。
6. **当前地图与slam_toolbox状态**：通过只读 `dynamic_map` 导出 OccupancyGrid原始CDR及PGM/YAML，通过已存在的 `/slam_toolbox/serialize_map` 导出 `.posegraph`+`.data`，服务返回 `result=0`。文件在 [assets/codex_slam_runtime_snapshot_20261021_2254/](assets/codex_slam_runtime_snapshot_20261021_2254/)，打包件为 [slam_state_and_map.tar.gz](evidence/slam_state_and_map.tar.gz)。`.posegraph` 4,975,333字节、`.data` 19,298字节；哈希和导出时间见 [slam_state_export.json](evidence/slam_state_export.json)。只是导出，没有调用deserialize/reset/pause/clear，也没有覆盖工程地图。地图与图状态是相邻时间的两个调用，不是全系统原子checkpoint。序列化成功和文件存在已验证，**未另开节点做反序列化恢复测试**。

**必须保留的当前有效参数重点：** 全量值以YAML为准，下表列最容易丢失或被launch覆盖的部分。

| 节点 | 应随数据一同保留的参数与本次实际值 |
|---|---|
| Point-LIO | `common.lid_topic=livox/lidar`、`common.imu_topic=livox/imu`、`time_diff_lidar_to_imu=0`；`preprocess.lidar_type=1`、`scan_line=4`、`timestamp_unit=3`、`scan_rate=10`、`blind=0.5`；`point_filter_num=8`；`mapping.imu_en=true`、`use_imu_as_input=false`、`acc_norm=1.0`、IMU/LiDAR间隔/噪声/饱和阈值、重力与初始化值；`mapping.extrinsic_R=I`、`extrinsic_T=[-0.011,-0.02329,0.04412]`、`extrinsic_est_en=false`；体素/滤波/匹配参数；`publish.tf_send_en=false`、`odometry.publish_odometry_without_downsample=true`、`prop_at_freq_of_imu=true`；`prior_pcd.enable=false`；`pcd_save.pcd_save_en=true`、`interval=-1`。|
| loam_interface | `state_estimation_topic=aft_mapped_to_init`、`registered_scan_topic=cloud_registered`、`odom_frame=odom`、`base_frame=base_footprint`、`lidar_frame=front_mid360`；源码会在首次成功取到模型外参后缓存初始对齐，启动顺序与最初关节角也需保留。|
| sensor_scan_generation | `base_frame=base_footprint`、`lidar_frame=front_mid360`、`robot_base_frame=gimbal_yaw`；同步策略/队列及TF失败返回identity的行为在源码中，不能只存参数。|
| terrain_analysis | `scanVoxelSize=.05`、`decayTime=.5`、`minRelZ=-1.5`、`maxRelZ=.5`、`vehicleHeight=.5`、`quantileZ=.2`、`clearDyObs=true`、动态物体/无数据/衰减/刷新参数，以及运行期间清图事件。|
| terrain_analysis_ext | `scanVoxelSize=.1`、`decayTime=.2`、`lowerBoundZ=-1.5`、`upperBoundZ=.5`、`localTerrainMapRadius=4`、`vehicleHeight=1`、`checkTerrainConn=false`、`clearingDis=20` 等全量值。|
| pointcloud_to_laserscan | remap `cloud_in:=terrain_map_ext`、`scan:=obstacle_scan`；`target_frame=base_footprint`、`queue_size=16`、`transform_tolerance=.01`；height=[-.5,6]、intensity=[.1,2]、range=[.3,10]；angle≈[-π,π]、increment=.00872665、`scan_time=.05`、`use_inf=true`。|
| slam_toolbox | `mode=mapping`、frames map/odom/base_footprint、`scan_topic=obstacle_scan`、`resolution=.05`、`map_update_interval=.5`、`minimum_time_interval=.2`、`minimum_travel_distance=.1`、`minimum_travel_heading=.1`、`scan_queue_size=10`、`tf_buffer_duration=10`、`transform_timeout=.2`、量程[.3,10]；`use_scan_matching=false`、`do_loop_closing=false`、`transform_publish_period=0`；solver/全部匹配及回环阈值也应保留。|
| Livox驱动 | `xfer_format=4`、`publish_freq=20`、`multi_topic=0`、`data_src=0`、`frame_id=front_mid360`；实际MID360 JSON路径与内容、固件/设备标识/时钟模式。JSON里的设备外参为0，不等于机器人安装外参为0。|
| JSP/RSP/串口/静态TF | JSP `rate=10`、`source_list=[]`；RSP `publish_frequency=20`、`frame_prefix=''`、`ignore_timestamp=false`；串口端口by-id、波特率115200与协议/角度定义；map→odom静态单位变换。|

**不能只保存仓库nav2_params.yaml的直接证据：** Point-LIO进程额外传入的 `/tmp/launch_params_3u9ff1kd` 和 `/tmp/launch_params_schdg08m` 把prior关闭、PCD保存打开；它们已归档。JSP/RSP实际读到 `/tmp/launch_params__oqb0_0c`（内容是导航参数集合），不是预期的robot_description参数集合。仓库与install中的 `robot_description.yaml` 写着JSP `rate=200`、`source_list=[serial/gimbal_joint_state]`、RSP `publish_frequency=200`，实际却是10/空/20。仅复制这个YAML会得到与本次现场不同的系统。没有在本次调查中修改该启动传参问题。

**当前地图/PCD使用情况：** SLAM分支启动slam_toolbox并强制 `prior_pcd.enable=false`，路径为空；`map_file_name=''`，未见加载既有pose graph。外层launch虽仍有world/map/prior_pcd_file默认配置，不能据此说当前加载了它们。现有 `point_lio/PCD/scans.pcd` 大小110,078,269字节，mtime是2026-10-20 21:06:20（UTC+8），是历史产物，不是今晚运行中状态的证据。本次参数 `pcd_save.interval=-1`，源码只在进程正常退出后的保存分支把累积点云写到该文件；不能为了得到今晚PCD去停掉用户正在运行的SLAM，也不能把历史文件当成今晚新图。

**rosbag本身不能自动保存或保证：** 初始参数与所有服务请求/响应、launch替换结果、URDF/设备配置/固件/外参来源、Git源码差异、自编译二进制与容器依赖、系统时钟/PTP/NTP状态、slam_toolbox内部pose graph、Point-LIO滤波器/体素地图/初始化缓存及调度顺序。`/parameter_events` 不能补齐录制前的参数，`/map` 不能替代 `.posegraph/.data`，PCD也不是原始逐点时序或IMU历史。消息定义与对应type support尤其要保留CustomMsg/CustomPoint，否则只拿Humble标准包无法重建当前订阅类型。

### 当前存在的不确定项

1. 本次是若干2–5秒短窗口，不是长时间负载/运动实验，也不是各级严格同时采样；约642/648Hz等差别不能直接解释为跨级丢包。不能证明长时间无丢包、时间戳永不回跳或改算法后确定性重现。
2. 关节反馈断链已由graph/参数/数值对照确认；为什么launch传参落入导航YAML有强证据，但未修改/重新启动验证修复；实车真实机械方向、零位和外参精度仍需实体标定。串口orientation来源及硬件采样延迟也无法靠ROS Header独立还原。
3. IMU `livox_frame` 缺TF、原始加速度单位约为g、`/odometry`速度尖峰均已保留证据。它们对移动建图质量的影响尚未受控验证，不能宣称整链物理正确。
4. 远端端点发现没有提供可用history/depth：统一UNKNOWN/0。源码构造表明驱动共享队列256，Point-LIO SensorDataQoS默认深度5；LIO输出深度20，loam pub/sub5，sensor pub2/sub1，terrain pub2/sub5，转换scan SensorDataQoS深度5/input16，TF常用100/静态1。**这些深度是源码/参数推导，不是graph直接读出的有效队列承诺**；节点内还存在同步/消息过滤/处理队列。未来工具不能把0当成有效配置。
5. Generic transform_listener_impl节点没有在graph中声明“属于哪个业务节点”的父子关系，附录保留其真实节点名；关键TF的**发布者**已经由GID实测对应，没有靠猜测分配。SLAM/Point-LIO空TF端点在抽样窗口之外是否会因运行参数改变而发消息，需要事件日志持续记录。
6. 晚于Point-LIO启动才开始录制的原始数据，无法恢复它之前的初始化、局部地图和初始对齐；本次没有Point-LIO可恢复checkpoint接口。已导出的slam_toolbox state只恢复二维算法已有状态，不能恢复Point-LIO/terrain缓存或传感器历史。
7. 保全的地图很小、抽样map内容不变。没有安排机器人移动，不能证明新场景持续插入/回环质量；slam_toolbox保存state成功，但未验证反序列化和跨版本兼容。
8. 设备固件版本、实际PTP/GPS同步状态、传感器硬件时基与主机时钟的物理误差未完全获取。主机/容器时钟信息另存，不能只因为Header贴近接收时间就宣称硬件同步已校准。
9. 尚未运行rosbag录制吞吐/掉包/磁盘容量测试；未验证MCAP插件（当前Debian清单未见mcap）。未来应先按本机实际storage插件实施，不把一个未经测试的存储格式写成既成环境。

### 建议的 SLAM rosbag profile

**这里只列建议，不创建profile文件、不启动record。** 分层保存可以让“从原始重算”和“直接重放历史输出看故障”使用同一份证据，但两种播放方式必须区分。

当前建议默认清单（完整真实名称；包括原始输入、实际模型输入、关键级间输出与故障旁证）：

```text
/livox/lidar
/livox/imu
/serial/gimbal_joint_state
/serial/imu
/joint_states
/tf
/tf_static
/aft_mapped_to_init
/cloud_registered
/lidar_odometry
/registered_scan
/odometry
/terrain_map
/terrain_map_ext
/obstacle_scan
/map
/rosout
/parameter_events
/diagnostics
```

确实只做纯Point-LIO算法原始重算时，传感器最小清单是 `/livox/lidar`、`/livox/imu`，但必须配套本次模型/参数/软件版本；若重跑全部下游，还要恢复有效关节状态和静态变换。默认清单保留串口两路是为比较机械反馈与当前错误/默认模型输入，而不是宣称它们已被当前算法消费。

可选追加：

```text
/sensor_scan
/livox/lidar/pointcloud
/pose
/map_metadata
/robot_description
/slam_toolbox/graph_visualization
/slam_toolbox/scan_visualization
/slam_toolbox/update
```

事件输入按实际任务启用：`/joy`、`/map_clearing`、`/cloud_clearing`。当前无publisher，不虚构有效数据；之后若接入清图/手柄操作，就必须保留这些改变算法状态的外部事件。`/cloud_registered_body`、`/cloud_effected`、`/Laser_map`、`/path` 只有在确认开关/实现路径已发布有效消息后再加。

录制端QoS建议：在实车容器/工控机本地写盘；`/tf_static` 采用RELIABLE+TRANSIENT_LOCAL，保留所有writer快照；`/map` 若要录制器晚启动仍取得已有图，也选RELIABLE+TRANSIENT_LOCAL；`/obstacle_scan` 是BEST_EFFORT writer，不能请求RELIABLE订阅。原始雷达/IMU可先按当前算法相同的BEST_EFFORT/VOLATILE读端做短时吞吐验证，以减少额外可靠读端阻塞驱动的风险，但BEST_EFFORT不保证无丢包；需要用Header/CustomMsg timebase与offset连续性、消息计数和录制性能证据验收，不能把QoS选项当作完整性证明。无需为了这些清单再跑 `record -a`。

重跑全链：仅把原始测量和所需外部输入送给新算法；由新RSP/LIO/sensor_scan/slam节点重算自己的输出，隔离旧 `/tf`、`/map`、Odometry和点云输出，避免双发布。若只重跑slam_toolbox，可用已录 `/obstacle_scan` 与当时所需 TF，停用旧SLAM输出；若只重跑terrain，可用 `/registered_scan`、`/lidar_odometry` 及清理事件。按需要生成 `/clock` 并让重跑节点 `use_sim_time=true`，保留原Header；不得把包含过去命令的bag直接回放进仍连着实车串口的ROS域。Point-LIO与terrain等源码里存在墙钟/接收顺序相关行为，固定Header并不保证完全位级复现。

归档完整性：见 [SHA256SUMS.txt](SHA256SUMS.txt)。完整归档现已复制并校验到本地工程目录 `/home/tungchiahui/UserFolder/MySource/ysu2026_rm_sentry/ysu2026_rm_sentry_nav/slam_audit_20261021_2248/`。工控机上的原副本保留。本目录是调查资料，不是功能包；工程代码及 synced sources 只读引用区没有被改动。
