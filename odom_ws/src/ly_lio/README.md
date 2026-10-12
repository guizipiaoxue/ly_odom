# ly_lio

基于 Small Point-LIO / Point-LIO 的独立 ROS 2 激光惯性里程计。当前恢复
**30 维 IMU 输出观测模型与逐点 point→map 激光更新**，关键算法含中文注释。
之前的相邻 10 ms 平面匹配方案在静止实机中发散，已退出业务流程。

静止 IMU 标定完成后收集初始地图；跟踪时按照每个点的真实时间预测状态，交织 IMU
观测，并用历史地图的局部 PCA 平面修正当前位置和姿态。采用多点体素 LRU 地图、
保留真实坐标/时间对的空间降采样、IMU 饱和轴屏蔽和完整协方差传播。
同一扫描的后验点在扫描结束后统一入图，避免用本帧点给自己提供历史约束。
默认每帧末点发布一次 Odometry/TF，更新频率与发布频率分别控制。
标准 PointCloud2 模式不依赖 PCL 或 IKFoM。

默认直接订阅 `/livox/lidar` 和 `/livox/imu`，状态原点为 IMU，测量轴为
`imu_frame=livox_frame`。`extrinsic_R/T` 只描述 lidar→IMU，当前 MID360 沿用雷达与
IMU 同源参考系的单位外参假设。安装外参只用于输出 baselink 位姿，保留
`pcl_publisher` 发布的 `/bashlink/pcl` 与 `/baselink/imu` 给其他使用者。发布：

| 接口 | 类型 / 作用 |
| --- | --- |
| `/ly_lio/odometry` | nav_msgs/Odometry，含机体系速度与协方差 |
| `/ly_lio/cloud_registered` | PointCloud2，逐点补偿后的 odom 点云 |
| `/ly_lio/path` | nav_msgs/Path，有界轨迹 |
| `/tf` | odom → baselink，机体原点位姿 |
| `/tf_static` | baselink → livox_frame，固定安装关系 |
| `/ly_lio/map_save` | Trigger，保存累积地图，需启用 save_pcd |
| `/ly_lio/reset` | Trigger，清空并重新初始化 |

```bash
source /opt/ros/humble/setup.bash
cd /home/hustlyrm/ly_odom/odom_ws
colcon build --packages-select ly_lio --parallel-workers 1 --symlink-install
source install/setup.bash
ros2 launch ly_lio ly_lio.launch.py \
  body_config_path:=/home/hustlyrm/ly_odom/pcl_ws/config/lidar2baselink.yaml
```

也可在仓库根目录执行 `./scripts/odom.sh`，统一启动 MID360 驱动、转换节点和里程计，读取同一份
源码安装外参。PTP 由 systemd 开机自启，启动入口只检查其状态；Ctrl+C 仅关闭 ROS 节点。
默认同时将当前 ROS 域全部话题录制到仓库 `bags/odom_日期_时间_随机后缀/`，包括
原始输入、转换输出、里程计、注册点云、Path 与 TF。用 `bag_dir:=/目录` 更改录包父目录，
`record_bag:=false` 关闭录包。Ctrl+C 后等待 rosbag 写盘与 metadata 保存完成。
回放原始 rosbag 或使用外部驱动时用 `./scripts/odom.sh start_lidar:=false`，跳过驱动
及 PTP 检查；普通回放可再加 `record_bag:=false`。里程计不依赖转换节点的输出。
配置位于 `config/mid360.yaml`；默认静止收集
200 个 IMU 样本，`acc_norm=1.0` 对应 Livox 原始加速度单位 g。
初始化同时估计重力方向、陀螺零偏与加速度模长零偏，运动窗口等待后续静止。
门限使用 SI 单位；单姿态不能完成加速度计全轴标定，启动时仍须保持设备静止。
输入加速度为 m/s² 时改为 `acc_norm=9.81` 并核实饱和阈值。
激光与 IMU 观测前同步传播协方差；过程噪声按 `Q*process_noise_reference_dt*dt`
积分，参考周期默认 0.005 秒。此前多点地图修复的历史录包对比见
[PLIO对比与零漂修复](docs/PLIO对比与零漂修复.md)。

时间戳检查、IMU短长周期、逐点更新与TF输出策略见 [时间戳与发布频率](docs/时间戳与发布频率.md)。

本包按用户要求移除测试代码及测试构建依赖。构建后可检查实机话题：

```bash
ros2 topic info /ly_lio/odometry
ros2 topic hz /ly_lio/odometry
ros2 topic echo /ly_lio/odometry --once
```

静止零漂、真实数据集精度、极端运动和速度倍数需用实机或同步 rosbag 评估。
本轮同源静止60秒末端0.92cm、默认启动独立实机0.91cm；完整口径与限制见
[双雷达与单雷达对比](docs/双雷达与单雷达对比.md)。
Point-LIO 的备用 24 维 IMU 输入滤波路径不在本包中。
来源及许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## IMU 状态与机体输出

外参方向：`extrinsic_R/T` 为 T_IL（lidar→IMU），安装文件的
`lidar_to_gimbal_rotation/translation` 当前作为 T_BL（lidar→baselink）读取。
节点推导 T_BI = T_BL × inverse(T_IL)，发布 T_WB = T_WI × inverse(T_BI)。
默认固定 lidar→IMU 外参；也可用 `imu_to_body_R`（行优先
9 元数组）、`imu_to_body_T`（3 元数组）直接给出 T_BI。使用安装文件推导 IMU→机体时必须固定激光外参；在线外参需直接给定 IMU→机体安装关系。
两种安装外参来源互斥。IMU 与输出帧不同却未配置安装外参时，启动明确报错。

基座原点在 IMU 中为 r_IB，世界系基座速度为 v_WB = v_WI + R_WI(omega_I × r_IB)。
位姿与 twist 协方差由完整 30 维 P 及输出雅可比计算，包括杆臂和姿态关联项。
地图、注册点云保持滤波器建立的 odom 坐标，Path 则记录输出 baselink 位姿。
默认 odom 原点仍为初始化 IMU 原点，故 baselink 的初始位置可非零，朝向可非单位旋转。

固定 TF 使用 `publish_sensor_tf` 控制；已有机器人状态发布器管理该安装关系时设为 false，
避免多个发布者。雷达与 IMU 使用不同帧名时，此节点只自动发布 baselink→imu_frame，
另行提供 IMU→lidar 的 TF；两者的标定仍由 extrinsic_R/T 供滤波使用。

现有安装文件键名指向 gimbal，项目仍按该目标就是 baselink 的约定使用；若实际是活动云台，
应另外建立 gimbal_link 并使用实时云台角度，固定安装模型仅适用于刚性连接。
同一原始帧名并不证明真实雷达与 IMU 原点完全重合，已知内部偏移应填入 extrinsic_R/T。

## 终端状态面板

默认 `./scripts/odom.sh` 显示与 sentry PLIO 相同的终端边框、`Point-LIO Odometry` 标题、
字段顺序、空行和数值精度，每秒刷新10次。通过独立墙钟定时器持续刷新，断流时也显示当前状态；
刷新只读取快照，不改变滤波和话题时间戳。面板布局依次为日期和运行时间、输入/处理频率、
点云计数、健康状态、机体运动状态、距离、配准/时间、耗时和进程资源。

```bash
./scripts/odom.sh terminal_enabled:=false
./scripts/odom.sh terminal_clear_screen:=false terminal_rate:=1.0
```

默认只在交互TTY清屏，文件和管道自动输出普通文本；不清屏模式保留同一终端的驱动、录包日志。
统一脚本和独立launch均支持 `terminal_enabled`、`terminal_clear_screen`、`terminal_rate`，
留空沿用YAML。直接 `ros2 run` 使用ROS参数 `terminal.enabled`、`terminal.clear_screen`、
`terminal.refresh_rate_hz`，刷新率允许(0,10]Hz，默认10Hz；`terminal.stale_timeout`默认1秒。
这些参数仅在启动时读取，关闭面板后不创建打印定时器。

面板中的 `{W}` 表示固定的初始机体显示坐标系 `baselink_initial`（记作B0），`{B}` 表示
当前 `baselink`。B0由首个有效发布的完整机体位姿确定：初始位置为原点，初始姿态确定轴向，
之后不随小车运动。设原始输出为 `T_WB`、首次有效输出为 `T_WB0`，终端显示
`T_B0B = inverse(T_WB0) * T_WB`；因此该首次位姿显示零位置和单位四元数（wxyz），后续显示
相对于启动机体的平移和旋转。这是终端内部的显示变换，不另发B0 TF；ROS话题和TF继续输出
原有 `odom` 下的位姿，时间戳也沿用实际输出。

`Position {W}` 和 `Orientation {W}` 对应当前 baselink 在B0中的位姿。`Lin Velocity {W}`
和 `Gravity {W}` 同时用首次姿态的逆旋转变换到B0，不能只改位置或四元数而混用不同坐标轴。
`Lin Velocity {B}`、`Ang Velocity {B}` 仍在当前baselink轴下表达；线速度始终对应已补偿
杆臂的baselink原点。`Accel Bias`、`Gyro Bias` 仍是原IMU偏置经 `R_BI` 旋转后的机体系
分量，不受B0显示变换影响，也不代表基座原点的加速度。上述物理量在同一次实际里程计发布
时采集，时间一致；尚未建立有效参考时显示 `n/a`。

B0在首次有效发布时建立，终端第一次刷新可能稍晚，届时已发生的运动会正常显示，不强制
第一屏归零。显式reset或自动重建清除旧B0，下次有效发布重新确定显示原点和初始姿态。

`Distance Traveled` 沿用参考PLIO的10cm门限：当前 baselink 与上一个累计锚点的距离达到
10cm才计入，并移动锚点，不能作为精确弧长。`Distance to Origin` 是相对于B0原点的距离，
等于终端显示位置的范数。日期取输出传感器时间按本机时区显示，Elapsed也是传感器经过时间。

状态：WAIT_LIDAR/WAIT_IMU等待有效输入，CALIBRATING等待静止IMU标定，BUILDING_MAP
收集静止初图；正常处理前5帧显示WARMUP，之后显示GOOD。CHECK提示非有限状态或低匹配率，
GOOD不保证定位精度。有效时间戳超过 `terminal.stale_timeout` 不再推进时显示STALE，重复
或回退消息不能维持GOOD；WAIT_OUTPUT表示输入有效但暂未输出新扫描位姿。断流继续刷新，
保留的位姿仅是最后一次结果。

- Sensor Rates和Processing Rate按递增传感器时间戳的最近窗口计算；后者是已完成扫描的
  采样频率，不是加速回放时的墙钟吞吐。odom/TF显示实际发布频率，两者不同时分别显示。
- Clouds recv/accept/proc分别为适配后进入核心的扫描数、成功入队数、完成跟踪处理数。
  不含适配器提前拒绝的非法布局/坐标系消息；接受后被裁剪或过期不撤销accept，初始化扫描
  不计入proc。因此三项差值不能直接当作网络丢包数。
- Cloud reject/pending沿用参考标签，reject为核心拒绝数，pending实际表示累计缓冲溢出
  丢帧数，不是当前队列长度。DDS message drops显示 `n/a`，表示尚未计量DDS丢包。
- Input是适配后、核心过滤前的点数，tracked是实际调用激光更新次数，Registration matches
  是激光更新返回成功次数；Point Time相对当前输出时刻，IMU lead是最新IMU到扫描末点的时间差。
- Computation Time包含核心扫描处理、地图插入和同步输出，不含适配/预处理；Avg/Max按当前
  初始化周期统计。CPU Load沿用参考PLIO定义，以全机逻辑核总容量为100%，Cores Utilized
  给等效占用核数；CPU Avg为最近100次刷新均值。RAM Allocation是进程驻留内存，MB按十进制
  计算，组件模式包含整个容器进程。
- 终端IO在独立回调组执行，复制快照后即释放锁，不在逐点循环中打印。显式reset清核心计数和
  B0参考；自动重建保留核心累计计数，清上一轨迹显示、B0、距离和本周期耗时。

此前绝对坐标面板验证：Release构建通过；101次面板的机体位姿/twist与原ROS输出在显示
精度内一致，85组Odom/TF完全一致，断流/reset及终端输出模式检查通过。引入B0显示变换后，
合成验证覆盖非零初始位姿、初始姿态旋转、世界速度/重力旋转、非法首帧和reset重锚点；
已有实机录包回放显示首个有效baselink建立B0后，Position/Orientation以B0表达且保持
ROS Odom/TF原始语义，Sensor Rates约10/200Hz、odom/TF约10Hz、节点正常退出。
回放模式在Livox已安装时加载可选消息库环境，避免新终端找不到CustomMsg运行库；不启动雷达。
