# ly_lio

**后续修改先查[未采用方案记录](docs/验证.md#12-未采用方案与复试条件)。没有新的故障证据、模型机制
及同源/留出验证，不得重复合入原失败方案。**

文档于2026-10-19整合，`docs`仅保留[ly建模.md](docs/ly建模.md)和[验证.md](docs/验证.md)；旧模型、调试报告与JSON已归入验证文档。

基于 Small Point-LIO / Point-LIO 的独立 ROS 2 激光惯性里程计。按2026-10-17用户要求，
状态及误差协方差为真正的 **24维 `SO(3)×R^21`**，代码顺序为
`[p,R,v,omega,a,g,bg,ba]`。IMU角速度和比力是状态，通过六维IMU输出观测更新；
饱和轴按95%量程单独屏蔽。固定LiDAR–IMU外参不进入状态，不融合轮速。

MID360使用 **1ms Batch**（`lidar_batch_duration: 0.001`），遇IMU或扫描末端提前结束，
同stamp先IMU。每批原始点补偿到真实批末，每点独立查询历史地图5近邻PCA平面，
计算标量点到面残差及H；同一批冻结先验，合并所有合格点的信息后只执行一次联合更新。
H直接包含`p/R/v/omega`12列，经完整24维P更新全部状态；独立各向同性点噪声使合格点
采用相同的单点测量方差。后验注册云与地图点按同一补偿模型重新计算，整扫描结束才
写入历史地图。没有合格匹配时继续预测和可用IMU更新。

模型、外参公式、原PCA门控与限制见[ly建模.md：当前模型与详细公式](docs/ly建模.md)。
目标历史地图仍视为固定参考；尚未建模持久地图误差及状态—地图相关性。
本轮保留逐点残差的Batch更新，不引入平面对平面观测，不恢复失败的相邻10ms窗口/
Schmidt、连续初始化、支持点对角方差或共享支持点GLS实验。
保留当前噪声/采样/地图数值，MID360速度噪声率仍为0.1 m²/s³；此前精度数字均为历史配置结果。

参数修改[config/mid360.yaml](config/mid360.yaml)，由节点直接读取，重启生效；
显式ROS参数可覆盖。配置位置见[建模与配置说明](docs/ly建模.md)。
云台中心Odometry由真实Batch后验按odometry_publish_rate（默认100Hz、有限正数）选择输出，以扫描末为锚反向分槽；扫描末必发、同stamp去重，稀疏数据不补造样本。publish_odometry_per_batch=true发布全部真实批末状态。注册云仍按扫描末发布且在baselink中。LIO不再广播odom→baselink动态TF，保留baselink→livox_frame静态安装TF。整帧等IMU覆盖后集中处理发送，100Hz是传感器时间目标，不等于墙钟每10ms实时输出；不新增传播/预测，不用now重盖旧状态。
ScanSession随每个已选Odometry同stamp发布（reliable/transient_local/depth1000），覆盖全部扫描末云；header.frame_id仍为odom，表示会话固定参考系。reset/重建/重启换UUID。后端按完整纳秒stamp关联标签和T_OB，不插值/外推，不以到达顺序猜测；复用stamp跨会话或位姿冲突时保守拒绝。
标准PointCloud2模式不依赖PCL或IKFoM。本轮验证结果见[24维Batch与固定外参验证](docs/验证.md)。

默认直接订阅 `/livox/lidar` 和 `/livox/imu`，状态原点为 IMU，测量轴为
`imu_frame=livox_frame`仍匹配原始驱动的测量轴标签。MID360的`extrinsic_T`保存激光系下
IMU芯片位置`[0.011,0.02329,-0.04412]`m，按显式约定换算为内部lidar→IMU平移。
安装外参用于输出 baselink 位姿和扫描末 baselink 注册云。独立 `pcl_publisher` 已删除，不再发布 `/bashlink/pcl` 与 `/baselink/imu`。发布：

| 接口 | 类型 / 作用 |
| --- | --- |
| `/ly_lio/odometry` | nav_msgs/Odometry，含机体系速度与协方差 |
| `/ly_lio/cloud_registered` | PointCloud2，reliable/depth10，逐批后验补偿并统一转到扫描末 baselink 的 XYZ 点云 |
| `/ly_lio/path` | nav_msgs/Path，有界轨迹 |
| `/tf` | LIO不创建动态发布者；建图链路由后端发布map → baselink |
| `/tf_static` | baselink → livox_frame，固定安装关系 |
| `/ly_lio/map_save` | Trigger，保存累积地图，需启用 save_pcd |
| `/ly_lio/reset` | Trigger，清空并重新初始化 |

```bash
source /opt/ros/humble/setup.bash
cd /home/hustlyrm/ly_odom/odom_ws
colcon build --packages-select ly_lio ly_bag_recorder --parallel-workers 1 --symlink-install
source install/setup.bash
ros2 run ly_lio ly_lio_node
```

也可在仓库根目录执行 `./scripts/odom.sh`，统一启动MID360驱动、里程计和独立录包节点。需要同时回环建图时改用 `./scripts/map.sh`，两者不要叠加运行。launch统一位于scripts/launch，共用加载逻辑位于scripts/lib；唯一Recorder继续使用ly_bag_recorder包内配置。
各节点代码直接读取所属配置，安装外参位于`odom_ws/src/ly_lio/config/lidar2baselink.yaml`。
PTP由systemd开机自启，驱动仅作只读检查；Ctrl+C仅关闭ROS节点。
默认同时将当前 ROS 域全部话题录制到仓库 `bags/odom_日期_时间_随机后缀/`，包括
原始输入、里程计、注册点云、Path 与 TF。修改录包包内`odom_ws/src/ly_bag_recorder/config/rosbag.yaml`更改录制与QoS选项，
`record_bag:=false`关闭录包。Ctrl+C 后等待 rosbag 写盘与 metadata 保存完成。
回放原始 rosbag 或使用外部驱动时用 `./scripts/odom.sh start_lidar:=false`，跳过驱动
及 PTP 检查；普通回放可再加 `record_bag:=false`。去畸变与机体系注册云发布均在里程计节点内完成。
日常算法配置位于包内`config/mid360.yaml`，直接运行节点和统一启动使用相同默认配置。
默认静止收集
200 个 IMU 样本，`acc_norm=1.0` 对应 Livox 原始加速度单位 g。
初始化同时估计重力方向、陀螺零偏与加速度模长零偏，运动窗口等待后续静止。
门限使用 SI 单位；单姿态不能完成加速度计全轴标定，启动时仍须保持设备静止。
输入加速度为 m/s² 时改为 `acc_norm=9.81` 并核实饱和阈值。
激光与 IMU 观测前同步传播协方差；过程噪声按 `Q*process_noise_reference_dt*dt`
积分，参考周期默认 0.005 秒。可选的 `velocity_process_noise_rate` 单独规定速度
协方差增长率（m²/s³），其速度块增量为 `rate*dt`；要求有限且非负，代码默认0，
不改变其他噪声的积分方式。修改YAML或用ROS参数覆盖后重启生效；旧 `velocity_cov`
仍不生效，不自动换算。Batch继续使用全部通过现有预处理和观测门控的候选点。
参数说明见[现行建模与配置约定](docs/ly建模.md)。此前多点地图修复的历史录包对比见
[PLIO对比与零漂修复](docs/验证.md)。

时间戳检查、IMU短长周期、逐点更新与TF输出策略见 [时间戳与发布频率](docs/验证.md)。

本包按用户要求移除测试代码及测试构建依赖。构建后可检查实机话题：

```bash
ros2 topic info /ly_lio/odometry
ros2 topic hz /ly_lio/odometry
ros2 topic echo /ly_lio/odometry --once
```

静止零漂、真实数据集精度、极端运动和速度倍数需用实机或同步 rosbag 评估。
此前24维独立实机预热5秒后连续测量600秒：首末5秒均值位置差0.6215cm、
最大1.1963cm、RMS0.5098cm，末端姿态变化0.021623°。15组调参后的候选跨场景退化，
当轮保留原数值参数；这些旧配置数值不作为现行24维/1ms/非零固定外参的实机验收。降维与原固定外参模式数学等价，详见[10分钟验证](docs/验证.md)。
此前Batch独立实机60秒末端0.309cm、最大2.246cm；两份同源对比没有证明全面降低零漂，
详见[模型改进验证](docs/验证.md)。
修改前逐点模型同源静止60秒末端0.92cm、独立实机0.91cm；历史口径与限制见
[双雷达与单雷达对比](docs/验证.md)。
Point-LIO 的备用 24 维 IMU 输入滤波路径不在本包中。
来源及许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## IMU 状态与机体输出

内部外参方向为`p_I=R_IL*p_L+t_IL`。`extrinsic_R`始终为R_IL；
`extrinsic_translation_convention=imu_in_lidar`时，`extrinsic_T`是IMU原点在激光系的位置c_L，
读取时`t_IL=-R_IL*c_L`；`lidar_in_imu`时直接使用原Point-LIO的t_IL。
安装文件的
`lidar_to_gimbal_rotation/translation` 当前作为 T_BL（lidar→baselink）读取。
节点推导 T_BI = T_BL × inverse(T_IL)，发布 T_WB = T_WI × inverse(T_BI)。
固定T_IL/T_BL在启动时组合一次，重建不改变配置标定。
也可用 `imu_to_body_R`（行优先9元数组）、`imu_to_body_T`（3元数组）直接给出固定T_BI。
旧`extrinsic_est_en=true`和外参先验参数会明确拒绝启动；旧false仅告警兼容。
两种安装外参来源互斥。IMU 与输出帧不同却未配置安装外参时，启动明确报错。

基座原点在 IMU 中为 r_IB，世界系基座速度为 v_WB = v_WI + R_WI(omega_I × r_IB)。
位姿与 twist 协方差由完整24维P及输出雅可比计算，包括杆臂和姿态关联项。
内部匹配地图和可选LIO累计导出保持odom坐标；注册云在扫描末baselink，Path记录odom中云台中心位姿。
默认 odom 原点仍为初始化 IMU 原点，故 baselink 的初始位置可非零，朝向可非单位旋转。

固定 TF 使用 `publish_sensor_tf` 控制；已有机器人状态发布器管理该安装关系时设为 false，
避免多个发布者。此节点发布的是baselink→lidar_frame，不能把非零偏移的IMU原点
标成激光原点。固定安装关系发布为静态TF。
如需独立真实IMU坐标系，可另行发布不同名称的TF，原始驱动frame_id无需改名。

现有安装文件键名指向 gimbal，项目仍按该目标就是 baselink 的约定使用；若实际是活动云台，
应另外建立 gimbal_link 并使用实时云台角度，固定安装模型仅适用于刚性连接。
同一原始帧名并不证明真实雷达与 IMU 原点完全重合，已知内部偏移应填入 extrinsic_R/T。

## 终端状态面板

默认 `./scripts/odom.sh` 显示与 sentry PLIO 相同的终端边框、`Point-LIO Odometry` 标题、
字段顺序、空行和数值精度，每秒刷新10次。通过独立墙钟定时器持续刷新，断流时也显示当前状态；
刷新只读取快照，不改变滤波和话题时间戳。面板布局依次为日期和运行时间、输入/处理频率、
点云计数、健康状态、机体运动状态、距离、配准/时间、耗时和进程资源。

```bash
ros2 run ly_lio ly_lio_node --ros-args -p terminal.enabled:=false
# 日常统一启动时修改config/mid360.yaml中的terminal.*参数。
```

默认只在交互TTY清屏，文件和管道自动输出普通文本；不清屏模式保留同一终端的驱动、录包日志。
统一启动直接读取包内YAML的终端参数。直接 `ros2 run` 使用ROS参数 `terminal.enabled`、`terminal.clear_screen`、
`terminal.refresh_rate_hz`，刷新率允许(0,10]Hz，默认10Hz；`terminal.stale_timeout`默认1秒。
这些参数仅在启动时读取，关闭面板后不创建打印定时器。

面板中的 `{W}` 表示固定的初始机体显示坐标系 `baselink_initial`（记作B0），`{B}` 表示
当前 `baselink`。B0由首个有效发布的完整机体位姿确定：初始位置为原点，初始姿态确定轴向，
之后不随小车运动。设原始输出为 `T_WB`、首次有效输出为 `T_WB0`，终端显示
`T_B0B = inverse(T_WB0) * T_WB`；因此该首次位姿显示零位置和单位四元数（wxyz），后续显示
相对于启动机体的平移和旋转。这是终端内部的显示变换，不另发B0 TF；ROS Odometry继续输出
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
  采样频率，不是加速回放时的墙钟吞吐。odom显示真实位姿stamp统计的发布频率。
- Clouds recv/accept/proc分别为适配后进入核心的扫描数、成功入队数、完成跟踪处理数。
  不含适配器提前拒绝的非法布局/坐标系消息；接受后被裁剪或过期不撤销accept，初始化扫描
  不计入proc。因此三项差值不能直接当作网络丢包数。
- Cloud reject/pending沿用参考标签，reject为核心拒绝数，pending实际表示累计缓冲溢出
  丢帧数，不是当前队列长度。DDS message drops显示 `n/a`，表示尚未计量DDS丢包。
- Input是适配后、核心过滤前的点数，tracked是实际参与激光关联的点数，Registration matches
  是成功进入联合更新的匹配点数；Point Time相对当前输出时刻，IMU lead是最新IMU到扫描末点的时间差。
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
