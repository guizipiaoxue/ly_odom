# ly_lio 维护说明

- 本包是独立 ROS 2 Humble 包；Point-LIO、small_point_lio及外部sentry PLIO为只读参考。
  不从兄弟目录动态引用源码，保留MIT和第三方版权。关键算法、适配和节点逻辑使用中文注释。
- 首读README、docs/实现对照.md、docs/时间戳与发布频率.md和docs/双雷达与单雷达对比.md。
- **当前恢复30维Point-LIO输出观测模型与逐点 point→map 激光更新。**
  相邻10ms平面→平面模型已因静止实机严重发散停用，plane_features/Schmidt克隆不再参与流程。
  旧失败和历史结果见docs/PLIO对比与零漂修复.md，不能宣称它们是当前验证。
- 按真实时间处理IMU、跟踪点、稠密点、scan_end。每次观测与输出前同步传播P；
  Q*process_noise_reference_dt*dt默认参考周期.005s，稀疏FPFᵀ保留完整交叉块。
- 激光用历史地图5近邻PCA平面、点到面标量残差、位置/右扰动姿态雅可比。
  同扫描后验点到scan_end才入图；每格默认20点、5cm最小间距、7格查询与LRU。
  空间降采样选真实坐标/时间对，不对运动中不同时间点的坐标和时间做均值。
- 静止IMU窗口估计bg、ba模长偏差和重力方向；清掉标定前扫描后收集至少5帧初图。
  建初图期间继续核对静止，运动则重来。不能保证把极慢运动识别出来；无持续ZUPT或位姿锁定。
- 连续实际IMU时间缺段超过max_imu_gap=.1s或队列溢出，明确告警并重标定/重建odom。
  节点同步清理旧Path和导出地图；这会重设odom原点，下游必须处理重置，不可当连续轨迹。
- 默认订阅/livox/lidar、/livox/imu，在IMU原点滤波；保留转换节点/bashlink/pcl、/baselink/imu。
  两节点IMU订阅保留best effort，队列depth=1000，避免点云回调期间挤掉200Hz IMU。
  本项目Livox加速度单位g，acc_norm=1；sentry驱动已输出SI，接入时acc_norm=9.80665且
  gravity模长相同，satu_acc须同步改约29.42m/s²。不能只改比例却把重力轴误判饱和。
- Livox PC2 timestamp是header相对纳秒；CustomMsg timebase为绝对UTC纳秒。
  严格PTP未锁定就丢包，两路统一TAI减37秒。不能用now()、收包时间或均匀5ms网格替换。
  PTP不等于同时采样；静止数据不能标定运动时的物理LiDAR–IMU时延。
- 默认固定T_IL（lidar→IMU）。统一安装文件给T_BL，推导T_BI=T_BL*inverse(T_IL)，
  T_WB=T_WI*inverse(T_BI)。在线外参需直接给固定imu_to_body_R/T，不能从在线T_IL推导固定机体关系。
  位置、速度、协方差、Path、TF共同核对杆臂；注册点云仍在odom，不重复变换。
- 默认scan_end发布Odom/TF/注册点云，stamp为距离过滤前最晚有效输入点。
  publish_odometry_without_downsample=true可逐跟踪点发布Odom/TF，仍按原始消息成批交付。
  当前无独立实时IMU预测输出，不把约10Hz发布误写为10Hz激光更新。
- 原始frame名相同不能证明lidar/IMU原点重合；内部平移目前缺少设备标定，不能凭其他型号
  示例正负号盲填。活动云台真实底盘姿态需要动态关节外参；安装文件当前对应云台中心。
- 算法和reset互斥回调组，save服务独立组使用地图快照；没有回环/绝对位置约束。
- 统一启动scripts/odom.sh：PTP开机服务仅检查，驱动/转换/里程计/全话题录包一起启动。
  回放用start_lidar:=false record_bag:=false。PTP不随ROS停止或重启。
- 构建：source Humble；可选source Livox启用CustomMsg，再在odom_ws运行
  colcon build --packages-select ly_lio --symlink-install --parallel-workers 1。
  不构建ROS1参考包；禁用-fast-math，默认Release，可选LY_LIO_NATIVE_OPTIMIZATION。
- 用户要求不保留新增测试/回放/监视/分析程序；临时程序在/tmp或stdin运行并清理，
  只保留原始bag、配置、CSV、JSON、图、中文报告。历史测试数字不是本轮验收。

## 本轮验证

- LY Release与转换节点构建通过；同源60秒LY末端0.919cm、最大1.446cm。
- 默认真实.3启动60秒末端0.906cm、最大2.428cm，姿态末端0.191°。
- 转换IMU队列从5加到1000后，同源重放12000条IMU和599帧点云全部转换，时间与顺序一致；
  599组Odom/TF/注册云stamp相等、TF值差0，LY末端0.906cm/最大2.401cm，无发散。
- 驱动析构仍有已知SIGBUS(-7)，里程计/转换/录包正常退出；PTP active且PID不变。
- 静止之外的运动时间偏移/内部外参/精度仍未验收，不用历史测试数量代替本轮结果。


## PLIO风格终端打印

- 默认启用 `terminal.enabled=true`，10Hz独立wall timer、独立互斥回调组读取加锁快照。
  边框、Point-LIO Odometry标题、字段顺序、空行和数值精度与sentry PLIO一致；不改滤波或话题stamp。
- 面板 `{W}`=固定初始机体显示系baselink_initial（B0），`{B}`=当前baselink。首个有效发布的
  完整body位姿T0确定B0原点与轴向，显示T_rel=T0^-1*T；首次有效位姿为零位置/单位四元数。
  仅终端内部变换，不发布B0 TF；ROS话题和TF仍保持原odom输出，stamp不改。
- Position/Orientation来自已补偿安装杆臂的body_odometry，再转为B0表达；世界线速度和重力
  同时按初始姿态逆旋转。当前机体系线速度/角速度及bg/ba表达不变；bg/ba是R_BI旋转后的
  IMU偏置，不代表基座原点加速度。全部物理量在同一次发布取快照，不混用坐标系或时刻。
- Distance Traveled沿用10cm累计门限，不代表精确弧长；Distance to Origin是B0原点距离，
  等于显示位置范数。无有效参考时物理量为n/a。首次刷新可能晚于首个发布，不人为强制第一屏归零。
  日期按本机时区转换传感器时间，Elapsed也用传感器时间；reset/自动重建后以新有效输出重选B0。
- 保留等待/静止标定/建图/断流，正常前5个处理扫描为WARMUP，之后为GOOD，异常为CHECK。
  有效stamp超过terminal.stale_timeout=1s未推进显示STALE；重复/回退不能维持GOOD，GOOD不保证精度。
- scripts/odom.sh和包launch支持terminal_enabled、terminal_clear_screen、terminal_rate覆盖；
  留空沿用YAML。TTY默认持续清屏，文件/管道无ANSI；clear_screen=false保留其他节点日志。
  刷新率范围(0,10]Hz，默认10Hz；参数仅启动读取，关闭面板后不创建打印timer。
- recv/accept为适配后核心接收/成功入队计数，proc不含初始化扫描；适配器提前拒绝不在这些统计内。
  accepted后溢出/过期不撤销接受记录。Cloud reject/pending沿用参考标签，pending实际是累计
  缓冲溢出丢帧，非当前队列长度；DDS message drops=n/a表示未计量，不能伪造0或当作buffer-drop。
- 频率按递增传感器stamp窗口计算，不是加速回放墙钟吞吐；tracked/matches来自真实激光调用/成功数。
  扫描耗时含核心到同步输出、不含适配预处理。CPU按全逻辑核总容量100%，Cores给等效核数，
  CPU Avg为最近100次刷新；RAM是进程RSS的十进制MB，组件模式包括整个容器。
- 实现位于src/terminal_dashboard.*和core diagnostics，终端IO不持有滤波锁。显式reset清计数和
  B0参考；自动重建保留核心累计计数并清上一轨迹显示、B0、距离和扫描耗时周期。
- 此前绝对坐标面板验证：10Hz参考布局Release构建、24个字段标签/顺序检查通过；101次面板
  位姿/twist与原baselink输出在4位小数精度内一致，85组Odom/TF stamp与位姿完全一致。
  B0显示变换后，合成验证覆盖非零初始位姿、初始姿态旋转、速度/重力旋转、非法首帧和
  reset重锚点；已有录包回放确认首个有效baselink作为显示原点、ROS Odom/TF仍保持原始
  odom语义。使用已有录包，不启动雷达或操作PTP，不留测试脚本。
- 修复新终端回放时可选Livox CustomMsg运行库路径缺失：start_lidar=false时若Livox已安装则
  加载其环境，但仍不启动驱动、不检查PTP；标准PC2独立构建仍不要求存在Livox工作区。

## 云台转心外参更新（2026-10-11）

- 使用 `odom_20261011_211347_1014ce5a`，用户确认底盘固定且云台只转yaw；根据420组
  body位姿拟合固定转心，只修改 `pcl_ws/config/lidar2baselink.yaml` 的XY平移，保留R/z。
  新 `t_BL=[-0.006009591012854143,-0.14915516181160768,0] m`，旧值保存在该bag分析目录。
- 约定 `p_new=p_old+R_old*d`、`t_new=t_old-d`，d=[12.535988,-3.374777,0]mm。
  这是基于LIO估计的有效杆臂，不是完整六自由度机械标定；yaw不能可靠标定高度。
- 同一原始420帧轨迹相对首个有效baselink的最大偏移5.573→3.938cm，RMS2.166→1.405cm；
  前后半、正反向四组留出验证均改善，但动态残差仍在，末帧0.507→0.510cm基本不变。
- 已构建转换包并核对源码/安装YAML一致；独立ROS域1x回放旧/新配置412组输出满足精确
  杆臂关系（位置误差<5.2e-14m），Odom/TF一致，新静态TF加载正确。滤波、时间、PTP未改。
- 详见 `docs/云台转心外参标定.md` 及该bag的 `analysis/extrinsic_recalibration/`；
  原始db3不修改，只留配置、CSV/JSON、图和报告，未新增测试/回放脚本。
