# ly_odom 工作说明

## 当前30维状态与芯片外参（2026-10-13，最新用户要求优先）

- 按用户最新要求恢复Point-LIO的30维状态/P：[p,R,R_IL,t_IL,v,omega,a,g,bg,ba]。
  30维是状态与误差维度，IMU残差仍6维，单点激光残差1维；Batch模型及0.1ms默认保留。
- MID360 YAML保留用户提供的IMU芯片在激光系位置extrinsic_T=[-.011,-.02329,.04412]m，
  显式extrinsic_translation_convention=imu_in_lidar；代码读取时t_IL=-R_IL*extrinsic_T，
  内部p_I=R_IL*p_L+t_IL。单位旋转时t_IL=[.011,.02329,-.04412]m。
  L2/未声明约定的参数默认lidar_in_imu，直接使用原Point-LIO平移约定。
- extrinsic_est_en=false固定外参并清零P对应6行/列；true恢复12列激光联合观测与双SO(3)
  注入/reset。在线先验extrinsic_rotation_cov/translation_cov在所属YAML，Q外参为零。
- 在线Batch保留原LiDAR点与冻结补偿D，正确计算外参H；后验地图点/注册云只更新标定部分，
  不重新运动补偿。重建/reset恢复配置标定。
- 固定安装T_BL推导的T_BI随当前在线T_IL重算，机体输出P包含外参交叉项。
  baselink→lidar_frame静态TF严格是T_BL，不能把非零偏移IMU原点标成livox_frame。
  显式固定T_BI且在线T_IL时按Odom相同stamp发布动态传感器TF。
- 配置继续分包直接加载，轮速不接入。说明见包内docs/30维状态恢复与IMU安装外参.md。
  下方24维清理、旧零漂和配置迁移数目均为历史，不能替代本轮30维/非零外参的精度结果。
- 本轮Release构建、924次ESKF数值核对及256组生产机体输出协方差有限差分通过；
  三种ROS合成配置共108组Odom/TF/注册云stamp一致，TF值差0，reset恢复及SIGINT退出通过。
  指标见包内docs/30维数值验证.json、30维外参与机体输出验证.json、30维ROS验证.json。
  未启动硬件、未改PTP、未测新零漂；临时验证程序与数据已清理。

## 当前分包配置与直接加载（2026-10-13，优先于下方历史）

- 用户要求取消中央合并配置。里程计参数在`odom_ws/src/ly_lio/config/mid360.yaml`，
  Livox ROS/PTP参数在`livox_sdk/src/livox_ros_driver2/config/mid360.yaml`，SDK直接读取同目录
  `MID360_config.json`；共享安装外参恢复为`pcl_ws/config/lidar2baselink.yaml`。
- 录包配置独立放工作区外`config/rosbag.yaml`，业务包`ly_bag_recorder`直接读取它并使用
  原生rosbag2_transport::Recorder，保留全部/隐藏话题、持续发现、QoS和关闭刷新metadata。
- 各节点C++直接读取所属默认配置，源码存在时优先读取源码，缺源码用包share。
  日常修改后重启即可；ROS显式参数/参数文件可覆盖。统一shell/launch仅管理进程，
  不读取参数YAML，不生成临时YAML/SDK JSON，不向节点注入配置。
- scripts/odom.sh仅保留start_lidar与record_bag进程开关；旧config/config_path/bag_dir/
  terminal/PTP launch覆盖不再使用，修改所属文件或独立节点ROS参数。
  回放`start_lidar:=false record_bag:=false`不启驱动、不查询PTP。
- 驱动代码在启动SDK前只读检查独立PTP/PHC服务和JSON主机接收IP，绝不操作服务/时钟。
  Livox网络JSON是上游SDK要求的持久配置，不由脚本从YAML转换。
- 用户选定Batch **0.1ms=0.0001s**，严格0<duration<.010s；L2未调参仍2ms。
  24组600秒同源搜索采用100µs相对2ms位置RMS降低6.25%，部分姿态略退化，
  两静止/一旋转回归也小幅改善位置，无新实机采集。见包内docs/Batch时长调参与零漂.md。
- 四业务包增量构建与离线配置/录包验证通过：64项LIO、16项驱动、原生ROS覆盖、
  组件加载、隐藏发现/静态TF、内容保真和SIGINT/SIGTERM写盘；统一入口默认录包与
  录包单节点退出联动关闭均通过。未启动硬件或改PTP，临时程序/合成bag已清理。
- 分包加载及使用见odom_ws/src/ly_lio/docs/统一参数配置.md，指标见同目录分包配置验证.json；
  此前集中配置52项断言只属于历史实现。

## 24维模型历史（2026-10-12，已被文首30维要求替代）

- 当前ly_lio按docs/建模.md改为激光Batch点到历史地图联合更新，不融合轮速。
- 当前状态与P均压缩为24维[p,R,v,omega,a,g,bg,ba]；在线30维实验代码已按要求删除。
  激光→IMU外参仅作固定参数，旧extrinsic_est_en=true配置明确启动失败。
- 清理后的384次数值投影比较及旧bag738条输出逐数等价通过；新一轮地图/采样/初始化
  15组筛选的点距.02候选在动态复核中退化，仍保留原点距.05和原噪声。
  最新记录见odom_ws/src/ly_lio/docs/24维清理与10分钟零漂.md。
- 真24维独立实机bag odom_20261012_213853_3614d9c3，预热5s后完整600s：
  首末5s均值位置差0.6215cm、最大1.1963cm、RMS0.5098cm，姿态末端0.021623°。
  窗内6000组Odom/TF/注册云一致，无窗内重建；启动首输出前曾因IMU缺段重建一次。
  转换云6000/6000，IMU录包旁路119999/119998，少一配对不直接归因转换节点漏发。
  新bag同源回放原点距.05末端0.5609cm，.02候选0.9309cm，支持保留原参数。
  采集已停止，PTP active/PID不变；驱动仍已知SIGBUS(-7)，其他节点正常退出。
- 最长2ms一批，遇IMU观测提前分界；真实批末去畸变，冻结先验后一次联合更新。
- IMU输出观测保持，饱和阈值95%，Q只含omega/a/bg/ba；先P后x在区间起点线性化。
- 默认仍扫描末点发布，原始输入、机体输出杆臂和两路转换保留，PTP独立。
- 参数迁移、保留的工程差异与测试结果见
  odom_ws/src/ly_lio/docs/文档模型改进与零漂验证.md。
- 此前Batch模型独立实机bag odom_20261012_190342_53aa7a1c，预热5s后60s：末端0.309cm、
  最大2.246cm、末端姿态0.01563°；654组Odom/TF/注册云stamp一致，窗内转换链路完整。
- 两份同源比较均未证明全面降低零漂（旧bag末端0.910→1.002cm，新bag0.237→0.292cm）；
  不以新实机结果替代同源改进证明。驱动仍有已知SIGBUS，PTP保持active且PID不变。
- 以下“现行”“当前”标题中的逐点/相邻平面与数值均为既往记录，不是本轮Batch结论。
- 后续18组有限调参只用旧静止bag筛选，冻结候选后用新静止bag及运动数据验证。
  候选未稳定泛化，30维也未全面改善；该轮保留24维和原噪声。方法、数据与结果见
  odom_ws/src/ly_lio/docs/调参与30维状态对比.md；该历史轮使用既有录包，未启动硬件。

## 工作区与数据流

- ROS 版本：ROS 2 Humble。
- `livox_sdk`：Livox MID360 驱动工作区。
- `pcl_ws`：外参标定和点云/IMU 坐标变换工作区。
- `odom_ws`：里程计工作区；`ly_lio` 是独立 ROS 2 逐点激光惯性里程计包，
  `Point-LIO` 和 `small_point_lio` 为算法参考。
- 数据流：MID360 → Livox 驱动 → `/livox/lidar`、`/livox/imu` →
  `pcl_publisher` → `/bashlink/pcl`、`/baselink/imu`，保留供其他使用者。
- 里程计直接接 `/livox/lidar`、`/livox/imu`，在 IMU 原点滤波，组合固定安装外参
  输出 odom→baselink，并发布 baselink→livox_frame 固定 TF。

## 现行里程计模型（2026-10-11，10 ms相邻平面窗口）

- 标定后按真实激光采样时间划分10ms窗口，逐点只预测、IMU观测与去畸变；
  每窗提取平面，只与紧邻前窗一对一匹配，激光约束为plane→plane，不调用旧point→map。
- 首窗只建立参考，缺窗不跨跳；时间取最晚有效输入点（标签有效、坐标/时间有限，
  距离过滤前）。原始10Hz帧到达后可成批发布约100Hz传感器时间位姿。
- 法向两自由度与前窗IMU原点处的有符号距离形成3维残差；质心仅作关联门控。
  前窗Schmidt pose clone保留边缘与交叉协方差，创新项计入同源相关性；误差注入后做SO(3) reset。
- 只支持固定lidar→IMU外参，extrinsic_est_en=true会启动失败；旧地图匹配/逐点发布参数移除。
  保存地图只作导出，
  不作为当前激光约束。保留30维Point-LIO IMU输出观测状态、原始输入与机体输出链路。
- 本轮真实静止60秒采样已完成，末端52.454m、最大54.632m、姿态末端171.154°，
  模型未通过静止验收。正式采样在启动后约107秒，观察器先预热5秒，并非初始化后5秒。
  结果在bags/odom_20261011_180304_2966109d/analysis/plane_window_live/。
  不将下面旧模型1.94cm结果作为本轮结论，旧bag与本轮实机也不作同源A/B。
  6000组Odom/TF时间戳与位姿一致，5705注册云全为相同stamp子集，旁路未收全不代表节点漏发。
  PTP服务不变，转换话题保留；本轮转换/里程计/录包正常退出，驱动仍有-11退出异常。

## 本次完成的修改

1. `livox_sdk/src/livox_ros_driver2/config/MID360_config.json` 将雷达地址改为
   `192.168.1.3`；主机接收地址已按当前 enp86s0 改为 `192.168.1.50`。
2. `livox_sdk/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` 使用
   `xfer_format=0`，输出 `sensor_msgs/msg/PointCloud2`。原先的自定义点云格式
   `xfer_format=1` 无法直接接入此节点。
3. `pcl_ws/src/pcl_publisher/src/pcl_publisher.cpp` 实现独立的点云和 IMU 回调，
   每条消息到达后立即变换并发布，不要求两个传感器消息同步。
4. 完善包依赖、可执行程序安装、YAML 安装和
   `pcl_ws/src/pcl_publisher/launch/pcl_publisher_launch.py`。
5. `livox_sdk/src/livox_ros_driver2/src/lddc.cpp` 为 MID360 IMU 添加
   `orientation_covariance[0] = -1`，明确告知消费者设备未提供姿态。

## 坐标变换约定

- 外参唯一来源：`pcl_ws/config/lidar2baselink.yaml`。其中现有键名
  `lidar_to_gimbal_rotation`、`lidar_to_gimbal_translation` 分别作为 lidar 到
  baselink 的旋转 `R` 和平移 `t`；当前按 gimbal 即目标 baselink 理解。
  如果二者实际不是同一个坐标系，应先组合 gimbal 到 baselink 的外参。
- 点云：`p_baselink = R * p_lidar + t`，平移单位为米。
- IMU：`angular_velocity_baselink = R * angular_velocity_lidar`，
  `linear_acceleration_baselink = R * linear_acceleration_lidar`。
  平移不直接加到向量上；本节点将测量轴转换至 baselink，
  不补偿 IMU 与 baselink 原点之间旋转运动产生的杆臂加速度。
- 假定输入 IMU 的轴与 lidar 轴一致；若存在独立 IMU 安装外参，应另行组合。
- 协方差：`C_baselink = R * C_lidar * R.transpose()`。
  保留 ROS 的 `C[0] == -1` 不可用标记；全零协方差保持未知语义。
- 输入姿态若有效，按 `q_world_baselink = q_world_lidar * inverse(q_baselink_lidar)`
  转换并归一化；MID360 驱动未提供姿态，明确设置不可用标记。
  节点对未提供的姿态或无效四元数输出单位四元数，并设置
  `orientation_covariance[0] = -1` 表明姿态不可用。
- IMU 数值单位沿用驱动输入，不进行重力去除或单位换算。直接使用 Livox 原始
  加速度时，应核实驱动/设备的单位；需要 SI 单位的下游应确保输入为 m/s²。
- 两类消息保留输入时间戳，输出 `header.frame_id` 默认为 `baselink`。
  点云仅修改 XYZ，保留 intensity、tag、line、timestamp 以及点布局；支持
  FLOAT32 XYZ、大小端和带行填充的点云。非有限点保留并设置 `is_dense=false`。
- 输入 QoS 为 SensorDataQoS（best effort）：点云默认depth=5，IMU加深到1000以容纳点云回调期间数据；输出为 reliable、depth=10。
- 点云输出按需求保留字面名称 `/bashlink/pcl`；IMU 输出为 `/baselink/imu`。
  如需更改可使用 ROS remap，不要只在某一处静默改名。
- 参数 `config_path` 指定外参文件，默认读取包 share/config 中安装的 YAML；
  `target_frame` 指定目标坐标系名称。配置缺失、形状错误或旋转不合法时启动失败。
  错误的点云布局会被丢弃并输出限频警告。

## 构建与运行

首次构建 Livox 驱动（需要已安装 Livox SDK2）：

```bash
source /opt/ros/humble/setup.bash
cd /home/hustlyrm/ly_odom/livox_sdk/src/livox_ros_driver2
bash build.sh humble
```

驱动提供的 `build.sh` 会删除其工作区旧的 build/devel/install 目录。
已有构建时可在生成 `package.xml` 后直接增量构建：

```bash
source /opt/ros/humble/setup.bash
cd /home/hustlyrm/ly_odom/livox_sdk
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

构建变换节点：

```bash
source /opt/ros/humble/setup.bash
cd /home/hustlyrm/ly_odom/pcl_ws
colcon build --packages-select pcl_publisher --symlink-install
source install/setup.bash
```

日常从仓库根目录统一启动（主机网口为 `192.168.1.50`，雷达为 `192.168.1.3`）：

```bash
./scripts/odom.sh
```

PTP 是开机自启系统服务；此命令仅检查授时服务和接收地址，然后启动雷达驱动、
坐标转换、里程计与全话题 rosbag 录制。回放原始 rosbag 或使用外部驱动时运行
`./scripts/odom.sh start_lidar:=false`。Ctrl+C 同时通知录包与 ROS 节点退出，等待录包写盘结束；PTP 继续运行。

单独调试转换节点时也可执行 `ros2 launch pcl_publisher pcl_publisher_launch.py`。

也可直接指定源码外参，避免使用旧的安装副本：

```bash
ros2 run pcl_publisher pcl_publisher --ros-args \
  -p config_path:=/home/hustlyrm/ly_odom/pcl_ws/config/lidar2baselink.yaml
```

验证实际雷达输出：

```bash
ros2 topic info /livox/lidar
ros2 topic hz /bashlink/pcl
ros2 topic hz /baselink/imu
ros2 topic echo /baselink/imu --once
```

## 后续维护

- `scripts/odom.sh` 加载 Humble、`livox_sdk`、`pcl_ws` 和 `odom_ws` 环境，通过
  `scripts/odom.launch.py` 统一启动 MID360 驱动、`pcl_publisher` 与 `ly_lio`，
  Ctrl+C 关闭 ROS 节点，独立的开机自启 PTP 服务继续运行。
  默认读取源码外参和 MID360 里程计配置，支持 `config_path:=...`、`config:=...`；
  可从任意目录执行，进程工作目录固定为仓库根目录。
  默认先只读检查 PTP 两服务和接收 IP；回放 rosbag/外部驱动用 `start_lidar:=false`，
  此模式若 Livox 已安装则加载消息库环境，不启动驱动、不检查 PTP。PTP 不由 odom 脚本启动/重启/停止。
- 修改外参后重新构建，或用 `config_path` 显式指定源码文件；外参在启动时读取。
- 驱动继续保持零外参，避免与此节点重复施加变换。
- 变更话题、坐标系、消息类型或外参约定时同步检查驱动 launch、节点和本说明。
- `odom_ws/src/small_point_lio/config/mid360.yaml` 的现有默认值使用
  `livox_custom_msg`。后续接入本次输出时，应将其 `lidar_type` 配置为
  `livox_pointcloud2`，并将点云/IMU 输入改为 `/bashlink/pcl`、`/baselink/imu`；
  不要继续对已经变换的数据重复施加 lidar 到 baselink 外参。
- 按用户要求删除此前添加的测试及分析/回放代码；维护时使用构建、ROS CLI 和原始录包检查。
- 本次验证范围为节点编译与模拟消息测试；真实雷达网络、数据单位和机械外参
  仍需在设备上确认。
- 验证结果：`pcl_publisher` 和修改后的 Livox ROS 2 驱动均编译通过；7 个 ROS
  集成用例全部通过，`colcon test-result --verbose` 报告 0 errors、0 failures。
  环境中的 cppcheck 2.7 因 ament 已知性能问题自动跳过。
  已实际启动安装后的 launch，确认默认 YAML 加载成功且两个输出话题类型正确。

## ly_lio 初版实现历史（现行模型见文首）

- 在 `odom_ws/src/ly_lio` 吸收 Small Point-LIO 的固定 30 维 ESKF、标量激光更新、
  IMU 饱和轴屏蔽、在线外参估计、PCA 平面、稠密哈希 LRU 地图、7 邻域查询、
  OpenMP 降采样、四类雷达适配、ROS 2 组件、轻量 PCD 与压缩保存。
- 与 Point-LIO 的 IMU 输出观测模型核对状态、噪声、雅可比与残差。
  默认接入 `/livox/lidar` 和 `/livox/imu`，lidar→IMU 外参默认单位阵，输出
  `/ly_lio/odometry`、`/ly_lio/cloud_registered`、`/ly_lio/path`、odom→baselink TF。
- 修正逐点相对时间、原实现体素坐标混叠、初始化外参、后验地图插入、重置、
  并行体素跨块重复、坐标系重复变换以及保存成功语义。
- 算法、节点和配置已加入中文注释。完整改进清单、单位约定和复现范围
  见 `odom_ws/src/ly_lio/docs/实现对照.md`，包维护规则见其 `AGENTS.md`。
- 构建：在 `odom_ws` 中执行 `colcon build --packages-select ly_lio --symlink-install`；
  source 安装目录后运行 `ros2 launch ly_lio ly_lio.launch.py
  body_config_path:=/home/hustlyrm/ly_odom/pcl_ws/config/lidar2baselink.yaml`。
- 安装 YAML 是 lidar→baselink，节点用固定 lidar→IMU 外参推导 IMU→baselink；
  在线外参模式需要显式 imu_to_body_R/T。输出位置、速度、协方差、Path、TF
  同步转换到机体原点，地图不重复变换；IMU 输入检查 imu_frame。
- 原始帧名同为 livox_frame，仍需真实内部标定支持原点重合假设；活动云台不适用固定机体外参。
- 当前已验证独立标准点云构建、可选 Livox CustomMsg 构建、14 个算法用例和
  1 个 ROS 集成用例。真实数据集轨迹精度、极端运动和性能倍数尚未验证。
- 复现 30 维输出模型；原 Point-LIO 的备用 24 维 IMU 输入路径不包含在本包中。
- 原始输入与机体输出改造后验证：ly_lio 的 16 个算法用例和 ROS 集成用例通过
  （colcon 汇总 19 项、0 failures）；转换节点测试通过，保留原有两个发布话题。
  实际启动脚本确认原始订阅、baselink 发布、安装静态 TF 和 Ctrl+C 正常退出。

## PTP 时间同步

- `PTP/` 包含中文使用文档、hardware UDPv4 E2E MASTER 模板、systemd 单元、一次性部署/只读检查、
  驱动增量构建、原始包和 ROS 时间戳验收与本机检查记录。
- 当前 enp86s0/ptp0 已有 sentry-ptp4l 与 sentry-phc2sys 运行，系统 UTC→PHC 方向，
  复用现有服务，不同时安装竞争同一时钟的实例。主机接收地址为 192.168.1.50。
- MID360 launch 设 LIVOX_REQUIRE_PTP_SYNC=1；驱动未收到 PTP/gPTP time_type=1 时丢包，
  不回退为收包时间。构建用 ./PTP/build_driver.sh，日常统一启动用 ./scripts/odom.sh。
- 系统 UTC 不被本工具修改；PHC 初始化仅在采集前，运行 -F 0 -S 0 禁止阶跃。
- 不能用 MASTER/s2 代表雷达已锁定；sudo ./PTP/check_ptp.sh verify 和 ./PTP/check_ros.sh
  两层都需验收。管理员操作因当前 sudo 需要密码未自动执行，不宣称实机时间精度。
- PTP 实机首次发现 TAI 比 UTC 快37秒、PointCloud2误传绝对纳秒；已在驱动统一
  转 UTC（launch ptp_utc_offset=37）并发布相对逐点纳秒；目标IP过滤仅收 .3。
- 稳态实机 ROS 10秒验收通过（最终100点云/2001IMU），无回退、重复、错误年龄或逐点错误。
  原始包旁路验收仍需sudo；驱动关闭时观察到上游SIGBUS，未将其表述为正常退出。

## 启动职责调整

- PTP 系统服务开机自启，本机 sentry-ptp4l@enp86s0 与 sentry-phc2sys@enp86s0
  已 enabled/active；static bootstrap 由依赖启动。安装脚本复用部署时也补齐 enable。
- 删除 PTP/ptp.sh 和 PTP/start_lidar.sh，PTP/check_ptp.sh 仅支持 status/verify。
- 雷达驱动、转换节点与里程计由 scripts/odom.sh 统一启动；支持 start_lidar:=false
  回放、ptp_interface、ptp_utc_offset、target_ip 参数。保留两路 baselink 发布。
- 本次启动调整验证：Shell/Python 语法、任意目录参数展开、回放不访问 PTP、
  伺服未运行时拒绝启动均通过。实机统一启动后滤波器初始化，保留两个转换发布话题；
  5 秒 ROS 采样 50 点云/1001 IMU，passed=true，回退/重复/逐点错误均为 0。
  Ctrl+C 后 PTP 服务 PID 不变且仍 active；转换和里程计正常退出，驱动仍有已知 SIGBUS(-7)。

## 全话题录包

- scripts/odom.sh 默认 record_bag:=true，包含 ros2 bag record --all
  --include-hidden-topics，持续发现当前 ROS 域内全部话题，保存消息数据和原有 header。
  包括原始点云/IMU、转换点云/IMU、里程计、注册点云、轨迹、TF、参数事件与日志；
  不自动包含服务调用、节点参数快照、配置文件或 PTP 硬件日志。
- 默认 bags/odom_北京时间_随机后缀/，每次新建独立 sqlite3 录包，不覆盖旧数据，
  bags/ 已加入 gitignore。bag_dir:=... 可指定父目录；record_bag:=false 可关闭录包。
- scripts/rosbag_qos.yaml 对原始传感器设置 best_effort，对 /tf_static 设置
  reliable/transient_local，确保接收发布者缓存的静态安装变换。其他话题自动适配 QoS。
- rosbag 在统一 launch 内接受 Ctrl+C 并刷新缓存、写 metadata，SIGTERM/SIGKILL
  宽限各 30 秒；等待进程退出再关闭终端。录包意外退出时关闭采集，避免继续运行却未保存。
- 录制 cloud_registered/path 会产生真实订阅，原先按订阅者跳过的发布将启用；
  录包磁盘占用随运行增长，PTP 系统服务继续独立运行。
- 回放时通常用 scripts/odom.sh start_lidar:=false record_bag:=false，
  若需要保存重新计算的输出，可保留 record_bag:=true。
- 全话题录包实机验证：从 /tmp 调用脚本，生成
  bags/odom_20261011_154232_4d9325e8，3695 条消息，sqlite3 约 97.6 MB；
  原始输入、转换输出、Odometry、cloud_registered、Path、动态与静态 TF、rosout
  均有消息，/tf_static=1；/parameter_events 已订阅但无参数变更，计数为 0。
  metadata.yaml 写出，录包正常退出，PTP 仍 active；本轮驱动也正常退出，
  不代表此前间歇出现的 SIGBUS 已修复。禁用录包与录包异常退出关闭逻辑检查通过。

## 静止底盘、运动云台录包分析

- 用户明确雷达刚性安装在运动云台上，底盘静止。现有 lidar_to_gimbal 固定外参
  只能表达雷达→云台；当前 baselink 标签不能自动等价于真实底盘。
- 20261011_154652_bbb530d5 录包只读分析生成于该 bag 的 analysis/；
  99.30 秒有效输出，最大参考点位移22.03 cm、最终1.33 cm，旋转主要绕云台 Z。
  单一固定转心拟合残差 RMS4.45 cm，尚不能认定误差来源或真值精度。
- 已保留数值、轨迹和 TF 一致性分析结果；此前添加的分析代码按要求删除。
- 真正底盘输出需要有时间戳的云台关节角及完整动态轴心几何，不能将姿态清零冒充
  静止底盘；本次仅分析记录数据，不改变算法、安装配置、话题接口或原始录包。

## 共轴机构与点云频率检查

- 用户进一步明确底盘中心/云台中心Z轴重合。纯yaw且中心安装外参正确时，
  输出中心不应水平运动，不能仅以云台坐标系命名解释22 cm位置摆动。
- 已只读核对帧频/逐帧匹配/延迟，结果位于目标bag
  analysis/点云频率检查.md、frequency_metrics.json、frequency_analysis.png。
- 原录包有效输出994帧对应连续994帧原输入，无漏帧；输入/输出均约10 Hz，
  原始IMU约200 Hz；输入到输出录包接收差中位12.13 ms、最大30.70 ms。
- 同一Release核心离线基准每100 ms输入周期算法工作平均8.87 ms、最大13.19 ms，
  约3.33万次成功激光更新/传感器秒。基准排除DDS/序列化，回调顺序与原节点不等价，
  不能用它验证轨迹精度；此前添加的性能探针源码及二进制已删除。
- 100 ms帧内云台转动约14°，运动雷达系先做体素坐标/时间平均是待检验的精度机制，
  尚未通过分帧A/B证明原因。本次保持驱动频率、算法及噪声参数不变。

## 实机静止零漂基线

- 用户要求启动实机并保持静止，录包odom_20261011_161143_6cfa194f；
  测量器预热5秒后采样60秒，601条里程计/11999条IMU。首末各5秒均值比较，
  最终位移6.66 cm、最大8.66 cm；最终姿态0.390°、最大0.581°，输出约10 Hz。
- 重力补偿已启用：200样本静止估计方向，预测R*f+g；原始及转换IMU不预先扣重力。
  本次结果证明仍有静止残余误差，未据此认定具体原因，也不能外推长期漂移率。
- 实机测量结果保存在该bag/analysis/
  静止零漂报告.md、stationary_metrics.json、stationary_samples.npz与轨迹图。
- 测量后已停止本次采集；录包/转换/里程计正常退出、PTP保持运行；
  驱动析构本次出现SIGSEGV(-11)，不宣称退出异常已修复。

## 旧逐点地图模型PLIO同源对比与修复（2026-10-11）

- 参考实际路径 `/home/hustlyrm/sentry_test_py/odom/src/plio`，保持参考源码只读。
  用其生产估计器回放同一原始云/IMU；仅将加速度g→SI、点时间相对ns→s，
  禁用参考双雷达的IMU修正，不移植其他设备标定。输出组合本项目同一安装中心外参。
- ly_lio增加静止窗口SI门限与bg/ba初始化；地图只用连续静止段。不能把慢速旋转
  自动保证识别为运动，启动仍须保持静止；单姿态不等于完整加速度计标定。
- 体素降采样保留真实坐标/时间对；地图每格最多20个真实样本，默认5cm去重；
  本扫描逐点后验坐标在scan_end统一插入，积压扫描逐帧发布。
- 激光/IMU观测与扫描发布前传播协方差，下一IMU只传播剩余区间；稀疏F左右乘
  与同定义稠密FPFᵀ核对一致。Q使用Q*dt_ref*dt，dt_ref=.005s保持200Hz标定
  方差率，直接bg/ba随机游走不会因点密度变大而缩小；一阶F仍有离散误差。
- 确定性60.001秒对比：原版LY末端6.817cm/最大8.696cm，参考PLIO3.853/4.544cm，
  修复LY1.939/2.701cm；姿态末端0.4179°/0.1995°/0.1483°。
  只补零偏未改善稳态漂移；多点地图/点距是本段位置改善的主要因素。
- 云台转动共同99.10秒：最大中心偏移原版22.769cm→修复14.024cm，
  但末端2.106cm→4.699cm变差，未彻底解决旋转误差。没有静止锁定/强制零速度。
- 已完成ly_lio构建，32个算法GTest+1个ROS流程用例通过，colcon汇总35项0 failures。
  离线旋转核心每100ms周期工作平均15.90ms、最大24.79ms（不含SQLite/CDR）。
- 此前静止回放验收满足本段末端2cm、最大4cm目标；回放与对比代码现已按用户要求删除。
  既有轨迹、图和指标保留，不能当作独立新数据上的精度保证。
- 详细机制、分项A/B、单位、配置差异和历史结果见
  `odom_ws/src/ly_lio/docs/PLIO对比与零漂修复.md`；图/CSV/JSON在静止bag的
  analysis/ly_compare/final_comparison、analysis/ly_compare/gimbal_final、analysis/current_regression。
  原版/修复的轨迹与配置保留；归档源码、测试库和回放二进制已删除。
- 本轮未启动真实硬件或改PTP；保留 `/bashlink/pcl`、`/baselink/imu` 与原始输入流，
  安装外参未改。参数选择使用这两份录包，需独立实机/数据验证，不能声称长期精度保证。

## 改模前时间戳与逐点输出历史（2026-10-11）

- 驱动两路PTP统一减37秒转UTC；点云header是帧首点，timestamp是相对纳秒，
  ly_lio用header+offset*1e-9；动态TF/Odometry使用状态的传感器时刻，不能改now()。
- 逐点ESKF与ROS输出频率独立。默认publish_odometry_without_downsample=false在scan_end
  发布一次；true在每个参与滤波的点后发布，仍遵循space_downsample，并随整帧到达成批输出。
- 主滤波仅处理点云/IMU共同覆盖的时间，没有独立IMU实时预测TF路径。
  更高频实时输出应缩小输入批次或另设IMU预测输出，不能用重复旧位姿加新时间戳实现。
- 已删除助手加入的包内测试、测试依赖/目标、漂移分析/对比/回放/性能探针代码、
  对应缓存/临时程序/归档和旧测试构建目录，重新构建业务节点。
  原始db3、指标/图/CSV、参考实现及PTP运行检查保留。本文此前测试数字仅为历史结果。
- 两bag原始流无倒退/重复；约4900万点无double时间合并，回转最大舍入365ns。
  IMU约4.27/5.74ms短长交替，设备采样/打包时间语义仍需核实，不擅自重写5ms网格。
- 旧输出部分帧按较晚IMU时刻发布；改模前独立scan_end回放404组输出完全对应最后有效点，
  Odom/TF/注册云stamp一致，误差−320～304ns。详情见 `odom_ws/src/ly_lio/docs/时间戳与发布频率.md`。


## 第二轮相对传感器残差实机早停（仍未通过）

修正了残差的参考表达后重新启动实机。首个 Odometry 输出后预热 5 秒，因约 9.97 秒已出现
9.64 m 位移而提前停止，录包实际可用于统计 50.940 s（硬件在首输出后约 55.94 s 停止，
未完成目标 60 s）。首末各 5 s 均值为：末端 **29.131 m**、最大 **30.819 m**、位置 RMS
**21.832 m**；末端姿态 **171.753°**、最大 **179.974°**。在线约 20 s/30 s/50 s 时位置
偏移约 17.68/27.17/34.75 m，仍严重发散，不能宣称修复。

数据位于 `bags/odom_20261011_181443_d8decc71/analysis/plane_window_live_round2/`。
录包中 5095 组 Odometry/TF 的完整 stamp 序列一致；收到的 4838 条注册云均是相同 stamp
的 Odometry 子集，旁路订阅未收全不等于节点时间戳错误。传感器时间输出约 100 Hz，原始云
约 10 Hz。该早停结果与第一轮及旧逐点地图录包不是同源 A/B。

实测后仅将默认平面支持数提高到20、法向门限收紧到10°，并把噪声下限调为2°/5 cm；
这些保守参数尚未重新进行实机精度验收，不能据此宣称零漂改善。


## 当前优先级：单雷达Point-LIO（2026-10-11晚）

- 用户明确暂停相邻10ms平面建模，先对比sentry完整双雷达PLIO并做好单雷达。
  当前 `ly_lio` 已恢复逐点 point→map 30维ESKF；此前平面模型失败记录只作历史。
- 真正启动sentry双雷达驱动、融合标定、PLIO和录包，关闭GTSAM回环以测连续里程计。
  原始包 `bags/sentry_dual_20261011_comparison`；参考源码保持只读。
- 同源固定60s窗口：双雷达末端1.505cm/最大1.874cm，参考.3单雷达安装中心
  1.738cm/2.516cm；均首末各5s均值。晚段独立在线60s为1.102cm/2.112cm，不能混用时窗。
- PTP440/440 true，87519552个原始点无回退，融合45711041点的时间与raw转换逐位一致，
  未发现37秒时标错配或逐点时间丢失。静止不能标定内部物理采样时差。
- sentry驱动原始IMU已为m/s²，本项目驱动仍为g；回放sentry原始数据必须将acc_norm
  和gravity模长设为9.80665，并将satu_acc设为29.41995，避免重复乘g或误屏蔽重力轴。
- 恢复多点LRU地图、真点采样、扫描末端后验入图、静止标定后5帧初图、完整P传播。
  默认Odom/TF约10Hz扫描末点发布，激光滤波仍逐点；保留原始输入、机体安装外参和两路转换发布。
- IMU缺段超过0.1s或未处理队列溢出会明确重建odom，并清Path/导出地图；下游须识别重置。
- 业务使用 `./scripts/odom.sh`，PTP仍为独立开机服务；新代码与解释含中文注释。
  不保留新增测试、回放、分析脚本，仅保留bag、配置和结果。详见包内
  `docs/双雷达与单雷达对比.md`、`AGENTS.md`与根目录 `ekf.md`。

- 本轮当前LY同源60秒末端0.919cm/最大1.446cm；公共扫描时刻参考双雷达1.498/1.833cm、
  单参考1.730/2.489cm。另默认odom.sh独立实机60秒0.906/2.428cm，姿态末端0.191°。
  原始点时间、输出stamp/TF值一致，报告见对应bag/analysis和包内对比文档。
- 实机发现IMU转换链路少67/12000条，原始完整，缺失均处点云转换回调占用区间；
  converter与LY的IMU接收队列从5加深到1000，保留best effort/时间顺序。
- 本轮关闭里程计、转换、录包均正常；Livox驱动仍有已知析构SIGBUS(-7)，未宣称修复。
  PTP服务active且PID不变。静止改善不等于动态时间/外参或移动精度已验证。
- 深队列修复同源复核通过：12000/12000 IMU和599/599点云转换，stamp/顺序完全一致；
  当前LY末端0.906cm、最大2.401cm，599组Odom/TF/注册云stamp一致、TF值差0。
  临时对比程序已清理，保留bag、有效参数、CSV/JSON/图与中文报告。


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

- 用户确认 `bags/odom_20261011_211347_1014ce5a` 中底盘固定、云台只绕竖直Z轴旋转。
  420组body位姿拟合固定转心，`p_new=p_old+R_old*d`、`t_BL_new=t_BL_old-d`。
  d=[12.535988,-3.374777,0]mm；仅更新水平有效杆臂，保留安装旋转和高度。
- `pcl_ws/config/lidar2baselink.yaml` 的新平移为
  `[-0.006009591012854143,-0.14915516181160768,0] m`；两消费节点仍共用此文件。
  转换包已构建，安装副本与源码一致；下次启动生效。
- 同一原始420帧轨迹相对初始baselink的最大偏移5.573→3.938cm、RMS2.166→1.405cm；
  前后半和正反向留出验证均改善。末帧0.507→0.510cm未改善，残余动态误差不能靠外参归零。
- 420组原始Odom/TF stamp与位姿一致；合法末点对应误差≤512ns，未发现明显时标错配。
  独立ROS域旧/新配置1x回放412组输出符合精确杆臂关系，Odom/TF一致，新静态TF正确。
  回放重初始化前缀不同，其最大5.477→3.770cm只作启动链验证，不混入原轨迹标定指标。
- 这是基于LIO的有效XY杆臂修正，yaw数据不能可靠标定Z及完整安装角度；尚待独立新录包
  验证。记录见 `odom_ws/src/ly_lio/docs/云台转心外参标定.md` 和该bag的
  `analysis/extrinsic_recalibration/`（保留旧/新外参、CSV/JSON、图和报告）。
  未改PTP、原始录包、内部滤波或两路转换话题，也未留下新增测试/回放脚本。
