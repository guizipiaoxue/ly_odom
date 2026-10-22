# ly_odom 工作说明

更新：2026-10-21。本文是当前维护约束；历史模型、调参、实机和失败实验已汇总到 [验证.md](odom_ws/src/ly_lio/docs/验证.md)，不再在本文件叠加相互冲突的历史“当前”说明。

**修改模型、初始化、观测、门控、地图或精度参数前，必须先读 [验证.md 的未采用方案与复试条件](odom_ws/src/ly_lio/docs/验证.md#12-未采用方案与复试条件)。没有新的故障证据、不同的模型机制和冻结的同源/独立留出验证，不得重复合入已失败原方案。**

## 文档入口与维护范围

- [ly建模.md](odom_ws/src/ly_lio/docs/ly建模.md)：当前代码的坐标约定、24维状态、传播/观测/雅可比、Batch信息解、初始化、地图和机体输出公式。
- [验证.md](odom_ws/src/ly_lio/docs/验证.md)：所有旧docs的汇总、日期/参数边界、失败方案、结果路径、原JSON与来源哈希。历史数字不是本轮新验收。
- `odom_ws/src/ly_lio/docs` 只保留以上两个文件；不恢复旧 `建模.md`、论文转录或独立调试文档/JSON。后续结果归入 `验证.md`，现行公式归入 `ly建模.md`。
- 修改本仓库算法时同时遵守包内 [AGENTS.md](odom_ws/src/ly_lio/AGENTS.md)。保持中文关键注释、MIT与第三方版权。参考实现只读，不从兄弟包动态引用源码。

## 工作区与数据流

ROS 2 Humble；仓库根目录 `/home/hustlyrm/ly_odom`。

| 工作区/目录 | 职责 |
| --- | --- |
| `livox_sdk` | Livox MID360驱动 |
| `pcl_ws` | 仅保留历史标定目录；独立转换包已删除，不参与启动 |
| `odom_ws` | 独立里程计包ly_lio及业务录包包ly_bag_recorder |
| `location_ws` | ly_locate在线历史关键帧回环/建图；map→baselink，不反馈LIO |
| `PTP` | 时间同步基础部署、启动配置；维护经验见 [AGENTS.md](PTP/AGENTS.md) |
| `bags` | 原始录包及各bag的analysis结果，不改写原始db3 |

MID360→驱动发布 /livox/lidar、/livox/imu；ly_lio直接读取原始测量，在IMU原点滤波，经固定安装外参输出云台中心Odometry。独立pcl_publisher已删除，不发布/bashlink/pcl或/baselink/imu。

主要接口为/ly_lio/odometry、/ly_lio/cloud_registered、/ly_lio/path、/ly_lio/scan_session、/tf_static以及Trigger服务/ly_lio/reset、/ly_lio/map_save。云台中心Odometry由真实Batch后验按odometry_publish_rate（默认100Hz、有限正数）选择输出，以扫描末为锚反向分槽；扫描末必发、同stamp去重，稀疏数据不补造样本。publish_odometry_per_batch=true发布全部真实批末状态。注册云仍按扫描末发布且在baselink中。LIO不再广播odom→baselink动态TF，保留baselink→livox_frame静态安装TF。整帧等IMU覆盖后集中处理发送，100Hz是传感器时间目标，不等于墙钟每10ms实时输出；不新增传播/预测，不用now重盖旧状态。

ScanSession随每个已选Odometry同stamp发布（reliable/transient_local/depth1000），覆盖全部扫描末云；header.frame_id仍为odom，表示会话固定参考系。reset/重建/重启换UUID。后端按完整纳秒stamp关联标签和T_OB，不插值/外推，不以到达顺序猜测；复用stamp跨会话或位姿冲突时保守拒绝。

## 当前模型（2026-10-17要求继续有效）

2026-10-22地图坐标约定：map固定为本次LIO静止标定和初图完成后首个真实Odometry对应的baselink系。`T_OB0=T_OI0*inverse(T_BI)`已含安装外参，`T_MO0=inverse(T_OB0)`；首关键帧先验左乘该变换，不把较晚关键帧误作首帧。ScanSession及LoopUpdate重复携带不可变origin_stamp/origin_pose，后端晚启动不改锚点；冲突拒绝。PCD/路径/TF/图位姿统一，内部odom及24维滤波不变。导出manifest v2记录固定原点，旧地图/会话消息不能直接混用。

- 状态/误差P真正24维 `[p,R,v,omega,a,g,bg,ba]`，流形 `SO(3)×R^21`；固定R_IL/t_IL只作参数，不保留30维外参占位，不在线估计，不接轮速。
- IMU采用角速度/比力输出观测，最多6维，达到配置量程95%逐轴屏蔽；不丢整条，不持续锁定速度、重力或零偏。a是IMU系比力，世界加速度为R*a+g。
- 激光逐点到历史地图五近邻PCA平面的标量残差；原点补偿到真实批末，同一先验计算各点残差/H并累加信息，一次联合更新完整24维P。H直接含p/R/v/omega十二列。
- MID360实际YAML和C++缺省为1ms，合法范围 `0<lidar_batch_duration<0.010s`，遇IMU/scan_end提前分界，同stamp先IMU。L2示例2ms，不套用MID360旧搜索结论。
- 先在区间起点状态传播P，再传播x；完整稀疏FPFᵀ保留交叉项。omega/a/bg/ba噪声仍按Q*reference_dt*dt，reference_dt=.005s；速度单独加rate*dt。
- `velocity_process_noise_rate`有限非负，单位m²/s³，代码缺省0、当前MID360 YAML明确0.1；旧velocity_cov停用，不自动迁移。改配置重启读取。
- 补偿是批末状态函数；激光H包含v/omega导数及正确姿态相消项，后验地图点/注册云重算。不能恢复冻结补偿却只对位姿求导的旧版。
- 保留原独立点噪声及目标PCA，不拟合源平面分组，不加2000点等观测上限；所有合格代表点参与，原预处理/关联门控仍有效。
- 当前扫描各批匹配同一个冻结历史地图，扫描末统一插入各批后验代表点，失败匹配点也可入图；后续批不回头平滑早批。真点降采样保留原坐标/时间对，不均值化时变激光点。
- 当前MID360采样.20m、地图.50m/每格128点/同格点距.05m、laserR=.001m²、plane_threshold=.01m、match_sqaured=25；运行以实际加载YAML/ROS覆盖为准，不以旧报告参数覆盖。
- 启动须保持静止：200样本SI门限估重力/bg及ba模长偏差，丢标定前扫描后继续静止收至少5帧/100点初图。运动则清初图重等，不恢复失败的连续初始化；单姿态不是全轴标定，极慢运动可能漏检。
- IMU实际缺段>.1s或队列溢出要求重标定/重建odom，并清旧Path/导出地图；原点会重设，下游不能当连续轨迹。固定外参和Q不变。
- 地图按固定参考处理，未建模持久地图误差、状态—地图交叉P、支持点共享噪声、随机批内轨迹或物理采样时延；滤波器内部无回环/绝对位置约束，独立ly_locate不回写状态/P，不能保证长期零漂或零误回环。

已失败并删除的方案包括10ms相邻平面窗/Schmidt、连续初始化、五支持点对角方差、批内共享GLS、稀疏源平面对平面。仅删PCA比值门控、2cm地图点距和特定噪声组合也未通过跨场景门槛。具体机制/证据/复试条件必须查验证文档，不能因参考项目有同名功能就恢复。

## 外参、安装中心与输出

- 内部始终 `p_I=R_IL*p_L+t_IL`。当前 `extrinsic_translation_convention=imu_in_lidar`：YAML的extrinsic_T是IMU原点在LiDAR系的位置c_L=`[.011,.02329,-.04412]`m，读取一次 `t_IL=-R_IL*c_L`。R_IL=I时内部t=`[-.011,-.02329,.04412]`m，不能重复取反。
- `extrinsic_R`总是LiDAR→IMU轴旋转；显式lidar_in_imu则直接将T解释为t_IL。旧extrinsic_est_en=false仅告警迁移，true及外参先验参数明确拒绝；当前不支持在线外参。
- 本包安装文件 `odom_ws/src/ly_lio/config/lidar2baselink.yaml` 的 `lidar_to_gimbal_rotation/translation` 表示固定T_BL。里程计组合 `T_BI=T_BL*inverse(T_IL)`、`T_WB=T_WI*inverse(T_BI)`，pose/twist P含完整24维交叉项和杆臂。
- 默认静态TF baselink→livox_frame严格为T_BL，不将非零偏移IMU原点误标为LiDAR原点。也可清空body_extrinsic_path并直接提供互斥的固定imu_to_body_R/T。
- 当前安装目标是云台中心；活动云台到真实底盘需要关节角和动态几何，不能用清零姿态或改名字替代。但共轴纯yaw固定中心的水平摆动仍是待解决误差。
- 旧yaw拟合只校正有效XY杆臂，不代表完整机械六自由度真值。当前安装平移 `[-.006009591012854143,-.14915516181160768,0]`m。

## 注册云与下游约定

/ly_lio/cloud_registered 为float32 XYZ；各Batch后验补偿到odom后统一以扫描末T_OB求逆转到baselink，stamp为真实扫描末点时间；loop直接下采样局部云，不重复求逆或施加安装外参。后端直接消费/ly_lio/odometry，不再订阅TF。loop只读取扫描末baselink注册云并保存局部关键帧；map只读取关键帧/完整图和轻量Odometry/会话标签，不订阅、转换或缓存实时整帧注册云。删除/location/cloud_registered及live_preview分支。固定map中逐帧按优化T_MB重建；已提交图的末关键帧给出T_MO=T_MB_k*inverse(T_OB_k)，同stamp当前位姿T_OB经T_MB=T_MO*T_OB直接发布map→baselink。odom仅保留为内部固定参考系和消息字段，不发布涉及odom的动态TF。未提交完整地图前没有map→baselink；旧图提交时允许同stamp发布修正更新。

旧odom系注册云不能直接混入新消费端，需从原始数据重算。原始IMU仍由LIO按原轴、原单位读取，不新增预先旋转/平移或扣重力的测量链路。

## 配置职责与统一启动

| 配置 | 读取者 |
| --- | --- |
| `odom_ws/src/ly_lio/config/mid360.yaml` | ly_lio节点/组件 |
| `livox_sdk/src/livox_ros_driver2/config/mid360.yaml` | 驱动ROS/PTP参数 |
| 同目录 `MID360_config.json` | SDK网络原生配置，驱动校验 |
| `odom_ws/src/ly_lio/config/lidar2baselink.yaml` | ly_lio直接读取固定安装外参 |
| `odom_ws/src/ly_bag_recorder/config/rosbag.yaml` | ly_bag_recorder直接读并使用原生rosbag2_transport::Recorder |
| `location_ws/src/ly_locate/config/location.yaml` | ly_locate的loop/map节点直接读取；ROS原生参数覆盖 |

业务YAML归属相关包的config目录，不放在仓库根目录或工作区外层。节点C++直接读默认配置，源码存在时优先源码、缺源码用share；ROS原生文件/CLI/组件参数覆盖。解析失败明确报错，日志记录实际路径，修改重启生效。SDK JSON是持久原生配置，不能由shell/YAML临时生成。

`scripts/odom.sh`加载Humble和已构建工作区，固定工作目录为仓库根，通过launch管理驱动/LIO/录包，不加载pcl_ws。已构建location_ws时只额外加载消息类型环境，回环/地图仍复用唯一ly_bag_recorder全话题录制，不新增Recorder或操作PTP服务。支持 `start_lidar`、`record_bag` 与 `use_sim_time` 开关；旧config/config_path/bag_dir/terminal/PTP launch注入已经取消。

```bash
./scripts/odom.sh
# 回放/外部驱动，不启动雷达、不查询PTP、不另录包：
./scripts/odom.sh start_lidar:=false record_bag:=false
# 独立实验用ROS原生覆盖：
ros2 run ly_lio ly_lio_node --ros-args --params-file /绝对路径/实验.yaml
```

scripts/map.sh 统一启动驱动/LIO/回环/建图/唯一Recorder；odom.sh 仍只启动里程计链路。两个入口共用进程锁，不能叠加运行。map节点默认save_on_shutdown=true，正常Ctrl+C/SIGTERM退出时把最新已收齐关键帧图重建并原子导出至maps/location_export；maps/location仅是回环归档。无有效关键帧跳过，缺帧/写盘失败明确报错，不把旧版本冒充最新；保存预留30秒宽限，等完整版本已保存再关终端。launch 统一位于 scripts/launch，公共加载逻辑在 scripts/lib/launch.sh；PTP服务不随ROS启停。回放须另加 use_sim_time:=true 并由外部发布/clock。

回放模式Livox已安装时加载其可选消息库，不启动驱动。默认全/隐藏话题持续发现录包到独立bags/odom_北京时间_随机后缀，不覆盖旧数据；静态TF reliable/transient_local，原传感器best_effort。录包不自动保存配置/参数快照或PTP硬件日志，应另存实际配置。任一采集进程意外退出时联动停止，其余节点Ctrl+C后等待Recorder刷缓存/metadata，退出宽限30秒；PTP继续独立运行。

## 时间同步和传感器单位

主机enp86s0/ptp0、接收192.168.1.50、目标MID360 192.168.1.3。复用既有sentry-ptp4l/phc2sys系统服务，不新建竞争授时实例，不修改系统UTC、不因启停ROS操作服务/时钟。驱动SDK前只读检查PTP/PHC及JSON接收IP；严格PTP未锁time_type=1就丢包，不回退收包时间。TAI→UTC按驱动所属配置减37秒。MASTER/s2不等于雷达锁定；历史验收不替代当前检查。

Livox PC2点时间=header+相对纳秒；CustomMsg=timebase绝对UTC纳秒+offset；IMU保留原始header。不能用now、接收时间或均匀5ms网格替换。PTP正确不等于物理LiDAR–IMU采样时延已标定。

本驱动加速度输入g，acc_norm=1；Sentry驱动已SI，接入时gravity模长与acc_norm均设9.80665，satu_acc同步设设备SI量程约29.42m/s²，不能再乘一次g或误把重力轴屏蔽。

## 构建和验证规则

优先增量构建，只构建业务包，不构建ROS1参考。首次驱动build.sh会删除其旧build/devel/install，应先核实必要性。

```bash
source /opt/ros/humble/setup.bash
cd /home/hustlyrm/ly_odom/livox_sdk
colcon build --packages-select livox_ros_driver2 --parallel-workers 1 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
cd /home/hustlyrm/ly_odom/odom_ws
colcon build --packages-select ly_lio ly_bag_recorder --symlink-install --parallel-workers 1
```

保留C++20/默认Release/OpenMP，不启用fast-math；本机指令优化只用已有可选开关。用户要求不保留新增测试/回放/监视/分析程序及二进制，必要临时工具在/tmp或stdin执行后清理；保留原始bag和结果/配置/图，不声称已删除的测试仍可运行。只改文档时检查公式/链接/清单，不冒充新构建或实机验收。

当前最近归档：Release、数值、58组ROS三联及三包8828输出降维回归通过；未启新硬件/PTP，降维等价不代表精度提升。所有旧30维/0.1ms/600秒数据按验证文档所属日期解释。未知采集尾差、驱动关闭异常、机械外参/动态时延/精度问题保留为未解决，不能通过隐藏失败窗口或重置轨迹包装为通过。
