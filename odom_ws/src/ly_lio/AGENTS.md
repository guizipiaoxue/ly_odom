# ly_lio 维护说明

更新：2026-10-21。根目录 [AGENTS.md](../../../AGENTS.md) 的数据流、配置、PTP及构建规则同时适用。

**修改模型、初始化、观测、门控、地图或精度参数前，先读 [验证.md 的未采用方案与复试条件](docs/验证.md#12-未采用方案与复试条件)。没有新的故障证据、不同的模型机制和冻结的同源/独立留出验证，不得重复合入原失败实现。**

## 唯一文档入口

`docs`只保留 [ly建模.md](docs/ly建模.md) 与 [验证.md](docs/验证.md)。前者按当前源码解释公式，后者集中历史调试、验收与失败证据及原JSON；不要再新增旧式独立调试报告或恢复论文转录。README提供运行入口。历史报告的“当前”仅指记录日期，不覆盖当前代码/YAML及最新用户要求。

## 当前模型必须保持的语义

- 真正24维状态/P：`[p,R,v,omega,a,g,bg,ba]`、`SO(3)×R^21`。唯一SO(3)是R_WI，右扰动注入后对完整P及交叉块做切空间reset。固定R_IL/t_IL不是状态，不保留30维占位，不在线估计，不接轮速。
- IMU是六维输出观测：gyro-omega-bg、acc*scale-a-ba；a是比力、g是世界重力，预测加速度R*a+g。量程95%逐轴屏蔽H和残差，不整条丢弃；量程比较在输入原单位，不能提前扣重力。
- 按IMU/Batch/scan_end真实时间推进。先P后x，F在区间起点线性化，稀疏FPFᵀ保留完整交叉块。名义位置为p+=v*dt，不擅自把二阶位置积分写进现行说明。
- omega/a/bg/ba过程噪声仍Q*process_noise_reference_dt*dt，参考周期.005s；速度块额外velocity_process_noise_rate*dt。rate单位m²/s³，有限非负，代码默认0/MID360 YAML为.1，旧velocity_cov停用，不隐式迁移。
- MID360与C++缺省1ms，合法严格0<duration<.010s，L2示例2ms；遇下一IMU/扫描末提前分界，同stamp先IMU。批末取代表点/稠密点集合的真实最晚点，不造固定时间格。
- 激光逐点到历史地图五近邻PCA平面标量残差。保留原LiDAR点和相对批末dt，以补偿状态函数计算H，直接含p/R/v/omega十二列；姿态项必须消去Rᵀv*dt的虚假项。
- 一批冻结同一先验/地图/关联，逐点累加信息后只注入/reset一次完整24维状态/P；先验子块白化解十二维，秩亏观测不直接逆Lambda或完整P。
- 独立各向同性点噪声laser_point_cov，单位方差m²；共享补偿状态误差已经由H/P及跨点交叉项进入创新，不重复加到独立R。目标平面/地图不确定性尚未建模。
- 保留PCA重合/共线、lambda0/lambda1比值、支持点距离与原距离相关残差门控；match_sqaured原拼写/表达式保持。没有源平面分组、最少5源点条件或2000点上限，所有合格代表点参与。
- 后验地图/注册点按更新后R/p/v/omega重新补偿；同扫描各批历史地图冻结，scan_end统一插入代表点，失败匹配点也可入图；不回头平滑早批。
- 空间降采样保留真实坐标/时间对，不均值化不同时刻测量。匹配地图多真实点、7格候选、同格点距去重/满格替换/插入刷新缓存策略保持，导出均值地图不反馈估计。
- 当前MID360 YAML为采样.20m、地图.50m、每格128点、同格点距.05m、laserR=.001、plane_threshold=.01、match_sqaured=25；代码缺省/L2/旧报告不完全同值，实跑以实际文件/ROS覆盖为准。

## 外参和机体输出

内部始终p_I=R_IL*p_L+t_IL。当前缺省约定imu_in_lidar，配置extrinsic_T填c_L=`[.011,.02329,-.04412]`m，边界一次t_IL=-R_IL*c_L；R_IL=I时内部t=`[-.011,-.02329,.04412]`m。extrinsic_R只表示LiDAR→IMU轴旋转；显式lidar_in_imu直接使用t_IL。

固定T_BL由本包config/lidar2baselink.yaml给出，启动组合T_BI=T_BL*inverse(T_IL)。body位置/速度/pose与twist P均使用完整24维交叉P和杆臂；静态TF baselink→lidar_frame为实际T_BL，不误标IMU芯片原点。直接固定imu_to_body_R/T与安装文件互斥。reset/缺段重建不改变配置外参和Q。

旧extrinsic_est_en=false仅告警兼容，true和外参先验键启动拒绝。continuous_initialization、init_imu_duration、plane_noise_model已无实现，历史快照不能当现用示例。

## 启动、输出与工程约束

静止200IMU样本SI门限估bg、ba模长残差及g方向，丢标定前扫描后持续静止收至少5帧/100点初图；期间检测运动清图重等。当前不是连续传播初始化，无ZUPT/速度/姿态/偏置锁定。fix_gravity_direction是初始化开关，不是持续固定重力状态。

IMU实际缺段>.1s或队列溢出明确告警重标定/重建odom，节点清旧Path/导出图，不能把重置当漂移降低。循环回放需reset，不改传感器时间；默认订阅原始/livox/lidar与/livox/imu，独立转换节点已删除。IMU订阅best effort/depth1000。

Livox PC2为header+相对ns，CustomMsg为timebase绝对UTC ns+offset；g单位输入acc_norm=1，SI数据需相应acc_norm/gravity/satu_acc。严格PTP未锁丢包，不能回退收包时间。PTP服务独立，只读检查，不为回放启硬件/操作时钟。

云台中心Odometry由真实Batch后验按odometry_publish_rate（默认100Hz、有限正数）选择输出，以扫描末为锚反向分槽；扫描末必发、同stamp去重，稀疏数据不补造样本。publish_odometry_per_batch=true发布全部真实批末状态。注册云仍按扫描末发布且在baselink中。LIO不再广播odom→baselink动态TF，保留baselink→livox_frame静态安装TF。整帧等IMU覆盖后集中处理发送，100Hz是传感器时间目标，不等于墙钟每10ms实时输出；不新增传播/预测，不用now重盖旧状态。
ScanSession随每个已选Odometry同stamp发布（reliable/transient_local/depth1000），覆盖全部扫描末云；header.frame_id仍为odom，表示会话固定参考系。reset/重建/重启换UUID。后端按完整纳秒stamp关联标签和T_OB，不插值/外推，不以到达顺序猜测；复用stamp跨会话或位姿冲突时保守拒绝。
后端直接消费/ly_lio/odometry，不再订阅TF。loop只读取扫描末baselink注册云并保存局部关键帧；map只读取关键帧/完整图和轻量Odometry/会话标签，不订阅、转换或缓存实时整帧注册云。删除/location/cloud_registered及live_preview分支。固定map中逐帧按优化T_MB重建；已提交图的末关键帧给出T_MO=T_MB_k*inverse(T_OB_k)，同stamp当前位姿T_OB经T_MB=T_MO*T_OB直接发布map→baselink。odom仅保留为内部固定参考系和消息字段，不发布涉及odom的动态TF。未提交完整地图前没有map→baselink；旧图提交时允许同stamp发布修正更新。

终端10Hz独立wall timer读取同一次发布快照，B0为首有效完整body位姿，仅用于显示；不改变ROS/TF。Distance Traveled有10cm累计门限，GOOD不保证精度，DDS drops=n/a不可伪造0；pending面板标签是累计缓冲丢帧，不是当前队列。reset/重建清B0和上一轨迹显示。IO不持滤波锁，算法/reset互斥，保存服务读地图快照。

活动云台真正底盘输出需要动态关节外参，固定安装当前对应云台中心；共轴纯yaw时中心水平漂移仍是误差，不通过改名解释。ly_lio内部无回环、绝对真值或完整地图相关性模型；独立ly_locate后端只消费输出维护map→baselink，不回写24维状态/P。

## 构建、代码与验证

本包是独立ROS2 Humble/C++20/默认Release包，保留中文关键注释和MIT/第三方许可；Point-LIO、small_point_lio和外部Sentry PLIO只读参考，不动态引用源码，不构建ROS1参考。禁用fast-math，可选LY_LIO_NATIVE_OPTIMIZATION；标准PC2独立构建不强依赖Livox自定义消息。

```bash
source /opt/ros/humble/setup.bash
# 可选source Livox环境以启用CustomMsg；不启动驱动
cd /home/hustlyrm/ly_odom/odom_ws
colcon build --packages-select ly_lio --symlink-install --parallel-workers 1
```

默认参数由节点/组件C++直接读config/mid360.yaml，源码优先/share回退，ROS原生文件/CLI/NodeOptions覆盖。shell/launch只管进程，不解析/生成/注入参数；统一入口仅start_lidar/record_bag。回放用 `./scripts/odom.sh start_lidar:=false record_bag:=false`。

按用户要求不保留新增测试/回放/监视/分析源码和二进制，临时工具在/tmp或stdin运行并清理，保留原bag、配置、CSV/JSON、图和中文报告。验证结果统一记录日期、模型、实际参数、共同窗、计数及边界。先冻结候选再看留出，不在同一静止曲线无限调参，不删观测掩盖性能。

最近2026-10-17归档Release/数值/58组ROS三联/三包8828输出降维回归通过，仅证明实现/接口及相同固定模型等价；未采新硬件，不代表精度提升。旧24/30维、0.1ms/2ms和600秒静止数值不能作当前独立实机验收。失败方案及重新评估条件必须保留在验证文档，不以新的通过项覆盖旧失败。
