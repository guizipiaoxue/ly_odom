# ly_lio 维护说明

**修改模型、初始化、门控或精度参数前，先读[未采用方案记录](docs/未采用方案记录.md)。
没有新的故障证据、模型机制和同源/留出验证，不得重复合入已失败的原方案。**

- **2026-10-17本轮要求（优先于全部下文历史）**：真正24维状态/P为
  `[p,R,v,omega,a,g,bg,ba]`，流形为`SO(3)×R^21`。激光→IMU外参仅为固定配置，
  不保留30维状态占位、不估计外参；IMU仍为6维输出观测，逐轴95%饱和屏蔽，不融合轮速。
  激光保留每点到历史地图PCA平面的标量残差。原始点统一补偿到实际批末，各点在同一
  先验下计算残差/H并累加联合信息，一次更新完整24维状态/P；H包含`p/R/v/omega`
  12列，独立点噪声保持原定义。当前MID360及C++缺省为1ms，遇IMU/扫描末端提前分界，
  同stamp先IMU。具体观测与时间语义见`docs/建模.md`“本项目当前实现约定”。
  本轮不引入平面对平面观测，不恢复失败的10ms相邻窗/Schmidt克隆或平面噪声实验。
  本轮Release、独立数值、58组ROS三联和三份录包8828输出同源回归通过，匹配/地图计数
  一致，状态/P投影在数值容差内；报告docs/24维Batch与固定外参验证.md。
  未启硬件或操作PTP，降维等价不代表精度提升，不移用历史精度作为当前实机验收。
- **固定外参约定**：内部`p_I=R_IL*p_L+t_IL`；用户本轮选定
  `extrinsic_translation_convention=imu_in_lidar`，配置`extrinsic_T`是IMU原点在LiDAR系
  的位置`c_L=[.011,.02329,-.04412]m`，读取时只做一次`t_IL=-R_IL*c_L`。
  固定`T_BL`推导固定`T_BI`，机体输出
  用完整24维P及杆臂雅可比，不包含在线外参协方差。下文30维、外参状态/先验、在线动态
  传感器TF描述均为历史；本轮数值/ROS结果见新报告，不移用旧测试数字。
- **配置与结果边界**：以节点实际加载的YAML和ROS覆盖值为准。历史0.1ms、每格点数、
  地图分辨率和噪声参数不是本轮新模型的验收证据；不以旧600秒静止结果证明新模型通过。

- **2026-10-16连续启动/平面观测历史实验（未通过，代码已删除）**：连续启动、PCA支持点
  对角方差、批内共享支持点GLS及其诊断接口已删除；continuous_initialization、
  init_imu_duration、plane_noise_model不再是可用参数。历史参数仅用于解释归档实验。
  当时保留原启动与独立点噪声、所有合格观测、原PCA、30维/0.1ms/初始P/rate0.1/
  地图/采样/外参，不加点数上限；这些是历史实验基线。
  124845共同窗闭合14.625→15.018/15.087/15.371cm，10cm运动目标未达到；三启动包
  未过1cm；新600秒静止步长/速度只降约4%/12%，位置漂移退化，Sentry运动回归超+5cm。
  因此删除失败实验。删除前默认关闭数值逐位兼容、导数/稠密GLS/2101全点/30组三联ROS/
  参数覆盖/非法值/reset及Release构建曾通过，不代表功能仍存在或精度通过。
  串行短窗性能不代替完整内存/压力验收。
  本轮授权启实机采集后已停止，PTP不变，驱动仍已知-11；公共600秒6000组三联一致，
  全bag录包存在16条Odom/TF尾差，不能宣称全序列三联数量相同。报告见
  docs/连续启动与平面观测模型验证.md，结果见124845包analysis/model_repair_20261016。

- **2026-10-16距离优先修复（历史模型验收，参数语义保留）**：velocity_process_noise_rate单位m²/s³，
  要求有限非负，代码默认0、MID360采用0.1。Q速度块为rate/reference_dt，使速度
  协方差直接增加rate*dt；其余四组噪声/传播不变。旧velocity_cov仍停用并告警，
  ROS文件/CLI可覆盖，重启读取，reset和IMU缺段重建保留Q。无新增观测上限或采样变更。
  预定0/.03/.1中.03因Sentry闭合降幅89.986597%未达90%，.1两包闭合
  .262327/.303594m（原5.42877/4.74658m），PLIO位置差RMS .33133/.71639、
  最大.78814/1.56243m。PCA-off虽闭合改善，LY最大差退化，保留原PCA条件。
  冻结.1+原PCA后的运动留出闭合.278997→.136635m；近静止闭合.005015→.004986m，
  最大/RMS增加2.60/1.06mm（允许+1cm），全部通过。姿态仅记录，无状态锁定或限幅。
  两包rate0归档54字段与旧基线全段相同；完整176566事件状态/P/Q/计数与独立旧Q
  机制逐位相同；.1与旧Qv20全段54字段相同。180组稠密传播与参数/reset/缺段检查、
  66组生产ROS三联stamp通过，12组完整回放无新增丢帧/重建。L2保持原配置。
  当前说明docs/距离优先速度噪声修复.md，结果在问题LY包analysis/distance_repair_20261016。
  未启动硬件或操作PTP，PLIO为比较参考而非真值；历史零漂结果不是本配置验收。

- **2026-10-15点到面补偿状态导数历史（优先于更早冻结补偿历史）**：Batch保留原点和相对批末时间，
  补偿依赖批末v/omega/R/外参；H覆盖前18维[p,R,R_IL,t_IL,v,omega]，固定外参选择
  [p,R,v,omega]12维求解，在线外参18维求解，状态/P仍30维。共享补偿状态误差通过完整
  P与交叉项进入联合更新，不重复加入独立测量R。姿态H消除R^T*v*dt项，后验入图与注册云
  用更新后的状态重算补偿。当前仍采用批内匀速/匀角速模型；独立批内随机轨迹、地图和时间
  不确定度未建模。公式及本轮验证见docs/Batch运动补偿状态雅可比.md。
  Release构建、600组状态有限差分、240组稠密Kalman、8组生产PCA更新及4种核心扫描
  配置通过；结果见docs/Batch运动补偿数值验证.json。未测新零漂，临时程序已清理。

- **2026-10-13模型历史（已被文首24维要求替代）**：恢复Point-LIO30维状态/P顺序
  [p,R,R_IL,t_IL,v,omega,a,g,bg,ba]；IMU观测仍6维，Batch保持0.1ms，不融合轮速。
  extrinsic_est_en=false清零外参P行/列并固定，true恢复12列激光观测、外参注入和双SO(3)reset。
  MID360的extrinsic_T保存IMU在激光系位置[-.011,-.02329,.04412]m，约定imu_in_lidar；
  参数边界转换t_IL=-R_IL*c_L，内部始终p_I=R_IL*p_L+t_IL。L2显式lidar_in_imu。
  在线Batch保留原点/D，后验云/入图点与观测使用同一模型；reset恢复启动标定。
  固定T_BL安装在线组合T_BI，body pose/twist P含外参交叉项；staticTF是实际T_BL，child为lidar_frame。
  显式固定T_BI+在线T_IL则发布同stamp动态sensor TF。说明见docs/30维状态恢复与IMU安装外参.md。
  以下24维/禁止在线分支/芯片原点未标定描述均已被本项替代；历史零漂结果不作为本轮验收。
  本轮Release构建、924次ESKF数值核对、256组生产机体输出协方差有限差分及三种ROS
  合成配置通过；108组Odom/TF/注册云stamp一致，TF值差0，reset恢复与SIGINT退出正常。
  结果见docs/30维数值验证.json、30维外参与机体输出验证.json、30维ROS验证.json。
  未启动硬件或修改PTP，未测新零漂，临时程序/数据已清理。

- **2026-10-13分包配置加载（加载职责保留，参数值按文首与实际YAML）**：节点C++直接读取包内config/mid360.yaml，包含算法/IMU/地图/
  初始化/话题/frame/输出/终端等参数；不再读取根config/ly_odom.yaml（已移除）。
  Livox参数与SDK JSON在其驱动包，录包配置独立在工作区外config/rosbag.yaml。
  shell/launch只管理进程，不解析/生成/注入参数；独立ros2 run与组件加载同样读默认YAML。
  ROS显式参数优先于默认文件。安装外参两消费者直接共用pcl_ws/config/lidar2baselink.yaml。
  用户选定Batch=.0001s（0.1ms），严格0<duration<.010；L2示例未调参仍2ms。
  24组搜索最小RMS的5µs不作为默认；采用100µs相对2ms在600秒同源位置RMS降低6.25%，
  两静止/一旋转回归也小幅改善位置，部分姿态略退化，无新实机采集。
  配置链及结果见docs/统一参数配置.md、docs/Batch时长调参与零漂.md。
  四业务包构建与离线直接读取/原生覆盖/组件加载/录包内容及退出检查通过；
  未启动硬件、未改PTP。指标见docs/分包配置验证.json，不是新零漂测量。
- 本包是独立 ROS 2 Humble 包；Point-LIO、small_point_lio及外部sentry PLIO为只读参考。
  不从兄弟目录动态引用源码，保留MIT和第三方版权。关键算法、适配和节点逻辑使用中文注释。
- 首读README、docs/建模.md、docs/统一参数配置.md；docs/实现对照.md、
  docs/时间戳与发布频率.md和docs/双雷达与单雷达对比.md含历史审计，不覆盖文首要求。
- **2026-10-12历史：按docs/建模.md实现Batch点到历史地图联合更新和IMU输出观测。**
  不接轮速；状态和P均为真正24维[p,R,v,omega,a,g,bg,ba]，不保留30维存储空壳。
  在线外参载体、12列更新、外参误差注入/先验参数已删除；旧extrinsic_est_en=true启动拒绝。
  384次数值投影比较及738条旧bag输出逐数等价；点距.02候选动态退化，保留默认.05。
  最新调参和600秒静止测量见docs/24维清理与10分钟零漂.md。
  213853独立实机预热5s+600s：末端0.6215cm、最大1.1963cm、RMS0.5098cm、姿态0.021623°。
  6000组Odom/TF/注册云一致，窗内无重建；启动首输出前IMU缺段曾重建一次。
  转换云全齐，IMU录包旁路少1个配对（119999/119998），不能直接归因为转换漏发。
  新600s同源回放原.05/候选.02点距末端为0.5609/0.9309cm，仍保留原参数。
  Batch最长lidar_batch_duration=.002s，遇IMU事件提前分界，同stamp先IMU；统一补偿到
  实际批末IMU系，冻结先验的omega/R^T v，联合信息更新后只用后验R/p变世界。
  详见docs/文档模型改进与零漂验证.md。
  相邻10ms平面→平面模型已因静止实机严重发散停用，plane_features/Schmidt克隆不再参与流程。
  旧失败和历史结果见docs/PLIO对比与零漂修复.md，不能宣称它们是当前验证。
- 按真实时间处理IMU、激光Batch、scan_end；P先于x传播以使用区间起点线性化；
  Q*process_noise_reference_dt*dt默认参考周期.005s，稀疏FPFᵀ保留完整交叉块。
- 激光为每点到历史地图5近邻PCA平面的标量残差，通过原PCA和匹配门控才参与观测。
  同一Batch冻结先验，逐点计算残差/H后合并信息，一次联合更新完整24维状态。
  同扫描后验点到scan_end才入图；保留7格查询与LRU，格长/每格容量/最小点距以实际配置为准。
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
  T_WB=T_WI*inverse(T_BI)。也可直接给固定imu_to_body_R/T；不支持在线T_IL估计。
  节点通过核心只读接口读取固定T_IL，使用固定T_BL组合T_BI。
  位置、速度、协方差、Path、TF共同核对杆臂；注册点云仍在odom，不重复变换。
- 默认scan_end发布Odom/TF/注册点云，stamp为距离过滤前最晚有效输入点。
  publish_odometry_per_batch=true可按真实批末额外发布Odom/TF，仍按原始消息成批交付。
  旧publish_odometry_without_downsample=true告警映射为按批；velocity_cov已停用并告警。
  Q的omega/a/bg/ba仍用原参考周期离散；2026-10-16起可选直接速度方差率见文首。
  IMU按配置量程95%屏蔽饱和轴。
  当前无独立实时IMU预测输出，不把约10Hz发布误写为10Hz激光更新。
- 原始frame名相同不能证明lidar/IMU原点重合；当前应用用户给出的芯片位置，旋转仍假设同轴。
  活动云台真实底盘姿态需要动态关节外参；安装文件当前对应云台中心。
- 算法和reset互斥回调组，save服务独立组使用地图快照；没有回环/绝对位置约束。
- 统一启动scripts/odom.sh：PTP开机服务仅检查，驱动/转换/里程计/全话题录包一起启动。
  回放用start_lidar:=false record_bag:=false。PTP不随ROS停止或重启。
- 构建：source Humble；可选source Livox启用CustomMsg，再在odom_ws运行
  colcon build --packages-select ly_lio --symlink-install --parallel-workers 1。
  不构建ROS1参考包；禁用-fast-math，默认Release，可选LY_LIO_NATIVE_OPTIMIZATION。
- 用户要求不保留新增测试/回放/监视/分析程序；临时程序在/tmp或stdin运行并清理，
  只保留原始bag、配置、CSV、JSON、图、中文报告。历史测试数字不是本轮验收。

## 历史Batch模型验证（2026-10-12，不自动覆盖本轮24维固定外参配置）

- 后续18组有限调参、24/30维同源及留出比较见docs/调参与30维状态对比.md；
  候选未稳定泛化，该轮保留原噪声与固定24维。该历史轮用既有bag，未启动硬件；
  在线30维代码现已按用户要求删除。
- Release构建、80组联合/稠密Kalman等价、SO(3)/传播/观测雅可比数值核对通过，
  7项核心时间顺序、空点云、地图冻结、输出与参数流程检查通过。
- 旧静止bag同源60秒：旧末端0.910cm/最大2.405cm，新1.002/2.382cm，未全面改善。
- 新独立实机bag odom_20261012_190342_53aa7a1c：首输出后5s预热+60s，600输出，
  末端0.309cm/最大2.246cm，姿态末端0.01563°。654组Odom/TF/注册云stamp一致。
- 实机窗内原始/转换IMU12000/12000、云599/599；完整bag的启动发现前缀有计数差。
- 新bag同源离线旧末端0.237cm→新0.292cm；不能将不同初始化的现场输出当同源旧/新。
- 本轮模型稳定但不宣称全面改善静止零漂或动态精度；详见新报告和各bag/analysis。
- 采集已停止，里程计/转换/录包正常退出，驱动已知SIGBUS(-7)仍在；PTP active、PID不变。

## 修改前逐点模型验证（本轮Batch模型见新文档）

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
- 面板参数直接在config/mid360.yaml的terminal.*设置，或通过原生ROS参数覆盖。TTY默认持续清屏，文件/管道无ANSI；clear_screen=false保留其他节点日志。
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
