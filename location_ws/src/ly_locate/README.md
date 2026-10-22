# ly_locate 在线回环与建图

更新：2026-10-22。ROS 2 Humble，C++20，默认Release，无预建地图。re_location.cpp保留空占位，不参与构建。

## 数据流与坐标

云台中心Odometry由真实Batch后验按odometry_publish_rate（默认100Hz、有限正数）选择输出，以扫描末为锚反向分槽；扫描末必发、同stamp去重，稀疏数据不补造样本。publish_odometry_per_batch=true发布全部真实批末状态。注册云仍按扫描末发布且在baselink中。LIO不再广播odom→baselink动态TF，保留baselink→livox_frame静态安装TF。整帧等IMU覆盖后集中处理发送，100Hz是传感器时间目标，不等于墙钟每10ms实时输出；不新增传播/预测，不用now重盖旧状态。

ScanSession随每个已选Odometry同stamp发布（reliable/transient_local/depth1000），覆盖全部扫描末云；header.frame_id仍为odom，表示会话固定参考系。reset/重建/重启换UUID。后端按完整纳秒stamp关联标签和T_OB，不插值/外推，不以到达顺序猜测；复用stamp跨会话或位姿冲突时保守拒绝。

后端直接消费/ly_lio/odometry，不再订阅TF。loop只读取扫描末baselink注册云并保存局部关键帧；map只读取关键帧/完整图和轻量Odometry/会话标签，不订阅、转换或缓存实时整帧注册云。删除/location/cloud_registered及live_preview分支。固定map中逐帧按优化T_MB重建；已提交图的末关键帧给出T_MO=T_MB_k*inverse(T_OB_k)，同stamp当前位姿T_OB经T_MB=T_MO*T_OB直接发布map→baselink。odom仅保留为内部固定参考系和消息字段，不发布涉及odom的动态TF。未提交完整地图前没有map→baselink；旧图提交时允许同stamp发布修正更新。

局部云表达的是扫描末云台中心（当前frame名baselink），不是活动底盘。完整地图保持固定map参考系，不能把不同时刻的局部云直接相加。后端不回写24维滤波器。

### 固定map参考系

地图固定参考系约定（2026-10-22）：令`t0`为LIO静止标定、初图完成后本会话首个真实Odometry的时间。LIO先组合安装外参`T_BI=T_BL*inverse(T_IL)`，得到`T_OB0=T_OI0*inverse(T_BI)`；map定义为该时刻的baselink，固定初始变换`T_MO0=inverse(T_OB0)`。图的首关键帧先验为`T_MO0*T_OB_k`；它可能晚于`t0`，不能强制清零该关键帧。无回环时`T_MB(t)=T_MO0*T_OB(t)`，首个LIO机体位姿为单位阵；回环后仍在这个固定map中优化历史。map轴向跟随`t0`的机体安装轴，不额外对齐重力。

`ScanSession`每条消息携带`body_frame/origin_stamp/origin_pose`（源会话首个`T_OB0`），`LoopUpdate`重复携带固定原点。后端晚启动、首帧云无效或丢帧不重新选择原点；同源原点冲突拒绝建图。map.pcd、路径、图位姿和map→baselink统一使用该坐标系；局部关键帧仍在各自采集时刻的baselink。session.yaml和manifest.yaml记录原点整数纳秒及固定`map_from_odom_translation/quaternion_xyzw`（double 17位），该固定定义不同于回环后由末关键帧计算的实时修正。导出格式为`ly_locate_keyframes_v2`；旧会话消息/地图不能直接混用，应从原始传感器数据重算，不能仅修改frame_id。LIO内部odom、24维状态与P不受此坐标定义改变。

## 防误回环

loop.cpp独立维护历史局部关键帧、Scan Context和GTSAM iSAM2图。关键帧须满足最短时间间隔，并达到平移或旋转阈值；没有静止定时关键帧，防止静止重复确认。关键帧在线保存PCD；可通过服务补取。

接纳顺序：

1. 时间≥30s、累计行程≥15m、排除最近30关键帧，优化轨迹候选半径≤8m。
2. 20环×60扇区最大高度Scan Context，ring key选20候选，循环列余弦距离；信息bin/扇区不足拒绝。SC≤0.10；不同位置次优候选须通过0.03间隔与0.75比率。yaw相隔≥30°仍有近似最优解也拒绝。
3. 当前单帧到独立历史单帧的正反ICP，避免目标混入当前云。双向重叠≥70%、各≥150内点、RMSE≤0.10m；不使用单个fitness替代双向重叠。
4. 正反闭合≤0.08m/0.035rad；相对预测和SC初值修正≤1.5m/0.35rad；点到平面信息矩阵须满秩，单墙/地面等退化拒绝。只是后端门控，不改ly_lio观测模型。
5. 至少3个连续当前关键帧、不同历史关键帧提供一致修正，确认期间运动≥1m；允许正向/反向回访。一致性≤0.25m/0.08rad，随后10关键帧冷却。
6. 约束用Cauchy核加入GTSAM副本，优化后核查新回环和全部历史因子残差，异常整次回滚。图的sigma为独立后端工程权重，不冒充ly_lio完整相关协方差。

默认配置是待真实录包冻结验证的保守起点，不能保证零误回环。重复大棚/走廊结构可能同时通过所有几何检查。单帧MID360稀疏、视场差异、非水平运动、大角度或大漂移将增加漏检。当前没有多帧子图描述、全局重定位、人工审核或绝对约束，不能以闭合变好作为安全验收。

## ROS接口与异步处理

| 接口 | 类型 | 语义 / QoS |
| --- | --- | --- |
| /location/loop | ly_locate/msg/LoopUpdate | reliable/transient_local/depth1，完整图快照，每关键帧发布，含session/revision、原始/优化位姿、时间和累计约束 |
| /location/keyframe | ly_locate/msg/Keyframe | reliable/depth10，body局部XYZ，id/session/stamp关联 |
| /location/get_keyframe | ly_locate/srv/GetKeyframe | 按session/id补取，支持地图晚启动/遗漏关键帧；仅当前进程当前会话历史 |
| /location/map | sensor_msgs/msg/PointCloud2 | reliable/transient_local/depth1，单完整图版本重建的关键帧地图，map坐标 |
| /location/path | nav_msgs/msg/Path | reliable/transient_local/depth1，和地图同图版本，历史采样stamp |
| /tf | tf2_msgs/msg/TFMessage | 仅map→baselink，T_MO修正真实100Hz目标Odometry；不订阅TF |
| /location/map_save | std_srvs/srv/Trigger | 保存完整一致版本到新目录，缺帧/尚未重建完成返回失败 |

LoopUpdate.header.stamp为末关键帧采样时刻，绝非优化完成now。loop_closed只标记本次版本新接纳事件；累计constraints保留历史回环。LoopConstraint.relative_pose为T_Bfrom_Bto，带SC、RMSE、overlap、accepted_revision。完整快照允许跳过中间版本，旧revision/退休session拒收。

注册云发布/loop订阅DDS队列深度均为10，独立于高频轻量Odometry。map不保留实时云；持久局部关键帧和完整地图仍占必要内存。

map/path各消息完整但跨话题非分布式原子到达，版本看图快照。源会话切换清图隔离旧原点，已有TF缓存不能由本协议撤销。odometry_wait_seconds控制精确stamp等待超时。

## 推荐数据格式

主地图资产采用“关键帧局部XYZ二进制压缩PCD + 纳秒stamp/原始与优化SE(3)位姿 + 图版本/约束”，不只保存合并PCD。ly_lio没有强度输出，不虚构XYZI；位姿CSV使用double 17位，stamp_ns用整数。

- loop自动归档：maps/location/<session>/keyframes/<id>.pcd、poses.csv、loops.csv、graph.g2o、session.yaml。每文件原子替换，session.yaml最后写；崩溃可能发生在文件之间，不宣称整个目录事务一致。当前不自动恢复iSAM2会话。
- map_save导出：maps/location_export/<session>_r<revision>_<random>/map.pcd、keyframes/*.pcd、poses.csv、loops.csv、manifest.yaml。先写临时新目录，完整后rename，不覆盖旧地图。保留局部云才能再修正历史。
- 默认save_on_shutdown=true：Ctrl+C/SIGTERM正常退出时停止工作线程并自动导出最新收到的完整图到同一location_export目录，无需退出后调用ROS服务。即使周期重建尚未发布，退出也直接重建已齐全的最新版本；缺关键帧或写盘失败明确报错并非零退出，不回退旧版本冒充最新。没有有效关键帧时跳过，不生成空地图。launch预留30秒保存宽限，等待“完整版本已保存”后再关闭终端；SIGKILL/断电不能触发保存，最后尚未送达map节点的后端更新不保证纳入。可通过location.yaml关闭自动保存。
- graph.g2o信息顺序是translation/rotation，GTSAM是rotation/translation；g2o不保存Cauchy核，读取时须重新设置，不等价于iSAM2序列化。
- 原始证据继续使用现有rosbag2 SQLite3(db3)：保留原始激光、IMU、/tf、/tf_static及新session/loop输出。默认业务Recorder自动发现新话题；不改写原db3。PCD不能替代带时间和原始传感器的bag。
  全话题录制和 QoS 统一配置在 odom_ws/src/ly_bag_recorder/config/rosbag.yaml；本包不启动第二个 Recorder。scripts/odom.sh 已有录包节点时，只启动本包 loop/map 即可；不要重复运行 ros2 bag record 或独立 ly_bag_recorder。统一入口会加载已构建 location_ws 的消息类型环境。

完整快照/整图重建面向中小规模地图。最多20000关键帧；容量、磁盘或数值错误停止当前会话并报错，不静默删除历史。局部云常驻内存，图副本支持回滚，计算/内存随历史增长；无分层子图、磁盘缓存或大图性能验收。队列溢出告警，关键帧地图也不等于全扫描无损地图。

## 构建与启动

~~~bash
cd /home/hustlyrm/ly_odom
source /opt/ros/humble/setup.bash
source livox_sdk/install/setup.bash
cd odom_ws
colcon build --packages-select ly_lio --symlink-install --parallel-workers 1
source install/setup.bash
cd ../location_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-select ly_locate --symlink-install --parallel-workers 1
source install/setup.bash
cd ..
# 完整链路：驱动、LIO、回环、建图及唯一全话题Recorder。
./scripts/map.sh
# 外部回放（另行 ros2 bag play --clock），不启动硬件、不重复录包：
./scripts/map.sh start_lidar:=false record_bag:=false use_sim_time:=true
# 已有 odom 链路时只补回环/建图，不再执行 map.sh：
ros2 launch scripts/launch/location.launch.py
ros2 service call /location/map_save std_srvs/srv/Trigger '{}'
~~~

两个C++节点直接读config/location.yaml，源码优先/share回退，ROS原生--params-file/-p覆盖，日志记录路径，重启生效。launch统一放在scripts/launch，仅管节点与use_sim_time。不同时运行多个map→baselink广播者。map.sh与odom.sh共用进程锁，二选一启动；进程退出时内核自动释放锁。

本机实际构建采用 /opt/ros/humble 的GTSAM 4.2a9及其配套库。另有/usr/local GTSAM 4.3.2；切换时头/库必须一致并重建，不混用ABI。

## 本轮验证边界

最初构建/合成验收与后续600秒完整链路实机观察，分别记录在 odom_ws/src/ly_lio/docs/验证.md 第16、17、19节。600秒实机已运行并保存bag，但只有1关键帧、0回环，没有真实运动回环留出验收；启动一次TF超时和Livox退出段错误仍保留。复用既有PTP服务且未启停/改时钟，不能宣称实场精度提升或误回环率为零。临时验证通过stdin执行，不保留新增测试/分析程序。
