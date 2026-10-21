# 统一启动入口

在仓库根目录二选一启动：

- ./scripts/odom.sh：MID360驱动、ly_lio、唯一全话题Recorder。
- ./scripts/map.sh：上述全部节点，以及Scan Context/GTSAM回环与历史关键帧建图。

两个脚本共享进程锁，不要叠加运行。PTP复用已有systemd服务，启动/退出ROS不操作授时服务。Ctrl+C联动退出，给Recorder预留30秒刷缓存和metadata；等待退出完成后再关闭终端。

map.sh退出时默认自动保存完整关键帧地图到maps/location_export/<session>_r<revision>_<random>/，包含map.pcd、局部keyframes、poses.csv、loops.csv和manifest.yaml。maps/location/是运行中的回环归档，和完整地图导出不同。保存开关save_on_shutdown及路径在location_ws/src/ly_locate/config/location.yaml；等待日志“完整版本已保存”及进程退出后再关闭终端。没有有效关键帧时跳过，缺帧/写盘失败明确报错。

目录：map.sh和odom.sh是用户入口；launch/odom.launch.py、launch/location.launch.py、launch/map.launch.py保存ROS进程编排；lib/launch.sh共享环境加载、构建检查和启动锁。各节点仍直接读取自己包内业务YAML，不由启动脚本生成或修改配置。

默认启硬件并录包到仓库bags/，参数与QoS统一归odom_ws/src/ly_bag_recorder/config/rosbag.yaml。

外部驱动：./scripts/map.sh start_lidar:=false

已有rosbag回放：./scripts/map.sh start_lidar:=false record_bag:=false use_sim_time:=true，另在加载全部工作区环境的终端执行ros2 bag play /绝对路径/原始bag --clock。回放包含旧/location输出时须只选择本次需要的原始输入，避免旧回环/map→baselink与新计算结果混发。

已有odom链路时只补后端：加载location_ws/install/setup.bash后，从仓库根执行ros2 launch scripts/launch/location.launch.py；不要再运行map.sh，不另开Recorder或第二个map→baselink广播者。

可用--help查看脚本说明，--show-args查看launch参数。三个公共参数为start_lidar、record_bag、use_sim_time。Recorder的接收时间策略仍由自己的YAML读取；不为算法启用/clock而偷偷改写录包配置。

当前动态TF只有建图后端的map→baselink，LIO输出100Hz目标云台中心Odometry并保留静态baselink→livox_frame。无独立点云转换/预览节点；map不订阅整帧注册云，只有loop接收以形成局部关键帧。odom.sh不启动后端，因此只有Odometry与静态TF，不输出全局动态TF。
