# 独立录包节点

`ly_bag_recorder` 在C++节点内部直接读取本包 `config/rosbag.yaml`（仓库路径 `odom_ws/src/ly_bag_recorder/config/rosbag.yaml`）。默认从ament当前包share目录逐级向上查找包内源码配置，源码优先；不依赖当前工作目录或编译时的源码绝对路径。找不到源码配置时读取包share下的安装副本。显式指定但无效的配置会启动失败，不静默回退。launch只负责启动和退出宽限，不解析或生成配置。

```bash
source /opt/ros/humble/setup.bash
cd odom_ws
colcon build --packages-select ly_bag_recorder --symlink-install
source install/setup.bash
ros2 run ly_bag_recorder ly_bag_recorder
```

可用原生ROS启动参数 `config_file` 指定另一份配置，`output_dir` 覆盖保存父目录：

```bash
ros2 run ly_bag_recorder ly_bag_recorder --ros-args \
  -p config_file:=/绝对路径/rosbag.yaml -p output_dir:=/绝对路径/bags
```

输出目录和存储插件配置的相对路径均以实际读取的YAML父目录为基准；源码配置默认 `../../../../bags` 指向仓库bags目录。移机只部署安装包时，相对路径仍按实际安装的YAML位置解析，按部署需要修改配置或显式设置绝对 `output_dir`。每次生成独立的 `odom_北京时间_随机后缀` 子目录，原生writer拒绝覆盖既有包。

节点直接使用Humble的 `rosbag2_transport::Recorder` 和 `rosbag2_cpp::Writer`。默认持续发现全部话题及隐藏话题，原始传感器使用best effort，`/tf_static`使用reliable/transient local；未覆盖的话题由rosbag原生QoS适配。消息内容及header保持原值；bag接收时间沿用原生Recorder语义。配置中的话题列表与QoS话题必须使用完整绝对名称。

回环与地图录包也统一归本包负责，不在 ly_locate 中另起 Recorder。包内配置继续使用 `all: true`、空排除条件、隐藏话题和持续发现，因此包含后启动的 `/location/*` 以及其他业务话题；QoS 表只是订阅策略覆盖，不是录制白名单。已显式配置 `/ly_lio/scan_session`、`/location/loop`、`/location/map`、`/location/path` 的 transient_local，以及关键帧和实时注册云的 volatile。

统一入口 `scripts/odom.sh` 在 location_ws 已构建时加载其消息类型环境；不会因此启动额外的回环/建图节点或录包进程。使用统一入口时不要再执行本页的独立 `ros2 run` 示例，也不要另起 `ros2 bag record -a`。独立启动 Recorder 且需要录制回环数据时，应先加载 location_ws/install/setup.bash。录包晚加入只能取得发布者实际保留的持久消息；启动前已错过的 volatile 原始扫描/关键帧不能由录包补回。

禁用录包键盘处理。Humble的Recorder必须接收非空KeyboardHandler；本节点提供不启用终端的处理器，不读取stdin、不改变终端模式，也不安装信号处理器。SIGINT/SIGTERM由rclcpp结束ROS spin后，在主线程调用原生 `stop()` 刷新缓存并写出metadata；统一launch继续提供30秒退出宽限。异常返回非零，供统一launch关闭采集。没有自行实现订阅、序列化或Python录包循环，也不启动或操作PTP。

完整建图统一入口为 scripts/map.sh，复用 scripts/launch/odom.launch.py 中唯一的 Recorder；scripts/odom.sh 用于不启回环建图的里程计链路。两个脚本共享启动锁，不叠加运行，不启停既有PTP服务。所有launch位于scripts/launch。
