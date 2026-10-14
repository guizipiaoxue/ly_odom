# 独立录包节点

`ly_bag_recorder` 在C++节点内部直接读取仓库 `config/rosbag.yaml`。默认从ament当前包share目录逐级向上查找包含 `odom_ws/src/ly_bag_recorder` 的源码仓库，优先使用其外部配置；不依赖编译时的源码绝对路径。找不到源码配置时读取包share下的安装副本。显式指定但无效的配置会启动失败，不静默回退。launch只负责启动和退出宽限，不解析或生成配置。

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

输出目录和存储插件配置的相对路径均以实际读取的YAML父目录为基准；默认 `../bags` 指向仓库bags目录。移机只部署安装包时，安装副本的相对路径指向包share下的bags，按部署需要修改副本或显式设置绝对 `output_dir`。每次生成独立的 `odom_北京时间_随机后缀` 子目录，原生writer拒绝覆盖既有包。

节点直接使用Humble的 `rosbag2_transport::Recorder` 和 `rosbag2_cpp::Writer`。默认持续发现全部话题及隐藏话题，原始传感器使用best effort，`/tf_static`使用reliable/transient local；未覆盖的话题由rosbag原生QoS适配。消息内容及header保持原值；bag接收时间沿用原生Recorder语义。配置中的话题列表与QoS话题必须使用完整绝对名称。

禁用录包键盘处理。Humble的Recorder必须接收非空KeyboardHandler；本节点提供不启用终端的处理器，不读取stdin、不改变终端模式，也不安装信号处理器。SIGINT/SIGTERM由rclcpp结束ROS spin后，在主线程调用原生 `stop()` 刷新缓存并写出metadata；统一launch继续提供30秒退出宽限。异常返回非零，供统一launch关闭采集。没有自行实现订阅、序列化或Python录包循环，也不启动或操作PTP。
