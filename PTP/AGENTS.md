# PTP 维护规则

- 目标设备 MID360 192.168.1.3；当前有线网卡 enp86s0，主机 192.168.1.50，PHC ptp0。
- 先读 README；已有 sentry PTP 服务时复用，不能与 ly-ptp 服务同时调节同一 PHC。
- 默认 IEEE 1588v2、UDPv4、E2E、domain 0、hardware、固定 MASTER。
- 授时方向 CLOCK_REALTIME→PHC→雷达。不要修改系统时间、NTP 服务或任意网卡地址。
- PHC 初始化只在采集前；运行伺服 -F 0 -S 0 禁止阶跃。UTC/TAI 时差不能随意置零。
- 原始包 time_type=1 和 ROS 时间检查都要通过，不能以 MASTER/s2 替代雷达验收。
- Livox launch 默认严格 PTP，非 PTP 包入队前丢弃，不退回主机收包时间。
- 本机实测设备 PTP 为 TAI，launch ptp_utc_offset=37，两路统一转UTC；其他固件先测量再配置。
- PointCloud2 timestamp 应为绝对采样时间减帧基时的相对纳秒；目标IP只收配置的 .3。
- ROS稳态实机验收已通过，原始包抓取仍需sudo；驱动停止后观察到上游SIGBUS，文档记录限制。
- 点云 header+逐点纳秒 offset、IMU 时间戳必须保持同一时域，禁止盖 now() 掩盖异常。
- 实际硬件包验收需要管理员 AF_PACKET 权限；不占用驱动 UDP 接收端口。
- 按用户要求移除测试代码。变更解析器后通过 check_ptp.sh verify 和 check_ros.sh
  实机检查失锁、时间回退、UTC−TAI 时域和逐点相对时间。
- 保留 pcl_publisher 两个 baselink 输出；ly_lio 继续原始输入并输出机体位姿。

- PTP 由 systemd 开机自启，独立于 ROS 节点；install.sh 为一次性部署入口并确保 enable。
- 不恢复每次启动授时的脚本；ptp.sh、start_lidar.sh 已删除，check_ptp.sh 仅 status/verify。
- scripts/odom.sh 统一启动雷达驱动、转换和里程计，仅检查授时服务，不调整服务或时钟。
- Ctrl+C 关闭 ROS 节点后继续授时；start_lidar:=false 用于原始 rosbag/外部驱动。
