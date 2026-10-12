# MID360 PTP 时间同步

目标是使 MID360 点云、IMU 的采样时间与主机 ROS 系统时间处于同一时域，并在失锁时
拒绝退化为收包时间。原始话题为 `/livox/lidar`、`/livox/imu`；转换节点继续发布
`/bashlink/pcl`、`/baselink/imu`，里程计仍直接订阅原始话题。

## 当前机器

2026-10-11 检查结果：Ubuntu 22.04、linuxptp 3.1.1 已安装。雷达链路使用有线
`enp86s0`，PHC 为 `/dev/ptp0`，主机 IPv4 为 `192.168.1.50/24`，雷达为
`192.168.1.3`。驱动 JSON 的接收地址已同步为 `.50`；如换网卡/IP，请同步修改
`livox_sdk/src/livox_ros_driver2/config/MID360_config.json` 中全部接收地址。

主机已有并启用了 `sentry-ptp4l@enp86s0`、`sentry-phc2sys@enp86s0`、
`sentry-phc-bootstrap@enp86s0`。日志显示端口 MASTER，phc2sys 为 `s2`，
采样时 PHC 与系统时差在几十纳秒量级。这只证明主机 PHC 锁定，不能据此宣布雷达已锁定。
`systemd-timesyncd`、`chrony` 在本机被 masked；本工具不修改它们，也不修改系统 UTC。
主机是局部时间源，并没有据此获得外部绝对 UTC 精度。

本次执行身份没有免密 sudo，因此没有重新安装系统软件、替换运行服务、调节 PHC 或
修改网卡。`ethtool` 尚未系统安装，`install.sh` 会补装；已下载并临时解包只读查询，
确认 enp86s0 支持 hardware-transmit、hardware-receive、hardware-raw-clock，PHC 索引 0。
管理员原始包抓取验收需 sudo。

## 时间链路

```text
主机 CLOCK_REALTIME（UTC，只读源）
   → phc2sys → 有线网卡 PHC（PTP/TAI）
   → ptp4l（IEEE 1588v2，UDPv4，E2E，domain 0，MASTER）
   → MID360 内部时钟 → 点云和 IMU 采样时间 → ROS 消息 → ly_lio
```

主机发送 Sync/Follow_Up、响应 Delay_Req。雷达作为从时钟自动使用兼容的 PTP 信号，
当前驱动 JSON 不存在有效的 `enable_ptp` 开关，不能靠添加虚构配置项启用。
如果固件/网络不兼容，原始包的 `time_type` 不会通过验收。UDP 319/320 的组播及
Delay 消息必须能双向通过同一有线链路；不要用 Wi-Fi PHC 或普通 USB 网卡替代硬件授时网卡。

## 一次性部署与日常启动

PTP 是独立的系统授时服务，随系统开机启动，持续运行；不与 ROS 点云节点共用
启动脚本。当前机器的 sentry 主时钟与 PHC 伺服均为 enabled、active，已经满足要求。
bootstrap 是 static 单元，由主时钟的 Requires 依赖在开机时拉起，无需单独 enable。

首次部署或补装依赖时，在仓库根目录执行一次：

```bash
# 一次性安装依赖，复用现有配置并确保服务开机自启；新机器安装 ly 服务。
sudo ./PTP/install.sh enp86s0
# 含严格同步策略的 Livox 驱动增量构建。
./PTP/build_driver.sh
```

日常启动只需：

```bash
./scripts/odom.sh
```

该命令先只读检查同一部署的 PTP 主时钟、PHC 伺服和驱动接收地址，然后由一个 ROS
launch 启动 MID360 驱动、pcl_publisher 和 ly_lio，默认同时录制全部 ROS 话题到
仓库 `bags/`；用 `bag_dir:=/目录` 指定父目录，`record_bag:=false` 关闭录包。
Ctrl+C 后等待 rosbag 缓存刷新与 metadata 写出，再关闭终端。驱动始终启用严格 PTP 策略；
未同步数据直接丢弃。Ctrl+C 统一关闭 ROS 节点，PTP 服务继续运行。
本脚本不调用 sudo，不启动、重启或停止 PTP，也不调整时钟。

```bash
# 只读查看服务、开机自启状态与日志。
./PTP/check_ptp.sh status enp86s0
# 原始 rosbag 回放或外部驱动：跳过雷达启动及 PTP/网卡检查。
./scripts/odom.sh start_lidar:=false
# 更换网卡时显式指定；需另行更新网卡地址与驱动 JSON。
./scripts/odom.sh ptp_interface:=enp86s0
```

原先的 ptp.sh、start_lidar.sh 已删除；PTP 目录只提供一次性部署、构建与验收工具，
雷达和点云节点统一由里程计启动入口管理。启动驱动后等待初始化及发布稳定约 5 秒，
再进行下面的时间验收。

首次部署新机器时，安装脚本生成 `/etc/linuxptp/ly-odom/master-网卡.conf`，安装
`ly-ptp-bootstrap@`、`ly-ptp4l@`、`ly-phc2sys@`，并通过 `systemctl enable --now`
启用主时钟与 PHC 伺服；bootstrap 由服务依赖启动。
已经存在 sentry 服务时补齐依赖并启用已有实例，不覆盖其文件，也不重启运行中的服务。
其他未知 PTP 进程存在时拒绝接管，需先核对网卡与时钟方向。

新服务在发布 PTP 前初始化 PHC 为 `CLOCK_REALTIME + utc_offset`，仅在没有采集进程
和 PTP 伺服器时允许校时阶跃。持续伺服设置 `-F 0 -S 0`，只缓慢调整 PHC 频率，
禁止初始/运行阶跃；`--max_frequency=100000` 限制调频为 ±100 ppm。
方向始终是系统→PHC；系统时间只读，不能反过来把主机设为从时钟。

## 两层验收

必须先启动雷达驱动，使设备发送点云和 IMU，然后在另一终端执行：

```bash
# 旁路抓取原始 Livox UDP，不绑定或抢占驱动 UDP 56301/56401。
sudo ./PTP/check_ptp.sh verify enp86s0

# 检查实际 ROS header 和逐点 offset，默认采样 10 秒。
./PTP/check_ros.sh --seconds 10
```

两条命令都必须返回 0，并显示 `passed: true`。原始层检查两类包数量、
`time_type=1`、回退/重复、设备时间与主机 UTC 的差；ROS 层检查消息数量、
header 时间连续性、收包年龄、逐点相对纳秒偏移及 IMU/激光时间区间重叠。
检查失败返回非零，数据不足不算成功。连续运行可增加 `--seconds`；间隙最大值
会输出，但有限采样不能保证今后永不失锁。

ROS 点的真实时间保持：`header.stamp + point.timestamp * 1e-9`。
不要将相对纳秒改成绝对秒，不要在转换节点重盖 `now()`。仅检查 ROS header
看起来像 UTC，不能证明来源是采样时钟：旧驱动的收包时钟回退也可能通过该检查。

`PTP/monitor_packets.py` 判断 PTP/UTC 时域，而不是精确测量授时误差：接收差值包含
网络、内核队列和 Python 调度。真实精度需设备统计或硬件抓包；MASTER 不证明雷达锁定，
`time_type=1` 也不是某个微秒误差界限的保证。

## 防止失锁退化

修改后的 MID360 launch 默认设 `LIVOX_REQUIRE_PTP_SYNC=1`。驱动在入队之前
要求 `time_type=1`（PTP/gPTP），未同步/GPS 等其他时间源数据直接丢弃。
它不会用系统收包时间替代采样时间。`scripts/odom.sh` 通过组合 launch 包含此驱动
启动文件，并在节点启动前检查服务和本地接收 IP。旧安装二进制不包含此策略，需重新构建。

直接 `ros2 run` 时也应带 `LIVOX_REQUIRE_PTP_SYNC=1`；只有确需非同步实验时，才在
直接运行中取消它。严格模式不会消除设备复位或重新锁定时的真实时间阶跃。
PTP 改动、雷达复位、主机时间大幅修改前停止里程计/录包；恢复同步并通过验收后，
重新启动里程计或调用 `/ly_lio/reset`。已有过滤器对回退数据的丢弃不能恢复正确的时间轴。

PTP 是时钟同步，不会解决特征不足造成的几何退化，也不补偿机械外参或传输丢包。

## UTC 与 TAI 的 37 秒差

硬件 ptp4l 使用 PTP/TAI，主机系统/ROS 使用 UTC；配置 `utc_offset=37`，
`phc2sys -w` 根据 ptp4l 信息维护时域差，不能改成 `-O 0`。
以后发生闰秒，应按权威 TAI−UTC 更新配置并在停止采集后重新初始化。

本机实际启动雷达后，未转换时点云和 IMU 均比系统 UTC 快约 37 秒；与硬件
主时钟的 TAI 时标一致。当前 launch 显式设 `ptp_utc_offset=37`，对应环境变量
`LIVOX_PTP_UTC_OFFSET`，驱动在解析 PTP 包时统一减去该差值，覆盖点云基时和 IMU。
逐点字段改为绝对采样时间减帧基时的相对纳秒，避免原代码把绝对纳秒传给相对解码器。

换成输出 UTC 的固件/主时钟时，必须先测量确认，再启动：
`./scripts/odom.sh ptp_utc_offset:=0`；原始包监视器相应传
`--ptp-utc-offset 0`。不能同时更改 PHC 的标准 TAI 时标来掩盖设备时域问题。
当前 `check_ptp.sh verify` 按本机实测 TAI 配置检查（减 37 秒）。

同链路另有 192.168.1.5 的雷达，当前 launch 设置 `target_ip=192.168.1.3`，
驱动原始数据入口和设备配置回调都检查 `LIVOX_TARGET_IP`，避免未配置设备自动加入
共享话题或被自动启用。改变目标时必须同步驱动 JSON 中的 lidar_configs。

稳态实机 ROS 验收记录见 `ROS实机验收.json`：启动稳定后采样 10 秒，100 帧点云、
2001 条 IMU，回退/重复/错误年龄/逐点错误均为 0，验收通过。点云 header 接收年龄
约 103～105 ms（帧聚合时间包含在内），IMU 约 0.2～9 ms；它们不是 PTP 精度测量。
最后一轮同时运行 scripts/odom.sh，ly_lio 成功初始化，转换节点和里程计正常退出。
原始 AF_PACKET 旁路验收仍需管理员执行，当前用户无法免密 sudo。
实机停止驱动时上游析构路径出现 SIGBUS（exit -7），数据验收在退出前完成；
这是驱动关闭问题，不能将它当成授时成功或时间退化证据。

## 停止与实机检查

在里程计终端按 Ctrl+C 停止雷达、转换和里程计，开机自启的 PTP 保持运行。
仅管理员维护时才直接操作 systemd；校时前先停止采集。

```bash
./PTP/check_ptp.sh status enp86s0
sudo ./PTP/check_ptp.sh verify enp86s0
./PTP/check_ros.sh --seconds 10
```

测试代码已按用户要求移除。保留原始包和 ROS 实机检查，用于观察失锁、时域错误、
回退、重复和逐点纳秒偏移。
所有检查脚本只观察时间；只有管理员安装的新服务 bootstrap 会在采集前初始化 PHC。

## 本次启动职责调整验证

统一运行 `./scripts/odom.sh`，三节点启动，ly_lio 初始化；两个转换发布话题保留。
稳态采样 5 秒收到 50 帧点云、1001 条 IMU，ROS 时间检查 passed=true，
无回退、重复、错误年龄或逐点错误；这仍不代表硬件授时精度验收。
Ctrl+C 后 PTP 服务 PID 保持不变，仍为 active/ enabled；转换与里程计正常退出，
驱动关闭仍有前述 SIGBUS(-7)。参数展开及回放模式/缺失伺服门控也已检查。
