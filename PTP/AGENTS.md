# PTP 部署与维护经验

更新：2026-10-20。此目录只保留基础部署、启动和配置文件；经验集中在本文。
不保留独立调试、测试、监视、构建包装脚本及验收 JSON，不恢复每次启动 ROS 时重新授时的入口。
必要的临时检查在 `/tmp` 或 stdin 执行并清理，不能声称已删除的检查程序仍可运行。

## 文件职责与启动

| 文件 | 职责 |
| --- | --- |
| `install.sh` | 一次性安装依赖、复用已有服务或安装新服务，并启用开机自启 |
| `bootstrap.py` | 新部署服务在发布 PTP 前初始化 PHC，含必要的硬件及进程保护 |
| `master.conf.in` | IEEE 1588v2、UDPv4、E2E、domain 0、hardware、固定 MASTER 配置模板 |
| `systemd/ly-ptp-bootstrap@.service` | 一次性 PHC 初始化，由主时钟的依赖启动，无需单独 enable |
| `systemd/ly-ptp4l@.service` | 持续向雷达发布 PTP |
| `systemd/ly-phc2sys@.service` | 持续从只读系统时间同步 PHC，禁止阶跃 |

仓库根目录的一次性部署命令：

```bash
sudo ./PTP/install.sh enp86s0
```

省略网卡时按到 `192.168.1.3` 的路由选卡；更换设备或路由时显式指定有线网卡。
安装脚本补装 `linuxptp`、`ethtool`、`python3`，验证网卡的硬件时间戳及唯一 PHC。
已有 sentry 服务时复用，不覆盖其配置、不重启运行实例；新机器安装 ly 服务，生成
`/etc/linuxptp/ly-odom/master-网卡.conf`，部署 bootstrap 到 `/usr/local/libexec/ly-ptp/`。
遇未知 PTP 进程拒绝自动接管，先核对网卡、PHC 和时钟方向，避免多个伺服器同时调节同一 PHC。

日常在仓库根目录启动 ROS：

```bash
./scripts/odom.sh
# 回放或外部驱动，不启动雷达、不检查 PTP、不另录包：
./scripts/odom.sh start_lidar:=false record_bag:=false
```

PTP 由 systemd 开机自启，独立于 ROS。统一入口管理驱动、转换、里程计和录包；
驱动初始化 SDK 前只读检查已有 PTP/PHC 服务和本地接收 IP。Ctrl+C 后 PTP 继续运行，
等待 Recorder 刷缓存及 metadata 后再关闭终端。日常启动不调用 sudo、不启停授时服务、不调时钟。

## 本机配置与授时方向

- 本机使用 Ubuntu 22.04 / ROS 2 Humble；历史检查的 linuxptp 版本为 3.1.1。
- 有线网卡 `enp86s0`、PHC `/dev/ptp0`、主机 `192.168.1.50/24`、目标 MID360 `192.168.1.3`。
- 既有部署为 `sentry-ptp4l@enp86s0`、`sentry-phc2sys@enp86s0`、`sentry-phc-bootstrap@enp86s0`；不能同时启用 ly 实例调节同一 PHC。
- 授时方向固定为 `CLOCK_REALTIME（UTC，只读）→ phc2sys → PHC（PTP/TAI）→ ptp4l → 雷达`，禁止将系统时钟设为目标。不要修改系统 UTC、NTP 服务或任意网卡地址。
- 新服务在发布前将 PHC 初始化为 `CLOCK_REALTIME + utc_offset`；需要阶跃时，已有 PTP 或采集/里程计进程必须阻止初始化。
- 持续伺服保留 `-F 0 -S 0`，禁止初始及运行阶跃；`--max_frequency=100000` 限制为 ±100 ppm。
- 硬件网卡须支持 `hardware-transmit`、`hardware-receive`、`hardware-raw-clock`；UDP 319/320 组播及 Delay 消息必须双向可达。Wi-Fi 或普通 USB 网卡不能直接替代。
- `MASTER`、phc2sys `s2` 及很小的 PHC 偏差只说明主机链路状态，不证明雷达锁定，也不代表外部绝对 UTC 精度。

驱动参数在 [mid360.yaml](../livox_sdk/src/livox_ros_driver2/config/mid360.yaml)，SDK 原生网络配置在
[MID360_config.json](../livox_sdk/src/livox_ros_driver2/config/MID360_config.json)。节点 C++ 直接读取，
源码优先、安装 share 回退，ROS 原生参数可覆盖；修改配置后重启生效，不由 shell/launch 生成临时 JSON。
更换网卡更新 `ptp_interface`；更换主机 IP 同步 JSON 中全部非空接收地址；更换雷达同步
`target_ip` 与 `lidar_configs`。链路另有 `.5` 设备，目标过滤不能无故取消。
SDK JSON 没有有效的 `enable_ptp` 开关，不添加虚构配置项。

## 时域与检查经验

- 当前 `require_ptp_sync: true`，原始数据必须为 `time_type=1`（PTP/gPTP），其他时间源入队前丢弃；不退回主机收包时间。
- 本机设备实测 PTP 比 UTC 快 37 秒。模板 `utc_offset=37`，驱动 `ptp_utc_offset: 37` 将点云基时和 IMU 统一减至 UTC。`phc2sys -w` 保留标准时域关系，不能改成 `-O 0` 掩盖错误。
- 换成输出 UTC 的固件须先测量，再将驱动偏移设为 0；不能同时破坏 PHC 的标准 TAI 时标。未来闰秒按权威 TAI−UTC 更新配置，停止采集后重新初始化。
- PointCloud2 真点时间为 `header.stamp + point.timestamp * 1e-9`，字段是相对纳秒；CustomMsg 为绝对 UTC 纳秒 timebase 加 offset；IMU 保留采样 header。不能改成 now() 或均匀 5ms 网格。
- 原始包和 ROS 两层都要核查：原始点云/IMU 包数量、`time_type=1`、时间回退/重复、UTC/TAI 偏差；ROS header 连续性、逐点偏移及激光/IMU 时间区间重叠。数据不足不能判通过，单看 ROS header 可能漏掉收包时间回退。
- 原始包检查用旁路抓取，需要管理员 AF_PACKET 权限，不绑定或抢占驱动 UDP 56301/56401。接收年龄含帧聚合、网络、内核队列及调度延迟，不能当精确授时误差；`time_type=1` 也不保证某个微秒误差界限。
- 服务状态可用系统自带 `systemctl status`、`systemctl is-enabled`、`journalctl -u` 只读查看；按实际 sentry/ly 实例查询，不为检查新启一套服务。
- PTP 维护、雷达复位或主机时间大幅变化前停止采集。重新同步并核查后重启里程计或调用 `/ly_lio/reset`；丢弃回退数据不能恢复旧滤波器时间轴。
- PTP 不解决物理 LiDAR–IMU 采样时延、机械外参、几何退化或传输丢包。独立转换节点已删除；里程计直接订阅原始话题，发布扫描末baselink注册云与odom→baselink机体位姿。

## 驱动构建经验

增量构建只选 ROS 2 业务包，不运行会删除旧 build/devel/install 的上游 `build.sh`。
首次准备时，在仓库根目录确认 ROS 2 manifest 和 launch 已就位：

```bash
# 仅缺少 package.xml 时复制，保留现有自定义 launch 文件。
test -f livox_sdk/src/livox_ros_driver2/package.xml || \
  cp livox_sdk/src/livox_ros_driver2/package_ROS2.xml livox_sdk/src/livox_ros_driver2/package.xml
mkdir -p livox_sdk/src/livox_ros_driver2/launch
cp livox_sdk/src/livox_ros_driver2/launch_ROS2/*.py livox_sdk/src/livox_ros_driver2/launch/
source /opt/ros/humble/setup.bash
cd livox_sdk
colcon build --packages-select livox_ros_driver2 --parallel-workers 1 --symlink-install \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF
```

## 历史结果与未解决问题

以下均为 2026-10-11 历史记录，不代表 2026-10-20 新验收：

- 主机检查确认有线网卡的三项硬件时间戳能力及 PHC 索引 0；sentry 主时钟/伺服 active、enabled，PHC 日志偏差为几十纳秒。原记录中 `ethtool` 尚未系统安装，仅临时解包只读查询；timesyncd/chrony 当时为 masked，未修改系统时间或网卡。
- 设备未转换的两路时间比 UTC 快约 37 秒，据此采用上述 TAI→UTC 配置。驱动构建与当时 5 项时间检查通过；这些测试程序现已删除。
- 稳态 10 秒 ROS 样本为点云 100 帧、IMU 2001 条，回退、重复、错误年龄、逐点错误均为 0，两路时间区间重叠。点云最大间隔 100323720ns、接收年龄 102667291～105189137ns；IMU 最大间隔 6017070ns、年龄 239446～8919956ns。这些是消息时序统计，不是硬件同步精度。
- 统一启动另一次 5 秒样本为点云 50 帧、IMU 1001 条，上述异常均为 0，ly_lio 成功初始化；Ctrl+C 后 PTP PID 不变，转换与里程计正常退出。参数展开、回放模式和缺失伺服门控当时已检查。
- 管理员 AF_PACKET 原始包旁路验收未执行，不能用 ROS 通过补写为已完成。驱动停止时上游析构出现 SIGBUS（exit -7），仍是关闭路径问题，不能隐藏为整体通过。

已删除报告的来源 SHA256（用于识别历史来源，不作为仍可访问的文件链接）：

- 原 `本机检查.json`：`c23caecbb630e774fd02597c9392622192a1d02215df85fdf521fe073990bbba`。
- 原 `ROS实机验收.json`：`737c485346219182913b56d0fe262adfd7c6d56c9f2bbeb2ae0e031a8a9574df`。

本次仅整理目录、解除已删工具依赖并静态检查；未运行安装脚本、启停服务、调节时钟或做硬件验收。
