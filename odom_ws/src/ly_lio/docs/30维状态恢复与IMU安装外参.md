# 30维状态恢复与IMU安装外参

> 历史记录，2026-10-17起不再描述现行模型。当前为真正24维`SO(3)×R^21`和固定外参，
> 激光保留每点到历史地图平面的Batch联合更新；见[当前实现约定](建模.md#本项目当前实现约定)。
> 本文的30维/在线估计/外参先验、0.1ms与芯片位置数值均保留为当时配置及验证证据，
> 不得照抄为现用示例；当前MID360位置为`c_L=[0.011,0.02329,-0.04412] m`，
> `imu_in_lidar`读取时仍仅转换一次`t_IL=-R_IL*c_L`。下文“当前”“最新”均限于本文日期。

日期：2026-10-13。依据用户最新要求恢复Point-LIO的30维状态，并应用给定的MID360芯片位置。
此前24维清理和零漂结果保留为历史；本次不将其作为新配置的零漂验收。

## 状态和观测维度

状态及误差协方差P均为30维，顺序与Point-LIO输出观测模型一致：

| 状态 | 误差索引 | 含义 |
| --- | --- | --- |
| p、R | 0、3 | IMU原点的世界位置、姿态 |
| R_IL、t_IL | 6、9 | 激光→IMU外参旋转、平移 |
| v、omega、a | 12、15、18 | 世界线速度、IMU系角速度、IMU系比力 |
| g、bg、ba | 21、24、27 | 世界重力、IMU陀螺与加速度零偏 |

每项误差三维。R与R_IL均采用右扰动，注入后对两个SO(3)及其全部交叉协方差做reset。
IMU观测仍为6维`[omega_m-omega-bg, a_m-a-ba]`，单点激光仍为1维点到面残差。
一批N个有效激光点对应N个标量观测；不能将“恢复30维状态”理解为增加30个IMU观测。

`extrinsic_est_en: false`为当前默认：外参状态保留，P对应6行/列置零，外参不随观测更新。
`true`启用激光对外参的联合估计；初始先验由`extrinsic_rotation_cov`（rad²）和
`extrinsic_translation_cov`（m²）给出，默认均为0.01。外参没有过程随机游走Q。
单纯恢复30维存储并固定外参，物理24维子系统的数学模型不变；本次非零芯片安装位置会改变几何模型。

## 用户向量与Point-LIO内部约定

用户给的是IMU原点在激光坐标系中的位置：

`c_L = [-0.011, -0.02329, 0.04412] m`。

内部与Point-LIO统一使用：

\[
p_I=R_{IL}p_L+t_{IL},\qquad t_{IL}=-R_{IL}c_L.
\]

这是坐标变换平移与原点位置的区别。取激光点`p_L=c_L`必须得到IMU零点。
MID360配置保留用户原值，并显式说明位置约定：

```yaml
extrinsic_translation_convention: imu_in_lidar
extrinsic_T: [-0.011, -0.02329, 0.04412]
extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
extrinsic_est_en: false
extrinsic_rotation_cov: 0.01
extrinsic_translation_cov: 0.01
lidar_batch_duration: 0.0001
```

当前单位旋转下内部`t_IL=[0.011,0.02329,-0.04412]`米。代码在读取参数时只转换一次；
ROS参数查询仍显示YAML中的原位置向量，启动日志另打印内部平移。
`extrinsic_R`始终将激光测量轴旋转到IMU测量轴，不随平移约定反转。
目前R=I是测量轴同向的配置假设；给定芯片位置本身不能确定两者的旋转标定。

`extrinsic_translation_convention: lidar_in_imu`则直接把extrinsic_T当t_IL，
适配原Point-LIO参数。未声明该参数时读取边界默认此约定；包内MID360默认文件明确设为imu_in_lidar，
因此给MID360传旧约定实验文件时须同时覆盖约定。L2示例已显式设为lidar_in_imu，避免继承MID360选项。

## Batch与原始Point-LIO的不同

本次恢复状态排列和在线外参观测，继续使用《建模.md》的Batch更新，不恢复旧逐点注入。
默认最长0.1ms，配置必须严格满足`0 < lidar_batch_duration < 0.010`秒；遇IMU事件或扫描末端提前结束。

令冻结的批末先验为R、p、omega、v及E=R_IL、t=t_IL，点时间差为dt：

\[
D=\operatorname{Exp}(\omega dt),\quad b=R^T v dt,\quad
q_I=D(Ep_L+t)+b,\quad C=R^T n.
\]

历史地图平面提供残差`z=-(n^T(R*q_I+p)+d)`。固定外参只使用6列位姿雅可比；在线模式为12列：

\[
H=[n^T,\ (q_I\times C)^T,\ (p_L\times E^T D^T C)^T,\ (D^T C)^T].
\]

必须携带原激光点p_L与冻结旋转D，不能从去畸变后的点简单反推原点来计算外参导数。
dt=0时外参列退回原Point-LIO公式。每批冻结地图/先验、累加信息并联合注入一次。
固定模式解6维系统，在线模式解12维系统；均通过完整30维P将信息传递给其余状态。

在线注入后，用相同冻结运动补偿修正注册点及地图点：

\[
q_I^+=q_I^-+D[(E^+-E^-)p_L+t^+-t^-],\qquad p_W=R^+q_I^++p^+.
\]

整扫描结束再入图，不重新用后验omega/v去畸变。IMU输出观测、95%饱和屏蔽、
先P后x传播、Q仅omega/a/bg/ba、静止初始化、时间戳和默认scan_end发布方式继续保留。
reset或IMU缺段重建将恢复配置外参，避免上一段在线估计结果成为未记录的新先验。

## 机体输出与TF

安装文件`pcl_ws/config/lidar2baselink.yaml`给固定T_BL；滤波内部给T_IL：

\[
T_{BI}=T_{BL}T_{IL}^{-1},\qquad T_{WB}=T_{WI}T_{BI}^{-1}.
\]

使用安装文件时，每次发布都按当前外参重新组合T_BI。令`s=R_BL^T*t_BL`、
`r=t_IL-R_IL*s`为机体原点在IMU系的位置，机体速度包含`R_WI*(omega×r)`杆臂项。
pose/twist协方差使用完整30维P，加入外参误差及交叉项；终端取同一发布快照。

非零偏移后，不能将T_BI发布给`livox_frame`。当前静态TF明确发送固定T_BL：
`baselink→lidar_frame`。驱动的原始IMU仍使用`livox_frame`作为测量轴标签，节点据此接收；
该标签不证明芯片原点与激光原点重合。若需要独立真实IMU坐标系，应使用不同名称另建TF。

也可通过`imu_to_body_R/T`提供独立固定T_BI，并清空body_extrinsic_path。
此时机体输出不依赖T_IL；若同时在线估计外参，传感器T_BL按当前标定在Odom相同stamp发布动态TF。
已有机器人状态发布器管理安装关系时可关闭publish_sensor_tf。

## 配置与验证

配置继续由所属节点C++直接加载，修改后重启生效，原生ROS参数可覆盖：

- LIO：`odom_ws/src/ly_lio/config/mid360.yaml`。
- Livox ROS/PTP：`livox_sdk/src/livox_ros_driver2/config/mid360.yaml`；SDK读取同目录MID360_config.json。
- 安装外参：`pcl_ws/config/lidar2baselink.yaml`。
- 录包：三个ROS工作区外的`config/rosbag.yaml`。

shell/launch只管理进程，不读取参数或生成配置。分包方式详见[统一参数配置](统一参数配置.md)。

Release构建已通过。ESKF的924次数值核对全部通过，包括固定/在线Batch与堆叠Kalman、
稀疏传播、64种IMU饱和掩码、600步固定30维与独立24维公式投影及双SO(3)reset。
公式比较最大误差3.33e-16，双SO(3)有限差分误差1.40e-10。
没有独立24维源码存档，该项是公式/数值等价验证，不是旧源码轨迹逐数回归。
详细结果见[30维数值验证.json](30维数值验证.json)。

直接调用生产机体输出函数，256组随机完整30维协方差与数值有限差分`J*P*J^T`核对通过：
pose协方差误差9.44e-12，twist误差3.81e-10；固定外参时与固定T_BI路径协方差差5.55e-17。
生产参数读取的MID360及16组非单位旋转换算、安装组合核对通过。
独立公式512组含D的Batch雅可比有限差分误差3.38e-9，生产h_point带D公式另经源码复核。
条件、公式及边界见[30维外参与机体输出验证.json](30维外参与机体输出验证.json)。

ROS 2 Humble合成流程使用200Hz IMU、10Hz多平面PointCloud2验证三种配置：

| 配置 | Odom / 机体TF / 注册云数量 | 传感器TF |
| --- | --- | --- |
| 默认固定外参与共享安装文件 | 30 / 30 / 30 | 固定T_BL |
| 在线外参与共享安装文件 | 39 / 39 / 39 | 固定T_BL |
| 在线外参与显式固定T_BI | 39 / 39 / 39 | 动态T_BI*T_IL |

共108组输出的stamp序列逐条一致并严格递增，Odom与机体TF位姿数值差为0，
pose、twist、协方差和注册点全部有限；没有非预期IMU缺段重建。
三种配置均完成reset后重新静止初始化，三个节点均以SIGINT正常退出（退出码0）。
显式T_BI在线模式的39组传感器TF与Odom stamp一致，非零残差触发了标定更新；
reset后起始t_IL恢复误差4.07e-8m、旋转矩阵误差6.17e-10。
这些流程验证并不证明真实标定收敛或轨迹精度。
详细条件与结果见[30维ROS验证.json](30维ROS验证.json)。

本次未启动真实雷达、未操作PTP，未进行新零漂测量。历史bag当前不在工作目录，
未做原始实机数据的同源回放；不得将此前24维600秒结果标为本次30维或非零外参结果。
在线外参需要足够的旋转/平移激励，静止与单轴yaw不能保证完整可观；恢复维数本身不保证降低零漂。
后续精度判断应在同一原始数据上比较固定/在线配置，再用独立静止与运动录包验证。
临时验证程序按维护要求清理，只保留结果JSON与说明。
