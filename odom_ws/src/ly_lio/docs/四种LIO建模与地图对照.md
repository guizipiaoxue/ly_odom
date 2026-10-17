# Super-LIO、GenZ-LIO、FAST-LIVO2、Point-LIO建模与地图源码对照

审查日期：2026-10-17。对象是本工作区实际代码，不把项目论文、网上其他分支或注释当成已启用功能。

**最值得区分的是两件事：地图存的是点还是持久平面，以及“几何拟合可靠”是否等于“物体固定不动”。**
Super-LIO和本地Point-LIO主要存点，查询时临时拟合平面；FAST-LIVO2、GenZ-LIO存持久平面及其协方差。
四者的默认定位主路径都没有完整的棚膜运动追踪或跨帧静态概率估计。平整但随风移动的棚膜，仍可能被当成可靠定位参照。

本轮只读参考源码、分析指定静止录包并写文档，没有移植算法。下文的源码结论不代表已经用四套算法回放，也不代表任何一套已经通过本项目静止、启动或运动精度验收。

## 1. 版本与总览

| 本地目录 | 本轮审查提交 | 运行环境 |
| --- | --- | --- |
| `Super-LIO` | `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2` | ROS 2，MID360入口默认CustomMsg |
| `genz-lio` | `1b7320dfe866bcb71bc14ce7db1faa307c86b676` | 同时提供ROS 1/2封装，MID360默认CustomMsg |
| `FAST-LIVO2` | `0d2c0346107b75b59934975adec9a6eeeb913c64` | 本地checkout为ROS 1/catkin，含视觉 |
| `Point-LIO` | `4b86a469eb5572e70ed575af25b5f15dd06e8e3c` | 本地checkout为ROS 1/catkin，**地图为IVox** |

以下比较主定位路径。未指定配置项时，明确区分代码默认、设备示例配置和可选分支；参考树均保持只读。

| 项目 | Super-LIO | GenZ-LIO | FAST-LIVO2 | Point-LIO | 当前LY |
| --- | --- | --- | --- | --- | --- |
| 实际状态/误差维度 | 18 | 23；重力方向2自由度 | 19，含逆曝光 | 默认30；可选24 | 30 |
| IMU使用方式 | 输入积分 | 输入积分 | 输入积分 | 默认输出观测；可选输入 | 输出观测 |
| 激光更新时刻 | 名义扫描末，多轮迭代；时序近似见3.1 | 扫描末，配置迭代数4 | LIO扫描末；LIVO按图像切分 | 按点时间；相同时间点成组 | 最长0.1ms Batch，遇IMU提前分界 |
| 激光直接H | 位姿6列 | 位姿＋可选外参12列 | 位姿6列 | 位姿＋可选外参12列 | 位姿＋可选外参＋v/omega，共18列 |
| 地图主体 | OctVox哈希＋8子格代表点 | 哈希＋自适应八叉树平面/点 | 哈希＋八叉树平面；另有视觉地图 | IVox哈希＋LRU点地图 | 哈希＋LRU多真实点地图 |
| 平面来源 | 查询4或5近邻QR | 节点积累支持点PCA | 节点积累支持点PCA | 查询5近邻QR | 查询5近邻PCA |
| 激光测量权重 | 固定方差 | 点噪声＋平面/匹配点不确定度 | 点噪声＋平面不确定度＋方差底值 | 固定方差 | 点噪声投影，独立残差 |
| 显式动态棚膜识别 | 未见 | 未见 | 未见；视觉也不等于动态识别 | 未见 | 未实现 |

“直接H”只列残差显式依赖的状态；完整滤波仍能通过P的交叉协方差修正其他状态。H列数与完整状态维度不能混为一谈。

## 2. 共同公式及当前LY的关键区别

统一记号：`R,p`为IMU到世界的位姿，`R_IL,t_IL`将激光点变到IMU系，`v`为世界系速度，`bg,ba`为IMU偏置。公式统一记号不表示各仓库变量下标约定相同。

### 2.1 三套IMU输入模型

Super-LIO、GenZ-LIO、FAST-LIVO2的核心传播都是：

\[
\dot p=v,\qquad \dot R=R[\omega_m-b_g]_\times,\qquad
\dot v=R(a_m-b_a)+g.
\]

IMU测量减偏置后直接驱动运动传播；状态中没有独立的真实角速度、比力。加速度先按各自适配/初始化规则转成内部单位。

### 2.2 Point-LIO默认及当前LY的IMU输出观测模型

30维状态为：

\[
x=[p,R,R_{IL},t_{IL},v,\omega,a,g,b_g,b_a].
\]

传播与IMU观测分别为：

\[
\dot p=v,\qquad \dot R=R[\omega]_\times,\qquad \dot v=Ra+g;
\qquad r_g=\omega_m-\omega-b_g,\quad r_a=a_m^{SI}-a-b_a.
\]

其中`a`是IMU系比力，不是已扣重力的世界加速度。Point-LIO还提供24维IMU输入路径，默认`use_imu_as_input=false`，不能只挑可选路径代表其默认建模。

### 2.3 为什么其他项目激光不直接写v，而LY写v/omega

前三套先按IMU预测轨迹把点去畸变到统一末时刻，随后将去畸变点当作固定输入，激光迭代只展开位姿/可选外参：

\[
r=n^\top\{R(R_{IL}p_L^{deskew}+t_{IL})+p\}+d.
\]

Point-LIO在点自己的时间更新，不需要把一批不同时刻的点一起补偿到批末；函数中的`feats_undistort`名称不表示它做了前三套那种整帧末端去畸变。

当前LY保留原始点及相对批末时间`τ=t_point−t_end≤0`，把补偿写成批末状态的函数：

\[
y(x)=p+R\operatorname{Exp}(\omega\tau)(R_{IL}p_L+t_{IL})+v\tau,
\qquad r=n^\top y(x)+d.
\]

因此`H_v=τ nᵀ`，`H_omega`通常也非零，`τ=0`时两者为零。固定外参时实际观测求解选择`[p,R,v,omega]`12维，状态/P仍30维。
**只删除LY的v/omega列，不能称为等价采用上述项目的模型**：必须同时明确采样时刻、去畸变输入是否冻结及相应误差近似。
这也不能仅凭源码推断哪一种在大棚更准。

源码：[LY补偿及H，estimator.h:16](../src/ly_lio/estimator.h#L16)、[Batch调度及后验入图，ly_lio.cpp:170](../src/ly_lio/ly_lio.cpp#L170)、[现有补偿推导](Batch运动补偿状态雅可比.md)。

## 3. Super-LIO

### 3.1 状态、初始化和更新

- 实际状态是`[R,p,v,bg,ba,g]`，P为18×18。头文件另有17维DOF别名，但实际滤波使用18维P；更新后将g归一化到配置模长。
- 两相邻IMU均值用于积分，12维噪声为陀螺、加计及两类偏置；`Fw`带dt，按`Fw Q Fwᵀ`加噪声。
- 初始化累计至少50个IMU，gyro均值给bg，ba为0，加计均值给重力方向和尺度。未见基于样本方差的静止拒绝机制。
- 随后4帧以初始位姿建立地图，尚不做正常连续融合；不能把它作为已经解决连续启动漂移的依据。
- 正常流程：IMU传播/保存轨迹→整帧末端去畸变→体素采样→迭代点面更新→后验入图。MID360示例配置迭代数4。
- 激光H只有`[R,p]`6列；固定外参，速度/偏置/重力通过完整P间接更新。

源码：[ESKF状态，ESKF.h:15](../../Super-LIO/src/super_lio/include/lio/ESKF.h#L15)、[传播及注入，ESKF.cpp:100](../../Super-LIO/src/super_lio/src/lio/ESKF.cpp#L100)、[初始化，super_lio.cpp:98](../../Super-LIO/src/super_lio/src/lio/super_lio.cpp#L98)、[去畸变，:361](../../Super-LIO/src/super_lio/src/lio/super_lio.cpp#L361)。

时序实现有一处近似：主同步只传入时间≤扫描末的IMU，传播通常停在最后一个此类IMU时刻，没有显式补传播到精确扫描末；晚于最后预测状态的点只应用固定外参。
因此上面的“扫描末”是名义观测时刻，不能理解为每帧均严格传播至该时间。这是源码调用链可见的限制，尚未量化其精度影响。
见[ROSWrapper.cpp:548](../../Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L548)、[super_lio.cpp:364](../../Super-LIO/src/super_lio/src/lio/super_lio.cpp#L364)及[:390](../../Super-LIO/src/super_lio/src/lio/super_lio.cpp#L390)。

### 3.2 OctVox点地图怎样工作

地图根格默认0.5m，每根格分8个0.25m子格，每子格只存一个代表坐标及累计次数，最多8个代表点/根格。
新点与已有代表点相距≤10cm且计数<20时递推平均；达到20次后停止更新该代表点，直到根格被淘汰重建。
插入刷新LRU，容量到阈值删除尾部根格；MID360示例容量200万根格。

实际匹配调用`getTopK`，按预计算的邻近子格分组搜索，维护最多5近邻并可提前停止；另有邻域查询函数但不是当前Observe主路径。
查询到4或5近邻即可用列主元QR解`A n_raw=−1`，归一化后检查支持点到面距离≤0.1m，再检查查询点`range>81*r²`。
合格观测统一权重1000，相当于残差方差0.001m²，没有显式平面协方差或启用的Huber。
更新后整帧降采样点都进入地图，不只成功匹配点。

源码：[存储/平均，OctVoxMap.hpp:89](../../Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp#L89)、[LRU及查询，:275](../../Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp#L275)、[QR，super_lio.cpp:15](../../Super-LIO/src/super_lio/src/lio/super_lio.cpp#L15)、[观测和入图，:461](../../Super-LIO/src/super_lio/src/lio/super_lio.cpp#L461)。

### 3.3 对大棚的含义

平均次数不是静态可信度。10cm融合门限和20次冻结可以限制代表坐标被连续改写，但早期棚膜形状也可能被长期冻结；后来棚膜移动到另一子格还可能留下两层表面。
代码未保存每次观测的时间、法向变化、残差历史或墙面速度。

`PROB/P2P/MIX`枚举未被实际Observe读取，不能写成主路径已有概率/混合模型。
另有加载PCD并ICP初始化的`super_lio_reloc`路径，可选择不更新地图；这是已有地图定位，不是普通LIO自动识别动态墙。
见[reloc实现，super_lio_reloc.cpp:98](../../Super-LIO/src/super_lio/src/lio/super_lio_reloc.cpp#L98)。

## 4. GenZ-LIO

### 4.1 状态与观测

误差状态为`[p,R,R_IL,t_IL,v,bg,ba,g_S2]`，23维；重力三维向量只有两个方向误差自由度，不能按字段数量写成24维P。
采用IMU输入ESIKF，整帧IMU轨迹去畸变后冻结点，扫描末迭代激光观测，后验入图。
配置迭代数4；底层循环从−1开始，观测模型计算次数可能为5，不能把配置值当成绝对调用次数。
激光H为位姿及可选外参12列，固定外参时后6列置零，无直接速度列。

默认混合两种**标量**残差：

\[
r_{plane}=n^\top y+d,\qquad r_{point}=\|y-q_{map}\|.
\]

先查平面，未匹配平面再退回点到点搜索。点到点不是三维`y−q`同时观测，而是距离范数的一维约束，H沿当前差向量方向投影；零距离行丢弃。
这能补充不适合平面的区域，但不能天然识别摆动棚膜。

源码：[状态/传播，StateIkfom.hpp:18](../../genz-lio/cpp/genz_lio/core/StateIkfom.hpp#L18)、[整帧处理，ImuProcessor.cpp:124](../../genz-lio/cpp/genz_lio/core/ImuProcessor.cpp#L124)、[混合残差/H/R，HybridMetricUpdate.cpp:56](../../genz-lio/cpp/genz_lio/core/HybridMetricUpdate.cpp#L56)。

### 4.2 持久平面地图

- 根为哈希体素，内部自适应八叉树。节点记录支持点/点协方差、中心、法向、特征值、半径、6×6平面参数协方差和点到点候选。
- 累积支持点后用PCA拟合；最小特征值及特征值间隙判定平面，不平面则分裂。初始化阈值虽默认5，实际条件是`size>threshold`，不是恰好五点。
- 第一份可用去畸变扫描直接建图。后续定位用云与入图云分开，入图云更密；按点协方差对角平方和排序，优先放较确定的点。
- 平面支持点默认到100后停止普通更新。存在成熟平面收到离面点后的重拟合分支，但**默认`max_mature_points_size=max_points_size=100`使该分支关闭**。
- 点到点候选默认根格128，逐层预算除4，并检查点距；满后不循环替换。
- `map_range>0`才按位置范围删根格，默认−1不删；没有按时间自动遗忘动态表面。
- 自适应体素采样目标1000～4000点是反馈目标，不是把所有观测硬截断到4000；实际采样是坐标/时间质心，和LY真点采样不同。

源码：[节点及PCA，VoxelMap.hpp:34](../../genz-lio/cpp/genz_lio/core/VoxelMap.hpp#L34)、[成熟平面分支，:424](../../genz-lio/cpp/genz_lio/core/VoxelMap.hpp#L424)、[建图/排序/裁剪，GenZLIO.cpp:133](../../genz-lio/cpp/genz_lio/core/GenZLIO.cpp#L133)、[自适应采样，ScaleAwareVoxelizer.cpp:55](../../genz-lio/cpp/genz_lio/core/ScaleAwareVoxelizer.cpp#L55)、[MID360配置](../../genz-lio/ros/config/default/mid360.yaml)。

### 4.3 不确定度具体加在哪里

各向异性点噪声由量程和方向角噪声构造；PCA平面参数协方差用各支持点拟合雅可比传播：

\[
\Sigma_{plane}=\sum_i J_i\Sigma_iJ_i^\top.
\]

匹配检查平面半径及标准化点面距离，候选按高斯概率选。最终点面R包含平面项和当前激光点噪声投影；点到点R含匹配点/查询点协方差及离散搜索项。
入图点世界协方差使用位姿/外参P的块，但未保留完整交叉项；支持点独立累加，观测R也是对角形式。
**不是地图—当前状态联合估计，也不是共享平面残差的完整相关性模型。**
`Plane.id`是地图发布标识，不是动态目标ID。自适应点点门限依据距离与上一帧位姿修正，也不是墙面静态概率。

源码：[点噪声，VoxelMap.cpp:133](../../genz-lio/cpp/genz_lio/core/VoxelMap.cpp#L133)、[世界点协方差，StateTransforms.cpp:8](../../genz-lio/cpp/genz_lio/core/StateTransforms.cpp#L8)、[概率关联，VoxelPrunedSearch.cpp:33](../../genz-lio/cpp/genz_lio/core/VoxelPrunedSearch.cpp#L33)、[观测R，HybridMetricUpdate.cpp:109](../../genz-lio/cpp/genz_lio/core/HybridMetricUpdate.cpp#L109)。

## 5. FAST-LIVO2

### 5.1 LIO模型

状态为`[R,p,逆曝光,v,bg,ba,g]`19维。无独立omega/a，无在线LiDAR—IMU外参状态。
IMU输入传播，先保存轨迹，再将点补偿到本次更新末端，随后冻结补偿点进行激光迭代。激光H只有位姿6列，其他量经P间接更新。
`avia.yaml`示例激光迭代数5；纯LIO等待整扫描IMU覆盖，LIVO模式按图像时刻切分融合。

初始化用加计均值确定重力方向/尺度，但该checkout的初始bg赋均值语句被注释，实际设为0；没有静止方差门控。
`imu_int_frame`计数在每个IMU样本增加，不能按名字说成固定采集30帧激光。

源码：[19维及注入，common_lib.h:167](../../FAST-LIVO2/include/common_lib.h#L167)、[初始化，IMU_Processing.cpp:104](../../FAST-LIVO2/src/IMU_Processing.cpp#L104)、[传播，:334](../../FAST-LIVO2/src/IMU_Processing.cpp#L334)、[补偿，:492](../../FAST-LIVO2/src/IMU_Processing.cpp#L492)、[激光H/更新，voxel_map.cpp:409](../../FAST-LIVO2/src/voxel_map.cpp#L409)。

### 5.2 激光平面地图

哈希根体素内是八叉树，每平面保存中心、法向、半径、特征值、点数和6×6平面协方差。
节点积累点做PCA，最小特征值小于平面阈值则成面，否则继续细分；不是每个查询取五近邻QR。
新增超过5点时可重拟合，达到节点支持数上限后冻结平面并释放支持点。avia示例上限50，根格0.5m、最大层数2；这些不是MID360验证参数，也不是每帧观测数量上限。
后验位姿重新计算整帧降采样点及其世界协方差，再统一入图，不只使用匹配成功点。

匹配要求投影位于约3倍平面半径内，且点面距离通过默认3σ检查；最终逆方差为：

\[
w=\frac{1}{0.001+J_{plane}\Sigma_{plane}J_{plane}^\top+
n^\top\Sigma_{lidar,world}n}.
\]

关联时使用的查询点世界协方差含当前状态P，最终R的当前点项使用激光测量噪声；两者不能混为同一项。
地图点协方差引入建图位姿P块，但没有完整地图—当前状态交叉协方差，多个观测共享平面的相关性也未形成完整R。
可选滑动地图按空间立方体删除远处根格，avia示例`local_map/map_sliding_en=false`；这不是动态墙面清除。

源码：[平面字段，voxel_map.h:69](../../FAST-LIVO2/include/voxel_map.h#L69)、[PCA及平面协方差，voxel_map.cpp:55](../../FAST-LIVO2/src/voxel_map.cpp#L55)、[更新/冻结，:219](../../FAST-LIVO2/src/voxel_map.cpp#L219)、[权重，:447](../../FAST-LIVO2/src/voxel_map.cpp#L447)、[关联，:730](../../FAST-LIVO2/src/voxel_map.cpp#L730)、[地图滑动，:924](../../FAST-LIVO2/src/voxel_map.cpp#L924)、[后验入图，LIVMapper.cpp:413](../../FAST-LIVO2/src/LIVMapper.cpp#L413)。

### 5.3 视觉地图是另一套地图

另有0.5m哈希视觉点地图，保存世界点位置、法向、协方差、历史图像patch及参考观测。
利用LiDAR平面法向做patch单应变换，以亮度差作直接法残差；默认前向路径H含位姿＋逆曝光7列，可选逆组合路径为位姿6列。
新视觉点结合图像网格和Shi-Tomasi分数挑选；有深度不连续、patch误差门控。patch历史最多30份，低分观测可删；多次法向变化很小时标记收敛。
这些是视觉特征/参考图像管理，不是棚膜动静识别。NCC检查代码存在，但未见默认启用路径；LiDAR地图滑动也不能推断为视觉主地图同步清理。

源码：[VisualPoint，visual_point.h:28](../../FAST-LIVO2/include/visual_point.h#L28)、[视觉建点，vio.cpp:804](../../FAST-LIVO2/src/vio.cpp#L804)、[历史patch，:908](../../FAST-LIVO2/src/vio.cpp#L908)、[直接法残差/H，:1621](../../FAST-LIVO2/src/vio.cpp#L1621)。
指定大棚包无相机话题，因此不能用它验证FAST-LIVO2的视觉收益，只能未来明确关闭图像走LIO路径。

## 6. Point-LIO：本地版本的实现

### 6.1 默认30维、逐时间组更新

默认IMU输出观测模型；可选24维IMU输入模型。激光按真实点时间传播和更新，恰好同时间的点组成一组，不是所有情况下严格一行一更。
`ImuProcess::Process()`初始化后直接复制原始点；没有前三套整帧去畸变再冻结到末时刻的步骤。
激光H只含p/R/可选外参，无直接v/omega列；同一时刻点本身无需批内时间补偿链式项。
观测更新函数虽然含`iterated`，初始化中`maximum_iter=1`，不能描述为默认多轮整帧匹配。
IMU饱和判断本地为量程99%，LY为95%。

输出模型Q包含直接速度`vel_cov`（参数默认20）及omega/a/bg/ba；实际传播加`Q*dt²`。
当前LY速度单独加`rate*dt`，其他块加`Q*reference_dt*dt`，不能直接把20和0.1当成同单位参数。

源码：[状态，common_lib.h:25](../../Point-LIO/include/common_lib.h#L25)、[默认路径/参数，parameters.cpp:51](../../Point-LIO/src/parameters.cpp#L51)、[Q及传播方程，Estimator.cpp:42](../../Point-LIO/src/Estimator.cpp#L42)、[IMU预处理，IMU_Processing.cpp:102](../../Point-LIO/src/IMU_Processing.cpp#L102)、[时间组，common_lib.h:140](../../Point-LIO/include/common_lib.h#L140)、[事件主循环，laserMapping.cpp:590](../../Point-LIO/src/laserMapping.cpp#L590)、[单轮及P传播，esekfom.hpp:107](../../Point-LIO/include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp#L107)。

### 6.2 五近邻QR与IVox地图

查询地图5近邻，QR解`A n_raw=−1`，再归一化n/d，检查所有支持点到面的距离，最后用`range>match_s*r²`门控查询点。
激光残差使用固定`laser_point_cov`；`noise_state`及鲁棒权重相关代码是注释，没有启用。
QR是固定常数项的代数拟合；LY的PCA是中心化的正交距离拟合，能表达经过原点的平面。QR不天然更抗噪或更抗动态，应比较退化检测及实际数据，不能只比较算法名称。

**本地地图为IVox，不是其他Point-LIO分支常见的ikd-tree。**
哈希定位体素，主路径默认节点存`vector<PointT>`，插入追加多个点，节点本身无固定一点上限；PHC是可选编译分支，当前宏注释。
地图入口按`filter_size_map`格内已有代表点决定是否添加，属于空间去重；扫描结束统一调用`MapIncremental()`，不是把当前查询点立即入图与自身匹配。
根格插入刷新LRU，容量到阈值删尾。代码参数默认IVox分辨率0.2m、邻域类型18、容量100万格；Options结构自身邻域初值6，但运行加载会覆盖。
没有保存每点跨帧稳定性、时间衰减或动态概率。

初图累计到`init_map_size`（加载默认100点）后开始定位，不是LY固定五帧静止初图。

源码：[QR，common_lib.h:203](../../Point-LIO/include/common_lib.h#L203)、[激光残差/H，Estimator.cpp:218](../../Point-LIO/src/Estimator.cpp#L218)、[节点类型，parameters.h:27](../../Point-LIO/src/parameters.h#L27)、[多点节点，ivox3d_node.hpp:124](../../Point-LIO/include/ivox/ivox3d_node.hpp#L124)、[查询，ivox3d.h:139](../../Point-LIO/include/ivox/ivox3d.h#L139)、[插入/LRU，:261](../../Point-LIO/include/ivox/ivox3d.h#L261)、[入图筛选，laserMapping.cpp:119](../../Point-LIO/src/laserMapping.cpp#L119)、[扫描末入图，:1010](../../Point-LIO/src/laserMapping.cpp#L1010)。

## 7. 对大棚问题真正有参考价值的部分

### 7.1 三种“质量”应分开

| 要判断的事情 | 可以参考的证据 | 不能由它推出什么 |
| --- | --- | --- |
| 回波是否受到干扰 | MID360原始Tag、零坐标/无效回波、量程及空间分布 | 正常Tag不能证明棚膜固定 |
| 局部几何是否可靠 | 点的空间展布、平面厚度/特征值、拟合协方差、关联门控 | 小残差/小平面方差不能证明物体静止 |
| 这个结构能否长期作为定位锚点 | 相对固定柱/梁等参照的跨帧中心、法向和残差变化，重复观测覆盖与可观性 | 单帧五点、点数累计或LRU年龄不足以估计动静 |

四套代码主要覆盖前两层，第三层仍需单独诊断。Tag反映回波质量，风吹着移动的真实棚膜可以产生正常Tag、很多点和很薄的平面。

### 7.2 可以借鉴，但尚未证明能改善LY

1. **持久面片及支持范围。** FAST-LIVO2/GenZ保留局部平面身份、中心/法向/半径，适合检查同一处结构随时间变化；先要稳定关联并控制自身位姿误差造成的假变化。
2. **测量各向异性和标准化关联。** 量程/方向噪声、平面不确定度可以比统一R更细，但它们估计测量/几何误差，不是墙运动速度。
3. **非平面结构提供补充约束。** GenZ点到点路径能利用一部分不能成面的几何；重复棚架也有误关联风险，不能认为增加点点项一定降低漂移。
4. **对定位与入图分别判断。** 几套实现都会把未匹配点入图。若研究棚膜，应分别检查某点是否参与当前定位、是否改写长期地图，防止错误位姿和活动表面相互强化。

这里优先建议的下一步是**利用已确认静止的原始包检查局部结构的时间变化**：在原始雷达系固定空间区域统计棚膜和柱/梁的变化，保留相同视角/覆盖，和已录假位移比较；不能先用漂移轨迹对齐点云，再把对齐后的变化全部认作墙运动。
若只有水平地面可靠，它对水平位置和yaw约束有限；压低所有墙面权重可能让运动性能更差。定位和环境大范围同向运动还存在可辨识性限制。

### 7.3 与已经失败的尝试如何划界

必须先查[未采用方案记录](未采用方案记录.md)和[README中的Huber撤销记录](../README.md)。
现有PCA五支持点对角平面噪声、0.1ms批内共享支持GLS、连续初始化、仅删PCA比值门控和固定3cm Huber都不能因参考项目也出现类似名称而直接恢复。

持久平面地图与“给现有五点拟合加一个R”并非同一实现，但新的地图表示仍需要故障证据、明确误差机制和冻结的同源/运动留出测试。
Super-LIO的融合计数、FAST-LIVO2/GenZ的支持点上限和地图冻结只是地图管理策略，不能更名为已经验证的动态评分。
本轮没有新精度证据支持更改现有主算法，也没有新增实验参数或默认关闭分支。

## 8. 指定“大棚400s不动”包与后续比较规则

用户明确确认设备全程静止；包目录：[大棚400s不动](/home/hustlyrm/ly_odom/bags/大棚400s不动)。
录包接收跨度为395.113605022秒，总179636条消息；名字中的400s不是精确传感器评价时长。
原始点云为PointCloud2，原始IMU话题`/livox/imu`；另有已录LY的baselink里程计、TF和注册云，没有相机话题。
`/parameter_events`为0，也未发现参数快照，因此已录轨迹只能称为**采集时版本基线**，不能断言等于今天源码/YAML回放结果。

静止基线的传感器时间窗、结果及输入检查单独保存在包内
[analysis/lio_reference_20261017/已录静止基线.md](/home/hustlyrm/ly_odom/bags/大棚400s不动/analysis/lio_reference_20261017/已录静止基线.md)。
该目录只保留指标JSON、轨迹CSV、图和报告，不保留分析程序。

已录里程计的header时间覆盖391.500211秒，输出声明为`odom→baselink`：

| 静止指标 | 本包已录结果 |
| --- | ---: |
| 首末各5秒位置均值距离 | 14.8723cm |
| 首末单帧距离 | 15.0359cm |
| 相对首5秒均值的位置最大 / RMS | 16.4089 / 7.5737cm |
| 相邻输出位置差RMS | 2.8503mm |
| 消息线速度模长RMS | 9.0099cm/s |
| 首条已录输出起前10秒最大位移，相对首帧 | 1.3674cm |
| 末5秒首末变化 / 相对该窗首帧最大变化 | 0.6135 / 1.6436cm |
| Odom/TF/注册云精确同stamp三联 | 3916 / 3916组 |

位姿及twist均有限，同stamp的Odom和TF位姿差为0。消息速度是滤波器估计量，不等于轨迹差分速度；“前10秒”也不包含首条输出之前的内部初始化。
原始IMU间隔中位/最大为4.912685/6.046270ms；静止陀螺均值约`[-0.0164674,0.00827772,-0.00722654]rad/s`，其中包含零偏，不能未经估计扣除就直接积分归因。
原始云首/中/末3帧抽样均约2万点，XYZ全零占约21%；这只是抽样回波有效性指标，不是算法实际参与匹配点数，也未证明它造成14.87cm位移。
该包可以确认设备静止时的估计误移，尚不能单独分离IMU、棚膜运动、覆盖变化和地图反馈各自的贡献。
轨迹见[静止基线图](/home/hustlyrm/ly_odom/bags/大棚400s不动/analysis/lio_reference_20261017/stationary_baseline.png)。

### 8.1 接入前必须核对的代码差异

| 项目 | 本包直接接入的主要障碍 | 源码依据 |
| --- | --- | --- |
| Super-LIO | MID360默认订阅CustomMsg，本包为PC2；示例每3点取1、盲区2m、空间采样0.5m，末时刻取最后保留点 | [ROSWrapper.cpp:309](../../Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L309)、[:419](../../Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L419)、[配置](../../Super-LIO/src/super_lio/config/livox_360.yaml) |
| GenZ-LIO | 默认Livox CustomMsg；`livox_pcl`把PC2的timestamp当绝对纳秒并减header，本驱动为相对纳秒，不能直接原样使用 | [CloudConversion.cpp:129](../../genz-lio/ros/src/CloudConversion.cpp#L129)、[OdometryServer.cpp:203](../../genz-lio/ros/src/OdometryServer.cpp#L203) |
| FAST-LIVO2 | ROS1；无MID360专用配置；示例开相机，本包须关图像走ONLY_LIO；发布Odom/cloud用now而非传感器时间，离线应记录内部更新时间 | [package.xml:19](../../FAST-LIVO2/package.xml#L19)、[LIVMapper.cpp:887](../../FAST-LIVO2/src/LIVMapper.cpp#L887)、[:1340](../../FAST-LIVO2/src/LIVMapper.cpp#L1340)、[:403](../../FAST-LIVO2/src/LIVMapper.cpp#L403) |
| Point-LIO | ROS1；Livox入口是ROS1 CustomMsg，现有其他PC2入口不是本包Livox字段的等价适配 | [preprocess.cpp:316](../../Point-LIO/src/preprocess.cpp#L316)、[preprocess.h](../../Point-LIO/src/preprocess.h) |

Super-LIO、Point-LIO和GenZ的CustomMsg路径常用`tag&0x30`筛0/0x10，不等于解读全部SDK Tag质量位；GenZ的PC2转换未读取Tag，不能假设两入口接受点集相同。
GenZ/FAST-LIVO2按初始化加计模长缩放；本项目原始加计约以g为单位。任何适配须明确单位，不能重复乘9.81，ROS2还要匹配best-effort/reliable QoS。
首末点时间、加速度单位和外参错误均足以破坏对比，不能把接入错误归因于算法建模。

### 8.2 同源静止和运动评价

- 使用同一份原始单雷达/IMU数据，按传感器时间选共同窗；记录初始化尚无输出的区间，不用延迟显示隐藏启动漂移。
- 全部输出转换到同一个实际安装中心baselink，只做起点位姿对齐，不用全程配准/终点归零；同时保存原始IMU原点轨迹。
- 静止首末各5秒均值距离、相对首5秒均值的位置最大/RMS、相邻位置差RMS、速度RMS一起看；另列首10秒和末5秒。
- 完整核对传感器header、输出有限性、Odom/TF/注册云stamp、重建和处理计数；原始/转换话题全包数量不同，不能单凭数量认定节点漏发。
- 若未来比较整套项目默认配置，应明确采样、盲区、地图、R/Q和迭代等均不同，只能称“整套实现比较”。若定位某个机制的收益，应在同一LY基线上一次隔离一项。
- 只有这份静止包不能决定采用。候选还须通过124845闭合≤10cm和其他既定运动留出门槛；参考PLIO差异不是位置真值误差。

阅读时LY配置快照为`lidar_batch_duration=.0001`、空间采样`.20m`、地图`.50m/32点/5cm`、`laser_point_cov=.001`、`plane_threshold=.10`、`match_sqaured=81`、速度噪声率`.1`；不是本包已证明的采集配置。
[mid360.yaml](../config/mid360.yaml)的SHA256为`b6a22a93647f6e0c88b791e7915fc99cc611ec9147a7161af8db2564fff201ab`。
历史报告中的其他参数组合不能混作本轮基线；本轮未改配置、安装外参、PTP或参考源码。
