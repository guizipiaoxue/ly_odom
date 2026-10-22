# ly_lio 建模与实现说明

更新：2026-10-21。依据当前生产源码、MID360/L2配置及截至2026-10-17的模型变更记录重写。按数据流解释公式与代码，不沿用旧论文的章节、公式编号或轮速模型。调试、历史版本和失败方案统一见[验证.md](验证.md)。

## 1. 算法实际做什么

`ly_lio`在ROS 2 Humble下估计IMU原点状态。每个激光点补偿到所属Batch的真实批末，在同一先验下匹配历史地图PCA平面，累加逐点信息后更新一次完整状态。IMU角速度和比力是观测量，其对应物理量也在状态中，区别于直接将原始IMU作为输入的标准15维惯导。

当前状态流形和误差协方差均为24维，固定LiDAR—IMU外参不进入状态，不保留30维占位。滤波器内部没有轮速、在线外参、回环、绝对位置约束或平面对平面残差。2026-10-20新增独立ly_locate后端消费输出，维护map→baselink与关键帧图，不回写滤波状态/P。默认输入`/livox/lidar`和`/livox/imu`，独立转换节点已删除，注册云去畸变后在扫描末baselink中发布。

代码对应：`src/ly_lio/eskf.h`为状态/传播/更新，`so3_math.h`为SO(3)，`estimator.h/.cpp`为补偿和观测，`ly_lio.cpp`为事件/初始化，`body_output.h`为机体输出。以下源码路径均相对包目录。

## 2. 坐标、单位与固定外参

### 2.1 变换方向

记T_AB为B系到A系的变换：

$$
p_A=R_{AB}p_B+t_{AB},\qquad
T_{AB}=\begin{bmatrix}R_{AB}&t_{AB}\\0&1\end{bmatrix}.
$$

L为LiDAR，I为IMU，W为里程计世界系（ROS的odom），B为输出机体系（默认baselink）。W在启动建立，没有地理定位含义。初始姿态为单位阵，重力方向由静止数据表达在这个初始系，不会先把传感器转到水平世界系。

### 2.2 LiDAR→IMU

内部始终使用

$$
u=R_{IL}p_L+t_{IL}.
$$

| 配置约定 | YAML的extrinsic_T含义 | 内部处理 |
| --- | --- | --- |
| imu_in_lidar，当前缺省 | IMU原点在LiDAR系的位置c_L | t_IL=-R_IL*c_L，读取一次 |
| lidar_in_imu | LiDAR原点在IMU系的位置 | 直接作为t_IL |

extrinsic_R始终为LiDAR→IMU轴旋转。当前MID360为

$$
R_{IL}=I_3,\quad c_L=\begin{bmatrix}0.011&0.02329&-0.04412\end{bmatrix}^{T}\mathrm{m},
$$

$$
t_{IL}=-R_{IL}c_L=\begin{bmatrix}-0.011&-0.02329&0.04412\end{bmatrix}^{T}\mathrm{m}.
$$

不能把内部t_IL填回要求c_L的YAML再取反。固定外参只是确定参数假设，不证明真实标定误差为零；当前P不传播其不确定性。

### 2.3 IMU输入单位

角速度单位rad/s。加速度按静止模长换算为SI比力：

$$
s_f=\frac{\|g_{cfg}\|}{\texttt{acc_norm}},\qquad f_m=s_f a_{raw}.
$$

当前MID360驱动输入g，acc_norm=1.0、配置重力模长9.81m/s²。SI数据要相应设置acc_norm和原单位饱和量程。原始输入不预先扣重力，由状态传播中的Ra+g处理。

## 3. 状态、误差与SO(3)

### 3.1 排列与含义

$$
x=(p,R,v,\omega,a,g,b_g,b_a)\in SO(3)\times\mathbb R^{21}.
$$

| 状态 | 误差索引，从0开始 | 坐标/单位 |
| --- | --- | --- |
| p | 0–2 | IMU原点在W系的位置，m |
| R=R_WI | 3–5 | I→W旋转，误差为I系右扰动，rad |
| v | 6–8 | IMU原点W系速度，m/s |
| omega | 9–11 | I系真实角速度，rad/s |
| a | 12–14 | I系比力，m/s²，不是世界净加速度 |
| g | 15–17 | W系重力，m/s² |
| bg | 18–20 | I系陀螺零偏，rad/s |
| ba | 21–23 | I系比力零偏，m/s² |

旋转名义量存3×3矩阵但误差为三维。“24维”指流形维度和24×24的P，不是旋转矩阵展开后的存储数。定义

$$
R_{true}=R\operatorname{Exp}(\delta\theta),\quad p_{true}=p+\delta p,\quad v_{true}=v+\delta v,
$$

$$
\delta x=[\delta p,\delta\theta,\delta v,\delta\omega,\delta a,\delta g,\delta b_g,\delta b_a]^T.
$$

其余欧氏状态同样加性。误差注入为

$$
x^+=x^-\boxplus\delta x,\qquad R^+=R^-\operatorname{Exp}(\delta\theta).
$$

### 3.2 指数映射与雅可比

令叉乘矩阵满足[u]_×w=u×w，旋转向量phi的模长为theta：

$$
\operatorname{Exp}(\phi)=I+\frac{\sin\theta}{\theta}[\phi]_\times+\frac{1-\cos\theta}{\theta^2}[\phi]_\times^2,\quad \theta=\|\phi\|.
$$

$$
J_l(\phi)=I+\frac{1-\cos\theta}{\theta^2}[\phi]_\times+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2,\quad J_r(\phi)=J_l(-\phi).
$$

源码A_matrix(phi)对应J_l。小角度使用级数，例如

$$
\frac{\sin\theta}{\theta}\approx1-\frac{\theta^2}{6}+\frac{\theta^4}{120},\qquad
\frac{1-\cos\theta}{\theta^2}\approx\frac12-\frac{\theta^2}{24}+\frac{\theta^4}{720}.
$$

注入后从旧切空间转换协方差：

$$
G=\operatorname{diag}(I_3,J_r(\delta\theta),I_3,I_3,I_3,I_3,I_3,I_3),
$$

$$
P^+=\operatorname{sym}(G\widetilde P^+G^T),\qquad \operatorname{sym}(A)=\tfrac12(A+A^T).
$$

源码使用A_matrix(-delta_theta)。必须变换完整姿态行列及交叉块，不只改姿态3×3对角块。

## 4. 状态与协方差传播

### 4.1 连续模型和实际离散

$$
\dot p=v,\quad \dot R=R[\omega]_\times,\quad \dot v=Ra+g,\quad \dot g=0.
$$

omega/a/bg/ba的名义导数为零，用过程噪声描述变化；速度另允许直接方差增长。对正时间步长，实际使用区间起点状态：

$$
p_{k+1}=p_k+v_k\Delta t,\quad
v_{k+1}=v_k+(R_ka_k+g_k)\Delta t,\quad
R_{k+1}=R_k\operatorname{Exp}(\omega_k\Delta t).
$$

其余名义状态不变。位置没有半加速度乘时间平方项；不能把二阶理想积分写成当前生产模型。

### 4.2 误差转移矩阵

按八个三维状态块排列，未列项为零：

$$
F=\begin{bmatrix}
I&0&I\Delta t&0&0&0&0&0\\
0&E&0&A&0&0&0&0\\
0&S&I&0&B&I\Delta t&0&0\\
0&0&0&I&0&0&0&0\\
0&0&0&0&I&0&0&0\\
0&0&0&0&0&I&0&0\\
0&0&0&0&0&0&I&0\\
0&0&0&0&0&0&0&I
\end{bmatrix},
$$

$$
E=\operatorname{Exp}(-\omega\Delta t),\quad A=J_r(\omega\Delta t)\Delta t,\quad
S=-R[a]_\times\Delta t,\quad B=R\Delta t.
$$

每个IMU、激光批末和扫描末事件先在区间起点计算F推进P，再推进x到同一真实时间；只传播各自尚未覆盖的正时间段。稀疏左右乘是完整FPFᵀ，不删交叉协方差。

### 4.3 过程噪声

令tau为process_noise_reference_dt，q_v为velocity_process_noise_rate，代码构造

$$
Q=\operatorname{diag}(0_3,0_3,(q_v/\tau)I_3,q_\omega I_3,q_aI_3,0_3,q_{bg}I_3,q_{ba}I_3),
$$

$$
P^-_{k+1}=\operatorname{sym}(FP^+_kF^T+Q\tau\Delta t).
$$

速度块直接增加q_v*dt，q_v单位m²/s³；其余四组仍加q*tau*dt，不是直接q*dt或q*dt²。当前MID360 tau=.005s、q_v=.1、q_omega=1000、q_a=500、q_bg=q_ba=.0001；q_v的代码缺省为0，由MID360 YAML覆盖为.1。旧velocity_cov停用且不迁移。

直接随机游走累计量按总时长，不随点分段变密缩小；名义一阶积分、F及噪声传递仍有离散误差，不能说不同分段完整等价。

## 5. IMU输出观测与逐轴屏蔽

### 5.1 残差、雅可比、测量方差

$$
y_m=\begin{bmatrix}\omega_m\\f_m\end{bmatrix},\quad
h_I(x)=\begin{bmatrix}\omega+b_g\\a+b_a\end{bmatrix},\quad z_I=y_m-h_I(x).
$$

$$
H_I=\begin{bmatrix}
0&0&0&I&0&0&I&0\\
0&0&0&0&I&0&0&I
\end{bmatrix},\quad R_I=\operatorname{diag}(\sigma_\omega^2I_3,\sigma_a^2I_3).
$$

imu_meas_omg_cov与imu_meas_acc_cov为这里的方差，不是标准差；加计方差对应SI残差。其他状态经完整P交叉项更新。

### 5.2 饱和轴

令M为六轴有效掩码，启用check_satu时

$$
M=\operatorname{diag}(m_1,\ldots,m_6),\quad \bar z_I=Mz_I,\quad \bar H_I=MH_I,
$$

$$
m_{\omega,i}=\mathbf1(|\omega_{m,i}|<0.95s_\omega),\qquad
m_{a,i}=\mathbf1(|a_{raw,i}|<0.95s_a^{range}).
$$

等于量程95%也屏蔽；比较加速度原输入而非缩放后值。当前3g、35rad/s量程对应2.85g、33.25rad/s阈值。一轴饱和不丢整条消息。代码保留6×6系统，屏蔽轴H/创新归零，R保持正值，与删除该观测轴等价。

### 5.3 Kalman更新和Joseph协方差

$$
S_I=\bar H_IP^-\bar H_I^T+R_I,\quad K=P^-\bar H_I^TS_I^{-1},\quad \delta x=K\bar z_I,
$$

$$
\widetilde P^+=(I-K\bar H_I)P^-(I-K\bar H_I)^T+KR_IK^T.
$$

实现采用等价展开P-KHP-(KHP)ᵀ+KS_IKᵀ，随后统一注入/reset。非有限数、分解失败或系统非正拒绝该次更新，不硬改噪声强行输出。

## 6. 一帧点云怎样按真实时间分批

### 6.1 时间与采样

当前Livox PointCloud2的timestamp为相对header的纳秒：

$$
t_j=t_{header}+10^{-9}\,\texttt{timestamp}_j.
$$

CustomMsg为绝对UTC纳秒timebase加offset；L2示例为相对header秒。按各适配器解释，不以接收时间、now或均匀5ms网格替换。严格PTP未锁驱动丢包；时钟一致不等于物理采样同时。

适配后先检查有限坐标/时间和时间水位、排序，距离过滤前最晚有效点为scan_end。然后距离过滤、按原有序索引步长筛点、可选体素采样；体素取距中心最近的真实坐标/时间对，不平均不同时刻点。

scan.points为跟踪/地图代表点，scan.dense_points为距离合格注册云。两者共同决定批界，只有代表点参与激光信息和地图增量；过滤后无末段点也要传播至原scan_end。

### 6.2 分界和事件优先级

MID360/C++批长上限.001s，严格0<duration<.010s；L2示例.002s。从下一点t_0开始，入批条件为

$$
t_j\le\min(t_0+d_b,t_{scan,end}),\qquad t_j<t_{next\ IMU}.
$$

实际批末取入批代表点/稠密点中最晚真实点，不用上限补齐。遇IMU旧批提前结束，同stamp先IMU再点。算法等待IMU水位覆盖整扫描后才处理，但不会把未来IMU观测倒灌更新过去激光点。

## 7. 运动补偿、PCA平面、残差与完整H

### 7.1 批末状态函数

保留原始点和相对时间，在批内匀速/匀角速近似下：

$$
\Delta_j=t_j-t_e\le0,\quad u_j=R_{IL}p_{Lj}+t_{IL},\quad D_j=\operatorname{Exp}(\omega\Delta_j),
$$

$$
q_{e,j}=D_ju_j+R^Tv\Delta_j,\quad
y_j=p+Rq_{e,j}=p+RD_ju_j+v\Delta_j.
$$

q_e为批末IMU系坐标，y为世界坐标。共同线性化先验固定，但补偿仍是R/v/omega的函数，没有额外批内加速度/角加速度随机轨迹或独立时间误差状态。

### 7.2 五近邻PCA与门控

在当前格和六个面邻格的候选内选最近5点，不是无界全局最近邻。

$$
\mu=\frac15\sum_{i=1}^{5}m_i,\quad C=\frac15\sum_{i=1}^{5}(m_i-\mu)(m_i-\mu)^T,\quad
\lambda_0\le\lambda_1\le\lambda_2.
$$

n为最小特征值单位特征向量，d=-nᵀmu，平面可经过原点。恰好5个邻点且分解成功后，必须满足

$$
\lambda_1\ge10^{-6},\quad \lambda_1\ge10^{-4}\lambda_2,\quad \lambda_0\le0.1\lambda_1,
$$

$$
|n^Tm_i+d|\le\texttt{plane_threshold}\quad(\forall i).
$$

它们排除重合/近共线和过厚平面。对查询点构造

$$
h_j=n_j^Ty_j+d_j,\qquad z_j=-h_j,\qquad
\|q_{e,j}\|>\texttt{match_sqaured}\,h_j^2.
$$

match_sqaured保持原参数拼写和经验门控，左边是距离而非距离平方。每个源点独立匹配，不拟合源平面，不要求每批至少5源点。

### 7.3 对补偿状态求导

冻结共同先验与匹配平面，对预测h求导。令phi_j=omega*Delta_j：

$$
J_{y,j}=\left[I_3,\ -R[D_ju_j]_\times,\ \Delta_jI_3,\ -\Delta_jRD_j[u_j]_\times J_r(\phi_j)\right],
$$

$$
H_{o,j}=n_j^TJ_{y,j}\in\mathbb R^{1\times12},\qquad H_j=[H_{o,j},0_{1\times12}].
$$

直接顺序为[p,R,v,omega]。c=Rᵀn、N=Dᵀc时，源码等价形式为

$$
H_p=n^T,\quad H_R=(D_ju_j\times c)^T,\quad H_v=\Delta_j n^T,\quad
H_\omega=\Delta_j(u_j\times N)^TJ_r(\phi_j).
$$

姿态关键相消是R*(Rᵀv*dt)=v*dt，因此不能把包含平移项的完整q_e放入姿态叉乘。角速度项来自

$$
\operatorname{Exp}(\phi+\delta\phi)\simeq\operatorname{Exp}(\phi)\operatorname{Exp}(J_r(\phi)\delta\phi).
$$

Delta=0时v/omega直接H为零，退化到同一时刻点到面观测。a/g/bg/ba直接H为零，但经完整P交叉项修正。

### 7.4 独立点噪声和共享状态误差

原始点独立各向同性噪声及残差方差为

$$
\Sigma_L=\sigma_L^2 I_3,\quad \sigma_L^2=\texttt{laser_point_cov},\quad
G_{n,j}=-n_j^TRD_jR_{IL},
$$

$$
r_j=G_{n,j}\Sigma_LG_{n,j}^T=\sigma_L^2.
$$

单位法向和正交旋转使方差不变；源码用等价的-nᵀR投影。当前不加PCA支持点对角方差或共享支持点GLS。共享补偿状态误差已在H_j P H_kᵀ内，不能再当独立测量噪声加一遍。地图平面作为固定参考；建图姿态、地图—状态、跨批支持复用误差未由此自动建模。

## 8. 全批信息累加与12维白化求解

令C_o=[I_12,0]，H_j=H_oj*C_o。全部合格点累加

$$
\Lambda=\sum_jH_{o,j}^Tr_j^{-1}H_{o,j},\qquad \eta=\sum_jH_{o,j}^Tr_j^{-1}z_j.
$$

对应误差最小二乘问题：

$$
\min_{\delta x}\ \tfrac12\delta x^T(P^-)^{-1}\delta x+\tfrac12\sum_j(z_j-H_j\delta x)^Tr_j^{-1}(z_j-H_j\delta x).
$$

该式说明与堆叠Kalman的等价关系，生产代码不显式逆完整P或点数平方的创新矩阵。定义

$$
P_{oo}=C_oP^-C_o^T=LL^T,\quad P_{xo}=P^-C_o^T,\quad B_o=P_{xo}L^{-T},
$$

$$
V=L^T\Lambda L,\qquad A_o=I_{12}+V.
$$

完整24维更新量和注入前P为

$$
\delta x=B_oA_o^{-1}L^T\eta,\qquad \widetilde P^+=P^- -B_oA_o^{-1}VB_o^T.
$$

P_xo为24×12，故求解12维不等于只更新12维。单平面Lambda可能秩亏，I+LᵀLambda L仍正定；代码先Cholesky分解P_oo，再LDLT分解A_o。最后一次注入/reset。

一批固定名义先验、P、关联和平面，各点只累加，不逐点注入、不多轮重匹配。所有通过既有门控的代表点均参与，无2000点上限；无合格点或分解失败则不执行激光更新。

## 9. 后验点和地图更新

该批更新后，代表点和稠密点都用后验p/R/v/omega重算世界坐标。未匹配成功的代表点也形成地图增量，仍受入图点距/容量限制。扫描内地图冻结，只参考此前扫描；scan_end统一入图。早批只用其自身后验，后续批不回头平滑早批，扫描末一帧云不等于全扫描末状态重新优化。

地图为有符号三维整型键、稠密哈希和有界链表。每格多真实点，同格点距去重、满格循环替换、新格超容量淘汰尾格。插入/触及已有格刷新链表，近邻查询不刷新；LRU指这个实际策略。点距检查限同格，不保证跨格全局最小点距。导出用均值地图不反馈匹配地图。

当前MID360 YAML：采样0.20m、地图格0.50m、容量100万格、每格128点、同格点距0.05m、laser_point_cov=.001m²、plane_threshold=.01m、match_sqaured=25。C++缺省、L2和旧报告不全相同，不能以旧0.20m/20点或0.1ms配置代替当前有效值。

## 10. 初始化、初图和重建

启用IMU且fix_gravity_direction=true时，用末N条IMU（MID360 N=200）估SI比力/角速度均值和各轴std：

$$
\|\bar\omega_m\|\le0.05,\quad \max_i\operatorname{std}(\omega_{m,i})\le0.02,\quad
\max_i\operatorname{std}(f_{m,i})\le0.30,
$$

$$
\left|\|\bar f\|-\|g_{cfg}\|\right|\le0.75,\qquad \|\bar f\|\ge10^{-6}.
$$

以上是当前启动门限，单位分别rad/s和m/s²。初始化

$$
p=0,\ R=I,\ v=0,\ \omega=0,\quad
g=-\frac{\bar f}{\|\bar f\|}\|g_{cfg}\|,
$$

$$
b_g=\bar\omega_m,\qquad b_a=\bar f+g,\qquad a=-g.
$$

单姿态不能分离完整倾角和横向加计偏置。fix_gravity_direction仅控制启动，不是在追踪时锁死g；禁用分支使用配置重力/默认偏置，不代表运动启动已经验证。

reset令P=.01I_24，再将g/bg/ba对角块分别置.0001/.001/.001，交叉项零。标定前扫描全部丢弃，继续静止收至少5个含有效点扫描、至少100个成功入图点。初图阶段按固定初姿态收点，不正常连续传播融合；静止门限失败就清图重等。追踪从初图末扫描开始，丢已覆盖IMU。这是分阶段初始化；极慢运动仍可能漏检，不保证绝对静止。

IMU实际缺段>.1s或IMU队列溢出触发重标定/重建odom，原点重设；清旧Path/导出地图，固定外参和Q不改。扫描缓冲溢出丢完整旧扫描；重复/倒退IMU拒绝，循环bag须reset，不伪造中间测量或跨缺段连续轨迹。

## 11. 从IMU到机体输出及协方差

本包安装文件odom_ws/src/ly_lio/config/lidar2baselink.yaml给固定T_BL：

$$
T_{BI}=T_{BL}T_{IL}^{-1},\qquad T_{WB}=T_{WI}T_{BI}^{-1}.
$$

令R_IB=R_BIᵀ、r_IB=-R_IB*t_BI，即B原点在I系的位置：

$$
R_{WB}=RR_{IB},\quad p_B^W=p+Rr_{IB},\quad v_B^W=v+R(\omega\times r_{IB}),
$$

$$
\omega_B=R_{BI}\omega,\qquad v_B^B=R_{BI}(R^Tv+\omega\times r_{IB}).
$$

ROS twist为当前B系，pose旋转误差为W系。相对于24维误差的输出雅可比为

$$
J_{pose}=\begin{bmatrix}
I&-R[r_{IB}]_\times&0&0&0&0&0&0\\
0&R&0&0&0&0&0&0
\end{bmatrix},
$$

$$
J_{twist}=\begin{bmatrix}
0&R_{BI}[R^Tv]_\times&R_{BI}R^T&-R_{BI}[r_{IB}]_\times&0&0&0&0\\
0&0&0&R_{BI}&0&0&0&0
\end{bmatrix}.
$$

$$
P_{pose}=J_{pose}PJ_{pose}^T,\qquad P_{twist}=J_{twist}PJ_{twist}^T.
$$

保留姿态/速度/角速度和杆臂交叉项，不添加已删除的外参状态方差。核心注册点先在W=odom；发布时统一变到扫描末机体系：`p_B=T_BI*inverse(T_WI_end)*p_W`，即`R_BW=R_BI*R_endᵀ`、`t_BW=t_BI-R_BW*p_end`。包含安装旋转和杆臂，不只更改frame_id。LIO不广播动态TF；内部地图与可选LIO累计导出仍用odom点，不能累加机体系局部云。静态TF baselink→livox_frame严格为实际T_BL，不误标非零芯片偏移的IMU原点。也可直接给互斥的固定imu_to_body_R/T并清空安装文件参数。

云台中心Odometry由真实Batch后验按odometry_publish_rate（默认100Hz、有限正数）选择输出，以扫描末为锚反向分槽；扫描末必发、同stamp去重，稀疏数据不补造样本。publish_odometry_per_batch=true发布全部真实批末状态。注册云仍按扫描末发布且在baselink中。LIO不再广播odom→baselink动态TF，保留baselink→livox_frame静态安装TF。整帧等IMU覆盖后集中处理发送，100Hz是传感器时间目标，不等于墙钟每10ms实时输出；不新增传播/预测，不用now重盖旧状态。

## 12. 配置、显示和运行边界

各C++节点直接读所属文件，源码优先/share回退，原生ROS文件/CLI/组件参数覆盖，修改重启生效。算法config/mid360.yaml、驱动包ROS YAML与SDK JSON、录包包内odom_ws/src/ly_bag_recorder/config/rosbag.yaml、LIO包内安装文件分工明确；shell/launch不合并/生成/注入参数，统一入口仅start_lidar和record_bag。

extrinsic_est_en=true明确拒绝，旧false告警兼容；旧外参先验也拒绝。continuous_initialization、init_imu_duration、plane_noise_model均无当前实现；旧publish_odometry_without_downsample=true仅告警映射到按批，不代表原始逐点发布。

终端用首个有效完整body位姿T_0建立B0显示系，显示T_rel=T_0^-1*T_WB，世界速度/重力也按B0转轴。只影响显示，不改滤波、ROS/TF或stamp；GOOD不是精度保证，reset/重建重新选B0。

安装目标目前为云台中心。活动云台到真实底盘需有时戳的关节角与动态轴心几何；但用户确认共轴纯yaw时中心水平摆动仍是误差，不能只用frame命名解释。

## 13. 尚未覆盖的误差与验证边界

- 批内匀速/匀角速，不估独立随机轨迹、角加速度、点时间误差或LiDAR–IMU物理时延。
- 地图/PCA平面/关联固定，不含建图位姿误差、跨批持久地图不确定性、状态—地图交叉P和共享支持噪声。
- 单姿态及单平面存在可观性局限，联合更新不会凭空提供未观测方向信息。
- 固定外参可能有真实误差；无回环/绝对约束，不能保证长期零漂。
- 数值等价、ROS接口正确、同源改动回归、独立实机精度必须分开，旧600秒或参数搜索不覆盖当前独立验收。

后续改模型前先查[验证.md](验证.md)第12节，不以换名、默认关闭或单一指标改善恢复旧失败方案。


## 14. 输出会话契约（2026-10-21）

ScanSession随每个已选Odometry同stamp发布（reliable/transient_local/depth1000），覆盖全部扫描末云；header.frame_id仍为odom，表示会话固定参考系。reset/重建/重启换UUID。后端按完整纳秒stamp关联标签和T_OB，不插值/外推，不以到达顺序猜测；复用stamp跨会话或位姿冲突时保守拒绝。

后端直接消费/ly_lio/odometry，不再订阅TF。loop只读取扫描末baselink注册云并保存局部关键帧；map只读取关键帧/完整图和轻量Odometry/会话标签，不订阅、转换或缓存实时整帧注册云。删除/location/cloud_registered及live_preview分支。固定map中逐帧按优化T_MB重建；已提交图的末关键帧给出T_MO=T_MB_k*inverse(T_OB_k)，同stamp当前位姿T_OB经T_MB=T_MO*T_OB直接发布map→baselink。odom仅保留为内部固定参考系和消息字段，不发布涉及odom的动态TF。未提交完整地图前没有map→baselink；旧图提交时允许同stamp发布修正更新。

注册云是各Batch后验点在扫描末机体系的刚体表达，后端不能重新估计扫描内状态或修复去畸变。见[ly_locate说明](../../../../location_ws/src/ly_locate/README.md)。

## 15. 独立建图退出保存

### 15.1 固定map参考系（2026-10-22）

map取本次LIO会话静止标定和初图完成后首个真实输出时刻`t0`的机体baselink系。内部odom仍以初始IMU原点为参考，固定安装组合`T_BI=T_BL*inverse(T_IL)`不变，`T_OB0=T_OI0*inverse(T_BI)`已经包含安装旋转及杆臂。定义固定`T_MO0=inverse(T_OB0)`，将首关键帧先验左乘为`T_MO0*T_OB_k`，相邻里程计/回环因子仍使用局部相对SE(3)。首关键帧可能晚于`t0`，不能把后端首个收到的位姿或首个有效关键帧重新设为零。

无回环时`T_MB(t)=T_MO0*T_OB(t)`，因而`T_MB(t0)=I`；map轴向为`t0`的baselink轴向，不做额外重力水平化。回环在固定map中优化`T_MB_k`，实时修正仍为`T_MO(t)=T_MB_k*inverse(T_OB_k)`。各局部点按`p_M=T_MB_k*p_Bk`重建，路径、TF和导出PCD保持相同坐标约定。

节点在机体外参变换后的首个真实Odometry冻结`origin_stamp/origin_pose=T_OB0`，每条ScanSession重复携带并注明body_frame；reset/缺段重建时与UUID一起清除。LoopUpdate携带相同固定原点，晚启动也可恢复；原点冲突拒绝拼图。session.yaml和导出manifest.yaml以17位double记录固定`T_MO0`及整数纳秒原点时刻，manifest格式升为ly_locate_keyframes_v2。旧会话消息/地图需从原始输入重算，不能混喂新接口。此变换只定义后端坐标，不改变LIO估计、P或噪声。

### 15.2 退出保存

ly_locate的loop运行中按session归档局部关键帧PCD、poses.csv、loops.csv、graph.g2o与session.yaml到maps/location。此目录不等于拼接地图导出。map节点的/location/map_save服务导出已提交完整版本；默认save_on_shutdown=true，在正常Ctrl+C/SIGTERM导致executor停止后先join工作线程，再基于最新已收到且局部关键帧齐全的图快照进行文件导出，不依赖已停止的ROS服务，也不等待周期可视化重建限频。

导出根由location.yaml的save_directory指定，默认maps/location_export；新建session/revision/随机后缀目录，写XYZ二进制压缩map.pcd、keyframes/*.pcd、17位double位姿/整数纳秒stamp的poses.csv、loops.csv和manifest.yaml，临时目录完成后rename，不覆盖历史。空图不导出；缺局部关键帧/写盘错误报错并非零退出，不用旧图代替最新图。launch给map节点30秒SIGTERM升级宽限。最后尚未传给map的后端更新不保证纳入，SIGKILL或断电不会运行正常退出保存。该改动只补保存生命周期，不改变地图筛选、坐标公式和LIO估计。


## 16. 点云链路与内存

实时预览点云链路已删除，map只消费局部关键帧/完整图和轻量Odometry/会话标签。注册云DDS发布和loop订阅深度均为10，高频小消息使用独立队列。机体位姿、会话关联和直接map→baselink公式见第11、14节。

保留两个有独立职责的后端业务节点（loop与map）；无转换/预览点云节点、无TF监听隐藏节点。必要的局部关键帧和完整地图仍占内存，本轮未测RSS，不承诺固定降幅。
