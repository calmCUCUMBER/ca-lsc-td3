# CA-LSC-TD3 方法蓝图

## 研究边界

任务固定为 Lift+Cruise VTOL 在恒高条件下由悬停转换到固定翼巡航。强化学习调度唯一的连续转换分配因子：

\[
\lambda\in[0,1]
\]

\(\lambda=0\) 表示 multicopter-dominated，\(\lambda\to1\) 表示
fixed-wing-dominated。它同步分配两条低层路径：升力旋翼名义 collective
卸载和 PX4 MC/FW 姿态控制权转移；它不直接给出电机、舵面或 pusher 命令。
Pusher 始终由独立空速控制器控制，高度和姿态稳定仍由 PX4 低层回路完成。

PX4/低层 VTOL 飞控负责稳定跟踪合法的 \(\lambda_d\)，算法层不直接控制 pitch、
elevator、pusher 或 motor RPM。外部命令有效时，同一个
\(\lambda_{exec}\) 决定 collective 和 MC/FW 权重；命令丢失时立即恢复旋翼支撑
并回退 PX4 原生空速融合。\(\eta_C\) 是 RL observation、physics prior 和
safety feature，不另建一条 \(\eta_C\)-aware PX4 blending 支路。

## 物理能力表征

论文和 Python 核心统一使用 SI 单位与“竖直向上为正”：高度 \(h\)、垂向速度 \(v_z\)、期望垂向加速度 \(a_{z,d}\) 和航迹角 \(\gamma\) 的正方向均对应上升；pitch 正方向为抬头。PX4 的 local-position NED \(z\) 与 \(v_z\) 必须在 telemetry adapter 边界取反，不能把 NED 符号直接送入公式。舵偏和力矩采用带符号的机体纵向约定，并与实际气动数据库保持一致。

动压与机翼升力估计：

\[
\bar q=\frac12\rho V_a^2,\qquad
\hat L_w=\bar qS\hat C_L(\alpha)
\]

机翼竖直承载能力：

\[
\eta_L=\operatorname{sat}_{[0,1]}
\left(\frac{\hat L_w\cos\gamma\cos\phi}
{\hat m(g+a_{z,d})+\varepsilon}\right)
\]

算法只能使用估计质量 \(\hat m\)，实验必须显式加入质量估计误差。

Shadow fixed-wing pitch controller 始终在后台给出与 \(\lambda\) 解耦的总非饱和力矩需求。实现层先减去当前舵面估计力矩，将其转换为相对当前 elevator command 的增量需求 \(\Delta M_{\rm req}^{FW}\)。不能用“总需求”除以“剩余增量能力”，否则量纲虽相同但物理口径不一致。

工程遥测分两级：schema 13 记录的 `VehicleTorqueSetpoint.xyz[1]` 是 PX4
normalized demand，只能用来定位两套控制器何时争用、失权或撞限；论文公式的
\(\Delta M_{req}^{FW}\) 必须来自带物理尺度的 shadow-controller 输出或经独立
辨识的映射。在该映射完成前，任何 normalized proxy 都不得标作 \(\eta_C\)。

所需舵偏增量方向由下式决定，而不能只看力矩正负：

\[
\operatorname{sgn}(\Delta\delta_{e,\rm req})
=\operatorname{sgn}\left(
\frac{\Delta M_{\rm req}^{FW}}{\hat C_{m_{\delta_e}}}
\right)
\]

按照该方向计算 elevator 剩余舵量，得到：

\[
M_{e,\rm ava}=\bar qSc|\hat C_{m_{\delta_e}}|\Delta\delta_{e,\rm ava}
\]

纵向气动控制余度：

\[
\eta_C=g_q\operatorname{sat}_{[0,1]}
\left(1-\frac{|\Delta M_{\rm req}^{FW}|}{M_{e,\rm ava}+\varepsilon}\right)
\]

其中 \(g_q\) 是动压 smoothstep 门函数，避免低空速、低力矩需求时误判为已经具有控制能力；若 \(|\hat C_{m_{\delta_e}}|\) 或所需方向的剩余控制能力近零，则直接令 \(\eta_C=0\)。本文统一采用俯仰力矩、elevator 舵偏和 \(C_{m_{\delta_e}}\) 的带符号约定。

统一门函数为：

\[
\mathcal S(x;x_0,x_1)=
\begin{cases}
0,&x\le x_0\\
3\xi^2-2\xi^3,&x_0<x<x_1\\
1,&x\ge x_1
\end{cases},\quad
\xi=\frac{x-x_0}{x_1-x_0}
\]

## Physics prior 与 residual action

\[
g_L=\mathcal S(\eta_L;\eta_{L,0},\eta_{L,1}),\qquad
g_C=\mathcal S(\eta_C;\eta_{C,0},\eta_{C,1})
\]

\[
\lambda_{\rm phy}=g_Lg_C
\]

Actor 只输出 \(a_t^{RL}\in[-1,1]\)：

\[
\lambda_d=\operatorname{sat}_{[0,1]}
(\lambda_{\rm phy}+\delta_\lambda a_t^{RL})
\]

因此策略学习的是相对物理先验应提前或推迟多少，而不是从零学习整个卸载命令。

## 时序编码和四 Critic

12 维观测固定为：

\[
[V_a,\alpha,\gamma,\eta_L,\eta_C,e_h,v_z,\theta,q,
\lambda_{\rm exec},P_L,P_P]
\]

CfC 还接收真实采样间隔 \(\Delta t_t\)。推荐初始 Actor 为 `FC(64) -> CfC(64) -> FC(64) -> tanh(1)`；sequence length 为 20，burn-in 为 5。

上述 12 个量跨越 \([0,1]\) capability、角度、速度和瓦级功率等不同尺度。`pack_observation` 只固定字段顺序并输出原始 SI 值；进入任一网络前必须使用只由训练集拟合的 normalization/clipping，评估时冻结统计量，并让 MLP、LSTM、CfC 共享完全相同的预处理，避免网络对比受到尺度偏差影响。

TD3 使用两套 reward critic 与两套 safety critic：

\[
Q_R^{cons}=\min(Q_{R,1},Q_{R,2}),\qquad
Q_C^{cons}=\max(Q_{C,1},Q_{C,2})
\]

Actor 目标：

\[
J_\pi=\mathbb E[Q_R^{cons}-\beta Q_C^{cons}
-\kappa_r(a_t^{RL})^2]
\]

\(\beta\) 根据约束预算通过 Lagrange multiplier 自适应更新。Reward 必须使用 lift rotors 与 pusher 的总推进功率；safety cost 与 reward 分离，并包含 shield intervention \(|\lambda_d-\lambda_{exec}|\)。

## 连续转换分配与执行层

Hard shield 和非对称 rate limiter 得到 \(\lambda_{exec}\) 后，升力分配为：

\[
T_{L,\rm ff}=(1-\lambda_{exec})T_{L,\rm ref}.
\]

姿态分配使用可审计的单调映射：

\[
w_{FW}=S_\lambda(\lambda_{exec})=
\operatorname{sat}_{[0,1]}
\frac{\lambda_{exec}-\lambda_0}{\lambda_1-\lambda_0},\qquad
w_{MC}=1-w_{FW},
\]

当前冻结 \(\lambda_0=0.1,\lambda_1=0.9\)。PX4 的 roll/pitch/yaw 三轴均使用
该权重，避免空速融合与 \(\lambda\) 同时争用控制权；pusher 不与
\(\lambda\) 绑定。该同步分配是高层动作的低层执行定义，不替代
CA-LSC-TD3 的 CfC、physics residual 和 twin safety critics 创新。

Hard shield 根据空速、迎角、\(\eta_L\)、\(\eta_C\)、高度误差和下降率建立预定义命令包线：

\[
\lambda_{proj}=\min(\lambda_d,\lambda_{max}^{hard})
\]

非对称速率限制满足 \(\dot\lambda_{max}^{+}<\dot\lambda_{max}^{-}\)，即卸载慢、恢复快。若 hard envelope 突然收缩到当前 \(\lambda_{exec}\) 以下，硬边界优先并绕过常规恢复速率限制，保证实际命令不高于当前 \(\lambda_{max}^{hard}\)。论文只能宣称该层执行预定义转换命令包线，不能在没有不变集或 Lyapunov/CBF 证明时宣称飞行状态的数学安全保证。

## 必须在编码前定下的参数

- 选择纵向模型还是完整 6-DOF；
- 实际 Lift+Cruise 机体与 \(C_L(\alpha)\)、\(C_{m_{\delta_e}}\)；
- \(\eta_L\)、\(\eta_C\) 门限与动压门限；
- residual 幅值 \(\delta_\lambda\)；
- soft/hard envelope、约束预算和成功/失败条件；
- rotor 与 pusher 的推力/功率模型；
- recurrent target hidden-state 与 episode 边界处理。
