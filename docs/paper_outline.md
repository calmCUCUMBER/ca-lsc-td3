# 论文结构草案

暂定题目：

> Control-Authority-Aware Liquid Safety-Constrained TD3 for Safe and Energy-Efficient Transition Control of Lift+Cruise VTOL UAVs

中文题目可写为：

> 基于控制权感知液态安全约束 TD3 的 Lift+Cruise VTOL 安全节能转换控制

## 1 Introduction

1. Lift+Cruise 转换中“机翼托得住”和“气动舵面控得住”是两个不同的渐进过程。
2. 固定空速阈值不能充分表征质量、风和模型误差下的真实转换能力。
3. 标准 TD3 只优化 reward，瞬时 MLP 状态难以处理采样抖动、延迟与动态趋势，且外部 filter 无法证明 Actor 自身学会安全。
4. 给出三项贡献：
   - \(\eta_L+\eta_C\) 转换能力表征；
   - CfC + physics residual + twin reward/safety critics + Lagrangian 的 CA-LSC-TD3；
   - learned policy、hard command envelope 与低层 autopilot 的分层架构及系统实验。

避免使用“首次用 TD3 做 VTOL 转换”“严格保证安全”等未经证实的表述。

## 2 VTOL Transition Modeling and Problem Formulation

### 2.1 Lift+Cruise dynamics and propulsion power

定义坐标系、纵向或 6-DOF 方程、机翼气动力、lift rotors、pusher、舵面、执行器动态及总推进功率。明确所有符号和可测/估计/仿真真值的边界。

### 2.2 Hierarchical transition control

定义连续转换分配因子 \(\lambda\)。RL 只输出该单一高层量；allocator 用同一
\(\lambda_{exec}\) 同步 lift-rotor unloading 与 PX4 MC/FW attitude authority，
pusher 空速控制和高度/姿态稳定仍由低层完成。

### 2.3 Wing vertical-support capability

推导 \(\eta_L\)，并解释估计质量 \(\hat m\) 与实际质量 \(m\) 的区分。

### 2.4 Shadow controller and aerodynamic control authority

给出 shadow FW pitch controller、当前舵面力矩、增量力矩需求、考虑 \(C_{m_{\delta_e}}\) 符号的方向相关 elevator 剩余余度、动压门函数及 \(\eta_C\)。说明其与 \(\lambda\) 解耦，并避免把总力矩需求与剩余增量能力混用。

### 2.5 Constrained sequential decision problem

给出 12 维观测、residual action、performance reward、安全 cost、终止/成功条件与安全预算，形式化为带连续时间间隔的 constrained MDP/POMDP。

## 3 CA-LSC-TD3

### 3.1 Physics-guided transition prior

定义 smoothstep、\(g_L\)、\(g_C\) 与 \(\lambda_{phy}=g_Lg_C\)。

### 3.2 CfC residual actor

给出序列输入、\(\Delta t\)、隐藏状态更新和 \(\lambda_d=\lambda_{phy}+\delta_\lambda a^{RL}\)。说明 episode reset、burn-in 与在线 hidden-state 处理。

### 3.3 Asymmetric conservative reward and safety critics

推导 \(\min Q_R\) 与 \(\max Q_C\) 的 TD targets，说明两组 safety critic 必须独立参数化。

### 3.4 Lagrangian actor optimization

给出 Actor loss、residual regularization、约束预算和 \(\beta\) 更新。

### 3.5 Hard command envelope and rate limiter

定义 soft/hard envelope、projection、卸载慢/恢复快的速率限制和 emergency recovery。明确它是工程执行层，不算作 TD3 本体创新。

### 3.6 Algorithm 1

伪代码应同时覆盖 sequence replay、burn-in、target policy smoothing、reward/safety critic 更新、delayed actor update、Lagrange 更新、target soft update 和 checkpoint 排序。

## 4 Experimental Setup

依次写：机体与仿真器、\(\lambda\)-PX4 同步验收、功率/气动标定、nominal
task、F1--F5、训练 curriculum、baselines、公平训练预算、扰动分布、
Monte Carlo 和评价指标。同步 allocator 是低层执行定义，不替代
CA-LSC-TD3 本体创新。

## 5 Results

### 5.1 Capability representation validation

F1 能耗—安全热图、F2 环境敏感性，以及 F3 的
\([V_a]\)/\([V_a,\eta_L]\)/\([V_a,\eta_L,\eta_C]\) ROC-AUC/F1。

### 5.2 Nominal performance

能耗、时间、掉高、AoA 裕度、pitch RMSE、平滑度和时间历程。

### 5.3 Generalization under temporal imperfections

冻结策略，对比 MLP/LSTM/CfC 在 jitter、delay、dropout 下的性能与推理时间。

### 5.4 Monte Carlo robustness

1000 次试验的 SR、Mean、Std、P95、CDF/箱线图。

### 5.5 Ablation and safety intervention

A0–A6 消融，重点同时报告 SR 与 IR，证明 safety critic 降低了对 hard shield 的依赖。

## 6 Discussion

讨论模型误差、估计质量、功率模型精度、纵向/6-DOF 范围、sim-to-real、hard shield 非形式化保证以及计算资源限制。

## 7 Conclusion

只总结被实验直接支持的结论，不把命令包线执行扩写为形式化安全保证。
