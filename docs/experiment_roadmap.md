# CA-LSC-TD3 实验路线与验收顺序

主线恢复为：

```text
传统 VTOL/PX4 基线
-> lambda-PX4 同步验证
-> F1 Va-lambda 扫描
-> F2 载荷/风扰动
-> F3 eta_L
-> F4 eta_C
-> F5 状态信息增益
-> Physics Prior
-> Residual TD3
-> CfC
-> Twin Safety Critics + Lagrange
-> Hard Shield
-> 正式对比、泛化、Monte Carlo、消融
```

## 0. 低层架构冻结

1. Hover、Transition、Fixed-wing Cruise 三种状态先由传统控制器跑通。
2. nominal 工况固定为 (h_0=h_d=50\,m)、(V_{a,0}\approx0)，目标为恒高
   hover-to-cruise。
3. λ 定义为 **Continuous Transition Allocation Factor**：

   \[
   T_{L,ff}\propto1-\lambda_{exec},\qquad
   w_{FW}=S_\lambda(\lambda_{exec}),\qquad
   w_{MC}=1-w_{FW}.
   \]

4. 外部 λ 有效时 PX4 不再使用空速权重与其竞争；命令失联/非法时立即恢复
   旋翼支撑并回退原生 PX4 blending。Pusher 始终由独立空速控制器负责。
5. 先运行 9 点 `transition_allocation` 验证；每点必须实际达到目标 λ 并在目标
   带内保留至少 10 个同步样本，MC/FW 权重 RMSE 不大于 0.02，才冻结架构并
   重跑 F1。飞行性能是否满足 F1 判据不参与同步验收。架构变更前的
   v7/v8/v9 数据只作诊断。
6. 2026-09-02 已满足 9/9 点冻结条件。后续 F1 单点失败只解释为该工况的动态
   可行性/控制性能证据，不再反向修改 transition allocator；若确需修改底层，
   必须结束当前正式批次、升级架构版本并重新做 9 点验收。

## F1：(V_a-\lambda) 扫描

关闭 RL，固定机体、无风、固定高度和同一套 PX4/控制器参数：

\[
V_a=6,8,\ldots,20\;m/s,\qquad
\lambda=0,0.1,\ldots,1.
\]

每点记录高度误差、迎角、pitch/pitch-rate、lift thrust/rotor speed、pusher、
MC/FW 实际权重和功率 proxy。论文主图为
\((V_a,\lambda)\) outcome map、maximum confirmed feasible \(\lambda\) 与
代表 N/B/I/X 时序图。N/B/I/X 完整保留作审计；B 解释为 marginal feasible /
performance-limited feasible，不等同于单调几何边界。

2026-09-02 状态：正式 schema-13 F1 已完成，
`data/evaluation/f1_transition_allocator_v1` 共 88 点，全部可解析且
`protocol_consistent=true`。结果为 N=28、B=43、I=8、X=9，无 missing 或
protocol-invalid 点。F1 结论冻结为：\(\lambda\) 是有意义且强烈依赖
\(V_a\) 的 transition allocation decision variable，低速高 \(\lambda\)
形成 unsafe wedge，高速高 \(\lambda\) 区域恢复为 nominal feasible。正式能耗
结论仍必须使用经验证的功率模型；当前静态 motor proxy 只能诊断，不能宣称
节能百分比。

## F2：固定空速 schedule 的局限

不重复完整 F1 网格，而是在 F1 边界附近做缩小版 \(V_a-\lambda\) 扫描。
F2-A formal 已完成并冻结：论文主图为 `docs/f2_protocol.md` 中的 18 点、
三种真实质量、每点 5 个有效重复；额外 \((10,1.0)\) 只作诊断。F2-B 使用
\(V_a=10,12,14\) 的 9 个代表 cell 施加确定性 1-cos gust
\(A_g=0,4,8\,m/s\)，共 135 个有效 run；后续再加入气动 mismatch。

F2 的结论不是能耗最优，而是证明相同 \(V_a\) 下 N/B/I/X 区域会随载荷或风
移动，即 \(\lambda_{\max}^{feasible}\ne f(V_a)\) alone。WQ-v3 已确认 steady
Gazebo wind 与 wind-relative \(V_a\) 控制链路；从 F2 开始，论文中的 \(V_a\)
冻结为 `selected_airspeed_mps=|\mathbf V_g-\mathbf V_w|`，原始
`airspeed_mps` 只作为 PX4/native 诊断。

2026-09-04 状态：F2-A payload formal 与 F2-B deterministic-gust formal 均已
完成并冻结。F2-B 共 27 cells、135 个 valid repeats；4 m/s 主要将 N 推向可恢复
的 B，8 m/s 显著降低 recovery 并产生 run-level hard violations。F2 不再增加
F2-C、更多风型或湍流；后者留给泛化与 Monte Carlo。

## F3：验证 \(\eta_L\)

标定 (C_L(\alpha))，在不同空速、质量、迎角和风下比较机翼竖直承载能力
与可安全转换程度。必须区分算法使用的估计质量 \(\hat m\) 与仿真真实质量。
F3 专用 instrumented LiftDrag 直接发布 Gazebo 实际施加的两片主翼 lift，作为
独立 \(L_z^{GT}\)。2026-09-06 状态：F3-A physical validation 已完成并冻结；
完整定义、GT、门槛与归档见 `docs/f3_protocol.md`。

## F4：验证 \(\eta_C\)

读取与 λ 解耦的 PX4 virtual FW pitch demand，结合 Gazebo 实际 elevator joint、
动态压强、已标定需求映射和方向相关剩余舵量计算增量控制余度。2026-09-06 状态：
F3-B1 derivative/remaining-authority 与 F3-B2 端到端 behavior 均完成；最终合并
矩阵 9/9 valid，q-gate、demand、remaining-travel 三项均 PASS。\(\eta_C\) 继续只供
feature/prior/shield，绝不直接修改已经冻结的 PX4 blend。

## F5：状态信息增益

当前阶段。先复用已有 F1/F2/F3 telemetry，以冻结的 2 s 未来 hard-safety 标签
比较同一 logistic model family：\([V_a,\lambda]\) 与
\([V_a,\eta_L,\eta_C,\lambda]\)。必须按完整 condition group 做 5-fold split，
不得随机拆分连续 telemetry rows。完整协议见 `docs/f5_protocol.md`；若旧 schema
覆盖或类别平衡不足，只补 boundary 附近少量 current-schema runs。

## 训练与正式实验

训练严格逐级：Physics Prior Only -> MLP Residual TD3 -> CfC -> Twin Safety
Critics -> Lagrangian -> Hard Shield。CfC 使用 10 Hz、sequence length 20、
burn-in 5；Actor 初始 residual 接近 0。之后执行 nominal comparison、冻结
policy 的 realistic generalization、恰好 1000 次 Monte Carlo，以及 A0--A6
消融。核心指标包括 SR、能耗、转换时间、掉高、AoA 裕度、pitch RMSE、
λ 平滑度、constraint violation rate 和 shield intervention rate。
