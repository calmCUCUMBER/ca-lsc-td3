# 待确认决策与风险

以下事项在两份原始方案中没有定值，因此本次迁移没有替用户假设：

1. 使用纵向模型还是完整 6-DOF。若只做纵向模型，论文不能声称验证侧风控制。
2. 最终机体参数、巡航速度、气动数据库、舵面范围及推进功率模型。
3. \(\eta_L\)、\(\eta_C\)、soft/hard envelope 与成功条件的数值阈值。\(\eta_C\) 的软件接口已固定接收“相对当前 elevator moment 的增量需求”，后续 shadow controller 适配器必须显式完成总需求到增量需求的转换。
4. CfC 是直接封装已复制的上游实现，还是采用维护中的 `ncps` 实现。当前上游 `Cfc` wrapper 每次 `forward` 都从零初始化 hidden state；它适合整段 sequence 验证，但在线 Actor 若需跨 step 保持状态，应基于 `CfcCell` 写显式 stateful wrapper，并定义 reset/episode 边界。
5. 已决定并实现：\(\lambda\) 是 Continuous Transition Allocation Factor，
   不是离散 mode intent。外部命令有效时，它同步 lift collective unloading 与
   PX4 MC/FW roll/pitch/yaw authority；pusher 仍独立控制，失联回退原生 PX4。
6. 两份原始方案中的最新文献与“创新避撞”判断尚未进行独立检索，正式写论文前必须核验题目、日期、DOI 和具体贡献。
7. 12 维 observation 的 normalization/clipping 数值必须由标定/训练数据确定；接口已规定统计量只能用训练集拟合、评估时冻结，且所有网络 baseline 共用。

2026-09-02 决策：schema-13 的 9 点同步验收已完成，transition-allocation
architecture 正式冻结，正式 F1 获得授权。冻结证据与关键文件哈希记录在
`config/transition_allocation_freeze.json`。后续某个 \((V_a,\lambda)\) 失败默认
属于动态可行性/控制性能结果，不再据此改动底层 allocator。架构变更前所有 F1
数据仍只作诊断，不能和新 allocator 的结果混合统计。

2026-09-07 决策：x4 simulation-speed qualification 通过并可用于下一阶段
A0 ROS/PX4 小规模学习 pilot。资格测试固定 action=0.0、关闭 learning，只验证
simulation-time 调度、/clock 观测、action freshness、terminal propagation 与
RTF。结果位于 `data/training/a0_sim_speed_qualification_x4_v6`：RL-active 平均
RTF=4.005，observation dt=0.0500s sim-time，RL step dt=0.1030s sim-time，
stale fraction=0，action/step ratio=1.0。该结果不作为 TD3 性能证据。

2026-09-07 决策：A0 x4 learning-enabled runtime pilot 通过。针对上一轮暴露的
pre-RL keepalive、train_step/action scheduling、hard-failure terminal propagation
问题，已完成最小修复：clock-driven pre-RL safe action、record/publish/train 顺序、
optimizer warm-up、post-exit ROS queue drain，以及 hard-abort terminal fallback。
结果位于 `data/training/a0_ros_td3_x4_learning_pilot_v6`：3/3 terminal observed，
0 action timeout，RTF=3.86--4.02，RL step dt≈0.100s，replay=705，critic updates=450，
actor updates=225，loss finite。该结果冻结 x4 runtime infrastructure；下一步进入
A0 nominal learning pilot，不再修改 F1/F2/F3/F5 或 PX4 allocator。

2026-09-07 修订：`a0_ros_td3_x4_nominal_50ep_v1` 将上述 3-episode 结论限定为
短时 smoke，不再据此冻结 long-runtime infrastructure。50 次 launch 中出现 6 次
`a0_rl_action_timeout`、6 次 pre-RL 无效回合及 1 次 terminal 丢失，因此该批次定为
DIAGNOSTIC FAIL，checkpoint 禁止续训和论文使用。x4 基础仿真资格与 TD3 核心实现
仍保持 PASS；A0 policy convergence 为 NOT ESTABLISHED。训练器已修正跨 episode
clock reset，采用只读 control-actor snapshot 与后台 learner，将 infrastructure stop
编码为 truncation 且不给 task-failure penalty，并将 attempt 与 valid episode 分开。
下一门槛固定为从新随机初始化运行 10 个 valid RL_ACTIVE episode；在此门槛通过前
不调整 TD3、reward、network 或 exploration，也不改 F1/F2/F3/F5 和 PX4 allocator。

2026-09-07 决策：`a0_ros_td3_x4_runtime_clean_10valid_v1` 经原始 telemetry
复核后，按 corrected terminal semantics 判定 x4 long-runtime gate PASS。原 summary
显示的 7 次 `a0_rl_action_timeout` 均为 schema-30 terminal grace 期间的标签覆盖；
首次终止实际上是 6 次 `va_hold_vertical_speed_runaway` 和 1 次
`va_hold_altitude_runaway`，且终止当时 action 均 fresh。前 10 个 attempt 已构成
10 个有效 RL_ACTIVE episodes（5 次 forward timeout、5 次 vertical runaway），
terminal 10/10、genuine action timeout=0、RL stale fraction=0、观测/动作仿真周期
0.0501/0.1000 s。schema-31 已在 trainer 端锁存首个 terminal，并在 recorder 端
禁止终止宽限期覆盖既有物理原因。旧 summary 保留为审计原件，不续跑本门槛；
后续物理 runaway/任务失败作为有效 RL 终止学习，不再触发无意义 retry。

2026-09-07 决策：schema-31 的 3-episode terminal-latch smoke 达到
3 attempts=3 valid episodes、0 retry、0 action timeout、0 RL stale，retry 修复正式
冻结。复核同时发现原 `success` 只是 Va/lambda/高度/vz 的 handover-ready dwell；
其中一条随后被 PX4 判定 forward transition timeout，不能获得成功奖励。schema-32
将该 dwell 明确改为只释放 external test hold，真正 `success` 只由 command node 的
`TRANSITION_FW -> HOLD_FW`（PX4 fixed-wing confirmed）产生；handover 到最终状态之间
不发布非终止 A0 observation，避免 action-free transition 进入 replay。同时三类
runaway 的 0.5 s dwell 已从 wall time 改为 ROS simulation time，使 x1/x4 task
termination envelope 一致。TD3、reward 数值、PX4 allocator 及 F1--F5 冻结项未改。

2026-09-08 补充：A0 trainer summary/training_metrics 直接锁存终止瞬间的
`terminal_command_state`、`terminal_handover_ready`、
`terminal_handover_ready_dwell_s` 和 `terminal_actual_fw_confirmed`。从该版本起，
任意 `success=true` 的 episode 必须同时满足
`terminal_actual_fw_confirmed=true`，否则 smoke 自动失败；任务失败 terminal 仍作为
有效 RL episode 进入 replay。

2026-09-08 决策：`a0_ros_td3_x4_nominal_50ep_v2` 已取得 50 个有效 episode，
下一论文主线步骤固定为 A0 deterministic nominal evaluation。评价加载该批 final
policy 与配套 normalizer，在 x1、50 m、nominal mass、无风/无噪声/无失配条件下进行
10 次 cold restart；关闭 exploration、replay、normalizer update 与全部网络更新。
评价器以参数 digest、normalizer 前后状态和 update/replay 计数自动证明策略冻结。
10 次成功数的预注册决策为 0--2 或 3--5：A0 继续至 100 episode；6--8：A0 nominal
baseline candidate，完成安全审计后进入 A1；9--10：冻结 A0 nominal 并进入 A1。
本阶段不改高度、reward、TD3、PX4 allocator 或 x4 平台。

2026-09-08 决策：persistent exploration 的 100-valid-episode 训练恢复了
2 次 actual HOLD_FW success，但最终 checkpoint 的干净 x1 deterministic
evaluation 为 0/10，且确定性 `lambda_exec` 均值约 0.177。由此冻结
`learning_starts=2000` 与 5 s simulation-time persistent exploration，正式把下一
blocker 定义为 reward/credit assignment，不再机械续训。所有 A0--A6 共享的新 reward
版本为 `capability_free_transition_potential_v1`：保留原安全和 terminal 项，仅增加
`gamma*Phi(next)-Phi(current)`，其中 `Phi=1*g_V(Va)+4*g_lambda(lambda_exec)`；
不使用 eta_L/eta_C，不奖励绝对高 lambda，task terminal 的 absorbing potential 为 0。
对旧 100 条轨迹的重算保持 `R_runaway < R_timeout < R_success`，且三条持续高-lambda
timeout 的 progress shaping 全为负。下一步只做 20-valid-episode A0 reward pilot，
通过后再做 5 次 frozen x1 deterministic evaluation；不启动 A1。

2026-09-08 决策：`a0_ros_td3_x4_nominal_100ep_v1` 的 100 个有效训练 episode
取得 0 次 actual fixed-wing success；配套
`a0_deterministic_nominal_100ep_eval_v1` 在干净的 10 次 x1 cold restart 中同样
为 0/10，全部 `forward_transition_timeout`。正式诊断为 success-discovery failure：
旧协议在 256 transitions 后即学习，并使用每 0.1 s 独立重采样的探索，无法可靠覆盖
需要约 3.8 s 执行器上升时间加 1.0 s handover dwell 的持续高 lambda 动作。该失败批次
冻结归档且不续训；TD3 数值实现、debug reward、x4、PX4 allocator、schema-32 终止
语义及 F1--F5 保持不变。下一步只运行 10-valid-episode discovery pilot：
`learning_starts=2000`，warm-up action 与后续 Gaussian perturbation 均按 simulation
time 分段保持 5.0 s。若没有至少一次 `HOLD_FW` authoritative success，则停止长训练，
再单独预注册所有消融共享的 capability-free potential shaping reward。
