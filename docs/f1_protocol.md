# F1：\(V_a-\lambda\) capability map 协议

日期：2026-09-01；最后更新：2026-09-02

F1 的目标不是把 PX4 blending 另立为论文问题，也不要求得到一条完美单调
边界。\(\lambda\) 已定义为同步 lift unloading 与 MC/FW attitude allocation
的连续转换因子；F1 回答它是否显著改变转换可行性、飞行品质和安全边界，
从而值得由 TD3 在线优化。

\[
(V_a,\lambda)\rightarrow
\{\Delta h,\alpha,q,\text{stability},\text{safety},\text{effort proxy}\}
\]

论文需要从 F1 得到三条证据：\(\lambda\) 对 VTOL transition behavior 有显著
影响；允许/合适的 \(\lambda\) 明显依赖 \(V_a\)；过度或不合适的 allocation
会导致安全边界、局部不收敛或性能退化。PX4 权重/torque demand 只作为同步
审计和后续 \(\eta_C\) 标定诊断，不单独构成 F1 创新。正式能耗最优
\(\lambda_E^*\) 等功率结论必须等可信功率模型完成后再报告。

## 当前状态

- \(\eta_L\) 与 \(\eta_C\) 的论文级定义已在
  `src/ca_lsc_td3/physics/capabilities.py` 中实现。
- F1 暂不使用 \(\eta_L,\eta_C\) 参与控制；它们的正式验证属于 F3。
- F1 汇总器会额外输出 `eta_l_proxy_*`，这是由 measurement window 内
  \(V_a,\alpha,\gamma,\phi\) 和 SDF 模型 \(C_L(\alpha)\) 离线计算的诊断量；
  不能替代 F3。
- schema-v7 粗扫复核发现该 proxy 在多数高速点为零，说明迎角/升力模型
  符号或标定尚未闭环；汇总器因此固定输出
  `eta_l_proxy_reportable=false`，F3 修复前不得用于论文结论。
- telemetry schema 13 已加入 PX4 实际 `mc/fw pitch weight`、由
  \(\lambda_{exec}\) 计算的期望权重和同步 RMSE，以及 MC/FW virtual
  pitch torque setpoint。后两者是 normalized controller demand，不是 N m；在完成
  力矩尺度和 \(C_{m_{\delta_e}}\) 标定前，仍必须输出
  `eta_c_inputs_available=false`，不得代入论文级 \(\eta_C\)。
- schema 13 保留了 elevator 口径修正：GZ bridge 将 `servo_2` 当作弧度命令，
  `standard_vtol` 关节在 \(\pm0.53\,rad\) 裁剪。旧的 `abs(servo)>=0.95`
  只是 PX4 通道诊断；新的
  `va_hold_elevator_joint_limit_command_fraction` 才是物理关节限位命令代理。
- 正式 F1 `data/evaluation/f1_transition_allocator_v1` 已完成并重算：
  88/88 个网格点均为 schema 13，`protocol_consistent=true`，无 missing 或
  protocol-invalid 点。分类结果为 N=28、B=43、I=8、X=9；其中 B 表示
  marginal feasible / performance-limited feasible，不应简单解释为几何边界。

## Coarse scan

第一轮粗扫：

\[
V_a=\{6,8,10,12,14,16,18,20\}\,m/s
\]

\[
\lambda=\{0,0.1,0.3,0.5,0.7,0.9\}
\]

共 \(8\times6=48\) 个工况，每个工况 5 次独立重启，共 240 次。

F1 runner 会随机打乱每个 \((V_a,\lambda)\) 的执行顺序，并把顺序写入
`f1_run_order.tsv`。点级失败不会中止整张图，因为 abort/unsafe 本身就是 F1
边界数据。

F1 单次有效性使用 `f1_measurement_pass`。它检查固定 \((V_a,\lambda)\)
measurement window 的完成度、空速/\(\lambda\) 跟踪、恒高安全、遥测完整度和
pusher 外部控制状态，但不把 Phase-0.5 的“4 秒内达到 8 m/s”诊断作为硬门槛。
因此 \(V_a=6\,m/s\) 的有效稳态测量不会因主动保持在 8 m/s 以下而被误判。
`phase_0_75_va_hold_pass` 保留原定义，用于已完成的 Phase-0.75 工程验收。

从 telemetry schema v6 起，固定空速控制增加统一的恒高外环：

\[
v_{z,d}^{\rm down}=\operatorname{sat}
\left(k_h(h-h_d)+k_v v_z^{\rm up}\right).
\]

默认 `va_altitude_kp=0.35`、`va_altitude_vertical_speed_kd=0.5`、
`va_altitude_velocity_limit=1.5 m/s`。正值表示 NED 向下命令。该修订用于解决
高空速受控转换保持期间 collective 达到下限后仍持续爬升的问题，并记录
`va_hold_down_velocity_command_mps`。schema v5 及更早的 F1 结果只作为 pilot
数据，不与 v6 正式网格合并统计。

该外环采用单向修正：只允许给出向下命令；高度不足时保持 0，由 PX4 原生
高度环负责恢复。这样可避免转换模式切换瞬间很小的向上速度前馈导致 collective
积分到上限。

进一步的 PX4 日志诊断表明，标准 `FlightTaskTransition` 会把外部垂向速度参考
渐近置零，而 `Standard` 会把前转换俯仰固定为 `FW_PSP_OFF`。因此仅增加 ROS
垂向速度外环仍无法在高速段消除机翼过剩升力。从当前协议起，只有在
`VT_UNLD_TEST_EN=1` 且外部卸载命令新鲜有效时，PX4 会锁定进入保持时的 NED
高度，并按

\[
\theta_d=\operatorname{sat}\left[
FW\_PSP\_OFF+K_h(z-z_{ref})+K_vv_z
\right]
\]

生成传统高度—俯仰参考，再交给原生姿态/升降舵控制器。默认
`VT_UNLD_ALT_P=2 deg/m`、`VT_UNLD_VZ_D=2 deg/(m/s)`，俯仰限幅
`[-12,10] deg`。NED 约定下，爬升使两项为负，从而压低机头。普通 PX4
转换路径保持不变。

schema v7 的 64 点结果已用于粗扫和边界诊断，但其 measurement timer 在离开
条件后会重置并允许再次进入，部分点把 6--12 个不连续片段拼成一次测量。因此
v7 结果只作为 exploratory/pilot 数据，不再作为最终稳态 F1 表。

schema v8 完成了 88 点、440 次协议诊断，但其 measurement 开始后采用
“单个样本离开软容差带即中止”的规则。该规则把 167 次轻微空速越界放大为
`measurement_window_interrupted`，因此 v8 永久归档为
`F1 protocol-validation / envelope-stress batch`，不用于论文最终边界。

正式 F1 改用 telemetry schema v9。协议为：

1. 空速连续满足容差 `va_target_dwell_s`；
2. 空速与 \(\lambda\) 同时连续满足容差 2 s；
3. 随后固定、连续记录 `va_measurement_s=3 s`，不重置、不拼接，也不因一个
   软带宽样本越界而提前停止；
4. 窗口结束后要求空速与 \(\lambda\) 各自至少 90% 样本位于容差带，并要求
   `max_abs_airspeed_error <= 0.6 m/s`；
5. 高度、垂向速度、AoA、姿态和 PX4 failsafe 等 hard safety 仍实时检查并
   立即中止，与窗口质量判定分离。

## 指令

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

# 先重建修改后的 PX4（包含 lambda-attitude allocation）
make -C PX4-Autopilot px4_sitl_default BUILD_DIR=build_ca_make -j4

# 再只重建新论文运行链中的三个 ROS 包
source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws/log_ca build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build_ca \
  --install-base ros2_ws/install_ca \
  --packages-select px4_msgs vtol_px4_control ca_lsc_transition \
  --symlink-install

# 先做 9 个架构冻结点，证明 lambda_exec 与 PX4 权重同步
./scripts/run_transition_allocation_verification.sh \
  data/evaluation/transition_allocation_sync_v1 220

# 若首批仅 Va=14, lambda=0.7 未执行到目标，只补这一点；不要重跑其余 8 点
./scripts/run_transition_allocation_gap.sh \
  data/evaluation/transition_allocation_sync_v1 220

# 同步验收通过后，旧 F1 数据不复用，重新跑正式网格
./scripts/run_f1_full_grid_v9.sh \
  data/evaluation/f1_transition_allocator_v1 220
```

架构同步判据和 F1 可行性判据相互独立。`synchronization_pass=true` 要求每个
验证点实际进入目标 lambda 带内至少 10 个样本，再检查 MC/FW 实际权重相对
lambda 理论权重的 RMSE 不大于 0.02；不能用 lambda=0 的自洽前缀替代非零
目标点。F1 measurement 即使未通过，只要目标已执行且权重同步，仍可完成该点
的架构验证。

截至 2026-09-02，同步验收已完成 9/9 点，schema-13 架构正式冻结。正式 runner
启动时必须通过 `config/transition_allocation_freeze.json` 的证据与关键文件哈希
检查。任何底层映射、PX4 权重、DDS、recorder、固定窗口协议或低层参数变更，
都必须使用新版本号重新验证，禁止继续写入同一个正式 F1 数据根目录。

如果中途只想重算汇总表：

```bash
./scripts/summarize_f1_grid.sh data/evaluation/f1_full_grid_v9
```

输出：

- `f1_grid_points.csv`：每个 \((V_a,\lambda)\) 一行，适合画热图；
- `f1_grid_summary.json`：完整 JSON 汇总；
- `f1_outcome_classes.png`：只显示 N/B/I/X 四类物理结果；协议未收敛点为灰色，
  具体 attempt 质量只在 data-quality 图表显示；
- `f1_boundaries.csv`：逐空速的可行区间/右删失边界；功率代理不可信时
  正式能耗最优字段保持为空；
- `f1_va_lambda_max_altitude_error.png`：仅使用 data-quality-valid 点的
  跨重复试验 P95 高度误差图；
- `f1_va_lambda_max_altitude_error_diagnostic.png`：worst-run 粗扫诊断图；
- `f1_feasible_boundary.png`：带区间/右删失箭头的可行边界图；
- `f1_data_quality.png` 与 `f1_data_quality_cases.csv`：协议质量单独图表；
- `f1_pusher_power_sanity.png`：\(T_PV_{axial}/P_P\) 能量下界检查；
- `f1_va_lambda_total_power.png`：只保留为 static motor-model effort
  诊断图，不是论文可报告的机械/电功率；
- 每个点目标保留 5 个有效 run；如果发生协议无效，会继续保留对应 attempt，
  最多 10 个 `run_*` 目录。所有 telemetry、summary、console 和
  `repeat_summary.json` 都不得删除。

## F1 分类口径

论文主图只合并为 `Feasible` 与 `Unsafe/Infeasible`；彩色区域显示已通过资格
验证的能耗，灰色区域表示不可行。下面的 N/B/I/X 和 data-quality 分类完整保留
在审计附件中，但不作为论文主叙事。

正式 physical map 使用四类结果（另加未解析/missing）：

- `nominal_feasible`（N）：连续测量完成且飞行品质正常；
- `boundary_feasible`（B）：完成连续测量，但重复出现高度调节等软边界，或
  5 次中有一次非收敛；
- `nonconvergent`（I）：目标工况已建立，但少于 3/5 次能维持连续测量；
- `physical_unsafe`（X）：目标 \((V_a,\lambda)\) 已建立后发生高度、下降率、
  AoA、俯仰率或 PX4 failsafe 等硬物理越界；
- `missing`：未测试，图中为灰色。

`data_quality` 与 physical class 正交：每点预先规定 5 个有效重复、最多 10 次
attempt。协议无效 attempt 不参与物理投票，但永久保留并计入
`protocol_invalid_attempt_rate`。取得 5 个有效重复后标为 `valid` 或
`valid_with_retries`；10 次仍不足 5 个有效重复才标 `protocol_unstable`。
因此 `1P+5X` 必须显示为 X，而不是 P，也不允许无限补跑到只剩好结果。

`settle_timeout`、
`measurement_window_interrupted` 和旧日志的多 measurement segment 均属于 I，
不再写成 X。像旧数据中的 \((8,0.7)\) 那样在 \(\lambda_{exec}=0\) 时提前中止
的 run 属于 P，不能证明 \(\lambda=0.7\) 不安全。

每个 B/X 必须给出 `boundary_reasons` 或 `unsafe_reasons` 及重复次数。
飞行品质阈值逐 run 检查：单次软越界只写入
`single_run_outlier_reasons`，至少两个有效 run 重复同类恶化才形成 B。
只有目标工况已经建立后的硬物理中止才判 X，并由
`unsafe_repeatability=single_run/repeated` 区分一次异常与可重复失效。固定点
分类使用 measurement-window 的 rotor spike，而不是整个转换阶段的 spike。

跨 run 的每项统计同时保留 Mean、Std、Median、P90、P95、Min、Max；P90/P95
使用 \((n-1)p\) 位置的线性插值。协议异常 run 不进入物理统计。

当最高已测 \(\lambda=0.9\) 仍可行但 1.0 未测时，边界必须写成
`>=0.9`，不得写成精确 0.9。

## 功率 sanity check

当前 `pusher_thrust_n` 与 `pusher_power_w` 都由同一电机速度命令按静态模型

\[
T=k_T\omega^2,\qquad P=k_Tk_M\omega^3
\]

重建。该模型没有 advance-ratio/轴向来流修正。schema-v7 粗扫中，45 个有
measurement window 的点里有 37 个违反
\(P_P\ge T_PV_{axial}\)，且 \(V_a\ge10\,m/s\) 的点系统性出现表观效率大于 1。
因此 `power_model_reportable=false`，正式 `lambda_energy_optimal` 暂停输出；
`lambda_energy_optimal_proxy` 仅用于内部诊断。后续可采用经独立验证的
\(Q\omega\)、电压电流遥测或 \(C_P(J)\) map，但在来源、效率、适用范围、
重复性和 \(P\ge TV\) 样本级检查通过前，不得报告节能百分比。

schema v8/v9 的 `transition_*_energy_j` 物理字段固定为空；静态模型积分只保存在
明确带 `_proxy_j` 的字段中。schema 13 之前的 v7/v8/v9/transition-gap 数据全部
归档为开发/诊断数据；正式可行域采用
`data/evaluation/f1_transition_allocator_v1` 的 schema-13 结果。即便在正式
schema-13 F1 中，节能最优 \(\lambda_E^*\) 仍不得报告。

## 代表点体检

```bash
./scripts/analyze_f1_representatives.sh \
  data/evaluation/f1_transition_allocator_v1/f1_grid_summary.json
```

默认生成正式 N/B/I/X 四个代表点：
\((16,0.6)\)、\((10,0.8)\)、\((12,0.6)\)、\((8,0.9)\)。可行点按多指标
Median/MAD 距离自动选择代表 run；失败点按 modal abort reason 和中位 abort
时刻选择。选择清单写入 `f1_representative_runs.json`，逐点图写入
`representative_timeseries/`，合成图写入
`representative_timeseries/f1_representative_nbix_timeseries.png`，避免人工挑图。

## 需要重点报告的量

控制变量有效性：

- `va_hold_mean_airspeed_mps`
- `va_hold_std_airspeed_mps`
- `va_hold_mean_lambda_exec`
- `va_hold_measurement_pusher_external_active_fraction`

仅用于内部诊断的 effort proxy（不得写成论文机械/电能耗）：

- `va_hold_mean_lift_power_w`
- `va_hold_mean_pusher_power_w`
- `va_hold_mean_total_power_w`
- `va_hold_total_energy_proxy_j`

飞行品质/安全：

- `va_hold_altitude_rmse_m`
- `va_hold_max_abs_altitude_error_m`
- `va_hold_max_abs_vertical_speed_mps`
- `va_hold_max_abs_alpha_rad`
- `va_hold_pitch_rate_p95_rad_s`

瞬态/执行器负担：

- `va_hold_lift_rotor_spike_samples`
- `va_hold_lift_rotor_saturation_fraction`
- `va_hold_lift_collective_saturation_fraction`
- `va_hold_servo_saturation_fraction`（旧 PX4 通道 \(\pm1\) 诊断）
- `va_hold_elevator_joint_limit_command_fraction`（GZ \(\pm0.53\,rad\)
  命令裁剪代理）
- `va_hold_mean_mc_pitch_weight_actual`
- `va_hold_mean_fw_pitch_weight_actual`
- `va_hold_*_pitch_torque_demand_normalized`（normalized，不能写成 N m）
- `va_hold_pusher_throttle_upper_saturation_fraction`
