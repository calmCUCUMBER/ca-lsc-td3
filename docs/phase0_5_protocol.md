# Phase-0.5：F1 前修正与验收协议

> 历史工程记录：本页数据产生于 schema 13 的同步 transition allocator 之前，
> 只证明连续命令/遥测/高度保持链路，不能作为新 \(\lambda\) 定义下的正式 F1。

日期：2026-08-29

本阶段来自对 `data/evaluation/repeats_5x` 中 Native 5 次与 Manual 5 次试验的复核。旧 CSV 不修改、不覆盖；所有新试验使用新目录。旧数据证明连续卸载链路有物理效果，但也暴露出起转工况离散、一次 pusher 加速异常、升力旋翼尖峰、总能量被 8 m/s 前阶段主导，以及 PX4 提前完成转换导致 \(\lambda\) 只能到约 0.3 的问题。

## 已实现的修改

1. 控制节点仅在以下条件连续满足 2 s 后允许前转换：
   \(|h-50|\le1\,m\)、\(|v_z|\le0.2\,m/s\)、地速 \(\le0.2\,m/s\)。
2. recorder 使用控制节点发布的高度基准作为优先 datum，并显式记录
   `altitude_local_m`、`altitude_datum_local_m`、`altitude_relative_m`、
   `target_altitude_local_m` 和 `target_altitude_relative_m`。论文高度
   `altitude_m` 仍等于 datum-relative altitude。
3. 评估器分别输出 `transition_start_altitude_error_m` 和
   `additional_transition_drop_m`；前者是起转偏差，后者才是起转后的新增掉高。
4. DDS/CSV 新增 `actuator_motors`、pusher 与 lift thrust setpoint，并用消息年龄检查新鲜度。pusher 诊断要求峰值设定不低于 0.4，且 4 s 内达到 8 m/s。
5. 自动报告 4 个 lift rotor 的最大角速度、超过 1400 rad/s 的样本数和起转前 0.5 s 平均 lift 功率代理。
6. 自动报告 pitch-rate 的 RMSE、P95 和最大值；迎角在 5 m/s 以下标为无效，\(\eta_L\) 同样保守置零。
7. 能量名称固定为 `model_based_mechanical_proxy`，并拆成起转至 8 m/s 与 8 m/s 至 FW 两段；不得表述为实测电能。
8. PX4 参数 `VT_UNLD_TEST_EN=1` 可在外部命令有效期间阻止自动完成前转换。实验仅在 \(\lambda_{exec}\) 进入目标 \(\pm0.03\) 并持续 1 s 后释放测试保持。
9. \(\lambda\) 响应输出 command-to-active 延迟、settling time、平均/峰值执行速率、稳态偏差/误差和 overshoot。
10. CSV schema 升级到 v3，新增 `height_gate_ok`、`vz_gate_ok`、
    `groundspeed_gate_ok`、`stability_gate_ok` 和 `stability_dwell_s`。
    评估器会输出 `primary_failure_cause` 以及起转前各稳定门通过比例。
11. 5 次重复脚本现在单次失败后继续执行剩余 run，并生成
    `repeat_status.tsv` 与 `repeat_summary.json`；最终只要存在失败仍返回非零退出码。
12. PX4 `Standard::update_transition_state()` 中的 pusher ramp 已前移到
    `TRANSITION_TO_FW` 入口处执行，不再被后续 virtual attitude setpoint
    freshness 检查提前 `return` 截断。这样受控 \(\lambda\) 测试在
    \(\lambda=0\) 且 \(V_a<8\,m/s\) 时也会先正常启动 pusher 加速。
13. 控制节点将“起飞完成”和“起转前稳定”拆开：起飞完成要求
    \(|h-h_d|\le0.5\,m\) 且 \(|v_z|\le0.3\,m/s\)，进入 `HOLD_MC`
    后继续发布 \(h_d=50\,m\)，不再把进入 HOLD 时的瞬时高度捕获成
    hold altitude。CSV 同步记录 `/fmu/in/trajectory_setpoint` 的高度给定，
    用来审计控制器实际发送的高度参考。

## Run 3 复核修正

`data/evaluation/phase0_5_lambda_0p5_5x/run_3` 是 schema v2 历史失败样本。
复算后应解释为 `pretransition_stability_timeout`，不是 \(\lambda=0.5\)
跟踪失败。该 run 没有进入 `TRANSITION_FW`，所以 `lambda_published_samples=0`
和 `lambda_tracking_samples=0` 都是级联结果。

旧 CSV 的 HOLD_MC 段最长稳定驻留为 1.648 s，低于 2.0 s 门槛；逐条件通过比例为：
高度门 8.4%，垂速门 87.8%，地速门 100%。根因集中在高度参考不一致：
console 中 PX4 local altitude 约 50.4-50.6 m，而旧 telemetry 的 datum-relative
高度约 48.9 m。v3 schema 会直接记录 local/datum/relative/target 四组高度，
不再需要从跳变倒推。

`data/evaluation/debug_lambda_0p5_v3` 是高度参考修复后的第二个失败样本。
该 run 已通过起转前稳定门并进入 `TRANSITION_FW`，但 25 s transition
窗口内 `pusher_thrust_setpoint=0`、`omega_4_rad_s=0`、最大空速仅
2.439 m/s，导致 \(\lambda\) 从未发布。修正后的 evaluator 将其归类为
`primary_failure_cause=transition_pusher_undercommand`，并给出
`secondary_failure_cause=lambda_never_activated`。这不是 \(\lambda=0.5\)
跟踪失败，而是受控转换路径中的 pusher activation 死锁。

## 2026-08-29 全栈冒烟

`phase0_5_native_smoke3_20260829` 通过 Native、50 m nominal 和遥测验收：起转稳定驻留 2.052 s，起转高度 49.049 m，起转后新增掉高 0.006 m，转换 4.848 s。该次在起转后 0.148 s 检测到四个 lift rotor 同时尖峰：同一帧 lift collective setpoint 为 1.0、四路 actuator command 为 0.976–1.0、实测角速度为 1465–1500 rad/s。因此其直接来源被归类为 `commanded_collective_saturation`，不是只有 ESC 字段跳变的遥测映射假象。它与起转时从悬停点切换到远端 FW position setpoint 同时发生，后续需用重复试验确认这一控制设定跳变是否是稳定根因；当前保留原始瞬态，不作低通删除。

`phase0_5_lambda_0p5_smoke_20260829` 通过受控 \(\lambda=0.5\) 验收：PX4 external-active 覆盖 98.3%，command-to-active 延迟 0.048 s，settling time 1.900 s，平均正向执行速率 0.2448/s，稳态平均绝对误差 0.0020，无 overshoot。PX4 在目标驻留完成前保持 Transition，随后安全释放并进入 FW。

`debug_lambda_0p5_v4` 是 pusher ramp 修正后的单次回归测试，已通过受控
\(\lambda=0.5\) characterization：起转前稳定驻留 2.048 s，transition
time 6.652 s，3.152 s 达到 8 m/s，pusher setpoint 峰值 0.45，
pusher \(\omega\) 峰值 1575 rad/s，\(\lambda\) 发布 60 个样本，
external-active 覆盖 98.3%，settling time 1.900 s，稳态平均绝对误差
0.0020，最终进入 FW 并保持 6.0 s。全窗口 mean tracking error 为 0.167，
这是包含从 0 ramp 到 0.5 的瞬态，不作为稳态跟踪失败解释。

`phase0_5_lambda_0p5_v4_5x` 的重复试验显示 pusher deadlock 已修复，但
起转前高度仍有随机性：5 次中仅 3 次通过，失败 run 的
`primary_failure_cause=pretransition_stability_timeout`。复核发现
`HOLD_MC` 会捕获约 49 m 的瞬时高度作为 hold setpoint，使 50 m 高度门变成
运气题。

`debug_lambda_0p5_v5_takeoff_hold` 是起飞完成/hold altitude 修正后的单次
回归测试，已全项通过：`HOLD_MC` 平均高度 49.970 m，实际 trajectory
高度给定 50.000 m，5.000 s 后进入转换，起转高度误差 0.057 m；
pusher 峰值 0.45，3.152 s 达到 8 m/s；\(\lambda=0.5\) 的 settling time
1.900 s、稳态平均绝对误差 0.0020、无 overshoot。

`phase0_5_lambda_0p5_v5_5x` 是同一修正后的 5 次独立重启统计结果，5/5
通过。5 次均无 failsafe、无 pusher anomaly、无 lift rotor spike；
`HOLD_MC` 平均高度为 \(49.926\pm0.026\,m\)，trajectory 高度给定为
\(50.000\pm0.000\,m\)，起转高度误差为 \(0.009\pm0.050\,m\)，
transition time 为 \(6.610\pm0.124\,s\)，3.119±0.125 s 达到 8 m/s。
\(\lambda_{exec}\) 峰值均为 0.5，settling time 均为 1.900 s，稳态平均
绝对误差为 \(0.00203\pm0.00006\)，稳态最大误差不超过 0.0281，无
overshoot。该结果可作为 Phase-0.5 中“连续 \(\lambda=0.5\) 卸载接口
和 50 m 恒高起转链路”的当前验收样本。

## 2026-08-29 Native/Manual 对照与 \(\lambda\) 一维能力验证

根据 `phase0_5_lambda_0p5_v5_5x` 的阶段性通过结果，后续按同一
schema v3 与同一 50 m nominal 协议重跑 Native、Manual airspeed
对照，并补充受控 \(\lambda\in\{0.1,0.3,0.5,0.7\}\) 一维 transition
characterization。注意：这仍是“前转换过程中的一维卸载表征”，不是
固定 \(V_a=12\,m/s\) 驻留采样，也不能替代正式 F1 的二维
\((V_a,\lambda)\) 扫描。

Native 对照 `phase0_5_native_v5_5x`：5/5 通过 Native transition、
50 m nominal、遥测与 pusher 诊断。5 次均无 failsafe、无 pusher
anomaly、无 lift rotor spike；transition time 为
\(4.798\pm0.062\,s\)，3.110±0.036 s 达到 8 m/s，起转后新增掉高
\(0.004\pm0.006\,m\)，高度 RMSE 为 \(0.349\pm0.080\,m\)，最大
绝对高度误差为 \(0.673\pm0.123\,m\)。最大 lift rotor 角速度为
\(958\pm142\,rad/s\)，最高 1119 rad/s，低于 1400 rad/s spike
门槛。pusher 峰值均为 1575 rad/s。

Manual airspeed 对照 `phase0_5_manual_v5_5x`：5/5 通过 Native
transition、50 m nominal、遥测、manual \(\lambda\) 与 pusher 诊断。
5 次均无 failsafe、无 pusher anomaly、无 lift rotor spike；
transition time 为 \(4.820\pm0.052\,s\)，3.200±0.030 s 达到
8 m/s，起转后新增掉高 \(0.019\pm0.015\,m\)，高度 RMSE 为
\(0.342\pm0.152\,m\)，最大绝对高度误差为 \(0.600\pm0.391\,m\)。
该对照中外部 \(\lambda\) 命令主要随 airspeed schedule 增长，
\(\lambda_{exec}\) 峰值为 \(0.313\pm0.009\)，external-active 覆盖
\(95.1\%\pm1.4\%\)。pusher 峰值均为 1575 rad/s。

受控 \(\lambda\) 一维表征结果如下。所有点均 5/5 通过，且所有 run
均无 failsafe、无 pusher anomaly、无 lift rotor spike，pusher 峰值
均为 1575 rad/s。

| 目标 \(\lambda\) | 通过 | Transition time (s) | 达到 8 m/s (s) | 起转后新增掉高 (m) | 高度 RMSE (m) | 最大 \(|e_h|\) (m) | 最大空速 (m/s) | 机械能量代理 (J) | 最大 lift \(\omega\) (rad/s) | Settling (s) | 稳态平均误差 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.1 | 5/5 | \(5.061\pm0.059\) | \(3.159\pm0.068\) | \(0.007\pm0.009\) | \(0.340\pm0.139\) | \(0.641\pm0.202\) | \(13.825\pm0.205\) | \(11021\pm1032\) | \(991\pm164\) | \(0.300\pm0.000\) | \(0.0020\pm0.0001\) |
| 0.3 | 5/5 | \(5.820\pm0.041\) | \(3.120\pm0.025\) | \(0.049\pm0.058\) | \(0.304\pm0.163\) | \(0.557\pm0.252\) | \(15.987\pm0.256\) | \(10897\pm590\) | \(893\pm120\) | \(1.100\pm0.000\) | \(0.0020\pm0.0001\) |
| 0.5 | 5/5 | \(6.610\pm0.111\) | \(3.119\pm0.112\) | \(0.012\pm0.016\) | \(0.296\pm0.088\) | \(0.699\pm0.210\) | \(17.730\pm0.212\) | \(9985\pm489\) | \(849\pm123\) | \(1.900\pm0.000\) | \(0.0020\pm0.0001\) |
| 0.7 | 5/5 | \(7.430\pm0.039\) | \(3.151\pm0.054\) | \(0.044\pm0.063\) | \(0.401\pm0.131\) | \(1.220\pm0.233\) | \(19.551\pm0.059\) | \(10999\pm481\) | \(893\pm119\) | \(2.700\pm0.000\) | \(0.0020\pm0.0001\) |

这组数据支持一个明确的工程结论：连续升力旋翼卸载接口在
\(\lambda=0.1\) 到 0.7 范围内可以稳定接收、执行并驻留，稳态跟踪误差
约 0.002，没有 overshoot。随着目标 \(\lambda\) 增大，transition time
有所增加，但该现象受到固定 \(\lambda\) ramp rate、目标误差带和驻留判据
的直接影响，因此不作为 \(\lambda\) 产生实际飞行动力学作用的证据。
后续 Phase-0.75 将在固定空速条件下，通过 lift-rotor thrust、转速、
功率以及高度/姿态响应验证 \(\lambda\) 的实际物理作用。当前不修改
pusher ramp 的原生 fallback；Phase-0.75 只在外部命令新鲜且合法时覆盖
pusher throttle。

## F1 放行条件

正式 F1 仍不放行。当前已满足 Native/Manual 各 5 次独立重启、受控
\(\lambda=\{0.1,0.3,0.5,0.7\}\) transition characterization、pusher
诊断和 lift-rotor spike 统计。Phase-0.75 已新增固定空速实验入口，
但必须先由实飞/仿真数据证明其 \(V_a\) measurement window 达标；之后
仍需满足：

- 加入指定空速 \(V_a\) 的稳定驻留控制，使采样发生在 \((V_a,\lambda_{exec})\) 二维目标带内；
- 正式表格分别报告两段机械能量代理，不能用全转换总值直接宣称某 schedule 节能。

## Phase-0.75：固定 \(V_a=12\,m/s\) 的闭环表征

Phase-0.75 的目的不是重复 Phase-0.5 的 transition characterization，
而是在相同空速条件下验证 \(\lambda\) 是否真实改变 lift-rotor thrust、
转速和功率。新增 `schedule_mode=va_hold_target`，执行顺序为：

1. 进入 `TRANSITION_FW` 后先发布 \(V_{a,d}=12\,m/s\) 的速度 setpoint，
   同时发布 \(\lambda=0\) 保持 PX4 受控 transition gate。
   速度 setpoint 只作为轨迹/姿态侧辅助，不能当作 pusher 油门闭环证据。
   Phase-0.75 现使用独立的 pusher throttle override：
   `/fmu/in/pusher_throttle_setpoint`、`/fmu/out/pusher_throttle_status`
   和 `/fmu/out/pusher_throttle_active`。
   该 override 仅在 `VT_UNLD_TEST_EN=1`、前转换、命令新鲜且数值在
   `[0,1]` 内时接管 pusher；命令丢失或转换退出后自动退回 PX4 原生逻辑。
2. 当 \(|V_a-V_{a,d}|\le0.3\,m/s\) 连续 1 s 后，开始 ramp 到目标
   \(\lambda\)。
3. 当 \(|V_a-V_{a,d}|\le0.3\,m/s\) 且
   \(|\lambda_{exec}-\lambda_d|\le0.03\) 连续 1 s 后，进入 3 s
   measurement window。
4. measurement window 内继续保持同一 transition control mode；测完后
   停止刷新外部命令，释放 PX4 完成 FW 转换。

ROS 侧 pusher 控制为 PI 形式：
\[
u_P=\mathrm{sat}(u_{ff}+K_p(V_{a,d}-V_a)+K_i\int(V_{a,d}-V_a)dt)
\]
并带 anti-windup、非对称 slew rate、低通后的 \(V_a\) 反馈，以及接近目标
空速后的 bumpless handover。关键符号约定是：若 \(V_a>V_{a,d}\)，控制器
必须降低 \(u_P\)。

`debug_phase0_75_va12_lambda_0p3_smoke` 已证明旧方案失败：最大空速达到
30.656 m/s，高度误差达到 91.22 m，\(\lambda\) 从未进入 measurement
window。新评估器会将该类失败明确归因为
`primary_failure_cause=va_hold_settle_timeout`、`secondary_failure_cause=pusher_not_regulating_airspeed`，
而不是误判为 \(\lambda\) 跟踪失败。

CSV schema 升级到 v4，新增 `pusher_throttle_command`、
`pusher_throttle_status`、`pusher_throttle_external_active` 和对应 freshness
字段。评估器新增 `phase_0_75_va_hold_pass`，并只在 measurement window 内统计
\(\bar V_a\)、\(SD(V_a)\)、\(\bar\lambda_{exec}\)、pusher override active
比例、lift thrust、lift collective、lift/pusher/total power、高度误差、垂速、
AoA、pitch/q 和 servo saturation。固定空速点比较时优先使用平均功率而非总能量。
若 \(V_a>V_{a,d}+3\,m/s\)、\(|e_h|>5\,m\) 或 \(|v_z|>2\,m/s\) 持续
0.5 s，recorder 会提前中止该 run，避免再次出现无意义的 140 m 级爬升。

2026-08-30 复核 `phase0_75_va12_lambda_0p0_5x` 与
`phase0_75_va12_lambda_0p3_5x` 后发现，两组各有一次在 \(V_a<3\,m/s\)
早期 transition transient 中触发 `va_hold_vertical_speed_runaway`，尚未进入
pusher PI handover 或 \(\lambda\) measurement。为避免低速瞬态被误判为固定
空速测量失败，CSV schema 升级到 v5，新增
`va_vertical_abort_enabled` 和 `va_low_speed_vertical_transient`。高度 runaway
仍全程 hard abort；垂速 runaway 仅在 \(V_a\ge6\,m/s\) 或 pusher PI 已接管后
生效，低速超阈值垂速只记录为 diagnostic。

## 命令

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

./scripts/run_transition_test.sh none data/evaluation/<new-native> 180
./scripts/run_transition_test.sh manual_airspeed data/evaluation/<new-manual> 180
./scripts/run_lambda_characterization.sh 0.5 data/evaluation/<new-lambda-0p5> 180
```

三个脚本都拒绝覆盖已存在的 `telemetry.csv`，且每个输出目录使用独立 PX4 workdir。

当前版本的 5 次独立重启可统一执行：

```bash
./scripts/run_phase0_5_repeats.sh native data/evaluation/phase0_5_native_v5_5x
./scripts/run_phase0_5_repeats.sh manual data/evaluation/phase0_5_manual_v5_5x
./scripts/run_phase0_5_repeats.sh characterization data/evaluation/phase0_5_lambda_0p1_v5_5x 0.1
./scripts/run_phase0_5_repeats.sh characterization data/evaluation/phase0_5_lambda_0p3_v5_5x 0.3
./scripts/run_phase0_5_repeats.sh characterization data/evaluation/phase0_5_lambda_0p5_v5_5x 0.5
./scripts/run_phase0_5_repeats.sh characterization data/evaluation/phase0_5_lambda_0p7_v5_5x 0.7
```

Phase-0.75 先做单次 smoke，不要直接跑 25 次。顺序固定为：先
\((V_a,\lambda)=(12,0)\)，证明 pusher 闭环本身能稳住空速；再跑
\((12,0.3)\)，证明固定空速下能进入 \(\lambda\) measurement window。

```bash
./scripts/run_va_hold_characterization.sh 12.0 0.0 data/evaluation/debug_phase0_75_va12_lambda_0p0_pusher_pi_smoke 220
./scripts/run_va_hold_characterization.sh 12.0 0.3 data/evaluation/debug_phase0_75_va12_lambda_0p3_pusher_pi_smoke 220
```

上述两个 smoke 都通过后，再进入固定 \(V_a=12\,m/s\) 的 25 次独立重启：

```bash
./scripts/run_phase0_75_va_hold_repeats.sh data/evaluation/phase0_75_va12_lambda_0p0_5x 12.0 0.0
./scripts/run_phase0_75_va_hold_repeats.sh data/evaluation/phase0_75_va12_lambda_0p1_5x 12.0 0.1
./scripts/run_phase0_75_va_hold_repeats.sh data/evaluation/phase0_75_va12_lambda_0p3_5x 12.0 0.3
./scripts/run_phase0_75_va_hold_repeats.sh data/evaluation/phase0_75_va12_lambda_0p5_5x 12.0 0.5
./scripts/run_phase0_75_va_hold_repeats.sh data/evaluation/phase0_75_va12_lambda_0p7_5x 12.0 0.7
```
