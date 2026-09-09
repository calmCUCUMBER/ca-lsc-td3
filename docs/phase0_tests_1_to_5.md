# Phase 0：测试 1–5 验收记录

日期：2026-08-28

> 版本说明（2026-08-29）：本页保留 Phase 0 当时的验收记录，不应再被解释为
> Phase-0.5 或正式 F1 证据。对 5+5 次数据复核后新增的严格起转稳定门、分段
> 能量、pusher/旋翼尖峰诊断、低速迎角门控与受控 \(\lambda\) 模式，统一见
> `phase0_5_protocol.md`。其中旧数据及本页数值均未被覆盖。

这里的“测试 1–5”指开始 F1 扫描之前的五项工程验收，不是论文中的
F1–F5：

1. PX4 原生 Hover → Transition → Fixed-wing 基线；
2. 50 m nominal 恒高转换；
3. 连续升力旋翼卸载接口与推进/舵面遥测；
4. 手工空速–\(\lambda\) schedule；
5. 气动与推进功率模型标定。

## 结论

按 2026-08-28 的最小门槛，五项验收均通过；按 2026-08-29 的 Phase-0.5
严格门槛，需要使用新 schema 重跑，不能沿用本页的 PASS。测试使用 `/home/weicheng/ca_lsc_td3` 中重新配置的
PX4 构建和独立 ROS 2 构建，没有使用复制目录里指向旧 `vtol_sim` 的缓存。

| 项目 | 结果 | 主要证据 |
|---|---:|---|
| 1. 原生转换 | PASS | 状态按 HOLD_MC → TRANSITION_FW → HOLD_FW，4.800 s 完成，无 failsafe，FW 保持 6.0 s |
| 2. 50 m nominal | PASS | 转换段高度 RMSE 0.635 m，最大掉高 1.124 m，最低 48.876 m |
| 3. 接口与遥测 | PASS | 高度、空速、姿态角速度、3 路舵面、5 路电机角速度、\(\lambda_{exec}\) 和模型功率均满足值有限、消息年龄不超过 0.5 s；最低完整率 88.7% |
| 4. 手工 \(\lambda\) | PASS | 5.100 s 完成；转换态内 \(\lambda_d^{max}=0.411\)、\(\lambda_{exec}^{max}=0.302\)，33 个发布样本中 31 个由 PX4 确认 active |
| 5. 模型标定 | PASS | 从 stock `standard_vtol` SDF 提取质量、翼面、升降舵和 5 个旋翼参数，并生成三组曲线 |

## 对照数据

| 指标 | PX4 原生 schedule | 手工空速–\(\lambda\) schedule |
|---|---:|---:|
| 转换时间 (s) | 4.800 | 5.100 |
| 转换段高度 RMSE (m) | 0.635 | 0.479 |
| 最大绝对高度误差 (m) | 1.124 | 0.844 |
| 最大掉高 (m) | 1.124 | 0.381 |
| 转换段最大空速 (m/s) | 13.256 | 13.437 |
| 最大 \(\lambda_d\) | 无外部命令 | 0.4108 |
| 命令有效期间最大 \(\lambda_{exec}\) | 无外部命令 | 0.3018 |
| 平均 \(|\lambda_d-\lambda_{exec}|\) | 不适用 | 0.0457 |
| 模型推进能量积分 (J) | 8072 | 8244 |

能量值由 SDF 的理想机械模型
\(P=k_T k_M\omega^3\) 重建。当前仿真电池电流无效，因此这些数值不是实测
电功率，也不能用这两次试飞直接得出手工 schedule 更节能或更耗能的论文结论。
手工 schedule 比原生 schedule 更保守，保留了更多升力旋翼推力，能量积分更高是
本轮观测结果，后续 F1 扫描需在相同空速/\(\lambda\) 工况下作严格比较。

## 接口实现与安全语义

- 新增专用 uORB/DDS 输入 `lift_rotor_unloading_setpoint` 和回传
  `lift_rotor_unloading_status`，复用 `NormalizedUnsignedSetpoint` 消息结构。
- 外部 \(\lambda\) 只替换前转换期间的升力旋翼 collective 权重；PX4 原有
  roll/pitch/yaw 差动控制权重和 pusher 路径保持不变。
- 输入必须有限、位于 `[0,1]` 且处于时间窗内；越界值被拒绝而不是截成危险命令。
  500 ms 失联或 fixed-wing failure 都立即恢复 \(\lambda=0\)（全 collective
  support），退出前转换则解除外部接管并回到 PX4 原生逻辑。
- `lift_rotor_unloading_active` 独立回传 PX4 是否接受并正在使用外部命令；验收不再
  用 stock schedule 的状态值推断外部接口已接管。
- 卸载速率上限为 0.25/s，恢复升力速率为 2.0/s。
- 手工 schedule 只在 `TRANSITION_FW` 且空速超过 8 m/s 后接管；目标由 8–16 m/s
  smoothstep 映射到 `[0,0.6]`。这避免在 pusher 建立空速前用外部零命令抢占
  collective 路径。

## 验收规则

- 转换状态顺序完整、无 failsafe、25 s 内进入 FW，并连续保持至少 5 s；
- 转换段最低高度不低于 45 m；高度 RMSE 不高于 3 m，最大绝对误差不高于 5 m；
- 十五个必需遥测字段的“有限值且消息年龄不超过 0.5 s”比例均不低于 80%；
- 手工 \(\lambda\) 必须在 `TRANSITION_FW` 内至少有 5 个有效命令/执行配对，且
  \(\lambda_d^{max}\ge0.05\)、\(\lambda_{exec}^{max}\ge0.02\)，且 PX4 active
  覆盖率不低于 80%、平均/最大跟踪误差分别不高于 0.1/0.2。进入 FW 后 stock
  状态回报的 \(\lambda=1\) 不计入此判定。

## 标定结果

- 总质量：5.02500003 kg；主翼总面积：1.0 m²；
- 升降舵范围：[-0.53, 0.53] rad；
- SDF 量纲俯仰力矩导数：\(\partial M/\partial\delta_e/q=-0.06\,m^3\)；
- 四个 lift rotor：\(k_T=2.0\times10^{-5}\)、\(k_M=0.06\,m\)；
- pusher：\(k_T=8.54858\times10^{-6}\)、\(k_M=0.01\,m\)；
- 模型悬停角速度：784.844 rad/s；模型理想 lift 功率：2320.55 W。

`Cm_delta_e` 和平均气动弦长仍未声称已经辨识：SDF 只足以给出量纲导数，后续若
论文需要无量纲系数，应补充定义与辨识实验。

## 失败样本与限制

第一次手工试飞 `phase0_manual_lambda_50m_run1` 在进入转换前就持续发布外部
\(\lambda=0\)，pusher 未建立推力，25 s 超时。该失败样本保留，没有删除；修复
接管时机后，run2 首次通过；增加独立 active 回传与消息新鲜度验收后，当前
run3 再次通过。

当前代码版本的正式证据为 `phase0_native_50m_run3` 和
`phase0_manual_lambda_50m_run3`。此前 run2 是开发过程成功样本，但缺少独立
active 回传和消息新鲜度字段，不作为最终 Test 3/4 证据。

目前每种模式只有一次当前版本的正式成功全栈试飞，证明的是接口和最小可运行性，不是统计
重复性。进入正式 F1 前应再做 5 次独立重启的基线与手工 schedule 重复测试，并
冻结 PX4 参数、随机种子和每次 ULog。迎角字段目前由地速和姿态估算，只适用于
本轮无风空场；F2 有风实验必须改为空气相对速度。

## 复现命令

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"

./scripts/calibrate_airframe.py
./scripts/run_transition_test.sh none <new-output-directory> 120
./scripts/run_transition_test.sh manual_airspeed <new-output-directory> 120
./scripts/test_core.sh
```

脚本拒绝覆盖已有 `telemetry.csv`，每次测试需使用新的输出目录。
