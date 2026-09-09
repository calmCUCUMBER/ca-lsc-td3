# F1 高速基线修正与验收（schema v7）

> 历史诊断：schema 7 的 \(\lambda\) 尚未同步 PX4 MC/FW attitude weights；
> 本页不属于 transition allocator 架构下的正式论文数据。

日期：2026-08-31

## 根因

旧高速基线在 `Va=18/20 m/s, lambda=0` 下持续爬升。PX4 ULog 证明：

- `FlightTaskTransition` 将外部垂向速度参考渐近置零；
- standard VTOL 在整个受控卸载保持期间将俯仰固定为 `FW_PSP_OFF=2 deg`；
- lift collective 已到原生下限，继续发送向下速度不能消除机翼过剩升力；
- pusher、elevator 和 servo 并未先发生饱和。

因此这是受控卸载试验路径缺少传统固定翼高度—俯仰环，而不是 18/20 m/s
机体包线不可行。

## 修正

仅当 `VT_UNLD_TEST_EN=1`、处于前转换且外部 lambda 命令新鲜有效时：

1. 锁定进入保持时的 NED 高度 `z_ref`；
2. 计算 `theta_d = FW_PSP_OFF + K_h(z-z_ref) + K_v v_z`；
3. 将俯仰限制到 `[VT_UNLD_P_MIN, VT_UNLD_P_MAX]`；
4. 继续使用 PX4 原生姿态、固定翼力矩和舵面控制器。

冻结参数：`K_h=2 deg/m`、`K_v=2 deg/(m/s)`、俯仰限幅 `[-12,10] deg`。
普通 PX4 MC/FW 转换不走该实验分支。

## 五次独立重启结果

| Va | 完成率 | measurement Va (mean±std) | 高度 RMSE (mean±std) | 五次最大 |vz| 上界 | 饱和/spike |
|---:|---:|---:|---:|---:|---:|
| 18 m/s | 5/5 | 18.006±0.018 m/s | 1.382±0.340 m | 0.177 m/s | 0 |
| 20 m/s | 5/5 | 20.039±0.047 m/s | 2.159±0.234 m | 0.233 m/s | 0 |

五次中的最大绝对高度误差上界分别为 1.908 m 和 2.571 m。全部 run：

- `f1_measurement_pass=true`；
- 无 failsafe、无高度 runaway；
- 无 lift rotor、collective、pusher 或 servo 饱和；
- telemetry schema 为 7，四个高度—俯仰参数均随结果归档。

原始结果：

- `data/evaluation/f1_highspeed_baseline_v7_va18_lambda0_5x`
- `data/evaluation/f1_highspeed_baseline_v7_va20_lambda0_5x`
- 合并诊断：`data/evaluation/f1_highspeed_baseline_v7_summary`

## 决策

F1 有效空速上界保留 20 m/s，不把 18/20 m/s 作为超包线证据。由于控制律
已改变，schema v5 的旧粗扫只能保留为 pilot 数据；正式 F1 必须在 schema v7
下重跑完整 coarse grid，之后再补 0.1 lambda 分辨率。
