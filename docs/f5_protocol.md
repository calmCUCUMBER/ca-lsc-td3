# F5：Transition capability predictive utility 协议

日期：2026-09-06

## 冻结前提

F1、F2、F3-A、F3-B 已完成。continuous transition allocation 继续冻结：

\[
\lambda\uparrow\Rightarrow
\{T_L\downarrow,\;w_{MC}\downarrow,\;w_{FW}\uparrow\}.
\]

\(\eta_C\) 只作为 capability feature、physics prior 与 safety input，
`eta_c_modifies_px4_blending` 必须保持 0。F5 不修改 PX4，也不训练 TD3。

## 科学问题与公平比较

F5 只检验已经完成物理验证的 \(\eta_L,\eta_C\) 是否在空速之外提供未来
transition-feasibility 信息。主比较固定为同一 L2 logistic model family：

\[
\mathcal M_A=[V_a,\lambda_{exec}],\qquad
\mathcal M_B=[V_a,\eta_L,\eta_C,\lambda_{exec}].
\]

两边都必须包含实际执行的 \(\lambda\)，使用相同标准化、正则、fold 和阈值
选择方法。F1 阈值只允许在对应 training fold 上选择。

## 2 s 未来安全标签

对 telemetry 时刻 \(t\) 的真实 executed candidate \(\lambda_{exec}(t)\)，固定
\(T_p=2\,s\)。标签只读取 \([t,t+T_p]\) 的未来物理/安全量，不读取
\(\eta_L\) 或 \(\eta_C\)：

- 无 PX4 failsafe 或冻结的 hard abort；
- 有效 AoA 满足 \(|\alpha|\le0.3391428111\,rad\)（机翼 SDF stall angle）；
- \(|e_h|\le5\,m\)；
- \(|v_z|\le2\,m/s\)，坐标为 ENU/up-positive；
- telemetry 最大间隔不超过 0.5 s，关键安全通道 coverage 不低于 90%。

若 hard violation 在 2 s 内先发生，即使飞行因此提前终止，也标为 unsafe；若
既没有完整 2 s，又没有明确 hard violation，则该候选不贴标签，禁止默认为 safe。
数据以 10 Hz 抽样，原始 20 Hz 数据仍保留在 source telemetry 中。

## 防止数据泄漏

严禁随机拆分连续 telemetry rows。主分析按完整 operating-condition group 做
5-fold split；同一 \((V_a,\lambda,m,\mathbf V_w,A_g)\) 的所有独立 run
只能出现在同一个 fold。输出同时保留 `run_group`，便于审计单条 trajectory
从未跨 fold。

F3-B1 的 elevator identification dither，以及 F3-B2 的 shadow-demand / diagnostic
elevator-bias 注入，只用于物理辨识，必须出现在 audit 中但不得进入 F5 dataset。

## 指标与结论边界

报告 ROC-AUC、PR-AUC、F1、Brier score、calibration curve、逐 fold 差值和按
condition group 重采样的 bootstrap 95% CI。F5 不预设结果导向的最小提升幅度；
主方向证据为：

\[
ROC\text{-}AUC_B>ROC\text{-}AUC_A,\quad
PR\text{-}AUC_B>PR\text{-}AUC_A,\quad
Brier_B<Brier_A.
\]

若现有数据 class balance 或 current-schema coverage 不足，只在 F1/F2 已知边界
附近补少量 schema-23 runs，不重跑完整 F1/F2。

## 执行

先审计并生成 dataset：

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

PYTHONPATH=src python scripts/audit_f5_dataset.py \
  data/evaluation/f1_transition_allocator_v1 \
  data/evaluation/f2a_payload_formal_v1 \
  data/evaluation/f2b_gust_formal_v1 \
  data/evaluation/f3a_eta_l_validation_v2 \
  data/evaluation/f3b1_formal_v1 \
  data/evaluation/f3b2_eta_c_behavior_smoke_merged_v3 \
  --output-dir data/evaluation/f5_data_audit_v1
```

只有 `predictive_utility_ready=true` 时才运行：

```bash
./scripts/analyze_f5_predictive_utility.sh \
  data/evaluation/f5_data_audit_v1/f5_dataset.csv \
  data/evaluation/f5_predictive_utility_v1
```

输出包括 `f5_data_audit.json`、逐 run audit CSV、`f5_dataset.csv`、
`f5_predictive_utility.json` 和逐 fold CSV。

当前首次 audit 的冻结结论是：旧 schema-13/15/16 没有完整 capability features；
排除 F3-B1 dither 和 F3-B2 diagnostic injection 后，现有自然飞行数据只有 4 个
包含 unsafe future labels 的 physical condition groups，不足以支撑 5-fold F5。
使用下面的小型边界补充矩阵，而不是重跑完整 F1/F2：

```bash
mkdir -p data/evaluation/f5_boundary_supplement_v1
./scripts/run_f5_boundary_supplement.sh \
  data/evaluation/f5_boundary_supplement_v1 220 \
  2>&1 | tee data/evaluation/f5_boundary_supplement_v1/runner.log
```

该矩阵为 10 cells × 5 valid restarts：四组 \((V_a,\lambda)\) 分别比较
\(m/m_0=1.0,1.2\)，另在 \((14,0.9)\) 比较 0/8 m/s deterministic gust。
它只补 current-schema boundary trajectories，不改变或重复 F1/F2 的正式结论。
