# F2：载荷/风扰动下的边界敏感性协议

日期：2026-09-03

F2 不重复完整的 \(8\times11\times5\) F1 网格。F1 已经证明
\(\lambda\) 是有效的 continuous transition allocation factor；F2 的任务是证明
固定空速 schedule 不足以唯一决定安全/合适的转换状态：

\[
\lambda_{\max}^{feasible}\ne f(V_a)\ \text{alone}.
\]

因此 F2 在 F1 边界附近重复一张缩小版 \(V_a-\lambda\) 网格，改变真实质量和
阵风，观察同一 \(V_a\) 下 N/B/I/X 分类与有效重复
成功率是否移动。前面的少量代表点只属于 smoke，不能替代正式 partial scan。

## 当前前置条件

- schema-13 transition allocation 已冻结：外部 \(\lambda\) 同步控制
  lift unloading 与 MC/FW pitch-weight allocation。
- 正式 F1 已完成：`data/evaluation/f1_transition_allocator_v1`。
- F1 能耗仍只可使用 effort proxy；F2 暂不报告节能最优，只报告可行性、
  高度/空速/姿态/安全边界变化。
- WQ-v3 已通过：steady Gazebo wind 生效，且
  `airspeed_source:=relative_wind` 时 pusher PI、fixed-\(V_a\) dwell 和
  evaluator 均使用 `selected_airspeed_mps=vrel_norm_mps`。
- 从 F2 开始，论文中的 \(V_a\) 默认指 `selected_airspeed_mps`；原始
  `airspeed_mps` 只作为 PX4/native diagnostic。
- F2-A payload smoke 已完成。payload 通过生成模型中的真实 fixed payload link
  实现，不只修改算法估计质量。
- F2-A 正式实验必须使用每次试验的 `actual_model_mass_kg_config` 计算离线
  \(\eta_L\) proxy，并使用 `selected_airspeed_mps`；旧的固定 5.025 kg 与
  `airspeed_mps` 口径已废止。

## F2-A 正式 payload partial scan（已冻结）

论文主图固定以下 18 个点；额外采集的 ((10,1.0)) 只保留在原始数据和
supplemental diagnostic 中，不进入主图或主边界：

| \(V_a\) (m/s) | \(\lambda\) |
| ---: | --- |
| 6 | 0.3, 0.4, 0.5, 0.6 |
| 8 | 0.5, 0.6, 0.7, 0.8 |
| 10 | 0.7, 0.8, 0.9 |
| 12 | 0.6, 0.8, 1.0 |
| 14 | 0.7, 0.8, 0.9, 1.0 |

三种真实质量为 \(m/m_0=1.0,1.1,1.2\)。每个 cell 需要 5 个
data-quality-valid independent repeats，总计 \(18\times3\times5=270\) 个主实验有效
run。每个 cell 最多 10 次 attempt；不足 5 个有效重复时标记
`protocol_unstable`，不无限补跑。

## 扰动组合

F2 执行顺序为：

1. F2-A formal payload partial scan：\(m/m_0=\{1.0,1.1,1.2\},\ V_w=0\)
2. F2-B formal deterministic-gust partial scan：\(A_g=\{0,4,8\}\,m/s\)
3. F2 在 F2-B 后冻结；combined stress、turbulence 与 aero mismatch 留给后续
   domain randomization、泛化和 Monte Carlo，不再扩成 F2-C。

steady 0/4/8 m/s 已完成的实验是 Wind Qualification，只证明风与相对空速链路，
不作为正式 F2-B。F2-B 冻结为 9 个 cell：\(V_a=10\) 时
\(\lambda=0.5,0.6,0.7\)，\(V_a=12\) 时 \(\lambda=0.6,0.8,1.0\)，
\(V_a=14\) 时 \(\lambda=0.8,0.9,1.0\)。三档阵风、每 cell 5 个有效重复，
共 135 个有效 run。

## F2-B deterministic-gust 协议

名义质量下施加沿当前航向的确定性顺风脉冲：

\[
V_g(t)=\frac{A_g}{2}\left[1-\cos\left(2\pi\frac{t-t_g}{T_g}\right)\right],
\quad A_g\in\{0,4,8\}\,\mathrm{m/s}.
\]

固定 `baseline delay=0.5 s`、`gust duration=2.0 s`、`recovery=2.0 s`。
阶段 A 仍用既有 Va/lambda dwell 建立目标；在目标建立前失败属于 protocol-invalid
并重试。阶段 B 不使用 Va occupancy 判定阵风数据有效性，只检查 wind bridge/profile
以及硬安全包线。阶段 C 检查 2 s 内恢复；硬违规为 X，未恢复为 I，可恢复但重复
软退化为 B，其余为 N。

先重建新增的 Gazebo wind bridge 和 recorder：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws/log_ca build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build_ca \
  --install-base ros2_ws/install_ca \
  --packages-select vtol_px4_control ca_lsc_transition \
  --symlink-install
```

先只跑一个 \(A_g=4\) smoke：

```bash
mkdir -p data/evaluation/f2b_gust_smoke_v1
./scripts/run_f2b_gust_smoke.sh \
  data/evaluation/f2b_gust_smoke_v1 10 0.6 4 220 \
  2>&1 | tee data/evaluation/f2b_gust_smoke_v1/runner.log
```

确认 `gust_bridge_ready`、命令/回读风矢量和 `selected_airspeed_mps` 响应后，才运行
135 次正式矩阵：

```bash
mkdir -p data/evaluation/f2b_gust_formal_v1
./scripts/run_f2b_gust_grid.sh \
  data/evaluation/f2b_gust_formal_v1 220 \
  2>&1 | tee data/evaluation/f2b_gust_formal_v1/runner.log
```

正式输出包括三档阵风并排的 N/B/I/X、recovery pass-rate 与最大高度误差图。
N/B/I/X 颜色与 F1/F2-A 统一：绿/黄/橙/红，未解析为灰。

## F2-A payload smoke 命令

先跑最小边界点集，不要一上来铺满所有扰动：

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws/log_ca build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build_ca \
  --install-base ros2_ws/install_ca \
  --packages-select ca_lsc_transition \
  --symlink-install

mkdir -p data/evaluation/f2a_payload_smoke_v1

./scripts/run_f2a_payload_smoke.sh \
  data/evaluation/f2a_payload_smoke_v1 \
  220 \
  "6:0.5 8:0.7 10:0.9 12:0.6 14:0.9" \
  "1.0 1.1 1.2" \
  2>&1 | tee data/evaluation/f2a_payload_smoke_v1/runner.log
```

该脚本会为每个质量条件生成独立 Gazebo model/world，并把以下字段写入每个 run：

- `condition_id`
- `mass_scale_config`
- `nominal_model_mass_kg_config`
- `actual_model_mass_kg_config`
- `payload_mass_kg_config`
- `airspeed_source_config=relative_wind`

smoke 原始 telemetry 不需要重跑；修正统计代码后只离线重算 condition summary
和跨质量汇总。

## F2-A formal 运行命令

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

mkdir -p data/evaluation/f2a_payload_formal_v1
./scripts/run_f2a_payload_formal.sh \
  data/evaluation/f2a_payload_formal_v1 \
  220 \
  2>&1 | tee data/evaluation/f2a_payload_formal_v1/runner.log
```

正式脚本固定 18 个主 cell、三种质量及每 cell 5 个有效重复。它同时生成：

- `combined/f2a_payload_points.csv`：统一质量–空速–\(\lambda\) 表；
- `combined/f2a_payload_points_all.csv`：含额外 \((10,1.0)\) 诊断点的完整表；
- `combined/f2a_payload_outcome_maps.png`：三质量 N/B/I/X 并排图；
- `combined/f2a_payload_pass_rate_maps.png`：有效重复成功率图；
- `combined/f2a_payload_confirmed_feasible.png`：各质量的已确认可行上界；
- `combined/f2a_payload_boundaries.csv`：边界、右删失与非单调标记。

边界图只表达 partial grid 内的 maximum confirmed feasible，不强制随空速或质量
单调。局部 nonconvergent pocket 必须和 pass-rate 一起报告。

## 必须新增的工程接口

1. world 生成器或 launch 参数：
   - `wind_speed_mps`
   - `wind_direction_deg`
   - `wind_gust_enabled`
   - `wind_seed`
   - 第一版 steady wind world 可由
     `scripts/generate_wind_qualification_world.py` 生成；正式 F2 仍需将
     commanded/actual wind vector 写入 telemetry。
2. 机体质量变体：
   - `mass_scale`
   - 同步修改惯量或添加明确位置的 payload link；
   - 日志写入真实 \(m/m_0\)，不能只改算法里的 \(\hat m\)。
3. 气动 mismatch（第二版再做）：
   - `cl_scale`
   - `cm_delta_e_scale`
   - 明确区分仿真真实参数与算法估计参数。
4. F2 runner：
   - 使用版本化、冻结的 \((V_a,\lambda)\) 清单；
   - 每个 run 写入 `condition.json`、`summary.json`、`telemetry.csv`、
     `console.log`。
5. F2 汇总器（已实现 `ca_lsc_td3.evaluation.f2_payload`）：
   - 沿用 F1 的 N/B/I/X 与 data-quality 分离；
   - 输出同一 \((V_a,\lambda)\) 在不同扰动下的分类迁移表；
   - 输出 \(\Delta\lambda_{\max}^{feasible}\) 或边界退化方向；
   - 同时输出 pass-rate，不能只报告一条人为单调化的边界线。

## 论文报告口径

F2 不需要证明某个扰动一定导致失败；只要展示同一 \(V_a\) 下可行/边际/不安全
\(\lambda\) 区域随真实质量或风移动，就足以支持：

\[
V_a\ \text{alone is not a sufficient transition-state descriptor}.
\]

这一步完成后，才进入 F3：用 \(C_L(\alpha)\)、质量估计和姿态/速度信息验证
\(\eta_L\) 是否能解释这些边界移动。
