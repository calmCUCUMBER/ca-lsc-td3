# F3：Transition capability 指标物理验证协议

日期：2026-09-04

## 冻结边界

F1、F2-A、F2-B 已完成并冻结。F3 不再修改已通过 9/9 架构验证的
continuous transition allocation：

\[
\lambda\uparrow\Rightarrow
\{T_L\downarrow,\;w_{MC}\downarrow,\;w_{FW}\uparrow\}.
\]

尤其是，\(\eta_C\) 只作为 capability feature、physics prior 和 safety
shield 的输入，`eta_c_modifies_px4_blending` 必须恒为 0。

## F3-A：\(\eta_L\) physical validation

统一定义为

\[
\hat L_w=\tfrac12\rho V_a^2S\hat C_L(\alpha),\qquad
\hat L_z=\hat L_w\cos\gamma\cos\phi,
\]

\[
\eta_L=\operatorname{sat}_{[0,1]}
\frac{\hat L_z}{\hat m(g+a_{z,d})+\varepsilon}.
\]

- \(V_a\) 必须来自 `selected_airspeed_mps=|V_g-V_w|`；
- \(\gamma,a_{z,d}\) 采用 ENU / up-positive；
- 质量必须显式记录为 `eta_l_estimated_mass_kg`，禁止在计算中写死
  5.025 kg；物理模型验证阶段令 \(\hat m=m_{actual}\)，后续鲁棒性实验才注入
  mass-estimation mismatch；
- 在线 feature 与物理回归样本必须分层：`eta_l_available=1` 表示
  \(\eta_L\) 可作为 TD3 capability feature 使用，低速时可取
  \(\eta_L=0\)；`eta_l_force_model_valid=1` 才表示该样本进入
  \(\hat L_z\) vs \(L_z^{GT}\) 的物理回归；
- force-model gate 固定为：`selected_airspeed_mps>=5 m/s`、
  `vrel_body_u_mps>1 m/s`、\(|\beta|<=0.35 rad\)、\(|\alpha|<=0.60 rad\)；
- \(C_L\) 使用与机体 SDF 一致的 offset、线性段、stall 与 post-stall 分段律。

### 独立 ground truth

F3 专用模型只把两片主翼的 stock Gazebo `LiftDrag` 替换成方程等价的
`ca_lsc::InstrumentedLiftDrag`。该插件施加物理引擎实际使用的力，并同步发布
同一 lift vector；两片主翼 ENU-z 分量之和定义为 \(L_z^{GT}\)。它不是由飞机
总加速度反推，因此不混入旋翼推力、重力和惯性项。普通 F1/F2 模型不替换，
历史数据也不重写。

报告以下指标：

- force RMSE、NRMSE、bias、MAE、\(R^2\)；
- sample-wise absolute relative error 的 median 与 P95（只在
  \(|L_z^{GT}|\ge5\,N\) 时计算）；
- \(\eta_L\) RMSE 与 \(R^2\)；
- ground-truth freshness/coverage 和各飞行阶段的样本数。

阶段标签必须按实验类型解析：普通 `va_hold_target` 使用 `va_hold_phase`，
`gust_target` 才使用 `gust_phase`；固定窗口完成后的数据单列为
`post_measurement_release`，不得继续计作 `measurement`。

预注册的通过门槛为：至少 200 个样本、GT coverage 不低于 90%、force
\(R^2\ge0.90\)、NRMSE 不高于 20%、median relative error 不高于 20%。如果
门槛未通过，保留结果并修正气动估计模型，不能调低门槛掩盖偏差。
除 overall gate 外，每个 run 必须满足 GT coverage，每个 mass/wind condition
以及每个 \((condition,V_a,\lambda,A_g)\) operating point 都必须分别满足样本数、
coverage、NRMSE 和 median relative-error 门槛，防止总体平均掩盖局部失效。

取样覆盖完整 transition trajectory，而非只保留 successful measurement：
airspeed/lambda settle、固定窗口、gust exposure、recovery，以及 hard abort
之前的有效片段均保留。

### 执行顺序

先重建新增的 instrumented Gazebo plugin、bridge 和 schema-18 recorder：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws/log_ca build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build_ca \
  --install-base ros2_ws/install_ca \
  --packages-select vtol_px4_control ca_lsc_transition \
  --symlink-install
```

先跑单点 instrumentation smoke：

```bash
mkdir -p data/evaluation/f3a_eta_l_smoke_v1
./scripts/run_f3a_eta_l_smoke.sh \
  data/evaluation/f3a_eta_l_smoke_v1 220 \
  2>&1 | tee data/evaluation/f3a_eta_l_smoke_v1/runner.log
```

确认左右主翼 GT、真实 elevator joint、`eta_l`、`eta_l_gt`、body-relative
`u/v/w`、`\beta` 和 `eta_l_force_model_valid` 都连续有效后，再跑
正式 24 个 steady cells 加 3 个 dynamic probes、共 81 个 valid runs 的
validation matrix：

```bash
mkdir -p data/evaluation/f3a_eta_l_validation_v1
./scripts/run_f3a_eta_l_validation.sh \
  data/evaluation/f3a_eta_l_validation_v1 220 \
  2>&1 | tee data/evaluation/f3a_eta_l_validation_v1/runner.log
```

steady 矩阵覆盖 \(V_a=6,8,10,12,14,18\,m/s\)、\(m/m_0=1.0,1.2\)、
沿航向 wind 0/4 m/s，每 cell 3 个 data-quality-valid independent restarts。
analyzer 必须输出 `mean_wind_heading_parallel_mps` 与
`mean_wind_heading_cross_mps`；若旧数据的 wind4 实际为 crosswind，则只能作为
crosswind stress data，不计入预注册 longitudinal wind matrix。
另加 \((10,0.5,A_g=4/8)\) 和已知危险区域 \((12,1.0,A_g=8)\) 三个
dynamic probes，保证记录 gust exposure、recovery 和 hard abort 前样本。这里的
wind 只增加模型验证覆盖，不重开 F2-B 科学问题。

如果 strict validation 未通过，先用新 analyzer 离线重算已有 telemetry，分开报告
protocol-invalid attempts、force-model-valid samples、quasi-steady phases 与
gust exposure/recovery。不得仅因 overall \(R^2\) 好就宣布 F3-A pass。

物理验证通过后，用冻结的 F1/F2-A points 表生成描述性对比图：

```bash
PYTHONPATH=src python -m ca_lsc_td3.evaluation.f3_capabilities plot-outcomes \
  data/evaluation/f1_transition_allocator_v1/f1_grid_points.csv \
  data/evaluation/f2a_payload_formal_v1/combined/f2a_payload_points.csv \
  --output-dir data/evaluation/f3a_eta_l_outcomes_v1
```

图只说明 outcome 在 \(V_a-\lambda\) 与 \(\eta_L-\lambda\) 坐标中的聚集趋势；
正式 ROC-AUC/F1 信息增益属于 F5，不能仅凭可视化提前宣称。

验证器输出包括：

- `f3a_eta_l_validation.json`：完整门槛与 overall verdict；
- `f3a_eta_l_validation_runs.csv`：逐 run 指标、protocol-valid 状态、wind
  heading projection 与 body-relative flow diagnostics；
- `f3a_eta_l_validation_conditions.csv`：逐 mass/wind condition 指标；
- `f3a_eta_l_validation_operating_points.csv`：逐 \(V_a,\lambda,A_g\) 指标；
- `f3a_eta_l_validation_phases.csv`：settle、measurement、release、gust/recovery
  等分阶段指标。

## F3-B：\(\eta_C\) 实现与验证边界

schema 18 同步记录未加权的 PX4 virtual FW pitch demand、Gazebo 实际 elevator
joint position、方向相关剩余舵量和物理尺度 moment terms。定义：

\[
M_{e,ava}=\bar qSc|\hat C_{m_{\delta_e}}|\Delta\delta_{e,ava},
\]

\[
\eta_C=g_q\operatorname{sat}_{[0,1]}
\left(1-\frac{|\Delta M_{req}^{FW}|}{M_{e,ava}+\varepsilon}\right).
\]

这里的需求严格定义为相对于当前实际 elevator 状态的**增量力矩需求**，不能把
总需求除以剩余增量能力而双重计数。\(C_{m_{\delta_e}}\) 的符号决定需要向上限
还是下限计算剩余舵量；零动压、零导数或所需方向无剩余行程时 \(\eta_C=0\)。

F3-B 诊断不得影响 F3-A recorder：实际 elevator angle 轻微越过 Gazebo joint
limit 时，recorder 只允许 clip 后保留原始角度、令 `eta_c_valid=0` 或
`\eta_C=0`，不能抛异常终止 telemetry。

F3-B 正式结束前仍需对 normalized shadow-demand 到目标舵角/物理力矩的映射做
独立辨识与 hold-out 验证。当前 schema-18 先完成必需的原始量记录，不能把
`eta_c_valid=1` 解读为已完成科学验证。

### F3-B1：elevator incremental moment derivative validation

F3-B1 的目标不是验证整套 \(\eta_C\)，而是先验证 elevator 增量力矩斜率
\(C_{m_{\delta_e}}\) 的局部物理尺度。验证点固定为：

\[
(V_a,\lambda)=(6,0.3),(10,0.6),(14,0.6),(18,0.6).
\]

识别激励只允许在目标工况建立后的 fixed measurement window 内打开。起飞、
airspeed/lambda settle、release 和进入 FW hold 后必须关闭 dither，避免把工况建立
瞬态误认为舵面局部辨识样本。schema-21 记录
`elevator_id_dither_scale_command`，并通过 Gazebo dither-scale topic 控制
instrumented elevator plugin。

由于 elevator 绝对力矩包含 operating-point-dependent baseline moment，F3-B1 的主回归
使用

\[
M_e^{GT}=b+K_\delta(\bar q\,\delta_e)+K_\alpha(\bar q\,\alpha).
\]

其中 \(K_\alpha\) 只作为 nuisance term，用来分离 AoA/baseline moment；科学判据只看
\(K_\delta\) 是否匹配 SDF 配置的 \(C_{m_{\delta_e}}\)。单变量
\(M_e^{GT}=b+K(\bar q\delta_e)\) 和 fixed-slope \(R^2\) 只作为诊断输出，不再作为
阻塞门槛。

预注册 F3-B1 cell gate：

- measurement samples \(\ge50\)；
- \(\operatorname{span}(\bar q\delta_e)\ge0.5\,Pa\cdot rad\)；
- 二变量回归 \(R^2\ge0.95\)；
- 二变量回归 RMSE \(\le0.05\,Nm\)；
- \(|K_\delta-K_{\delta,config}|/|K_{\delta,config}|\le25\%\)。

若某个点没有进入 fixed measurement window，应标记为 protocol invalid 并重跑；不能用
无 measurement 的样本判断 \(C_{m_{\delta_e}}\) 失败，也不能通过调低激励门槛掩盖问题。

### F3-B1 第二半：directional remaining-authority validation

在 \(K_\delta\) 已通过局部辨识后，继续验证 \(\eta_C\) 分母中的可用增量力矩：

\[
M_{e,ava}^{est}=\bar q|K_{\delta,config}|\Delta\delta_{e,ava}.
\]

GT comparator 使用同一 measurement window 内由 instrumented elevator wrench 拟合出的
局部模型：

\[
M_e^{GT}=b+K_\delta(\bar q\delta_e)+K_\alpha(\bar q\alpha),
\]

并在相同 \(q,\alpha\) 下计算：

\[
M_{e,ava}^{GT}
=
\left|M_e^{GT}(\delta_{limit})-M_e^{GT}(\delta_e)\right|.
\]

方向限制 \(\delta_{limit}\) 必须与在线 \(\eta_C\) 符号逻辑一致：先用
\(M_{req}^{FW}/(\bar q K_{\delta,config})\) 判断所需 elevator 角度增量方向，再选择
上限或下限。不能只看 \(M_{req}^{FW}\) 的正负，因为 \(K_{\delta,config}<0\) 时会选反。

由于 \(M_{e,ava}\) 随动态压显著放大，绝对 RMSE 只作为诊断；主门槛使用相对误差：

- samples \(\ge50\)；
- recorded remaining travel 与重新计算的 directional remaining travel 匹配比例
  \(\ge95\%\)；
- normalized RMSE \(\le15\%\)；
- median relative error \(\le15\%\)；
- P95 relative error \(\le25\%\)。

该步骤只验证 \(\eta_C\) 的物理分母，不验证 shadow demand 是否足够代表真实未来控制需求；
后者属于后续 \(\eta_C\) behavior / information-gain 实验。

### F3-B2：eta_C end-to-end behavior validation

F3-B2 在 F3-B1 已冻结的 \(K_\delta\) 和 remaining-authority 口径上验证整套
\(\eta_C\) 行为。该阶段不得改 PX4 transition allocation；telemetry 中
`eta_c_modifies_px4_blending` 必须保持为 0。GT 指标定义为：

\[
\eta_C^{GT}
=
g_q\operatorname{sat}_{[0,1]}
\left(1-\frac{|M_{req,test}^{FW}|}{M_{e,ava}^{GT}+\varepsilon}\right).
\]

其中 \(M_{e,ava}^{GT}\) 优先使用 recorder 中按 F3-B1 冻结斜率计算的
`moment_available_gt_nm`；旧数据没有该字段时，分析器可回退到 F3-B1 的
run-intercept / within-run-centered 局部拟合。

F3-B2 smoke 包含三组因果隔离检查：

- `q_gate`：固定 \(\lambda=0.3\)，令 \(V_a=6,8,10\,m/s\)，验证
  \(\bar q\uparrow\Rightarrow g_q\uparrow\Rightarrow\eta_C\uparrow\)；
- `demand`：固定 \(V_a=14,\lambda=0.6\)，只改变 shadow diagnostic demand，
  验证 \(|M_{req}^{FW}|\uparrow\Rightarrow\eta_C\downarrow\)；
- `remaining_travel`：固定 \(V_a=10,\lambda=0.3\) 和
  \(M_{req,test}^{FW}=0.7\,Nm\)，只改变用于 \(\eta_C\) 诊断的
  `elevator_control_angle_for_eta_c_rad`，验证
  \(\Delta\delta_{e,ava}\downarrow\Rightarrow M_{e,ava}^{GT}\downarrow
  \Rightarrow\eta_C\downarrow\)。

为避免干预混杂，schema-23 将真实 shadow controller 输出与行为测试控制量分开：

- `shadow_fw_pitch_moment_req_nm` / `shadow_fw_pitch_moment_req_raw_nm`：
  原始 shadow FW pitch 增量力矩需求；
- `shadow_fw_pitch_moment_req_for_eta_c_nm`：F3-B2 行为 smoke 中实际送入
  \(\eta_C\) 公式的测试需求。

`remaining_travel_behavior_pass` 必须基于
`directional_remaining_for_eta_c_mean_rad`，并同时检查
`shadow_fw_pitch_moment_req_for_eta_c_nm` 在三阶段近似恒定。不能用 raw demand
被 bias 间接改变后的结果判断 remaining-travel 因果关系。
