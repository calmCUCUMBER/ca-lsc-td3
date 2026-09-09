# CA-LSC-TD3

这是新论文工程的独立运行路径，研究对象是 Lift+Cruise VTOL 的恒高悬停—巡航转换。高层强化学习只决定连续转换分配因子 \(\lambda\)；同一个执行值同步升力旋翼名义卸载与 PX4 MC/FW 姿态控制权转移。Pusher 仍由独立空速控制器负责，PX4 低层继续负责姿态、高度和速度稳定。新增测试只使用本目录的 `build_ca`、`install_ca` 和 `build_ca_make`；复制进来的旧 build/install 缓存不作为有效实验环境。

方法主线：

```text
eta_L + eta_C
  -> physics prior
  -> CfC residual actor
  -> twin reward critics + twin safety critics
  -> Lagrangian update
  -> hard shield + asymmetric rate limiter
  -> transition allocator (lift unloading + PX4 attitude allocation)
```

当前 schema-13 连续 transition allocator 已通过同步验收并冻结；F1/F2/F3/F5、
Physics Prior Only 与 A0 ROS/PX4 在线训练链路已经完成分阶段验证，目前处于 A0
Vanilla TD3 基线训练与确定性评估阶段。架构变更前的 v7/v8/v9 及
transition-gap 数据统一视为 development/diagnostic。
现阶段已经：

- 保存两份原始研究方案；
- 整理方法定义、实验顺序和迁移白名单；
- 复用 `vtol_nav` 环境定义；
- 复制 CfC 的 PyTorch 核心实现及 Apache-2.0 许可证；
- 在 PX4 中加入连续 lift-rotor collective 卸载输入与实际执行值回传；
- 新增无导航/无感知的 50 m 空场转换 launch、遥测 recorder 和验收器；
- 对原有 Native 5 次与 Manual 5 次数据完成复核，并保留原始数据不覆盖；
- 增加严格起转稳定门、转换增量掉高、pusher/旋翼尖峰、\(q\) 和分段能量诊断；
- 增加 PX4 受控转换测试模式，并完成 \(\lambda_{target}=0.5\) 的全栈冒烟；
- 实现并测试 `smoothstep`、\(\eta_L\)、\(\eta_C\)、physics prior、hard projection 与非对称速率限制。
- 冻结 `config/nominal_low_level.env`，正式 runner 不再按目标空速修改 PX4
  transition 或 fixed-wing airspeed 内参；
- 外部命令有效时由 \(\lambda_{exec}\) 同步 PX4 MC/FW roll/pitch/yaw 权重，
  命令失联时恢复旋翼支撑并回退原生 PX4 airspeed blending；
- 增加 schema 13 的实际/期望 MC/FW 权重、同步 RMSE、normalized shadow
  demand 和 elevator \(\pm0.53\,rad\) 限位命令诊断；
- 保留功率代理的物理一致性警告，未经独立标定不报告正式节能百分比。
- 完成 nominal F1、payload/gust F2、\(\eta_L/\eta_C\) F3 与预测效用 F5 的
  分阶段验证并冻结其底层实验口径；
- 完成 Physics Prior Only nominal qualification，并建立带真实 PX4 terminal
  语义的 A0 Vanilla TD3 在线训练与 deterministic evaluation pipeline。

## 目录

```text
config/                    实验与外部依赖配置
docs/                      方法、实验路线、迁移记录和原始方案
environment/               继续使用的 vtol_nav 环境定义
reference/                 从旧仓库复制的只读参考，不参与当前构建
src/ca_lsc_td3/            新方法代码
tests/                     不依赖 Gazebo 的快速单元测试
third_party/cfc/           上游 CfC PyTorch 文件、README、许可证
ros2_ws/src/ca_lsc_transition/  Phase 0 launch、logger、schedule、评估器
data/                      新实验输出（默认不进版本控制）
```

`ros2_ws/src` 中旧的 bringup/perception/rl/simulation/visualization 目录仅作
迁移参考。正式重建必须显式使用 `--packages-select px4_msgs vtol_px4_control
ca_lsc_transition`，避免把旧导航包混入新论文运行链路。

## 环境与快速检查

继续使用已有 Conda 环境，不创建第二套环境：

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
./scripts/check_environment.sh
./scripts/test_core.sh
```

如需修复环境，参见 `environment/README.md`。ROS 2 包的编译仍应在退出 Conda 后进行；训练与 Python 测试再进入 `vtol_nav`。

## 获取固定版本的外部依赖

PX4、Micro-XRCE-DDS-Agent 与 `px4_msgs` 体积较大，不直接提交到本仓库。
它们的上游版本固定在 `config/dependencies.repos`，CA-LSC 所需的 PX4 与
消息定义修改保存在 `patches/`。克隆本仓库后执行：

```bash
python -m pip install vcstool
./scripts/bootstrap_dependencies.sh
```

脚本会拉取固定提交、只应用一次项目补丁，并初始化 PX4 submodules。实验原始
telemetry、训练输出、checkpoint、ROS/PX4 构建目录同样不进入 Git；这些内容应
在本机或独立 artifact 存储中归档。

## Phase 0 / Phase-0.5 测试

已通过的测试 1–5、阈值、失败样本和限制见
`docs/phase0_tests_1_to_5.md`。机器可读汇总位于
`data/evaluation/phase0_tests_1_to_5_summary.json`。

复现时必须给每次试飞新的输出目录：

```bash
./scripts/calibrate_airframe.py
./scripts/run_transition_test.sh none data/evaluation/<new-native-run> 120
./scripts/run_transition_test.sh manual_airspeed data/evaluation/<new-manual-run> 120
```

评审意见对应的 Phase-0.5 协议、指标口径和当前证据见
`docs/phase0_5_protocol.md`。当前 schema v3 记录高度 datum/local/relative
和逐条件稳定门诊断；受控点可用：

```bash
./scripts/run_lambda_characterization.sh 0.5 data/evaluation/<new-lambda-run> 180
```

5 次独立重启用新目录，脚本会在单次失败后继续跑完并写出
`repeat_summary.json`：

```bash
./scripts/run_phase0_5_repeats.sh native data/evaluation/phase0_5_native_v3_5x
./scripts/run_phase0_5_repeats.sh manual data/evaluation/phase0_5_manual_v3_5x
./scripts/run_phase0_5_repeats.sh characterization data/evaluation/phase0_5_lambda_0p5_v3_5x 0.5
```

Phase-0.75 已实现指定空速驻留和连续 \(\lambda\) 控制。schema-v7 已完成
64 个探索性 F1 点，但旧 recorder 允许拼接不连续 measurement segments，且
缺少 24 个 0.1 网格点，因此不作为最终 F1。schema-v8 虽完成 88 点，但其
单样本软带宽中止规则导致大量协议性 I，仅作为诊断批次。schema-v9 把测量
放在 2 s 联合稳定之后，固定连续记录 3 s，并在窗口结束后使用 90% 带内占比
与 0.6 m/s 最大误差保护；P attempt 独立审计，不再覆盖物理分类。这些数据全部
是开发/诊断数据，不进入论文最终统计。schema-13 同步验收现已完成 9/9 点；
`Va=14 m/s, lambda=0.7` 的首次尝试在 lambda 仍为 0 时发生协议瞬态，随后
有效 retry 已达到目标并验证 (w_{MC}/w_{FW}=0.25/0.75)。底层架构已冻结，
正式 F1 已授权：

```bash
./scripts/run_f1_full_grid_v9.sh \
  data/evaluation/f1_transition_allocator_v1 220
```

正式 runner 会先校验 `config/transition_allocation_freeze.json`：9 点同步证据及
PX4、ROS、映射、协议文件哈希必须与冻结版本一致，否则在创建正式数据前停止。
正式网格包含
88 个 \((V_a,\lambda)\) 单元，每点目标 5 个有效重复、最多 10 次有审计的
attempt。当前 Gazebo pusher 静态功率代理在高速下违反 \(P\ge TV\)，所以旧
proxy 只保留内部诊断；独立功率标定完成前不得据此宣称节能比例。详见
`docs/f1_protocol.md`。

## 开发顺序

严格按以下顺序推进，每一步单独验收并保存结果：

```text
传统基线 + lambda-PX4 同步验收
-> F1 Va-lambda 扫描
-> F2 payload/wind sensitivity
-> F3 eta_L
-> F4 eta_C
-> F5 eta_L+eta_C 信息增益
-> physics prior only
-> residual MLP-TD3
-> CfC
-> twin safety critics + Lagrangian
-> hard shield
-> 正式对比、泛化、1000 次 Monte Carlo、消融
```

论文目录见 `docs/paper_outline.md`，详细定义见 `docs/method_blueprint.md`，实验顺序见 `docs/experiment_roadmap.md`。旧仓库源文件没有被迁移过程覆盖。
