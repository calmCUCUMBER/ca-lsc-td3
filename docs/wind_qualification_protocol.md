# F2-W：Gazebo wind qualification 协议

日期：2026-09-02

F2 的风必须首先进入 Gazebo 物理环境，而不是直接注入 PX4 estimator。目标是验证：

\[
\mathbf V_w\ \text{存在},\qquad
V_a \approx |\mathbf V_g-\mathbf V_w|,\qquad
\text{气动力随 }|\mathbf V_g-\mathbf V_w|^2\text{ 改变}.
\]

截至 WQ-v3：Gazebo steady wind、aircraft link wind mode、wind-relative
airspeed control 已经完成确认。正式 F2 中的空速定义冻结为：

\[
\boxed{V_a \equiv selected\_airspeed\_mps = |\mathbf V_g-\mathbf V_w|}
\]

原始 `airspeed_mps` 只保留作 PX4/native 诊断量，不再作为 wind case 的科学
控制空速。WQ-v3 的 no-wind、4 m/s tailwind、4 m/s headwind 三组均完成固定
measurement window，`selected_airspeed_mps` 维持约 12 m/s，ground speed 按风速
平移。因此 wind qualification 状态为：

\[
\boxed{\text{WQ-v3 PASS；steady-wind infrastructure 可用于 F2}}
\]

进入正式 F2 时，先做 F2-A payload sensitivity；steady wind scan 放在 payload
之后，gust 仍留到后续版本。

## 已加入的工程入口

生成 wind qualification 专用 world/model：

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

./scripts/generate_wind_qualification_world.py \
  data/evaluation/wind_qualification/wind_4_x \
  --wind-enu 4 0 0
```

该脚本会：

- 从 PX4 stock `standard_vtol` 复制一个 `standard_vtol_wind` 模型；
- 给 aircraft 的每个 link 写入 `<enable_wind>true</enable_wind>`；
- 生成包含 `<wind><linear_velocity>...</linear_velocity></wind>` 的 world；
- 写入 `condition.json`，记录 wind frame、wind vector、world、model 与
  resource path。

运行 launch 时需要把生成的模型路径交给 Gazebo：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install_ca/setup.bash

ros2 launch ca_lsc_transition nominal_transition.launch.py \
  world:=/home/weicheng/ca_lsc_td3/data/evaluation/wind_qualification/wind_4_x/worlds/standard_vtol_wind.sdf \
  model_name:=standard_vtol_wind \
  extra_gz_resource_path:=/home/weicheng/ca_lsc_td3/data/evaluation/wind_qualification/wind_4_x/models \
  condition_id:=wind_4_x \
  wind_model:=steady_gazebo_world_wind \
  wind_cmd_e_enu:=4.0 \
  wind_cmd_n_enu:=0.0 \
  wind_cmd_u_enu:=0.0 \
  airspeed_source:=relative_wind \
  output_csv:=/home/weicheng/ca_lsc_td3/data/evaluation/wind_qualification/wind_4_x/telemetry.csv \
  px4_workdir:=/tmp/ca_lsc_td3/wind_4_x_px4
```

## W0--W3

| Test | 条件 | 风 | 验收 |
| --- | --- | --- | --- |
| W0 | 静止/悬停或固定模型 | 0 m/s | baseline；airspeed/气动力无风响应 |
| W1 | 静止/悬停或固定模型 | 4 m/s | 气动响应明显非零 |
| W2 | 静止/悬停或固定模型 | 8 m/s | 气动响应相对 W1 近似按 \(V^2\) 增大 |
| W3 | 前飞 | 4 m/s 顺/逆风 | \(V_a\) 随 \(|\mathbf V_g-\mathbf V_w|\) 变化 |

第一版如果做不到锁定模型，允许先用受控 hover / fixed airspeed hold 做 smoke；
但论文正式 F2 前仍需要给出 wind state、relative airspeed 和 aerodynamic response
三层证据。

## 需要补充到 telemetry 的字段

正式 F2 前，`telemetry.csv` 已从 schema 14 开始增加：

- `condition_id`
- `wind_model_config`
- `wind_cmd_e_enu_mps`, `wind_cmd_n_enu_mps`, `wind_cmd_u_enu_mps`
- `wind_actual_e_enu_mps`, `wind_actual_n_enu_mps`, `wind_actual_u_enu_mps`
- `wind_actual_source`
- `vg_e_enu_mps`, `vg_n_enu_mps`, `vg_u_enu_mps`
- `vrel_e_enu_mps`, `vrel_n_enu_mps`, `vrel_u_enu_mps`
- `vrel_norm_mps`
- `airspeed_relative_error_mps = airspeed_mps - vrel_norm_mps`

当前 recorder 已有 NED local velocity (`vn_mps`, `ve_mps`, `vz_up_mps`) 与
`airspeed_mps`。schema 14 会额外写入 commanded steady wind 和
\(\mathbf V_g-\mathbf V_w\) 诊断。注意 `wind_actual_*` 当前来源为
`configured_steady_world`，还不是 Gazebo wind-state topic 的直接测量；后续
deterministic gust 必须桥接或发布实际 gust state。

## 第一阶段只做 steady wind smoke（已完成）

先确认 Gazebo world wind 和 aircraft link wind mode 生效：

```bash
./scripts/generate_wind_qualification_world.py \
  data/evaluation/wind_qualification/w0_no_wind \
  --wind-enu 0 0 0

./scripts/generate_wind_qualification_world.py \
  data/evaluation/wind_qualification/w1_wind4_x \
  --wind-enu 4 0 0

./scripts/generate_wind_qualification_world.py \
  data/evaluation/wind_qualification/w2_wind8_x \
  --wind-enu 8 0 0
```

生成/复跑后，用统一脚本汇总，不再按文件名猜工况：

```bash
./scripts/analyze_wind_qualification.py \
  data/evaluation/wind_qualification
```

输出：

- `wind_qualification_summary.json`
- `wind_qualification_summary.csv`

旧 schema-13 W0/W1/W2 数据可作为 physical wind smoke；schema-14 WQ-v3
进一步确认 `relative_wind` 控制口径。后续正式 F2 wind runs 必须继续使用
`airspeed_source:=relative_wind`。

## WQ-v3：相对风速控制确认

F2 中真实空速定义为：

\[
V_a \equiv V_{\rm rel}=|\mathbf V_g-\mathbf V_w|.
\]

从 schema 14 起，recorder 可通过：

```bash
airspeed_source:=relative_wind
```

让 `va_hold_target` 的 dwell、pusher PI、overspeed guard、AoA gate 和
`va_error_mps` 统一使用 `selected_airspeed_mps=vrel_norm_mps`。原始 PX4
`airspeed_mps` 仍保留在 CSV 中，便于对照。

WQ-v3 只做三组，不进入正式 F2：

| Case | Wind ENU | 目标 | 预期 |
| --- | --- | --- | --- |
| WQ3-0 | `[0,0,0]` | \(V_{rel,d}=12\) | \(V_g\approx12\) |
| WQ3-tail | `[+4,0,0]` | \(V_{rel,d}=12\) | \(V_g\approx16\) |
| WQ3-head | `[-4,0,0]` | \(V_{rel,d}=12\) | \(V_g\approx8\) |

WQ-v3 已满足该条件；wind-relative airspeed control 已冻结。

通过 steady wind 后，再实现 deterministic 1-cos gust：

\[
V_g(t)=
\frac{A_g}{2}\left[1-\cos\left(\frac{2\pi(t-t_g)}{T_g}\right)\right],
\qquad t_g\le t<t_g+T_g.
\]

F2 第一版推荐在 measurement window 开始后 0.5 s 触发 gust，持续 3--4 s。
这样研究的是已建立 \((V_a,\lambda)\) 工况对风扰动的承受能力，而不是把起飞、
settle、ramp 和 measurement 混在一起。

## 通过条件

Wind qualification 通过时，归档中必须能回答：

1. `condition.json` 中的 commanded wind 是否与实际 wind topic/profile 一致；
2. `airspeed_mps` 是否随 \(|\mathbf V_g-\mathbf V_w|\) 改变，而不是始终等于
   ground speed；
3. 气动响应或 lift proxy 是否随 \(V_{rel}^2\) 有合理趋势；
4. 坐标系是否明确为 ENU，并记录 headwind/tailwind 的符号。

未通过这些检查前，F2 图只能叫 wind-implementation debug，不能写作正式 F2。
