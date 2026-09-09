# 迁移清单

新工程根目录：`/home/weicheng/ca_lsc_td3`

源工程 `/home/weicheng/vtol_sim` 保持原位。本次迁移没有清理、覆盖或提交源工程中的任何文件；源工程本来就存在大量未提交和未跟踪内容，因此不把它当作可直接复制的干净模板。

## 已复制并启用

| 新位置 | 来源 | 用途 |
|---|---|---|
| `environment/` | `vtol_sim/environment/` | 继续采用 `vtol_nav` 的 Python 3.10、NumPy 1.x、Torch、Gymnasium 等直接版本；删除了名为 lock、实为不完整 direct pins 的冗余文件 |
| `config/dependencies.repos` | `vtol_sim/config/dependencies.repos` | 记录 PX4、Micro-XRCE-DDS 与 px4_msgs 的精确提交；迁移时删除了导航专用 XTDrone2 和非必需 CleanRL checkout，并把原来的短 SHA/移动分支解析为完整提交 |
| `third_party/cfc/torch_cfc.py` | `vtol_sim/CfC/torch_cfc.py` | 上游 CfC PyTorch 参考实现 |
| `third_party/cfc/{README.md,LICENSE}` | `vtol_sim/CfC/` | 来源说明与 Apache-2.0 许可保留 |
| `src/ca_lsc_td3/_vendor/` | 上述 CfC 核心与许可证的包内副本 | 让 editable/wheel 安装后的代码不依赖临时 `PYTHONPATH`；上游文件保持不修改 |
| `docs/source_material/*.txt` | 本轮两份附件 | 保存完整原始思路与实验步骤，便于追溯 |

CfC 来源：`https://github.com/raminmh/CfC.git`，源仓库当时提交为 `3ebaa0307a280c5a9ee8261fdf733ad9bd5636ad`。

## 已复制但只作参考

| 新位置 | 原因 |
|---|---|
| `reference/legacy_ros2/vtol_px4_control/` | 其中的 PX4 命令节点和状态监视器可帮助复用起飞、MC/FW 转换及状态订阅逻辑，但接口仍是导航任务接口 |
| `reference/legacy_ros2/single_vehicle.launch.py` | 可参考 Gazebo、PX4、DDS 的进程编排和固定翼转换参数；包含地图、目标点、导航 episode 等旧逻辑，不参与新项目构建 |
| `reference/legacy_scripts/` | 仅用于追溯旧启动和转换校准流程；路径和 Python 模块仍指向旧工程 |

这些参考文件必须先去除导航耦合并重新设计 \(\lambda\) 执行接口，不能直接作为新论文实验代码。

## 明确排除

- `data/evaluation/contract13_*.json`、旧 checkpoint、训练日志和速度测试；
- 深度相机、障碍物扇区、程序化地图、A*、碰撞代理和导航 reward；
- `build/`、`install/`、`log/`、rosbag、ULog 及其他生成物；
- 旧导航 Actor/observation/action head 和普通单步 replay buffer；
- 93 GB 的本地 PX4 构建树、CleanRL 完整仓库等大体积第三方 checkout。

第三方仓库通过 `config/dependencies.repos` 固定版本。开发初期可以显式设置外部路径复用本机已有 checkout；最终可复现实验应重新拉取干净版本，而不是依赖旧仓库的脏工作树。
