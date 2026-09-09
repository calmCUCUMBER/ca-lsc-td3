# Legacy reference only

本目录中的文件来自旧导航避障工程，只用于提取已经跑通过的 PX4/Gazebo/DDS 启动与 VTOL 模式状态机思路。

请不要从这里直接启动新论文实验：

- 脚本仍指向旧模块和旧目录；
- launch 文件仍包含地图、目标点和 navigation episode 参数；
- `vtol_command_node.cpp` 只提供速度命令和离散 MC/FW mode intent，不提供连续 \(\lambda\) 卸载接口；
- 现有遥测没有 \(C_L\)、\(C_{m_{\delta_e}}\)、shadow-controller demand、各推进功率或 actuator-level unloading 数据。

正式实现应在新包中按接口重新设计，同时用这里的文件核对 PX4 topic、QoS、进程关闭和转换状态处理。

