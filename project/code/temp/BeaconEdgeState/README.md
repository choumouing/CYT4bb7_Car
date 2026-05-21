# BeaconEdgeState 算法说明

本目录是新的信标凸起边缘检测实验模块，未直接替换现有 `Estimation/Beacon_Detection`。

核心思路：

1. 100Hz 用四轮编码器估算车体系前进/横移速度，并计算轮速高通扰动。
2. 1000Hz 用 IMU 计算三个边缘特征：速度方向投影后的 roll/pitch 角速度、加速度模长误差、水平加速度。
3. 从 41ms 窗口内取峰值，得到边缘冲击分数。
4. 把 300ms 内的有效边缘峰归为一次凸起通过事件。
5. 用姿态弧、加速度方向、通过距离和轮速高通抑制急加减速、原地晃动和随机场地移动误检。

离线验证：

```powershell
python "CYT4bb7_Car/project/code/temp/BeaconEdgeState/offline_beacon_edge_eval.py"
```

嵌入式接入建议：

- 1kHz 调 `beacon_edge_state_update_from_project_1000HZ()`。
- 100Hz 调 `beacon_edge_state_update_from_project_100HZ()`。
- 读取 `g_beacon_edge_state.enter_event` 和 `g_beacon_edge_state.exit_event` 判断上/下信标。
- `enter_tick_ms` 和 `exit_tick_ms` 是真实边缘峰时刻，事件输出会有确认延迟。
