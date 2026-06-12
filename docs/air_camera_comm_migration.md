# CYT4BB7 Air/Camera 通信移植说明

## 当前职责

- `CYT4BB7_Air/project/code/Protocols/CameraSpi/`：Air CM7_1 作为 SPI 主机，轮询两块 CYT2BL3 图像板。
- `CYT4BB7_Air/project/code/Estimation/Image/`：Air CM7_1 汇总前摄、中摄、后摄图像结果并运行旧 Mode2 三摄融合逻辑。
- `CYT4BB7_Air/project/code/IPC/ipc_image_data.*`：Air CM7_1 将融合结果通过共享内存同步给 CM7_0。
- `CYT4BB7_Air/project/code/Protocols/AirComm/`：Air CM7_0 将融合后的 Mode2 数据扩展到 RUN_DATA 后发送给 CAR。
- `CYT4bb7_Car/project/code/Controller/car_mode2.c`：CAR 不再访问旧 CameraSpi 或旧 beacon_fusion，只消费 AirComm 下发的 Mode2 字段并执行旧 Mode2 控制逻辑。

## 摄像头映射

- SPI board 0：前摄 CYT2BL3。
- Air 本地图像：中摄。
- SPI board 1：后摄 CYT2BL3。

CYT2BL3 仍只输出本板 `4 beacon + 1 car_lamp` 图像结果，不做跨板融合。SPI beacon 第三个 `float` 为面积 `area`，本地保留的 `radius` 不进入板间协议。

## RUN_DATA 扩展

Air -> CAR 的 RUN_DATA 当前为 22 个 `float`：

| 索引 | 字段 |
| --- | --- |
| 0 | `tof_fused_height_mm` |
| 1 | `euler_roll` |
| 2 | `euler_pitch` |
| 3 | `euler_yaw` |
| 4 | `pos_est_vel_x` |
| 5 | `pos_est_vel_y` |
| 6 | `CRSF_LINK_UP` |
| 7..14 | `CRSF_STD[0..7]` |
| 15 | `mode2_target_valid` |
| 16 | `mode2_target_x` |
| 17 | `mode2_target_y` |
| 18 | `mode2_car_lamp_valid` |
| 19 | `mode2_car_lamp_cx` |
| 20 | `mode2_car_lamp_cy` |
| 21 | `mode2_lamp_angle_deg` |

CAR 端使用 `mode2_car_lamp_cx/cy` 和 `mode2_lamp_angle_deg` 计算旧版车灯参考点，再由 `mode2_target_x/y` 计算 raw delta 和旋转后的控制 delta。

## 调度

- Air CM7_1：10 ms 调用 `air_image_fusion_update_100HZ()`，内部完成 SPI 轮询、本地下视图像更新、三摄融合和 IPC 发布。
- Air CM7_0：100 Hz 调用 `air_comm_air_update_100HZ()` 并发送 22-float RUN_DATA。
- CAR：AirComm RUN_DATA 回调解析 Mode2 字段，`CAR_MODE_2` 在 100 Hz 调用 `car_mode2_update_100HZ()`。

## 保留约束

- CAR 不恢复与 CYT2BL3 的 SPI 通信，不保留第三块 CYT2BL3 链路。
- 旧 Mode2 控制参数已恢复为本地菜单参数，可通过 `Mode2 Img` 菜单调整。
- AirComm RUN_DATA 最大容量为 32 个 `float`，当前 22 个字段在协议容量内。
