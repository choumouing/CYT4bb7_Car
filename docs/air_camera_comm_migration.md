# CYT4BB7 Air/Camera 通信移植说明

## 模块落位

- `project/code/HW_Drivers/CameraSpi/`：封装三路 CYT2BL3 从机 SPI 硬件访问，小车 CYT4BB7 为 SPI 主机。
- `project/code/Protocols/CameraSpi/`：负责三从机轮询、帧解析、CRC、上下行 payload、在线状态和目标快照。
- `project/code/Protocols/AirComm/`：负责小车主控与飞机端主控的串口协议，保持旧帧格式 `AA AA 55 55 | Type | Seq | Len | Payload | CRC_L | CRC_H`。
- `project/code/Menu/menu_air_support.*`：保存 Air 参数镜像、Air Flash 槽、远程调参与批量同步。
- `project/code/Estimation/Image/`：通过 `beacon_fusion_update_100HZ()` 对外提供摄像头信标融合结果，协议层不直接改控制目标。

## 调度

- `car_loop_init()` 初始化菜单、CameraSpi、AirComm。
- `100HZ` 调用 `camera_spi_update_100HZ()`、`air_comm_car_update_100HZ()`、`beacon_detection_update_100HZ()`、`menu_update_100HZ()`。
- 主循环尾部调用 `camera_spi_poll()` 和 `air_comm_car_poll()`，协议解析不阻塞控制节拍。
- `PIT_CH0` 1ms 中断只做 AirComm tick 和 100Hz 标志，100Hz 分支调用菜单按键扫描。
- `UART_3` ISR 只读取字节并喂给 AirComm RX 队列。
- `P02_4/P01_1/P19_1` EXTI ISR 只通知三块 CYT2BL3 ready。

## Flash 地图

- Car 菜单槽：page `72-79`，每槽 2 页。
- Air 菜单槽：page `80-87`，每槽 2 页。
- 旧 Car 菜单槽：page `88-95` 只做兼容读取兜底，不再写回，避免继续碰 IMU 校准 page `95`。

## 安全限制

- Air 参数编辑和批量同步只允许在 Air 在线且车端未使能，或急停状态下执行。
- AirComm ACK 仍是单 pending，菜单层操作必须串行等待 ACK。
- `air_comm_car_exec_func()` 会等待飞机 ACK；Go 类操作后续接入时必须先确认 ACK 成功，再切车端运动模式。
- CameraSpi 采用异步传输和 poll 超时退出，不启用旧库的 `PIT_CH20`，不在 ISR 解析帧。

## 硬件默认值

- Camera SPI：`SPI_0/SCB7`，CLK `P02_2`，MOSI `P02_1`，MISO `P02_0`。
- Camera CS：`P02_3/P01_0/P19_0`。
- Camera INT：`P02_4/P01_1/P19_1`。
- AirComm UART：`UART_3`，TX `P17_2`，RX `P17_1`，波特率 `1152000`。
