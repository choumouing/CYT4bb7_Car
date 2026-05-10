# CYT4BB7 迈轮车架构重构建议

## 1. 目标

当前车项目已经不是逐飞空工程级别的小例程，继续把业务代码按外设名字平铺在 `project/code` 下，会让控制、估计、通信、驱动互相乱咬。重构目标不是为了目录好看，而是为了后面加 4BB7 串口通信、3 个 2B13 SPI 从板、WiFi 调参、全局惯导和双核任务时，不把主循环写成一锅粥。

目标结构沿用 `E:/CYT4BB7_Air/project/code` 的习惯：

- `HW_Drivers`：硬件外设驱动层，只做设备初始化、采样、输出，不做业务决策。
- `Estimation`：状态估计层，放姿态、里程计、全局惯导、信标检测和传感器融合。
- `Controller`：车体控制层，放 PID 内核、模式切换、主循环调度和运动控制。
- `Menu`：菜单与参数入口，放本地菜单框架、控制参数注册、参数默认值和后续 WiFi 调参共用入口。
- `Protocols`：通信协议层，放 WiFi、遥控接收机解析，以及后续 4BB7 串口、2B13 SPI 主从通信等非硬件驱动的协议逻辑。
- `IPC`：核间通信层，预留 CM7_0 与 CM7_1 的共享数据和事件通知。

## 2. 当前项目盘点

当前业务代码主要在 `project/code`，目录如下：

```text
project/code/
  Attitude/
  Beacon_Detection/
  control/
  encoder/
  imu/
  menu/
  motor/
  odometer/
  s_curve/
  target_follow/
  uwb/
  wifi/
  wireless_control/
```

当前入口主要集中在 `project/user/main_cm7_0.c`：

- 初始化：菜单、电机、编码器、里程计、信标检测、IMU、控制、遥控、WiFi、UWB、目标跟随。
- 1kHz：`IMU_Update_1000HZ()`。
- 40ms：UWB 解析、遥控状态、模式切换、目标跟随、航向保持。
- 20ms：Yaw rate 控制。
- 10ms：编码器、里程计、信标检测、速度环、电机输出、WiFi JustFloat 遥测。
- 空闲区：`wifi_core_Poll()`。

`project/user/main_cm7_1.c` 基本为空，说明当前工程实际是“0核干活，1核空置”。这对早期跑通没问题，但后续通信和估计一多，0核实时控制会被 WiFi、协议解析、文本命令、调参回包拖出抖动。

当前明显问题：

- `main_cm7_0.c` 承担了调度器、业务模式、控制状态机、通信触发、遥测采集等太多职责。
- `control` 目录同时依赖姿态、IMU、编码器、菜单参数和电机驱动，层级方向不清晰。
- `odometer` 既做编码器速度换算，又用姿态和加速度做融合，应该属于估计层，不该跟硬件编码器平级。
- `Beacon_Detection` 使用 IMU、编码器、里程计相关信号，应归到估计/检测层。
- `uwb` 里既有原始协议解析 `ALX_AOA`，又有跟随控制 `uwb_follow`，协议和控制混在一起。
- `wifi` 里既有链路收发，也有 IMU 校准命令，后续 WiFi 参数调车时需要更明确的命令路由边界。
- `Accel_Calibration.c`、`beacon_detection.c`、`odometer.c` 等文件偏大，后续应拆成算法、状态、参数、命令/持久化几个小文件。

## 3. 基于现有代码的目标目录

建议第一阶段只移动当前已经存在的 `.c/.h` 文件，不预留空模块，不创建暂时没人用的壳文件。先把这堆代码按实际职责归类，算法内部耦合后续再拆。

```text
project/code/
  HW_Drivers/
    Motor/
      motor.c
      motor.h
    Encoder/
      encoder_control.c
      encoder_control.h
    IMU/
      ICM42688.c
      ICM42688.h
    UWB/
      ALX_AOA.c
      ALX_AOA.h

  Estimation/
    Attitude/
      Accel_Calibration.c
      Accel_Calibration.h
      IMU_Filtter.c
      IMU_Filtter.h
      IMU_TOP.c
      IMU_TOP.h
      MahonyAhrs.c
      MahonyAhrs.h
    Position/
      odometer.c
      odometer.h
    Beacon_Detection/
      beacon_detection.c
      beacon_detection.h

  Controller/
    PID/
      pid.c
      pid.h
    Control/
      control.c
      control.h
    Planner/
      s_curve_planner.c
      s_curve_planner.h
    Modes/
      target_follow.c
      target_follow.h
      target_follow_config.c
      target_follow_config.h
      uwb_follow.c
      uwb_follow.h

  Menu/
    menu_config.c
    menu_config.h
    menu_core.c
    menu_core.h

  Protocols/
    Wifi/
      wifi_core.c
      wifi_core.h
      wifi_cmd/
      wifi_justfloat/
      wifi_cal_imu/
    WirelessControl/
      wireless_control.c
      wireless_control.h
```

说明：

- `HW_Drivers` 只放真实硬件驱动。信标灯检测当前是算法检测，不是驱动，所以归到 `Estimation/Beacon_Detection`。
- `Controller` 是车体控制总框架。这个名字更准确，别让车项目顶着飞控壳子到处乱跑。
- `Menu` 和 `Controller` 平级。当前菜单不只是 UI，它还承载控制参数注册、默认值和 Flash 存档，后续 WiFi 调参也应该接到这个参数入口上。
- `Protocols/WirelessControl` 放遥控接收机解析，因为它本质是输入协议；真正模式决策放 `Controller/Modes`。
- `HW_Drivers/UWB/ALX_AOA` 本质是 UWB 串口通信驱动和 AOA 数据帧解析；`uwb_follow` 当前输出跟随速度目标，归到 `Controller/Modes`。
- `menu_core` 当前是本地屏幕、按键、Flash 存档、参数编辑菜单框架；`menu_config` 当前注册轮速环、Yaw、UWB 跟随等控制参数。两者不要拆太散，统一归到顶层 `Menu`。
- `wifi_params`、4BB7 串口、2B13 SPI、IPC 文件当前不存在，第一阶段不在目标树里预留文件。后续真正写代码时再按职责放入 `Protocols` 或 `IPC`。

## 4. 现有文件迁移映射

建议迁移映射如下：

| 当前路径 | 目标路径 | 说明 |
| --- | --- | --- |
| `project/code/motor/*` | `project/code/HW_Drivers/Motor/*` | 有刷电机 PWM 与方向控制 |
| `project/code/encoder/*` | `project/code/HW_Drivers/Encoder/*` | 编码器采样与滤波 |
| `project/code/imu/*` | `project/code/HW_Drivers/IMU/*` | ICM42688 SPI 驱动 |
| `project/code/uwb/ALX_AOA.*` | `project/code/HW_Drivers/UWB/*` | UWB 串口通信驱动与 AOA 数据帧解析 |
| `project/code/Attitude/*` | `project/code/Estimation/Attitude/*` | 姿态估计与 IMU 校准 |
| `project/code/odometer/*` | `project/code/Estimation/Position/*` | 里程计与惯导融合入口 |
| `project/code/Beacon_Detection/*` | `project/code/Estimation/Beacon_Detection/*` | 信标灯/碰撞/异常算法检测，不是硬件驱动 |
| `project/code/control/pid.*` | `project/code/Controller/PID/*` | PID 内核 |
| `project/code/control/control.*` | `project/code/Controller/Control/*` | 速度环、航向环、麦轮运动控制 |
| `project/code/s_curve/*` | `project/code/Controller/Planner/*` | Jerk 受限 S 曲线速度规划 |
| `project/code/target_follow/*` | `project/code/Controller/Modes/*` | 目标点跟随模式 |
| `project/code/wifi/*` | `project/code/Protocols/Wifi/*` | WiFi 链路、命令、遥测 |
| `project/code/uwb/uwb_follow.*` | `project/code/Controller/Modes/*` | UWB 跟随控制，输出速度目标 |
| `project/code/wireless_control/*` | `project/code/Protocols/WirelessControl/*` | 遥控接收机协议解析 |
| `project/code/menu/menu_config.*` | `project/code/Menu/*` | 控制参数注册和默认值 |
| `project/code/menu/menu_core.*` | `project/code/Menu/*` | 本地屏幕、按键、Flash 存档、参数编辑菜单框架 |

## 5. 依赖方向

重构后依赖方向必须单向：

```text
HW_Drivers
  -> Estimation
  -> Controller
  -> Protocols telemetry output

Menu
  -> Controller parameter interface

Protocols input
  -> Controller command/state request
  -> Menu parameter request

IPC
  <-> CM7_0 / CM7_1 boundary only
```

更具体一点：

- 驱动层不能 include 控制层、估计层、通信层。
- 估计层可以读取驱动层数据，但不能直接写电机。
- 控制层可以读取估计结果和协议输入，但电机输出统一走 `HW_Drivers/Motor`。
- 菜单层负责参数注册、编辑、保存和查询，控制层只通过参数接口读取参数，不要让菜单直接钻进 PID 内部状态乱改。
- 协议层不要直接修改控制内部 PID 状态；调参要走参数接口，模式切换要走命令接口。
- IPC 只传结构化快照和事件，不传一堆散乱全局变量地址。

## 6. CM7_0 与 CM7_1 任务划分

### 6.1 CM7_0：实时控制主核

CM7_0 必须保持实时性，建议保留这些任务：

- 系统时钟、基础硬件初始化。
- 1kHz IMU 采样、滤波、姿态解算。
- 100Hz 编码器更新、里程计、车体速度估计。
- 50Hz/25Hz 模式状态机、目标选择、遥控输入快照。
- 50Hz 或 100Hz 速度环、航向环、麦轮解算。
- 电机 PWM 输出和紧急停机。
- 必要的低开销遥测快照生成。

CM7_0 不建议承担：

- 大段 WiFi 文本命令解析。
- 大量字符串格式化回包。
- 上位机复杂协议路由。
- 信标灯图像/复杂检测预处理。
- 未来视觉、地图、路径搜索这类耗时任务。

### 6.2 CM7_1：非强实时协处理核

CM7_1 适合放这些任务：

- WiFi 文本命令解析、参数查询、参数保存请求。
- JustFloat/调试遥测打包、降频发送。
- 4BB7 串口协议收发、复杂帧解析。
- 3 个 2B13 SPI 从板数据汇总、离线检测、错误统计。
- 信标灯检测预处理或低频融合辅助。
- 后续视觉、目标列表维护、路径点管理。

CM7_1 禁止直接做：

- 直接写电机 PWM。
- 直接清 PID 积分。
- 直接修改 CM7_0 的控制模式内部状态。

CM7_1 应该通过 IPC 发请求，例如：

```text
CM7_1 -> CM7_0:
  set_mode_request
  set_target_velocity
  set_target_point
  set_param_request
  telemetry_enable_request

CM7_0 -> CM7_1:
  car_state_snapshot
  estimator_snapshot
  control_snapshot
  fault_snapshot
  param_snapshot
```

## 7. 建议调度框架

建议把 `main_cm7_0.c` 瘦身成三件事：

```c
int main(void)
{
    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
    }
}
```

`car_loop_poll()` 内部再按频率分发：

```text
1kHz:
  Attitude_Update1000Hz()
  IMUCalib_Update1000Hz()

100Hz:
  Encoder_Update()
  Position_Update()
  Control_SpeedLoopUpdate()

50Hz:
  ProtocolInput_UpdateSnapshot()
  Mode_Update()
  TargetFollow_Update()
  Telemetry_BuildSnapshot()

25Hz:
  UWB_Update()
  BeaconDetection_Update()
  Fault_Update()

idle:
  Protocols_Poll()
  IPC_Poll()
```

这样主循环只看得见“任务调度”，看不见一堆控制细节。后续要查 bug，先看任务频率，再进模块，别在 `main` 里翻半天。

## 8. 参数与调参建议

当前 `menu_config` 和控制参数耦合偏重，但它和 `menu_core` 都围绕同一个菜单/参数体系工作，第一阶段不要再拆成 `Params`、`UI_Menu` 两层。建议统一放到顶层 `Menu`：

```text
Menu/
  menu_config.c
  menu_config.h
  menu_core.c
  menu_core.h
```

参数分组：

- `wheel_pid`：四轮速度环共用或独立参数。
- `yaw_pid`：角度环、角速度环参数。
- `mecanum`：轮距、轮径、编码器每米计数、轮向符号。
- `estimation`：里程计融合、加速度偏置、粗糙路面判断。
- `uwb_follow`：目标跟随 PID、限速、死区。
- `telemetry`：JustFloat 通道、发送频率、开关。

当前第一步先保留 `menu_config.*` 和 `menu_core.*` 的原始行为，只搬到顶层 `Menu`。后续如果要做 WiFi 参数表，再从 `Menu` 里抽出真正的参数中心，别现在就造一个不存在的新参数模块壳子。

## 9. IPC 数据结构建议

第一版 IPC 不要上复杂框架，先用共享内存 + 序号 + CRC/版本号：

```c
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t seq;
    uint32_t timestamp_ms;
    uint32_t checksum;
} ipc_header_t;
```

每类数据一个 payload：

- `ipc_car_state_t`：模式、使能、故障、时间戳。
- `ipc_estimator_state_t`：姿态、速度、位置、UWB 坐标。
- `ipc_control_state_t`：目标速度、实际速度、电机输出、PID 分项。
- `ipc_command_t`：调参、模式切换、目标点设置。

关键点：

- CM7_0 发布实时状态快照。
- CM7_1 发布请求，不直接修改实时状态。
- 每个 payload 都带 `seq`，读端发现序号变化才拷贝。
- 共享数据尽量定长，不用动态内存。
- 双核访问先关闭 DCache 或明确 cache clean/invalidate 策略。

## 10. 迁移步骤

### 阶段 0：冻结基线

- 保留当前可运行版本。
- 记录当前入口、周期、外设引脚、通信口、遥测通道。
- 不改行为，只写文档和迁移表。

### 阶段 1：只搬目录，不改逻辑

- 只新建当前承载已有文件的目录，先不要建空壳模块。
- 按映射移动文件。
- 更新 include 路径和 IAR 工程文件。
- 编译通过后再进入下一步。

这一步最容易被路径折腾，艹，IAR 对路径改动很敏感，建议一次只搬一组目录。

### 阶段 2：瘦身 `main_cm7_0.c`

- 后续新增 `Controller/Scheduler` 调度模块。
- 把 1kHz、10ms、20ms、40ms 逻辑拆成命名清晰的任务函数。
- `main_cm7_0.c` 只保留初始化和 `car_loop_poll()`。

### 阶段 3：拆协议和控制耦合

- `ALX_AOA` 移到 `HW_Drivers/UWB`，作为 UWB 串口通信驱动和 AOA 数据帧解析。
- `uwb_follow` 移到 `Controller/Modes`。
- `wireless_control` 只输出遥控快照。
- 模式切换统一由 `car_mode` 处理。

### 阶段 4：整理菜单与参数入口

- 第一阶段把 `menu_config.*` 和 `menu_core.*` 一起移到顶层 `Menu`，保持控制参数注册、屏幕、按键、Flash 存档逻辑不变。
- 后续再抽象统一参数中心，WiFi 参数表到那时再接入。

### 阶段 5：启用 CM7_1

- 先建立 IPC 空框架。
- CM7_0 发布状态快照。
- CM7_1 只读取并通过 WiFi 发遥测。
- 稳定后再把 WiFi 文本命令、4BB7 串口、2B13 SPI 放到 CM7_1。

## 11. 编码规范建议

- 模块对外只暴露 `Init + Update/Poll + GetState + Reset`，别到处 `extern` 一堆全局变量。
- 全局状态统一 `g_module_state`，模块内部状态统一 `s_module_xxx`。
- 参数、状态、命令结构体分开，别拿一个结构体啥都装。
- ISR 里只收字节、置标志、清中断，不做复杂解析。
- 控制闭环里禁止字符串格式化、Flash 写入、阻塞等待。
- 新增模块先写 `.h` 的接口，再写 `.c` 实现。
- 头文件组织参考 `E:/CYT4BB7_Air/libraries/zf_common/zf_common_headfile.h`：自定义库函数头文件统一收口到公共总头文件；各 `.c` 文件只 include 自己的 `.h`，其他模块头文件再 include 总头文件，避免每个 `.c` 前面堆七八行杂乱相对路径。
- 注释语言保持中文为主，底层库版权头保持原样。

## 12. 风险与注意事项

- IAR 工程文件引用的是显式文件路径，搬目录必须同步更新 `.ewp`。
- 当前 include 有大量相对路径，迁移时建议统一改成从 `project/code` 根开始的包含路径。
- 双核启用后要处理 DCache 一致性，否则共享内存数据会出现玄学错误。
- WiFi SPI 和控制闭环抢 CPU 时，要优先保证控制闭环。
- 参数 Flash 版本要递增，避免旧参数结构误读。
- 电机、编码器、轮序、正反转必须单独记录，迁移后第一件事就是低速架空测试。

## 13. 推荐最终边界

最终期望是：

```text
CM7_0:
  实时采样 -> 状态估计 -> 控制模式 -> 电机输出 -> 状态快照

CM7_1:
  通信解析 -> 调参/遥测/外部协处理 -> IPC 请求

共享边界:
  IPC 快照 + IPC 命令请求
```

一句话：0核负责车别翻，1核负责话别断。控制实时性必须永远排第一，通信、调参、遥测都只能围着它转。
