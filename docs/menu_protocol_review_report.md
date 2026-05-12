# 菜单系统、参数配置与机间通信协议审查报告

> 审查范围：菜单与参数系统、控制环参数配置、车-机通信协议
> 审查方法：3 个独立 Explore Agent 初审 + 2 个交叉验证 Agent 逐条确认
> 所有列出的问题均已通过阅读源代码交叉验证，标注 VERIFIED

---

## 一、总体结论

| 维度 | 评价 |
|------|------|
| 控制环参数可调性 | 4 个环共 24 个参数可通过菜单修改并即时生效 |
| 传感器数据显示 | **完全缺失** — 无法在屏幕上查看任何传感器数据 |
| 参数持久化 | 本地参数无校验，Air 参数有校验，**不一致** |
| 机间通信 | 可工作，但存在**阻塞主循环**和**同步失败无回滚**等严重设计缺陷 |
| MODE_2 PID 可调性 | **硬编码 #define**，无法在菜单中调整 |
| 代码质量 | 存在死代码、显示坐标反转、按键双触发等问题 |

---

## 二、菜单与参数系统问题

### BUG-1: 本地 Flash 保存无 Magic Number 和校验和 **[HIGH]**

- **文件**: `Menu/menu_core.c` 保存函数 (line 891-929) 和加载函数 (line 827-886)
- **问题**: 保存时直接写入 24 个 float 原始值，无 magic number、无版本号、无 CRC/校验和。加载时仅检查 NaN 和 ±1000000 范围。若 Flash 含旧应用残留数据，可能加载垃圾值直接送入 PID 控制器，导致车辆失控。
- **对比**: Air 参数系统有完整的 magic number (0x41495250 "AIRP")、版本号和 CRC 校验，两套系统严重不一致。
- **状态**: VERIFIED

### BUG-2: 无法在屏幕上查看传感器数据 **[HIGH]**

- **文件**: 整个 `Menu/` 目录
- **问题**: 菜单系统纯粹是参数编辑器，没有任何页面显示 IMU 姿态角、陀螺仪数据、编码器速度、UWB 位置、PID 误差/输出等实时信息。调 PID 如同盲调。
- **状态**: VERIFIED（grep `g_euler`, `g_odometer`, `encoder`, `IMU` 在 Menu 目录无匹配）

### BUG-3: MODE_2 目标跟随 PID 硬编码，不可菜单调整 **[MEDIUM]**

- **文件**: `Controller/car_mode.h` (line 35-38), `Controller/car_mode2.c` (line 22-35)
- **问题**: `TARGET_FOLLOW_POS_KP=120`, `TARGET_FOLLOW_POS_KI=0`, `TARGET_FOLLOW_POS_KD=30` 是编译期 `#define` 常量，直接传入 `PositionalPID_Init()`。菜单中无法调整，必须重新编译。
- **对比**: MODE_1 的 UWB PID 有完整的 10 个菜单参数可调。
- **状态**: VERIFIED

### BUG-4: 显示居中计算宽高反转 **[MEDIUM]**

- **文件**: `Menu/menu_core.c` lines 1100-1101, 1120-1121, 1142-1143
- **问题**: IPS114 屏幕为 240×135 (宽×高)。代码中：
  ```c
  uint16_t x = (135 - msg_len * 8) / 2;  // 用高度算水平居中，错误
  uint16_t y = (240 - 16) / 2;           // 用宽度算垂直居中，错误
  ```
  正确应为 `x = (240 - msg_len * 8) / 2` 和 `y = (135 - 16) / 2`。
  影响 `menu_show_success()`、`menu_show_error()`、`menu_show_progress()` 三个函数。
- **状态**: VERIFIED

### BUG-5: 本地 PID 参数可在车辆运行时修改，无安全保护 **[MEDIUM]**

- **文件**: `Menu/menu_core.c` lines 700-712
- **问题**: `MENU_TYPE_AIR_PARAMETER` 有 `menu_can_edit_air_params()` 保护（需要车辆停止），但 `MENU_TYPE_PARAMETER`（所有 24 个本地 PID 参数）无任何运行状态检查。误触将 wheel_kp 从 3.6 改为 36.0 会导致电机输出暴增。
- **状态**: VERIFIED

### BUG-6: 按键长按首次触发双发 **[LOW]**

- **文件**: `Menu/menu_core.c` lines 529-591
- **问题**: KEY_1/KEY_2 按下时，`key_get_state()` 检测到 KEY_SHORT_PRESS 触发一次 `menu_key_handler()`，随后长按循环中 `key_hold_ticks[i]==1` 又触发一次。同一按键事件在首 tick 处理两次。
- **状态**: VERIFIED

### BUG-7: 可见性检查硬编码 15 而非常量 8 **[LOW]**

- **文件**: `Menu/menu_core.c` line 1213
- **问题**: `if(item_index >= display_offset + 15)` 应为 `display_offset + MENU_MAX_VISIBLE_LINES`（值为 8）。允许渲染屏幕外项目。
- **状态**: VERIFIED

### BUG-8: `menu_show_debug_info()` 为死代码 **[LOW]**

- **文件**: `Menu/menu_core.c` line 301
- **问题**: 声明并定义但从未被任何地方调用。
- **状态**: VERIFIED

### BUG-9: S-curve 规划器为整体死代码 **[LOW]**

- **文件**: `Controller/Planner/s_curve_planner.c`
- **问题**: `s_curve_planner_init()` 和 `s_curve_planner_step()` 在整个项目中无调用者。关联的 3 个 `s_curve_*` 参数在 `menu_config.c` 中声明但从未注册到菜单。
- **状态**: VERIFIED

### BUG-10: `car_mode0_reset()` 为空函数 **[LOW]**

- **文件**: `Controller/car_mode0.c` line 9-11
- **问题**: 模式切换时被调用但什么也不做。与 mode1/mode2 的有意义 reset 不一致。
- **状态**: VERIFIED

### BUG-11: 按下编辑后无法撤销 **[DESIGN]**

- **文件**: `Menu/menu_core.c` lines 770-779
- **问题**: KEY_ENTER 和 KEY_BACK 都会保存当前值并退出编辑。无"取消修改/恢复原值"功能。
- **状态**: VERIFIED

### BUG-12: Air 参数与本地参数分开保存 **[DESIGN]**

- **问题**: 保存 "Slot0" 只保存 24 个本地参数，4 个 Air 参数有独立的保存/加载系统。用户可能误以为一次保存包含所有配置。
- **状态**: VERIFIED

---

## 三、机间通信协议问题

### BUG-13: `air_comm_car_set_param()` 阻塞主循环 **[CRITICAL]**

- **文件**: `Protocols/AirComm/air_comm_car.c` lines 691-703
- **问题**: 同步忙等循环，最长阻塞 850ms。在此期间：
  - 控制环停止更新
  - 摄像头 SPI 轮询被阻塞
  - 用户界面冻结
- 当前 4 个 Air 参数全量同步最坏阻塞 3.4 秒 (4 × 850ms)。
- **状态**: VERIFIED

### BUG-14: 同步失败无回滚、无详情 **[HIGH]**

- **文件**: `Menu/menu_air_support.c` lines 248-251
- **问题**: `menu_sync_all_air_params()` 遇到任一参数同步失败后，仅显示 "Sync Failed"，不告知哪个参数失败。部分参数已下发到飞机，部分未下发，无回滚机制，用户无法知道飞机上哪些参数是新的、哪些是旧的。
- **状态**: VERIFIED

### BUG-15: 无自动同步（飞机重连时） **[MEDIUM]**

- **文件**: `Menu/menu_air_support.c`
- **问题**: 飞机掉线重连后，不会自动重新下发参数。飞机可能使用旧参数（如重启后 Flash 中的旧值），但车端无检测、无提示。
- **状态**: VERIFIED

### BUG-16: 单个 Air 参数编辑阻塞主循环 **[MEDIUM]**

- **文件**: `Menu/menu_air_support.c` line 192 → `air_comm_car_set_param()`
- **问题**: 用户旋钮调节 Air 参数时，每次转动触发 850ms 阻塞发送+等待 ACK，导致 UI 卡顿。
- **状态**: VERIFIED

### BUG-17: Float 序列化字节序不一致 **[LOW]**

- **文件**: `Protocols/AirComm/air_comm_car.c` lines 100-128
- **问题**: `air_comm_write_float()` 使用 `memcpy` 按平台字节序序列化，`air_comm_write_u32()` 显式使用小端序。当前平台 Cortex-M7 为小端序所以碰巧一致，但如果飞机端平台不同，float 解析会出错。
- **状态**: VERIFIED

### BUG-18: ACK 重试增加 tx_frame_count 统计失真 **[LOW]**

- **文件**: `Protocols/AirComm/air_comm_car.c` line 545
- **问题**: 重传帧也计入 `tx_frame_count`，使诊断统计包含重传，无法区分"成功发送帧数"和"总发送字节数"。
- **状态**: VERIFIED

---

## 四、控制环参数配置问题

### BUG-19: 修改运行中 PID 参数时积分项不复位 **[MEDIUM]**

- **文件**: `Controller/control.c` 中的 `*_apply_params()` 函数
- **问题**: `*_apply_params()` 只更新 kp/ki/kd/i_limit/output_limit，不复位 `integral` 或 `output`。若 ki 从 0 改为较大值，之前累积的积分项立即产生大输出，导致输出突变。
- **状态**: VERIFIED

### BUG-20: PositionalPID 二次 P 项对负误差不对称 **[LOW]**

- **文件**: `Controller/pid.c` lines 36-43
- **问题**: 正误差 `p = kp_2*err² + kp_1*err`，负误差 `p = -kp_2*err² + kp_1*err`。二次项符号反转导致正负方向响应曲线不同。当前所有使用处 kp_2=0 所以处于休眠状态，但一旦启用就会出问题。
- **状态**: VERIFIED

### BUG-21: `IncreamPID_Update` 函数名拼写错误 **[LOW]**

- **文件**: `Controller/pid.h` line 47
- **问题**: 应为 `IncrementPID_Update`，写成了 `IncreamPID_Update`。
- **状态**: VERIFIED

### BUG-22: CAR_MODE_1 分发到 car_mode2 而非 car_mode1 **[MEDIUM]**

- **文件**: `Controller/car_mode.c` line 88-89
- **问题**: `case CAR_MODE_1: car_mode2_update_25HZ(now_ms);` — MODE_1 实际运行 MODE_2 的逻辑。MODE_2 内部再调用 `car_mode1_update_25HZ()` 作为子程序。用户无法独立运行纯 UWB 跟随（不含目标点导航）。
- **状态**: VERIFIED

---

## 五、问题严重度汇总

| 严重度 | 数量 | 编号 |
|--------|------|------|
| CRITICAL | 1 | BUG-13 |
| HIGH | 3 | BUG-1, BUG-2, BUG-14 |
| MEDIUM | 8 | BUG-3, BUG-4, BUG-5, BUG-15, BUG-16, BUG-19, BUG-22 |
| LOW | 7 | BUG-6, BUG-7, BUG-8, BUG-9, BUG-10, BUG-17, BUG-18, BUG-20, BUG-21 |
| DESIGN | 2 | BUG-11, BUG-12 |

---

## 六、回答用户原始问题

### Q1: 是否可以通过菜单查看传感器的数据？
**否。** 菜单系统没有任何传感器数据实时显示功能。无法在屏幕上看到 IMU 姿态、编码器速度、UWB 位置等。

### Q2: 是否可以修改控制环的参数？
**部分可以。** 4 个级联 PID 环（轮速、偏航角速率、偏航角、UWB 位置）共 24 个参数可通过菜单修改并即时生效。但 MODE_2 的目标跟随 PID（5 个参数）是硬编码的 `#define`，不可菜单调整。

### Q3: 是否可以保存控制环的参数？
**可以，但有风险。** 通过 "Save Slot" 保存到 Flash，但本地参数无 magic number 和校验和，存在加载垃圾数据的风险。Air 参数有完整校验。

### Q4: 是否可以和飞机进行通信？
**可以。** 通过 UART_3 (1152000 baud) 与飞机通信，支持参数下发、函数调用、心跳保活和遥测接收。

### Q5: 协议设计是否存在 bug 缺陷？
**存在。** 最严重的两个：(1) 参数下发阻塞主循环最长 850ms/次；(2) 同步失败无回滚、无具体错误报告。

### Q6: 修改的参数是否可以同步到飞机？
**可以，但不可靠。** 手动触发 "Sync Air" 可将 4 个 Air 参数同步到飞机，但无自动同步、无部分失败处理、无脏标记提醒。
