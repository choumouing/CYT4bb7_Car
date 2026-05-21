# 车端信标坐标约束修正惯导漂移长期任务

你现在要在 CYT4BB7 麦克纳姆轮小车工程里，搭建“信标检测结果 + 已知信标全局坐标 + 当前惯导坐标”的位置修正基础框架。请始终使用简体中文回复，先读现有代码再动手，先确认接口和调度顺序再新增模块。不要一上来搞复杂融合，当前目标是简单、稳定、能开关、能上车验证的 v1 框架。

## 项目背景

当前项目路径：

```text
D:\HDUASC-SmartCar-21st-FlyOverMinefield
```

已有两个可用基础模块：

```text
CYT4bb7_Car\project\code\Estimation\Position\odometer.h
CYT4bb7_Car\project\code\Estimation\Position\odometer.c

CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.h
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.c
```

`odometer` 已经实现状态可用的惯性/编码器里程计，外部主要直接读取全局结构体 `g_odometer`，其中包含：

```c
g_odometer.position[x]
g_odometer.position[y]
g_odometer.vel[x]
g_odometer.vel[y]
```

`beacon_detection` 已经实现基础信标灯检测功能，外部主要读取全局结构体 `g_beacon_detection`。本任务 v1 只消费检测结果，不重新研究信标检测算法，不修改检测阈值和敏感度。

当前主循环调度位置在：

```text
CYT4bb7_Car\project\code\Controller\car_loop.c
```

当前 100Hz 顺序大致为：

```c
encoder_update_100HZ();
odometer_update_100HZ();
beacon_detection_update_100HZ();
```

后续接入 `fixator` 时必须重新检查调度顺序。由于 v1 使用 `g_beacon_detection.enter_event` 触发修正，`fixator_update_100HZ()` 原则上应在 `beacon_detection_update_100HZ()` 之后运行，修正方案再由 `odometer` 应用，具体接入前必须读代码确认。

## 信标灯物理信息

赛道上的信标灯是固定在地面上的圆形凸起结构，不是平面标记，也不是长条障碍。

物理参数：

- 外形：圆形盘状。
- 外径：`260 mm`。
- 整体形态：微拱形。
- 边缘厚度：`1.8 mm`。
- 中心最高处总厚度：`15 mm`。

对麦克纳姆轮小车的影响：

- 小车经过信标灯时，轮子可能整体打滑。
- 车身可能出现短时倾斜、俯仰、横滚冲击。
- 平地标定得到的惯导/编码器里程计参数，在压过信标灯时容易失准。
- 长期运行后，惯导位置可能产生严重漂移。

因此，信标灯既是干扰源，也是可利用的固定地标。由于信标坐标可以提前登记，检测到“上信标灯”时可以用已知坐标修正当前全局惯导位置。

## 坐标系与极性要求

必须修正 `odometer` 对外坐标极性。最终语义固定为：

- `g_odometer.vel[x] > 0`：小车往前走。
- `g_odometer.vel[y] > 0`：小车向右移动。
- `g_odometer.position[x] > 0`：小车位于原点前方。
- `g_odometer.position[y] > 0`：小车位于原点右侧。

注意：当前 `odometer.h` 注释和 `odometer.c` 横移公式原先表达的是“左为正”。改极性时不能只改注释，必须同步检查所有受影响位置。

至少要全局搜索并检查：

```text
g_odometer.vel[y]
g_odometer.position[y]
g_beacon_detection.vel[1]
body_vel[y]
horizontal_vel[y]
strafe
velocity_strafe_feedback_mps
```

重点风险点：

- `CYT4bb7_Car\project\code\Estimation\Position\odometer.c`
  - 横移速度公式。
  - 机体坐标转水平坐标后的 Y 输出。
  - `g_odometer.position[y]` 积分。
- `CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.c`
  - 内部也计算 `g_beacon_detection.vel[1]`。
  - `beacon_detection_location_from_motion()` 使用横移速度判断 `FRONT / RIGHT / LEFT / REAR`。
  - 如果只改 `odometer` 不改这里，方向判断可能反。
- `CYT4bb7_Car\project\code\Controller\car_mode1.c`
  - 速度闭环反馈使用 `g_odometer.vel[y]`。
  - 目标横移速度 `velocity_strafe_target_mps` 与反馈极性必须一致。
- `CYT4bb7_Car\project\code\Menu\menu_config.c`
  - 菜单显示 `Odo Y` 和速度显示语义要同步。
- WiFi/日志语义
  - 如果日志中输出 Y 速度或位置，说明必须跟随“右为正”。

## 本轮新增模块

在以下目录新增两个基础库：

```text
CYT4bb7_Car\project\code\Estimation\Position
```

### 1. beacon_config

文件：

```text
beacon_config.h
beacon_config.c
```

职责：

- 保存赛道上信标灯总个数。
- 保存每个信标灯的全局 XY 坐标，单位为 `m`。
- 保存小车上电/初始化时的初始全局 XY 坐标，单位为 `m`。
- 让小车上电后不再固定从 `(0, 0)` 开始，而是从配置值开始，便于统一“信标全局坐标”和“小车全局坐标”的坐标系。

配置形式固定为：

- 使用 C 源文件内的静态/常量数组登记信标坐标。
- 不做文件系统。
- 不做 Flash 持久化。
- 不做菜单运行时改参。
- 不做动态内存。

坐标含义固定为：

- `beacon_config` 中登记的信标坐标表示“车体中心位于该信标中心时”的全局坐标。
- v1 不做轮组接触点偏置补偿。
- v1 不根据 `FRONT / RIGHT / LEFT / REAR` 给车体几何补偿。

初始位姿范围固定为：

- 本轮只配置初始 `position[x]` 和 `position[y]`。
- 本轮不配置全局 yaw。
- `odometer_reset()` 后仍以当前车头方向作为全局 X 正方向的航向基准。

对外接口必须极简。建议只保留初始化、reset、只读配置读取这类最少接口。不要为了未来乱加接口。

### 2. fixator

文件：

```text
fixator.h
fixator.c
```

职责：

- 读取 `g_beacon_detection.enter_event`。
- 读取当前 `g_odometer.position[x/y]`。
- 读取 `beacon_config` 中登记的所有信标坐标。
- 当检测到上信标灯事件时，以当前惯导坐标为圆心，在可调半径内寻找最近信标。
- 如果范围内存在信标，则认为小车实际车体中心应位于该信标坐标，输出惯导位置修正方案。
- 如果范围内没有信标，则不修正。

v1 基础算法固定为：

```text
if enter_event:
    找到距离当前 g_odometer.position 最近的信标
    if 最近距离 <= FIXATOR_MATCH_RADIUS_M:
        输出修正方案：position[x/y] = 该信标坐标
    else:
        不修正
```

默认参数：

```c
#define FIXATOR_MATCH_RADIUS_M (0.5f)
```

`0.5 m` 是初始值，后续必须通过实车日志再调。当前没有可用融合日志，不能把这个参数吹成最终最优值。

重复修正策略：

- v1 不额外按信标 ID 去重。
- v1 只依赖 `enter_event` 单脉冲触发一次修正。
- 必须在文档或调试输出中说明风险：如果 `beacon_detection` 对同一信标产生重复 `enter_event`，`fixator` 也可能重复修正。
- 后续如果实车发现重复修正，再增加冷却或上次信标 ID 去重，不要本轮提前复杂化。

检测联动策略：

- v1 不反向修改 `beacon_detection` 阈值。
- v1 不做“惯导可信度高时降低检测敏感度”的动态阈值。
- 可以在设计说明中保留未来扩展方向：惯导可信区域内没有信标时，未来可降低检测触发敏感度或抑制误检。
- 当前先完成单向闭环：检测到信标后修正惯导。

## odometer 接入要求

`odometer.h` 最前面必须新增一个总开关宏，用于一键启停信标矫正功能，避免修正算法出错影响系统运行。

建议形式：

```c
#define ODOMETER_BEACON_FIXATOR_ENABLE (1U)
```

要求：

- 宏关闭时，`odometer` 行为必须退回纯惯导/编码器里程计。
- 宏开启时，`odometer` 才允许应用 `fixator` 给出的修正方案。
- 初始全局坐标也应受 `beacon_config` 影响，但必须保证逻辑清晰：如果关闭矫正，是否仍启用初始坐标配置，需要在实现中写清楚。推荐：初始坐标配置属于 `beacon_config` 的地图坐标能力，可以独立于信标修正开关；但实现前必须让代码注释讲明白。

`fixator` 与 `odometer` 的关系固定为：

- `fixator` 负责判断是否需要修正，输出修正方案。
- `odometer` 负责应用修正到 `g_odometer.position`。
- 不要让多个模块随便直接改 `g_odometer`，否则后面调试会乱成一锅粥。

可以采用最小接口，例如：

```c
void fixator_init(void);
void fixator_reset(void);
void fixator_update_100HZ(void);
uint8 fixator_get_position_fix(float position[2]);
```

这里只是建议，不要求逐字照抄。最终接口必须更少、更清楚，而不是更多。

## 代码风格与工程约束

必须遵守现有工程风格：

- C 语言。
- 4 空格缩进。
- 函数和控制块大括号独占一行。
- 模块内部状态用 `static` 文件作用域变量。
- 不引入动态内存。
- 不引入大数组。
- 不引入复杂模型。
- 不引入现在用不到的未来接口。
- 对外函数越少越好，优先 `init`、`reset`、`update_100HZ`。
- 注释使用中文，但不要写废话。
- 算法必须简单、可解释、嵌入式实时可跑。

新增头文件规则：

- `beacon_config.h` 和 `fixator.h` 内部只能 include：

```c
#include "zf_common_headfile.h"
```

- 不要在业务头文件里私自 include 其他业务头。
- 新增头文件必须加入：

```text
CYT4bb7_Car\libraries\zf_common\zf_common_headfile.h
```

新增 C 文件规则：

- `.c` 文件优先 include 自己的同名头文件，例如：

```c
#include "fixator.h"
```

- 如果需要访问其他模块公开 API，应通过总头文件已经暴露的接口访问。

IAR 工程规则：

- 新增 `.c/.h` 必须加入：

```text
CYT4bb7_Car\project\iar\project_config\cyt4bb7_cm_7_0.ewp
```

- 推荐加入 `Estimation -> Position` 分组。
- 当前 `cyt4bb7_cm_7_1.ewp` 暂未挂载这些估计模块，不要无脑改第二核工程。

禁止事项：

- 禁止执行 `git commit`、`git push`、`git reset`、`git checkout`，除非用户明确要求。
- 禁止覆盖原始日志。
- 禁止为了本任务大改 `beacon_detection` 算法。
- 禁止添加 Flash、菜单、通信协议等超出 v1 的功能。
- 禁止把复杂离线算法硬塞进车端 C 代码。

## 具体任务拆分

### 任务 1：读代码确认现状

先读以下文件：

```text
CYT4bb7_Car\AGENTS.md
CYT4bb7_Car\libraries\zf_common\zf_common_headfile.h
CYT4bb7_Car\project\code\Estimation\Position\odometer.h
CYT4bb7_Car\project\code\Estimation\Position\odometer.c
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.h
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.c
CYT4bb7_Car\project\code\Controller\car_loop.c
CYT4bb7_Car\project\code\Controller\car_mode1.c
CYT4bb7_Car\project\code\Menu\menu_config.c
CYT4bb7_Car\project\iar\project_config\cyt4bb7_cm_7_0.ewp
```

必须确认：

- `g_odometer` 当前结构和更新逻辑。
- `g_beacon_detection.enter_event` 的保持周期和清零方式。
- `car_loop.c` 的 100Hz 调度顺序。
- Y 轴极性修改会影响哪些模块。

### 任务 2：修正 odometer 坐标极性

把对外语义改为“右为正”。

必须同步更新：

- `odometer.h` 坐标系注释。
- `odometer.c` 横移速度计算或最终输出符号。
- 与 `g_odometer.vel[y]` 闭环反馈相关的控制逻辑。
- 与 `g_beacon_detection.vel[1]` 和方向判断相关的检测逻辑。
- 菜单显示和日志说明。

验收时必须能解释：

- 为什么右移时 `g_odometer.vel[y] > 0`。
- 为什么右移后 `g_odometer.position[y]` 增大。
- 为什么控制层 `strafe` 目标和反馈没有反号打架。
- 为什么信标方向判断没有被 Y 极性改反。

### 任务 3：新增 beacon_config

新增：

```text
CYT4bb7_Car\project\code\Estimation\Position\beacon_config.h
CYT4bb7_Car\project\code\Estimation\Position\beacon_config.c
```

最低要求：

- 定义信标坐标结构，包含 `x/y`，单位 `m`。
- 定义信标总数。
- 定义信标坐标数组。
- 定义初始全局坐标。
- 提供极简只读接口。
- `beacon_config_reset()` 或等价接口必须能恢复默认配置状态。

实现要点：

- 坐标数组先填示例值或空配置都可以，但必须让用户能很容易在 `.c` 文件中登记比赛前的坐标。
- 如果默认无信标，`fixator` 必须自然不修正。
- 不要搞运行时增删信标接口。

### 任务 4：新增 fixator

新增：

```text
CYT4bb7_Car\project\code\Estimation\Position\fixator.h
CYT4bb7_Car\project\code\Estimation\Position\fixator.c
```

最低要求：

- `fixator_init()`。
- `fixator_reset()`。
- `fixator_update_100HZ()`。
- 一个让 `odometer` 获取修正方案的最小接口。

基础算法：

- 只在 `g_beacon_detection.enter_event != 0U` 时尝试匹配。
- 当前点为 `g_odometer.position[x/y]`。
- 遍历 `beacon_config` 中所有信标。
- 计算平方距离，避免不必要的 `sqrtf()`。
- 找到 `FIXATOR_MATCH_RADIUS_M` 内最近信标。
- 命中则输出修正方案：目标位置等于该信标坐标。
- 未命中则不输出修正。

建议输出状态至少能支持调试：

- 是否有有效修正。
- 命中的信标索引。
- 修正前位置。
- 修正后位置。
- 匹配距离或距离平方。
- 修正次数。

这些调试字段可以放在一个全局状态结构里，也可以尽量少放。目标是能看懂，不是堆字段。

### 任务 5：接入 odometer 和主循环

要求：

- `odometer_init()` 或 `odometer_reset()` 时应用 `beacon_config` 的初始全局坐标。
- `ODOMETER_BEACON_FIXATOR_ENABLE` 开启时，`odometer` 才应用 `fixator` 修正方案。
- `fixator_update_100HZ()` 必须被 100Hz 调度调用。
- 调度顺序必须保证 `fixator` 能读到本周期有效的 `enter_event`。
- 不要让 `fixator` 和 `odometer` 互相造成头文件循环依赖。

推荐思路：

- `car_loop.c` 中在 `beacon_detection_update_100HZ()` 后调用 `fixator_update_100HZ()`。
- `odometer_update_100HZ()` 在积分完成后或下一周期读取 `fixator` 修正方案并应用。
- 如果为了时序简单，也可以在 `fixator_update_100HZ()` 内只记录方案，下一次 `odometer_update_100HZ()` 应用；但必须注释说明这会有一个 100Hz 周期级延迟。

具体采用哪种时序，以读完现有代码后的最小改动为准。

### 任务 6：同步公共头文件和 IAR 工程

必须修改：

```text
CYT4bb7_Car\libraries\zf_common\zf_common_headfile.h
CYT4bb7_Car\project\iar\project_config\cyt4bb7_cm_7_0.ewp
```

要求：

- `zf_common_headfile.h` 的 Project Code 区域加入：

```c
#include "Estimation/Position/beacon_config.h"
#include "Estimation/Position/fixator.h"
```

- `cyt4bb7_cm_7_0.ewp` 的 `Position` 分组加入：

```text
$PROJ_DIR$\..\..\code\Estimation\Position\beacon_config.c
$PROJ_DIR$\..\..\code\Estimation\Position\beacon_config.h
$PROJ_DIR$\..\..\code\Estimation\Position\fixator.c
$PROJ_DIR$\..\..\code\Estimation\Position\fixator.h
```

## 验收标准

### 代码结构验收

- 新增 `beacon_config.c/h`。
- 新增 `fixator.c/h`。
- 新增模块位于 `Estimation/Position`。
- 新增头文件加入总头文件。
- 新增 C/H 文件加入 IAR CM7_0 工程。
- 对外函数保持极简，没有无意义接口。

### 极性验收

必须能通过代码检查确认：

- 前进：`g_odometer.vel[x] > 0`。
- 右移：`g_odometer.vel[y] > 0`。
- 原点前方：`g_odometer.position[x] > 0`。
- 原点右侧：`g_odometer.position[y] > 0`。

必须检查控制层和信标检测方向判断没有因为 Y 极性变更出现反号问题。

### 算法验收

至少用人工构造或静态逻辑检查覆盖：

1. `beacon_config` 信标数量为 0 时，`fixator` 不输出修正。
2. `enter_event == 0` 时，`fixator` 不输出修正。
3. `enter_event != 0` 且 `0.5 m` 内无信标时，不输出修正。
4. `enter_event != 0` 且 `0.5 m` 内有一个信标时，输出该信标坐标。
5. `enter_event != 0` 且 `0.5 m` 内有多个信标时，选择最近信标。
6. `ODOMETER_BEACON_FIXATOR_ENABLE == 0U` 时，`odometer` 不应用信标修正。
7. `odometer_reset()` 后初始 `position[x/y]` 来自 `beacon_config` 配置。

### 构建验收

优先使用 IAR 完整构建 `cyt4bb7_cm_7_0`。

如果当前环境没有 IAR，只能做以下替代检查，并必须如实说明：

- `git diff --check`。
- 可行时使用 `arm-none-eabi-gcc -fsyntax-only` 做语法检查。
- 不能声称 IAR 已通过。

## 风险与后续优化方向

当前没有专门用于“信标坐标修正惯导”的实车日志，所以 v1 只能先搭框架。

已知风险：

- `0.5 m` 匹配半径可能过大或过小。
- 信标密集时，最近信标匹配可能误选。
- `beacon_detection` 如果重复产生 `enter_event`，v1 可能重复修正。
- 小车压过信标时，车体中心不一定精确位于信标圆心；v1 暂不做轮组/车体几何偏置补偿。
- 只修正 XY，不修正 yaw。
- 不根据累计里程、打滑程度动态估计惯导可信半径。

后续可优化，但本轮不要提前实现：

- 按信标 ID 去重或冷却。
- 结合累计行驶距离估计惯导可信半径。
- 结合 `g_beacon_detection.location` 做车体几何偏置补偿。
- 使用信标坐标反向抑制误检。
- 根据地图信标分布动态调整检测敏感度。
- 增加日志通道记录修正前后位置、命中信标 ID、匹配距离。

## 最终交付说明要求

完成任务后，必须用简体中文说明：

- 新增了哪些文件。
- 修改了哪些已有文件。
- `ODOMETER_BEACON_FIXATOR_ENABLE` 在哪里。
- Y 轴极性改了哪些地方。
- `beacon_config` 如何登记信标坐标和初始坐标。
- `fixator` 何时触发，如何选择信标。
- `odometer` 如何应用修正。
- 是否同步了 `zf_common_headfile.h` 和 IAR 工程。
- 跑了哪些检查或构建。
- 哪些风险需要实车日志继续验证。

不要提交 git。不要声称没有实际跑过的验证已经通过。
