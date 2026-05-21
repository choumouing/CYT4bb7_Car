你现在要重新研究并长期优化车端信标检测算法。请始终使用简体中文回复，先读代码和日志，再写离线分析脚本，再基于数据设计和修改算法。不要只围绕当前 `beacon_detection.c` 的阈值小修小补；如果数据证明当前算法方向不对，可以直接替换为一套新的、简单可解释、可嵌入式实时实现的检测算法。

## 项目路径

```text
D:\HDUASC-SmartCar-21st-FlyOverMinefield
```

重点代码：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.c
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.h
```

第二次标定日志目录：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\第二次算法的标定数据
```

数据提交节点：

```text
6cdceaeb9484627ef0141bf7eca2c91e45e17d3f
```

该节点提交信息为：`运行了29次,记录了29份日志`。

## 信标灯物理信息

信标灯是一个整体的小圆形凸起，不是长坡道：

- 外形：圆形盘状。
- 外径：260 mm。
- 边缘厚度：1.8 mm。
- 中心最高处总厚度：15 mm。
- 通过过程很短。正常实车经过时，上信标灯到下信标灯不应该拖到几秒后；如果算法几秒后才报事件，应按失败处理。

方向语义：

- `信标在车正前方`：上信标灯时，车体应先出现抬头趋势；下信标灯时，车体应出现低头/降头趋势。
- `信标在车正后方`：上/下信标灯的俯仰变化方向与正前方相反。
- `信标在车正右方`：上信标灯时，车体右侧应上抬；下信标灯时，车体应向右侧下落/倾斜。
- `信标在车正左方`：上/下信标灯的横滚变化方向与正右方相反。
- 文件名里的方向描述的是上信标灯瞬间车体与信标灯的相对位置，不代表车只沿单一轴运动。

## 总目标和优先级

最终目标是让车端 C 代码稳定识别每一次经过信标灯的事件。优先级固定如下：

1. 无信标日志必须 0 误触发。
2. 准确、及时识别上信标灯 `enter_event`。
3. 准确、及时识别下信标灯 `exit_event`，并和对应 `enter_event` 配对。
4. 正确维护 `on_beacon` 状态，不能长期卡住，也不能几秒后才补报。
5. 在前面目标满足后，再优化方向识别：`FRONT / RIGHT / LEFT / REAR`。

方向识别是二级目标。能精确识别最好，但不能为了方向结果牺牲“是否正在信标上”和“上/下信标时刻”的稳定性。

## 日志数据集

本轮必须以 `第二次算法的标定数据` 里的 29 份 CSV 为主验收集，不再以旧 28 份日志作为主目标。

29 份日志分三类：

1. 短日志：整份日志保证只压过 1 次信标灯。文件名写明信标在车的正前方、正后方、正左方、正右方，以及快速/中速/慢速工况。
2. 长日志：4 份，每份经过 10 个信标灯，方向分别全在车前边、后边、左边、右边。
3. 无信标日志：`全程没有碰到信标灯.csv`，全程不应检测到任何信标事件。

### 已扫描到的日志概况

这 29 份 CSV 均为 40 通道日志。行数如下：

| 文件名 | 行数 | 通道数 |
| --- | ---: | ---: |
| `全程没有碰到信标灯.csv` | 469344 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车前边.csv` | 45490 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车后边.csv` | 56108 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车右边.csv` | 96172 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车左边.csv` | 51097 | 40 |
| 其余 24 份短日志 | 每份约 3078 到 18984 行 | 40 |

## 40 通道顺序

日志来自车端 `wifi_justfloat`，通道顺序必须按以下定义解析：

1. `tick_1000us_cnt`
2. `ICM42688.acc_x`
3. `ICM42688.acc_y`
4. `ICM42688.acc_z`
5. `ICM42688.gyro_x`
6. `ICM42688.gyro_y`
7. `ICM42688.gyro_z`
8. `g_imufilter_1000hz.accx`
9. `g_imufilter_1000hz.accy`
10. `g_imufilter_1000hz.accz`
11. `g_imufilter_1000hz.gyrox`
12. `g_imufilter_1000hz.gyroy`
13. `g_imufilter_1000hz.gyroz`
14. `g_euler.roll`
15. `g_euler.pitch`
16. `g_euler.yaw`
17. `left_front`
18. `right_front`
19. `left_rear`
20. `right_rear`
21. `accel_x_g`
22. `accel_y_g`
23. `accel_z_g`
24. `gyro_x_dps`
25. `gyro_y_dps`
26. `gyro_z_dps`
27. `sample.tilt_deg`
28. `g_beacon_detection.bump_detected`
29. `g_beacon_detection.confidence`
30. `g_beacon_detection.location`
31. `g_beacon_detection.wheel_mask`
32. `g_beacon_detection.score`
33. `g_beacon_detection.enter_event`
34. `g_beacon_detection.exit_event`
35. `g_beacon_detection.on_beacon`
36. `g_beacon_detection.impact_robust_z`
37. `g_beacon_detection.speed_mps`
38. `g_beacon_detection.vel[0]`
39. `g_beacon_detection.vel[1]`
40. `g_beacon_detection.wheel_highpass_count`

离线脚本必须兼容旧 32 通道日志和新 40 通道日志，但本轮主评估必须使用上述 29 份 40 通道日志。

## 当前旧算法基线问题

必须先评估当前算法，不能跳过。已知当前车端输出在第二次标定数据上存在明显问题：

| 文件/类别 | 当前表现 |
| --- | --- |
| `全程没有碰到信标灯.csv` | `bump_detected` 有 12 次上升沿，`enter_event` 8 次，`exit_event` 8 次，属于严重误触发。 |
| `此日志一共经过10个信标灯,信标方向都在车前边.csv` | `enter_event` 10 次，`exit_event` 10 次，但 `bump_detected` 14 次，存在重复/额外蜂鸣器触发。 |
| `此日志一共经过10个信标灯,信标方向都在车后边.csv` | `enter_event` 10 次，`exit_event` 10 次，但 `bump_detected` 17 次，存在重复/额外蜂鸣器触发。 |
| `此日志一共经过10个信标灯,信标方向都在车右边.csv` | `enter_event` 6 次，`exit_event` 6 次，漏检严重。 |
| `此日志一共经过10个信标灯,信标方向都在车左边.csv` | `enter_event` 4 次，`exit_event` 4 次，漏检严重。 |
| 多个短日志 | 出现应为 1 次却检测 0 次、2 次、4 次的情况。 |

旧算法不能作为最终方案。后续必须重点解决：

- 平地晃动、急转、结构振动导致的无信标误触发。
- 真正上信标灯时漏检。
- 下信标灯未及时识别，几秒后才补报。
- `bump_detected` 同时代表 enter/exit，导致蜂鸣器含义混乱。
- 左右方向和横滚相关工况明显漏检。

## 验收标准

### 事件数量

- 短日志：每份期望 1 次 `enter_event` 和 1 次 `exit_event`。
- 10 信标长日志：每份期望 10 次 `enter_event` 和 10 次 `exit_event`。
- 无信标日志：期望 0 次 `enter_event`、0 次 `exit_event`、0 次等价蜂鸣器触发。

### 事件时序

- 同一次信标的 `enter_event -> exit_event` 配对间隔必须在 `0.2s - 1.0s` 内。
- 超过 `1.0s` 才出现 `exit_event`，或者几秒后才响蜂鸣器，直接判为失败。
- 检测时刻应贴近真实冲击/姿态特征峰，实车感知尺度上要求在 `0.5s` 内输出。
- 不追求毫秒级人工标注，但不能出现用户可明显感知的延迟。

### 方向识别

- 方向输出只允许：
  - `UNKNOWN`
  - `FRONT`
  - `RIGHT`
  - `LEFT`
  - `REAR`
- 短日志方向标签来自文件名。
- 方向识别为二级指标，报告中必须统计正确率，但不能为了方向牺牲事件检测和误触发控制。

### 蜂鸣器语义

- 当前实车允许上信标灯和下信标灯都响。
- 但算法评估不能只看 `bump_detected`。必须区分：
  - `enter_event`：上信标灯。
  - `exit_event`：下信标灯。
  - `on_beacon`：当前是否认为车正在信标上。
- 如果保留上下都响，报告必须说明每一次蜂鸣器对应 enter 还是 exit。不能让“响了一下”成为含糊事件。

## 离线分析要求

必须优先用 Python 做离线分析，严禁一上来直接改 C。

建议使用：

- `pandas`
- `numpy`
- `scipy`
- `matplotlib`
- 必要时可用 `numba`、`parquet/npz` 缓存提升速度。

必须建立或更新：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis
```

该目录用于放：

- 数据加载脚本。
- 批量评估脚本。
- 事件明细 CSV。
- 扫描摘要 CSV。
- 分析报告。
- 关键曲线图。
- 缓存文件。

注意：

- 不要破坏原始 CSV。
- 不要覆盖原始日志。
- 可以覆盖 `analysis/output` 下由脚本生成的结果文件。

## 必须分析的曲线和特征

对典型有信标、无信标、误检、漏检日志，必须画图或输出窗口统计：

- `gyro_x_dps / gyro_y_dps / gyro_z_dps`
- `gyro_xy = sqrt(gyro_x_dps^2 + gyro_y_dps^2)`
- `accel_norm_error = abs(norm(accel_x_g, accel_y_g, accel_z_g) - 1)`
- `roll / pitch / yaw`
- `tilt_deg`
- `tilt_rate_dps`
- 四轮速度：`left_front / right_front / left_rear / right_rear`
- 车体速度：`forward / strafe / speed`
- 轮速高通特征：`wheel_highpass_count`
- 当前车端输出：`bump_detected / enter_event / exit_event / on_beacon / score / location`
- 新算法候选输出：候选峰、enter、exit、配对关系、方向、置信度。

必须重点比较：

- 快速、中速、慢速下冲击峰值和持续时间。
- 前、后、左、右方向下俯仰/横滚特征差异。
- 有信标事件与无信标平地晃动、急转、结构振动的边界。
- 当前旧算法误触发点附近的 IMU 和轮速特征。
- 漏检样本附近是否存在更可靠的低幅特征。

## 真值和自分析要求

本轮不要求先人工逐帧标注所有 enter/exit 时间。要求算法先自分析，因为数据中的信标特征应当足够明显。

实现方式：

1. 根据文件名确定期望事件数、方向、速度等级。
2. 对每份日志自动寻找候选 enter/exit 峰。
3. 按物理约束配对：同一次信标的 enter/exit 间隔优先落在 `0.2s - 1.0s`。
4. 输出每个候选事件前后至少 `1.5s` 的关键特征摘要。
5. 对无法满足数量或时间约束的样本，自动列入“需要人工复核”列表。
6. 人工复核只针对争议样本，不作为前置条件。

## 新算法设计方向

不要只调几个阈值。至少设计并批量比较 2 套候选算法。

候选方向可以包括但不限于：

1. 多特征窗口峰值检测：
   - 结合 `gyro_xy`、`tilt_rate_dps`、`accel_norm_error_g`、`wheel_highpass_count`。
   - 使用短窗口峰值而不是单点阈值。
2. 自适应噪声基线：
   - 在无信标和正常跑动中估计噪声边界。
   - 使用 robust z-score 或类似抗异常指标。
3. enter/exit 分离：
   - 不允许一个 `bump_detected` 混掉上下信标。
   - enter 检测负责开始，exit 检测负责闭合。
4. 物理时间约束状态机：
   - `idle -> enter_candidate -> on_beacon -> exit_candidate -> cooldown`
   - 或者等价但更稳的状态机。
   - 必须防止几秒后才把 exit 当成当前信标的闭合事件。
5. 方向判定：
   - 正前/正后重点看 pitch/俯仰和前后速度。
   - 正左/正右重点看 roll/横滚和横向速度。
   - 方向不确定时允许 `UNKNOWN`，不要硬判导致错误传播。

## C 端实现约束

最终 C 代码必须符合嵌入式实时要求：

- 不能引入动态内存。
- 不能引入大数组。
- 不能引入高阶复杂模型。
- 不能把离线 Python 复杂逻辑无脑搬进 C。
- 可以使用少量静态状态变量。
- 可以继续保持当前 `1kHz IMU 更新 + 100Hz 轮速更新` 结构。

建议职责：

- `beacon_detection_update_1000HZ()`：
  - 更新 IMU 特征。
  - 推进窗口峰值。
  - 推进 enter/exit 状态机。
  - 锁存事件。
  - 输出 40 路调试日志。
- `beacon_detection_update_100HZ()`：
  - 更新四轮速度。
  - 更新车体 `forward / strafe / speed`。
  - 更新轮速高通特征。
  - 更新方向辅助信息。

`beacon_detection_data_t` 至少应保留：

- `bump_detected`
- `on_beacon`
- `enter_event`
- `exit_event`
- `confidence`
- `location`
- `wheel_mask`
- `hold_ticks`
- `event_count`
- `enter_count`
- `exit_count`
- `score`
- `impact_baseline`
- `impact_robust_z`
- `speed_mps`
- `vel[2]`
- `gyro_xy_dps`
- `gyro_z_abs_dps`
- `tilt_rate_dps`
- `tilt_deg`
- `accel_norm_error_g`
- `wheel_highpass_count`

## 输出成果要求

最终必须输出以下成果：

1. 日志扫描结果：
   - CSV 数量。
   - 每个文件行数。
   - 通道数。
   - NaN/Inf。
   - tick 跳变/丢帧情况。
2. 离线脚本路径和运行方法。
3. 当前旧算法基线评估：
   - 每份日志 `bump_detected / enter_event / exit_event / on_beacon` 统计。
   - 漏检、误检、重复触发、延迟闭合。
4. 新候选算法评估表：
   - 文件名。
   - 期望 enter 数。
   - 期望 exit 数。
   - 实际 enter 数。
   - 实际 exit 数。
   - 误触发数。
   - enter-exit 配对间隔。
   - 方向识别结果。
   - 备注。
5. 参数选择依据：
   - 为什么能覆盖快速/中速/慢速。
   - 为什么能压住无信标平地晃动。
   - 为什么不会几秒后才补报。
6. 最终算法说明：
   - 使用哪些特征。
   - 窗口大小。
   - 阈值。
   - 状态机逻辑。
   - enter/exit 配对逻辑。
   - 方向判定逻辑。
7. C 代码改动总结：
   - `beacon_detection.c`
   - `beacon_detection.h`
   - 如需调整蜂鸣器触发处，也必须说明原因。
8. 最终风险和后续补采建议。

## 工作顺序

严格按以下顺序做：

1. 读 `AGENTS.md` 和现有 `beacon_detection.c/h`。
2. 扫描 `第二次算法的标定数据` 下 29 份 CSV。
3. 建立或更新 Python 数据加载器，兼容 32/40 通道。
4. 复现当前车端输出，先评估旧算法。
5. 针对无信标误检日志画图，找误触发边界。
6. 针对短日志漏检/多检样本画图，找 enter/exit 共同特征。
7. 针对 4 份 10 信标长日志评估连续事件稳定性。
8. 设计至少 2 套新候选算法并批量评估。
9. 选择最稳的一套，说明参数依据。
10. 转成 C 端实时实现。
11. 用同一批 29 份日志复测。
12. 输出报告和代码改动总结。

## 禁止事项

- 禁止直接从当前算法阈值小修小补开始。
- 禁止不读日志就改 C。
- 禁止只看有信标日志，不看无信标误触发。
- 禁止只追求方向识别，忽略 enter/exit 时序。
- 禁止把几秒后的补报当成正确检测。
- 禁止覆盖原始 CSV。
- 禁止执行 `git commit`、`git push`、`git reset`、`git checkout`，除非用户明确要求。
- 没有 IAR 环境时，不要声称车端编译通过。

## 最终必须明确回答

每次阶段性完成后，必须明确说明：

- 是否跑通 29 份日志。
- 无信标日志是否 0 误触发。
- 4 份 10 信标长日志分别识别到多少次 enter/exit。
- 短日志是否每份都识别到 1 次 enter/exit。
- 是否存在超过 `1.0s` 的 enter-exit 闭合。
- 当前方向识别正确率。
- 哪些样本仍需人工复核。
- 后续还需要补采哪些实车日志。
