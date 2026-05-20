你现在要长期优化车端信标检测算法。请始终使用简体中文回复，先读代码和日志，再写分析脚本，再基于数据改算法。不要只围绕现有 beacon_detection.c 的阈值小修小补，你可以优化当前算法，也可以完全替换成一套新的检测算法。目标是：基于现有 28 份日志，离线复现并优化信标检测，最终让车端 C 代码能精确识别每一次“上信标灯”和“下信标灯”，并把位置结果简化为前、右、左、后四类。

项目路径：
D:\HDUASC-SmartCar-21st-FlyOverMinefield

重点代码：
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.c
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.h

日志目录：
CYT4bb7_Car\project\code\Estimation\Beacon_Detection

这个目录下有 28 份日志。日志来自车端 wifi_justfloat，通道顺序如下：

1. tick_1000us_cnt
2. ICM42688.acc_x
3. ICM42688.acc_y
4. ICM42688.acc_z
5. ICM42688.gyro_x
6. ICM42688.gyro_y
7. ICM42688.gyro_z
8. g_imufilter_1000hz.accx
9. g_imufilter_1000hz.accy
10. g_imufilter_1000hz.accz
11. g_imufilter_1000hz.gyrox
12. g_imufilter_1000hz.gyroy
13. g_imufilter_1000hz.gyroz
14. g_euler.roll
15. g_euler.pitch
16. g_euler.yaw
17. left_front
18. right_front
19. left_rear
20. right_rear
21. accel_x_g
22. accel_y_g
23. accel_z_g
24. gyro_x_dps
25. gyro_y_dps
26. gyro_z_dps
27. sample.tilt_deg
28. g_beacon_detection.bump_detected
29. g_beacon_detection.confidence
30. g_beacon_detection.location
31. g_beacon_detection.wheel_mask
32. g_beacon_detection.score

日志语义：

- 有一份长日志，名称含义类似“经过20次信标灯”。这是小车正常运行，全程一共碾压/经过信标灯 20 次。它适合做最终离线仿真验证：算法应该识别到 20 次有效信标事件，并尽量准确识别每次上信标灯和下信标灯的时间。
- 日志名称“快速跑，没有经过信标灯”表示全程没有碰到信标灯，离线算法不应该误识别任何信标事件。
- 日志名称“实际没有信标灯”同样表示全程没有碰到信标灯，也不应该误触发。
- 其他名称较统一的短日志，例如“信标在正后方，直接经过快速.csv”。这里“正前方/正后方/左/右”等描述的是上信标灯那一瞬间，车体与信标灯的相对位置关系，不代表车只做单轴运动。小车可能往前、后、左、右、斜向等多个方向运动。
- 文件名里的“快速/中速/慢速”表示经过信标灯时的速度等级。不同速度下 IMU、欧拉角、轮速冲击特征不同，必须分别分析。
- 文件名后缀如“中速2.csv”表示同一场景重复采集的第二次日志，应该作为同类样本处理。

核心目标：

1. 不要局限于当前 beacon_detection.c 的 classic_bump、axis_bump、edge_bump、hand_push 等规则。
2. 必须先把所有 CSV 日志读通，建立统一数据加载器。
3. 必须建立离线仿真框架：给定一份日志，输出检测到的上信标灯事件、下信标灯事件、事件时间、事件方向、score/confidence。
4. 必须能批量跑 28 份日志，输出每份日志的识别结果统计。
5. 对无信标日志，误触发次数应为 0，除非你有非常充分的数据理由说明某处疑似真实冲击。
6. 对“经过20次信标灯”的长日志，应重点检查是否能识别 20 次有效事件，并尽量分析每次上/下信标的时间。
7. 对短日志，应结合文件名里的方向、速度标签，评估方向识别是否正确。
8. 最终修改 C 代码时，优先保证嵌入式实时性，不能引入复杂动态内存、大数组、大量浮点高阶模型或难以维护的代码。

Python 分析要求：

- 优先使用 Python 做离线分析。
- 推荐使用 pandas、numpy、scipy、matplotlib；如果需要参数搜索或分类评估，可以使用 scikit-learn。
- 数据量较大时，不要写低效 Python for 循环硬扫全量数据。尽量使用 numpy 向量化、pandas rolling、scipy signal、numba 或缓存 npz/parquet。
- 第一步先写数据加载脚本，自动发现 Beacon_Detection 目录下的日志文件。
- 给每个 CSV 自动加列名，列名按上面的 32 个通道定义。
- 检查采样周期、tick 是否跳变、是否有丢帧、是否有异常 NaN/Inf。
- 建议建立 analysis 子目录，例如：
  CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis
  里面放 Python 脚本、分析报告、生成图表或缓存文件。
- 不要破坏原始日志，不要覆盖 CSV。

必须做的分析：

1. 画出典型日志里的关键曲线：
   - 校准后 gyro_x_dps/y_dps/z_dps
   - gyro_xy = sqrt(gx^2 + gy^2)
   - accel_norm_error = abs(norm(accel_x_g/y_g/z_g) - 1)
   - tilt_deg
   - tilt_rate_dps，可离线由 tilt_deg 或 roll/pitch 差分得到
   - 四轮速度 left_front/right_front/left_rear/right_rear
   - 车体速度 forward/strafe/speed
   - 轮速高通特征 wheel_highpass_count
   - 原算法输出 bump_detected/score/location
2. 对有信标日志，找出上信标灯和下信标灯附近的共同特征。
3. 对无信标日志，找出正常跑动时 IMU 和轮速的最大噪声边界，重点避免误触发。
4. 比较不同速度：快速、中速、慢速下，冲击峰值、持续时间、窗口宽度是否不同。
5. 比较不同方向：前、后、左、右经过时，四轮冲击模式和车体速度方向是否可用于方向判断。
6. 评估当前算法在所有日志上的表现：漏检、误检、重复触发、方向错误、上/下信标混淆。
7. 尝试新算法方案，不要只调阈值。可以考虑：
   - 多特征窗口峰值检测
   - 自适应噪声基线 + robust z-score
   - IMU 冲击候选 + 轮速/速度门控
   - 状态机：idle -> on_beacon_candidate -> on_beacon_hold -> off_beacon_candidate -> cooldown
   - 上信标和下信标分开识别
   - 基于速度方向和轮速冲击分布判断前/后/左/右
8. 对候选算法做批量评估，给出参数表和选择理由。

最终 C 代码目标：

- 修改 beacon_detection.h，简化 beacon_bump_location_t。不要保留左前、右前、左后、右后、对角线这些复杂结果。只需要：
  BEACON_BUMP_LOCATION_UNKNOWN
  BEACON_BUMP_LOCATION_FRONT
  BEACON_BUMP_LOCATION_RIGHT
  BEACON_BUMP_LOCATION_LEFT
  BEACON_BUMP_LOCATION_REAR
- beacon_detection_data_t 里保留必要结果字段，重点支持：
  - 是否正在信标上
  - 上信标事件锁存
  - 下信标事件锁存
  - location：前/右/左/后
  - confidence
  - score
  - event_count，最好区分 enter_count/exit_count，如果结构体需要调整就说明原因
  - speed_mps、vel[2]
  - 关键 IMU 特征和轮速特征，方便调试
- beacon_detection.c 可以继续保持 1kHz IMU 更新 + 100Hz 轮速更新的结构。
- 1000Hz 函数负责 IMU 特征、窗口峰值、状态机推进、事件锁存。
- 100Hz 函数负责四轮速度、车体速度、轮速高通特征。
- 不要把离线 Python 里复杂的东西无脑搬进 C。C 代码必须简单、可解释、实时性好。
- 如果需要新增少量静态状态变量可以，但要保持模块薄、清楚、可维护。

输出成果要求：

1. 先给出你对日志文件的扫描结果：发现多少 CSV、每个文件大概多少行、是否能正常解析 32 通道。
2. 给出离线分析脚本路径和使用方法。
3. 给出批量评估结果表，至少包含：
   - 文件名
   - 期望事件数
   - 检测到的上信标次数
   - 检测到的下信标次数
   - 误触发次数
   - 方向识别结果
   - 备注
4. 给出你最终选择的算法说明，不要空泛，要说明用了哪些特征、窗口大小、阈值、状态机逻辑。
5. 给出参数搜索或参数选择依据：为什么这些阈值适合快速/中速/慢速和无信标日志。
6. 修改 beacon_detection.c/h，并说明改动点。
7. 不要执行 git commit、git push、git reset、git checkout。
8. 如果没有 IAR 环境，不要尝试编译。可以跑 Python 离线脚本和静态检查。
9. 最终必须明确说明：
   - 离线算法是否跑通 28 份日志
   - 无信标日志是否 0 误触发
   - “经过20次信标灯”长日志识别到了多少次
   - 当前仍有哪些风险，需要后续补哪些实车日志

工作顺序：

1. 读 AGENTS.md 和现有 beacon_detection.c/h。
2. 扫描 Beacon_Detection 目录下所有 CSV 日志。
3. 写 Python 数据加载和校验脚本。
4. 建立当前 C 算法的离线复现版本，先评估旧算法。
5. 画关键曲线，分析误检/漏检。
6. 设计至少 2 套新候选算法，并批量评估。
7. 选择最稳的一套，转成 C 实现。
8. 再用同一批日志离线复测新算法。
9. 输出简洁但完整的报告和代码改动总结。

注意：

- 不要一开始就改 C。先用日志把问题看清楚。
- 不要只凭一两份日志调参。必须批量跑全部日志。
- 不要只追求识别有信标日志，必须同时压住无信标日志误触发。
- 不要迷信当前算法，必要时直接换掉。
- 最终目标不是花哨，而是实车稳定识别每一次上信标灯和下信标灯。
