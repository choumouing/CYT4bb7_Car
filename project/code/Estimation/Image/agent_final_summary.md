# 三摄像头灯点融合最终汇总

## 输入与预处理

- 输入只使用 `C:/Users/23055/Downloads` 下三份任务指定 CSV。
- 不参考本目录既有 `.csv` 数据。
- 日志字段：`I0..I26` 为三摄像头各 3 个槽位的 `(x, y, radius)`；`I27..I37` 为采集时旧流程输出，仅用于字段说明。
- 强制预处理：若单帧任意摄像头识别灯数大于数据集标注灯数 `N`，整帧丢弃；小于或等于 `N` 的帧保留。

## 数量准确率对比

| 数据集 | 标注灯数 | 总帧 | 保留帧 | 丢弃帧 | Agent 1 基线 | Agent 2 优化 | Agent 3 全新算法 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `100cm_1灯.csv` | 1 | 7176 | 7169 | 7 | 72.70% | 100.000000% | 100.000000% |
| `2灯_(-175,50)_(-40,-100).csv` | 2 | 10250 | 10244 | 6 | 69.71% | 100.000000% | 100.000000% |
| `3灯_(-175,50)_(-40,-100)_(-85,200).csv` | 3 | 6629 | 6629 | 0 | 81.75% | 100.000000% | 100.000000% |
| **汇总** | - | 24055 | 24042 | 13 | 73.92% | 100.000000% | 100.000000% |

## 方案结论

- Agent 1：离线复刻当前 `beacon_fusion.c` 核心流程，证明现有算法会在欠检、空帧、跨相机匹配失败和重复拆分时输出错误数量。
- Agent 2：保留现有工程输入/输出边界，新增期望灯数配置，把最终输出数量锁定为 `N`；距离用半径反比代理，只做远近定性。
- Agent 3：完全独立于现有工程算法，按摄像头内稳定排序和跨摄像头同序合并完成去重，专注数量准确，不计算距离和角度。

## 正式交付文件

- Agent 1：`agent1_beacon_baseline.py`、`agent1_report.md`、`agent1_summary.csv`、`agent1_frame_detail.csv`、`agent1_field_map.csv`
- Agent 2：`agent2_beacon_fusion_optimized.c`、`agent2_beacon_fusion_optimized.h`、`agent2_validate.py`、`agent2_report.md`、`agent2_results.csv`、`agent2_frame_detail.csv`
- Agent 3：`agent3_beacon_fusion.c`、`agent3_beacon_fusion.h`、`agent3_validate.py`、`agent3_design.md`、`agent3_report.md`、`agent3_results.csv`、`agent3_fusion_detail.csv`
- 补充审计：`agent1_baseline_*`、`agent2_optimized_*`、`agent3_independent_*`

## 工程原则说明

- KISS：Agent 2/3 都把“数量正确”作为独立约束处理，避免继续堆复杂几何门限。
- YAGNI：没有引入训练、外部依赖或未要求的物理测距模型。
- DRY：字段解析、预处理规则和验证口径在脚本中集中复用。
- SOLID：新增文件均为 `agent*_` 独立交付，不修改原 `beacon_fusion.c/h`，便于人工审查后再选择接入。

