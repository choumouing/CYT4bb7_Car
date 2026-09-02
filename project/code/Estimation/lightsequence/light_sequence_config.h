/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#ifndef LIGHT_SEQUENCE_CONFIG_H
#define LIGHT_SEQUENCE_CONFIG_H

#include "zf_common_headfile.h"

#define LIGHT_SEQUENCE_BEACON_COUNT        (BEACON_CONFIG_BEACON_COUNT) /* 使用Predata地图的信标数量 */
#define LIGHT_SEQUENCE_PRESET_COUNT        (4U)    /* 预设亮灯序列数量 */
#define LIGHT_SEQUENCE_MAX_ROUNDS          (40U)   /* 单套序列最多保存的轮数 */
#define LIGHT_SEQUENCE_MATCH_RADIUS_M      (0.5f)  /* 熄灯事件坐标匹配半径，单位m */
#define LIGHT_SEQUENCE_DUPLICATE_WINDOW_MS (1000U) /* 同一灯号的时间去重窗口，单位ms */

/**
 * @brief 一套预设亮灯序列。
 */
typedef struct
{
    uint8 round_count;                              /* 有效轮数 */
    uint16 round_lights[LIGHT_SEQUENCE_MAX_ROUNDS]; /* 各轮灯号组合，例如16表示1号和6号灯 */
} light_sequence_preset_t;

/* 预设亮灯序列配置。 */
extern const light_sequence_preset_t g_light_sequence_presets[LIGHT_SEQUENCE_PRESET_COUNT];

#endif /* LIGHT_SEQUENCE_CONFIG_H */
