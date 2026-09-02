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
#ifndef _FIXATOR_H_
#define _FIXATOR_H_

#include "zf_common_headfile.h"

#define FIXATOR_MATCH_RADIUS_M (0.5f)
#define FIXATOR_NO_BEACON_INDEX (0xFFFFU)
#define FIXATOR_SOURCE_NONE (0U)
#define FIXATOR_SOURCE_AIR_BEACON (1U)
#define FIXATOR_SOURCE_BEACON_DETECTED (2U)

typedef struct
{
    uint8 pending_fix;
    uint8 last_match_valid;
    uint8 counts_in_sequence;       /* 1=计入信标序列，0=只做位置校准 */
    uint8 fix_source;               /* 0=无，1=Air 灭信标灯 */
    uint16 beacon_index;
    uint16 previous_beacon_index;
    float before_position[2];
    float fixed_position[2];
    float match_distance2_m2;
    uint32 fix_count;
    uint32 sequence_count;
} fixator_data_t;

extern fixator_data_t g_fixator;

void fixator_init(void);
void fixator_reset(void);
void fixator_update_100HZ(void);
uint8 fixator_get_position_fix(float position[2]);

#endif
