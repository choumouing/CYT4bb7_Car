#ifndef _FIXATOR_H_
#define _FIXATOR_H_

#include "zf_common_headfile.h"

#define FIXATOR_MATCH_RADIUS_M (0.5f)
#define FIXATOR_NO_BEACON_INDEX (0xFFFFU)

typedef struct
{
    uint8 pending_fix;              /* odometer 尚未消费的修正方案 */
    uint8 last_match_valid;         /* 最近一次 enter 边沿是否命中信标 */
    uint16 beacon_index;            /* 最近命中的信标索引 */
    float before_position[2];       /* 修正前位置 [m]，[x]=右，[y]=前 */
    float fixed_position[2];        /* 修正目标位置 [m]，[x]=右，[y]=前 */
    float match_distance2_m2;       /* 匹配距离平方 [m^2] */
    uint32 fix_count;               /* 已输出修正次数 */
} fixator_data_t;

extern fixator_data_t g_fixator;

void fixator_init(void);
void fixator_reset(void);
void fixator_update_100HZ(void);
uint8 fixator_get_position_fix(float position[2]);

#endif
