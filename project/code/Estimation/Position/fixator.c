#include "fixator.h"

fixator_data_t g_fixator = {0};

static uint32 s_last_enter_count;

static float fixator_distance2(float ax, float ay, float bx, float by)
{
    /* 全局坐标统一为 x=右、y=前；距离计算不关心轴名，只要求两端语义一致。 */
    float dx = ax - bx;
    float dy = ay - by;

    return (dx * dx) + (dy * dy);
}

void fixator_init(void)
{
    fixator_reset();
}

void fixator_reset(void)
{
    memset(&g_fixator, 0, sizeof(g_fixator));
    g_fixator.beacon_index = FIXATOR_NO_BEACON_INDEX;
    s_last_enter_count = g_beacon_detection.enter_count;
}

void fixator_update_100HZ(void)
{
    uint8 enter_now = (g_beacon_detection.enter_event != 0U) ? 1U : 0U;
    uint32 enter_count_now = g_beacon_detection.enter_count;
    uint16 count;
    uint16 i;
    uint16 best_index = FIXATOR_NO_BEACON_INDEX;
    float best_distance2 = FIXATOR_MATCH_RADIUS_M * FIXATOR_MATCH_RADIUS_M;
    beacon_config_point_t beacon;

    /*
     * v1 只对每次 enter_event 计数变化输出一次修正，避免保持期内重复修正。
     * 如果检测模块对同一信标再次产生新的 enter_event，仍可能重复修正，后续实车再决定是否加冷却。
     */
    if((enter_now == 0U) || (enter_count_now == s_last_enter_count))
    {
        return;
    }

    s_last_enter_count = enter_count_now;
    g_fixator.pending_fix = 0U;
    g_fixator.last_match_valid = 0U;
    g_fixator.beacon_index = FIXATOR_NO_BEACON_INDEX;
    g_fixator.before_position[0] = g_odometer.position[x];
    g_fixator.before_position[1] = g_odometer.position[y];
    g_fixator.fixed_position[0] = g_odometer.position[x];
    g_fixator.fixed_position[1] = g_odometer.position[y];
    g_fixator.match_distance2_m2 = best_distance2;

    count = beacon_config_get_count();
    for(i = 0U; i < count; i++)
    {
        float distance2;

        if(beacon_config_get_beacon(i, &beacon) == 0U)
        {
            continue;
        }

        distance2 = fixator_distance2(g_odometer.position[x],
                                      g_odometer.position[y],
                                      beacon.x,
                                      beacon.y);
        if(distance2 <= best_distance2)
        {
            best_distance2 = distance2;
            best_index = i;
            g_fixator.fixed_position[0] = beacon.x;
            g_fixator.fixed_position[1] = beacon.y;
        }
    }

    if(best_index != FIXATOR_NO_BEACON_INDEX)
    {
        g_fixator.pending_fix = 1U;
        g_fixator.last_match_valid = 1U;
        g_fixator.beacon_index = best_index;
        g_fixator.match_distance2_m2 = best_distance2;
        g_fixator.fix_count++;
    }
}

uint8 fixator_get_position_fix(float position[2])
{
    if((position == NULL) || (g_fixator.pending_fix == 0U))
    {
        return 0U;
    }

    position[0] = g_fixator.fixed_position[0];
    position[1] = g_fixator.fixed_position[1];
    g_fixator.pending_fix = 0U;

    return 1U;
}
