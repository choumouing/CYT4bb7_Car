#include "beacon_config.h"

#define BEACON_CONFIG_DEFAULT_COUNT (7U)
#define BEACON_CONFIG_ARRAY_SIZE(array) ((uint16)(sizeof(array) / sizeof((array)[0])))

typedef struct
{
    uint16 beacon_count;
    float initial_position[2];
} beacon_config_state_t;

/*
 * 比赛前在这里登记信标坐标。
 * 坐标含义：车体中心位于信标中心时的全局坐标，单位 m。
 * X 正方向为右侧。
 * Y 正方向为前方。
 * 发车区坐标作为 odometer 初始全局坐标。
 */
static const beacon_config_point_t s_default_beacons[] =
{
    {4.0f, 4.5f},
    {3.0f, 4.5f},
    {1.0f, 3.5f},
    {2.0f, 2.0f},
    {2.0f, 0.5f},
    {4.0f, 2.0f},
    {5.0f, 1.0f}
};

static const float s_default_initial_position[2] =
{
    6.0f,
    1.0f
};

static beacon_config_state_t s_beacon_config;

void beacon_config_init(void)
{
    beacon_config_reset();
}

void beacon_config_reset(void)
{
    uint16 count = BEACON_CONFIG_DEFAULT_COUNT;

    if(count > BEACON_CONFIG_ARRAY_SIZE(s_default_beacons))
    {
        count = BEACON_CONFIG_ARRAY_SIZE(s_default_beacons);
    }

    s_beacon_config.beacon_count = count;
    s_beacon_config.initial_position[0] = s_default_initial_position[0];
    s_beacon_config.initial_position[1] = s_default_initial_position[1];
}

uint16 beacon_config_get_count(void)
{
    return s_beacon_config.beacon_count;
}

uint8 beacon_config_get_beacon(uint16 index, beacon_config_point_t *beacon)
{
    if((beacon == NULL) || (index >= s_beacon_config.beacon_count))
    {
        return 0U;
    }

    *beacon = s_default_beacons[index];
    return 1U;
}

void beacon_config_get_initial_position(float position[2])
{
    if(position == NULL)
    {
        return;
    }

    position[0] = s_beacon_config.initial_position[0];
    position[1] = s_beacon_config.initial_position[1];
}
