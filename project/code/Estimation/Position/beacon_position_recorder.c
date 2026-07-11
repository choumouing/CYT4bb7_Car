#include "beacon_position_recorder.h"
#include "Controller/car_loop.h"

beacon_position_recorder_t g_beacon_position_recorder = {0};

static uint8 s_last_ch8_high;
static uint8 s_ch8_synced;

static uint8 beacon_position_recorder_ch8_high(void)
{
    return (g_air_crsf_std_ch8 > 0.5f) ? 1U : 0U;
}

static void beacon_position_recorder_record(void)
{
    uint16 index = g_beacon_position_recorder.point_count;

    if(index >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        g_beacon_position_recorder.full = 1U;
        return;
    }

    g_beacon_position_recorder.points[index][x] = g_beacon_position_recorder.position[x];
    g_beacon_position_recorder.points[index][y] = g_beacon_position_recorder.position[y];
    g_beacon_position_recorder.point_count++;
    if(g_beacon_position_recorder.point_count >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        g_beacon_position_recorder.full = 1U;
    }
}

void beacon_position_recorder_init(void)
{
    memset(&g_beacon_position_recorder, 0, sizeof(g_beacon_position_recorder));
    s_last_ch8_high = 0U;
    s_ch8_synced = 0U;
}

void beacon_position_recorder_enter(void)
{
    memset(&g_beacon_position_recorder, 0, sizeof(g_beacon_position_recorder));
    g_beacon_position_recorder.active = 1U;

    if(air_comm_car_is_run_data_fresh() != 0U)
    {
        s_last_ch8_high = beacon_position_recorder_ch8_high();
        s_ch8_synced = 1U;
    }
    else
    {
        s_last_ch8_high = 0U;
        s_ch8_synced = 0U;
    }
}

void beacon_position_recorder_exit(void)
{
    g_beacon_position_recorder.active = 0U;
    s_last_ch8_high = 0U;
    s_ch8_synced = 0U;
}

void beacon_position_recorder_update_100HZ(void)
{
    uint8 ch8_high;

    if(g_beacon_position_recorder.active == 0U)
    {
        return;
    }

    g_beacon_position_recorder.position[x] +=
        g_odometer.vel[x] * ODOMETER_UPDATE_DT_S;
    g_beacon_position_recorder.position[y] +=
        g_odometer.vel[y] * ODOMETER_UPDATE_DT_S;

    if(air_comm_car_is_run_data_fresh() == 0U)
    {
        s_ch8_synced = 0U;
        return;
    }

    ch8_high = beacon_position_recorder_ch8_high();
    if(s_ch8_synced == 0U)
    {
        s_last_ch8_high = ch8_high;
        s_ch8_synced = 1U;
        return;
    }

    if((ch8_high != 0U) && (s_last_ch8_high == 0U))
    {
        beacon_position_recorder_record();
    }

    s_last_ch8_high = ch8_high;
}
