#include "sbus.h"


#define SBUS_CENTER_DEADZONE            (50U)
#define SBUS_SWITCH_TOLERANCE           (50U)
#define SBUS_FRAME_TIMEOUT_MS           (100U)
#define SBUS_PERIOD_MS                  (40U)
#define SBUS_FRAME_TIMEOUT_CYCLES       ((SBUS_FRAME_TIMEOUT_MS + SBUS_PERIOD_MS - 1U) / SBUS_PERIOD_MS)

static const uint16 s_sbus_channel_min[SBUS_USED_CHANNEL_COUNT] =
{
    322U,
    204U,
    306U,
    306U,
    204U,
    240U
};

static const uint16 s_sbus_channel_center[SBUS_USED_CHANNEL_COUNT] =
{
    1028U,
    1025U,
    1040U,
    1028U,
    0U,
    0U
};

static const uint16 s_sbus_channel_max[SBUS_USED_CHANNEL_COUNT] =
{
    1807U,
    1805U,
    1777U,
    1754U,
    1807U,
    1807U
};

sbus_state_t g_sbus_state = {0};

static uint8 s_frame_timeout_counter = SBUS_FRAME_TIMEOUT_CYCLES;

static uint16 sbus_clamp_raw(uint16 value, uint16 min_value, uint16 max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

static int16 sbus_map_axis(uint16 raw_value,
                           uint16 min_value,
                           uint16 center_value,
                           uint16 max_value)
{
    uint16 value;
    uint16 lower_deadzone_edge;
    uint16 upper_deadzone_edge;
    int32 mapped;

    value = sbus_clamp_raw(raw_value, min_value, max_value);
    lower_deadzone_edge = center_value - SBUS_CENTER_DEADZONE;
    upper_deadzone_edge = center_value + SBUS_CENTER_DEADZONE;

    if(value < lower_deadzone_edge)
    {
        mapped = -((int32)(lower_deadzone_edge - value) * 1000) /
                 (int32)(lower_deadzone_edge - min_value);
        return (int16)mapped;
    }

    if(value > upper_deadzone_edge)
    {
        mapped = ((int32)(value - upper_deadzone_edge) * 1000) /
                 (int32)(max_value - upper_deadzone_edge);
        return (int16)mapped;
    }

    return 0;
}

static int16 sbus_map_switch_2pos(uint16 raw_value, uint16 up_value, uint16 down_value)
{
    int32 delta_up;
    int32 delta_down;

    delta_up = (int32)raw_value - (int32)up_value;
    delta_down = (int32)raw_value - (int32)down_value;

    if(delta_up < 0)
    {
        delta_up = -delta_up;
    }

    if(delta_down < 0)
    {
        delta_down = -delta_down;
    }

    if(delta_up <= (int32)SBUS_SWITCH_TOLERANCE)
    {
        return SBUS_STD_SWITCH_UP;
    }

    if(delta_down <= (int32)SBUS_SWITCH_TOLERANCE)
    {
        return SBUS_STD_SWITCH_DOWN;
    }

    return SBUS_STD_SWITCH_INVALID;
}

static void sbus_refresh_receiver_watchdog(void)
{
    if(1U == uart_receiver.finsh_flag)
    {
        s_frame_timeout_counter = 0U;
        g_sbus_state.frame_updated = 1U;
        uart_receiver.finsh_flag = 0U;
    }
    else
    {
        g_sbus_state.frame_updated = 0U;

        if(s_frame_timeout_counter < SBUS_FRAME_TIMEOUT_CYCLES)
        {
            s_frame_timeout_counter++;
        }
    }

    g_sbus_state.receiver_online =
        ((1U == uart_receiver.state) && (s_frame_timeout_counter < SBUS_FRAME_TIMEOUT_CYCLES)) ? 1U : 0U;
}

static void sbus_snapshot_channels(void)
{
    uint8 index;

    for(index = 0U; index < SBUS_CHANNEL_COUNT; index++)
    {
        g_sbus_state.raw_channel[index] = uart_receiver.channel[index];
        g_sbus_state.std_channel[index] = 0;
        g_sbus_state.channel_valid[index] = 0U;
    }

    g_sbus_state.std_channel[SBUS_CH1] =
        (int16)-sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH1],
                              s_sbus_channel_min[SBUS_CH1],
                              s_sbus_channel_center[SBUS_CH1],
                              s_sbus_channel_max[SBUS_CH1]);
    g_sbus_state.std_channel[SBUS_CH2] =
        sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH2],
                      s_sbus_channel_min[SBUS_CH2],
                      s_sbus_channel_center[SBUS_CH2],
                      s_sbus_channel_max[SBUS_CH2]);
    g_sbus_state.std_channel[SBUS_CH3] =
        sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH3],
                      s_sbus_channel_min[SBUS_CH3],
                      s_sbus_channel_center[SBUS_CH3],
                      s_sbus_channel_max[SBUS_CH3]);
    g_sbus_state.std_channel[SBUS_CH4] =
        sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH4],
                      s_sbus_channel_min[SBUS_CH4],
                      s_sbus_channel_center[SBUS_CH4],
                      s_sbus_channel_max[SBUS_CH4]);
    g_sbus_state.std_channel[SBUS_CH5] =
        sbus_map_switch_2pos(g_sbus_state.raw_channel[SBUS_CH5],
                             s_sbus_channel_min[SBUS_CH5],
                             s_sbus_channel_max[SBUS_CH5]);
    g_sbus_state.std_channel[SBUS_CH6] =
        sbus_map_switch_2pos(g_sbus_state.raw_channel[SBUS_CH6],
                             s_sbus_channel_min[SBUS_CH6],
                             s_sbus_channel_max[SBUS_CH6]);

    for(index = 0U; index < SBUS_USED_CHANNEL_COUNT; index++)
    {
        if((index == SBUS_CH5) || (index == SBUS_CH6))
        {
            g_sbus_state.channel_valid[index] =
                (SBUS_STD_SWITCH_INVALID != g_sbus_state.std_channel[index]) ? 1U : 0U;
        }
        else
        {
            g_sbus_state.channel_valid[index] = 1U;
        }
    }
}

void sbus_init(void)
{
    uint8 index;

    s_frame_timeout_counter = SBUS_FRAME_TIMEOUT_CYCLES;
    g_sbus_state.receiver_online = 0U;
    g_sbus_state.frame_updated = 0U;

    for(index = 0U; index < SBUS_CHANNEL_COUNT; index++)
    {
        g_sbus_state.raw_channel[index] = 0U;
        g_sbus_state.std_channel[index] = 0;
        g_sbus_state.channel_valid[index] = 0U;
    }
}

void sbus_update_25HZ(void)
{
    sbus_refresh_receiver_watchdog();
    sbus_snapshot_channels();
}

const sbus_state_t *sbus_get_state(void)
{
    return &g_sbus_state;
}
