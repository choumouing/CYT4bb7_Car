#include "wireless_control.h"

#include "zf_device_uart_receiver.h"

typedef enum
{
    WIRELESS_CH1 = 0,
    WIRELESS_CH2,
    WIRELESS_CH3,
    WIRELESS_CH4,
    WIRELESS_CH5,
    WIRELESS_CH6
} wireless_channel_index_t;

static const uint16_t g_channel_min[WIRELESS_CONTROL_CHANNEL_COUNT] =
{
    WIRELESS_CH1_LEFT_LIMIT,
    WIRELESS_CH2_DOWN_LIMIT,
    WIRELESS_CH3_DOWN_LIMIT,
    WIRELESS_CH4_LEFT_LIMIT,
    WIRELESS_CH5_UP_VALUE,
    WIRELESS_CH6_UP_VALUE
};

static const uint16_t g_channel_max[WIRELESS_CONTROL_CHANNEL_COUNT] =
{
    WIRELESS_CH1_RIGHT_LIMIT,
    WIRELESS_CH2_UP_LIMIT,
    WIRELESS_CH3_UP_LIMIT,
    WIRELESS_CH4_RIGHT_LIMIT,
    WIRELESS_CH5_DOWN_VALUE,
    WIRELESS_CH6_DOWN_VALUE
};

wireless_control_state_t g_wireless_control_state = {0};

static uint8_t g_frame_timeout_counter = WIRELESS_FRAME_TIMEOUT_CYCLES;

static uint16_t wireless_clamp_channel(uint16_t value, uint16_t min_value, uint16_t max_value);
static uint8_t wireless_channel_is_in_range(uint16_t value, uint16_t min_value, uint16_t max_value);
static uint8_t wireless_switch_is_enabled(uint16_t raw_value, uint16_t min_value, uint16_t max_value, uint16_t enabled_value);
static float wireless_map_upper_side(uint16_t value, uint16_t start_value, uint16_t max_value, float output_limit);
static float wireless_map_lower_side(uint16_t value, uint16_t min_value, uint16_t start_value, float output_limit);
static float wireless_map_center_channel(uint16_t value,
                                         uint16_t min_value,
                                         uint16_t center_value,
                                         uint16_t max_value,
                                         float output_limit,
                                         int8_t lower_side_sign,
                                         int8_t upper_side_sign);
static void wireless_update_receiver_watchdog(void);
static void wireless_snapshot_channels(void);
static void wireless_clear_targets(void);
static void wireless_force_estop(void);

void wireless_control_init(void)
{
    wireless_force_estop();
}

void wireless_control_task(void)
{
    uint8_t ch5_enabled;
    uint8_t ch6_remote_mode;
    uint8_t ch6_uwb_mode;
    float manual_rotate_speed;

    wireless_update_receiver_watchdog();
    wireless_snapshot_channels();

    g_wireless_control_state.receiver_online =
        ((1U == uart_receiver.state) && (g_frame_timeout_counter < WIRELESS_FRAME_TIMEOUT_CYCLES)) ? 1U : 0U;

    ch5_enabled = wireless_switch_is_enabled(g_wireless_control_state.raw_channel[WIRELESS_CH5],
                                             WIRELESS_CH5_UP_VALUE,
                                             WIRELESS_CH5_DOWN_VALUE,
                                             WIRELESS_CH5_DOWN_VALUE);
    ch6_remote_mode = wireless_switch_is_enabled(g_wireless_control_state.raw_channel[WIRELESS_CH6],
                                                 WIRELESS_CH6_UP_VALUE,
                                                 WIRELESS_CH6_DOWN_VALUE,
                                                 WIRELESS_CH6_UP_VALUE);
    ch6_uwb_mode = wireless_switch_is_enabled(g_wireless_control_state.raw_channel[WIRELESS_CH6],
                                              WIRELESS_CH6_UP_VALUE,
                                              WIRELESS_CH6_DOWN_VALUE,
                                              WIRELESS_CH6_DOWN_VALUE);

    if ((!g_wireless_control_state.receiver_online) || (!ch5_enabled) ||
        ((!ch6_remote_mode) && (!ch6_uwb_mode)))
    {
        wireless_force_estop();
        return;
    }

    g_wireless_control_state.control_enabled = 1U;
    g_wireless_control_state.emergency_stop_active = 0U;
    g_wireless_control_state.mode = ch6_uwb_mode ?
                                    WIRELESS_CONTROL_MODE_UWB_FOLLOW :
                                    WIRELESS_CONTROL_MODE_REMOTE;

    if(WIRELESS_CONTROL_MODE_UWB_FOLLOW == g_wireless_control_state.mode)
    {
        wireless_clear_targets();
        return;
    }

    g_wireless_control_state.forward_speed = (int16_t)wireless_map_center_channel(
        g_wireless_control_state.clamped_channel[WIRELESS_CH2],
        WIRELESS_CH2_DOWN_LIMIT,
        WIRELESS_CH2_CENTER,
        WIRELESS_CH2_UP_LIMIT,
        MAX_CONTROL_SPEED,
        -1,
        1);

    g_wireless_control_state.strafe_speed = (int16_t)wireless_map_center_channel(
        g_wireless_control_state.clamped_channel[WIRELESS_CH1],
        WIRELESS_CH1_LEFT_LIMIT,
        WIRELESS_CH1_CENTER,
        WIRELESS_CH1_RIGHT_LIMIT,
        MAX_CONTROL_SPEED,
        1,
        -1);

    manual_rotate_speed = wireless_map_center_channel(
        g_wireless_control_state.clamped_channel[WIRELESS_CH4],
        WIRELESS_CH4_LEFT_LIMIT,
        WIRELESS_CH4_CENTER,
        WIRELESS_CH4_RIGHT_LIMIT,
        MAX_ANGULAR_SPEED,
        -1,
        1);
    g_wireless_control_state.rotate_speed = manual_rotate_speed;
}

const wireless_control_state_t *wireless_control_get_state(void)
{
    return &g_wireless_control_state;
}

static uint16_t wireless_clamp_channel(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint8_t wireless_channel_is_in_range(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    return ((value >= min_value) && (value <= max_value)) ? 1U : 0U;
}

static uint8_t wireless_switch_is_enabled(uint16_t raw_value, uint16_t min_value, uint16_t max_value, uint16_t enabled_value)
{
    int32_t delta;

    if (!wireless_channel_is_in_range(raw_value, min_value, max_value))
    {
        return 0U;
    }

    delta = (int32_t)raw_value - (int32_t)enabled_value;
    if (delta < 0)
    {
        delta = -delta;
    }

    return (delta <= (int32_t)WIRELESS_SWITCH_TOLERANCE) ? 1U : 0U;
}

static float wireless_map_upper_side(uint16_t value, uint16_t start_value, uint16_t max_value, float output_limit)
{
    if ((value <= start_value) || (max_value <= start_value))
    {
        return 0.0f;
    }

    return ((float)(value - start_value) * output_limit) / (float)(max_value - start_value);
}

static float wireless_map_lower_side(uint16_t value, uint16_t min_value, uint16_t start_value, float output_limit)
{
    if ((value >= start_value) || (start_value <= min_value))
    {
        return 0.0f;
    }

    return ((float)(start_value - value) * output_limit) / (float)(start_value - min_value);
}

static float wireless_map_center_channel(uint16_t value,
                                         uint16_t min_value,
                                         uint16_t center_value,
                                         uint16_t max_value,
                                         float output_limit,
                                         int8_t lower_side_sign,
                                         int8_t upper_side_sign)
{
    uint16_t lower_deadzone_edge;
    uint16_t upper_deadzone_edge;
    float mapped_speed;

    lower_deadzone_edge = center_value - WIRELESS_CENTER_DEADZONE;
    upper_deadzone_edge = center_value + WIRELESS_CENTER_DEADZONE;

    if (value < lower_deadzone_edge)
    {
        mapped_speed = wireless_map_lower_side(value, min_value, lower_deadzone_edge, output_limit);
        return mapped_speed * (float)lower_side_sign;
    }

    if (value > upper_deadzone_edge)
    {
        mapped_speed = wireless_map_upper_side(value, upper_deadzone_edge, max_value, output_limit);
        return mapped_speed * (float)upper_side_sign;
    }

    return 0.0f;
}

static void wireless_update_receiver_watchdog(void)
{
    if (1U == uart_receiver.finsh_flag)
    {
        g_frame_timeout_counter = 0U;
        uart_receiver.finsh_flag = 0U;
    }
    else if (g_frame_timeout_counter < WIRELESS_FRAME_TIMEOUT_CYCLES)
    {
        g_frame_timeout_counter++;
    }
}

static void wireless_snapshot_channels(void)
{
    for (uint8_t index = 0U; index < WIRELESS_CONTROL_CHANNEL_COUNT; index++)
    {
        g_wireless_control_state.raw_channel[index] = uart_receiver.channel[index];
        g_wireless_control_state.clamped_channel[index] = wireless_clamp_channel(uart_receiver.channel[index],
                                                                                 g_channel_min[index],
                                                                                 g_channel_max[index]);
    }
}

static void wireless_clear_targets(void)
{
    g_wireless_control_state.forward_speed = 0;
    g_wireless_control_state.strafe_speed = 0;
    g_wireless_control_state.rotate_speed = 0.0f;

    for (uint8_t index = 0U; index < WIRELESS_CONTROL_WHEEL_COUNT; index++)
    {
        g_wireless_control_state.wheel_target[index] = 0;
        g_wireless_control_state.wheel_pwm[index] = 0;
    }
}

static void wireless_force_estop(void)
{
    g_wireless_control_state.control_enabled = 0U;
    g_wireless_control_state.emergency_stop_active = 1U;
    g_wireless_control_state.mode = WIRELESS_CONTROL_MODE_REMOTE;

    wireless_clear_targets();
}
