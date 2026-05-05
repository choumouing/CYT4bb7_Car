#include "wireless_control.h"

#include "../Attitude/IMU_TOP.h"
#include "../control/car_control.h"
#include "../menu/menu_config.h"
#include "zf_device_uart_receiver.h"

#define WIRELESS_CONTROL_PERIOD_SEC         (0.005f)

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

extern cascade_controller_t controller;

wireless_control_state_t g_wireless_control_state = {0};

static uint8_t g_frame_timeout_counter = WIRELESS_FRAME_TIMEOUT_CYCLES;
static uint8_t g_yaw_target_valid = 0U;

static uint16_t wireless_clamp_channel(uint16_t value, uint16_t min_value, uint16_t max_value);
static uint8_t wireless_channel_is_in_range(uint16_t value, uint16_t min_value, uint16_t max_value);
static uint8_t wireless_switch_is_enabled(uint16_t raw_value, uint16_t min_value, uint16_t max_value, uint16_t enabled_value);
static int16_t wireless_map_upper_side(uint16_t value, uint16_t start_value, uint16_t max_value);
static int16_t wireless_map_lower_side(uint16_t value, uint16_t min_value, uint16_t start_value);
static int16_t wireless_map_center_channel(uint16_t value,
                                           uint16_t min_value,
                                           uint16_t center_value,
                                           uint16_t max_value,
                                           int8_t lower_side_sign,
                                           int8_t upper_side_sign);
static float wireless_normalize_angle(float angle);
static void wireless_update_receiver_watchdog(void);
static void wireless_snapshot_channels(void);
static void wireless_clear_targets(void);
static void wireless_force_estop(void);
static void wireless_update_yaw_target(int16_t rotate_speed);
static void wireless_update_wheel_snapshot(void);

void wireless_control_init(void)
{
    wireless_force_estop();
}

void wireless_control_task(void)
{
    uint8_t ch5_enabled;
    uint8_t ch6_enabled;
    int16_t manual_rotate_speed;

    wireless_update_receiver_watchdog();
    wireless_snapshot_channels();

    g_wireless_control_state.receiver_online =
        ((1U == uart_receiver.state) && (g_frame_timeout_counter < WIRELESS_FRAME_TIMEOUT_CYCLES)) ? 1U : 0U;

    ch5_enabled = wireless_switch_is_enabled(g_wireless_control_state.raw_channel[WIRELESS_CH5],
                                             WIRELESS_CH5_UP_VALUE,
                                             WIRELESS_CH5_DOWN_VALUE,
                                             WIRELESS_CH5_DOWN_VALUE);
    ch6_enabled = wireless_switch_is_enabled(g_wireless_control_state.raw_channel[WIRELESS_CH6],
                                             WIRELESS_CH6_UP_VALUE,
                                             WIRELESS_CH6_DOWN_VALUE,
                                             WIRELESS_CH6_DOWN_VALUE);

    if ((!g_wireless_control_state.receiver_online) || (!ch5_enabled) || (!ch6_enabled))
    {
        wireless_force_estop();
        return;
    }

    g_wireless_control_state.control_enabled = 1U;
    g_wireless_control_state.emergency_stop_active = 0U;
    is_car_running = 1.0f;

    g_wireless_control_state.forward_speed = wireless_map_center_channel(
        g_wireless_control_state.clamped_channel[WIRELESS_CH2],
        WIRELESS_CH2_DOWN_LIMIT,
        WIRELESS_CH2_CENTER,
        WIRELESS_CH2_UP_LIMIT,
        -1,
        1);

    g_wireless_control_state.strafe_speed = wireless_map_center_channel(
        g_wireless_control_state.clamped_channel[WIRELESS_CH1],
        WIRELESS_CH1_LEFT_LIMIT,
        WIRELESS_CH1_CENTER,
        WIRELESS_CH1_RIGHT_LIMIT,
        1,
        -1);

    manual_rotate_speed = wireless_map_center_channel(
        g_wireless_control_state.clamped_channel[WIRELESS_CH4],
        WIRELESS_CH4_LEFT_LIMIT,
        WIRELESS_CH4_CENTER,
        WIRELESS_CH4_RIGHT_LIMIT,
        1,
        -1);
    g_wireless_control_state.rotate_speed = manual_rotate_speed;

    controller.vx_target = (float)g_wireless_control_state.forward_speed;
    controller.vy_target = (float)g_wireless_control_state.strafe_speed;
    wireless_update_yaw_target(manual_rotate_speed);
    wireless_update_wheel_snapshot();
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

static int16_t wireless_map_upper_side(uint16_t value, uint16_t start_value, uint16_t max_value)
{
    uint32_t numerator;
    uint32_t denominator;

    if ((value <= start_value) || (max_value <= start_value))
    {
        return 0;
    }

    numerator = (uint32_t)(value - start_value) * (uint32_t)MAX_CONTROL_SPEED;
    denominator = (uint32_t)(max_value - start_value);

    return (int16_t)(numerator / denominator);
}

static int16_t wireless_map_lower_side(uint16_t value, uint16_t min_value, uint16_t start_value)
{
    uint32_t numerator;
    uint32_t denominator;

    if ((value >= start_value) || (start_value <= min_value))
    {
        return 0;
    }

    numerator = (uint32_t)(start_value - value) * (uint32_t)MAX_CONTROL_SPEED;
    denominator = (uint32_t)(start_value - min_value);

    return (int16_t)(numerator / denominator);
}

static int16_t wireless_map_center_channel(uint16_t value,
                                           uint16_t min_value,
                                           uint16_t center_value,
                                           uint16_t max_value,
                                           int8_t lower_side_sign,
                                           int8_t upper_side_sign)
{
    uint16_t lower_deadzone_edge;
    uint16_t upper_deadzone_edge;
    int16_t mapped_speed;

    lower_deadzone_edge = center_value - WIRELESS_CENTER_DEADZONE;
    upper_deadzone_edge = center_value + WIRELESS_CENTER_DEADZONE;

    if (value < lower_deadzone_edge)
    {
        mapped_speed = wireless_map_lower_side(value, min_value, lower_deadzone_edge);
        return (int16_t)(mapped_speed * lower_side_sign);
    }

    if (value > upper_deadzone_edge)
    {
        mapped_speed = wireless_map_upper_side(value, upper_deadzone_edge, max_value);
        return (int16_t)(mapped_speed * upper_side_sign);
    }

    return 0;
}

static float wireless_normalize_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
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
    controller.vx_target = 0.0f;
    controller.vy_target = 0.0f;
    g_yaw_target_valid = 0U;

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
    g_wireless_control_state.forward_speed = 0;
    g_wireless_control_state.strafe_speed = 0;
    g_wireless_control_state.rotate_speed = 0;
    is_car_running = 0.0f;

    wireless_clear_targets();
    cascade_controller_reset(&controller);
}

static void wireless_update_yaw_target(int16_t rotate_speed)
{
    if (0U == g_yaw_target_valid)
    {
        cascade_set_target(&controller, -g_euler.yaw);
        g_yaw_target_valid = 1U;
    }

    if (0 != rotate_speed)
    {
        const float yaw_delta = (float)rotate_speed * WIRELESS_CONTROL_PERIOD_SEC;
        cascade_set_target(&controller, wireless_normalize_angle(controller.target_yaw + yaw_delta));
    }
}

static void wireless_update_wheel_snapshot(void)
{
    g_wireless_control_state.wheel_target[0] = (int16_t)controller.wheel_count_target[0];
    g_wireless_control_state.wheel_target[1] = (int16_t)controller.wheel_count_target[1];
    g_wireless_control_state.wheel_target[2] = (int16_t)controller.wheel_count_target[2];
    g_wireless_control_state.wheel_target[3] = (int16_t)controller.wheel_count_target[3];

    for (uint8_t index = 0U; index < WIRELESS_CONTROL_WHEEL_COUNT; index++)
    {
        g_wireless_control_state.wheel_pwm[index] = controller.pwm_output[index];
    }
}
