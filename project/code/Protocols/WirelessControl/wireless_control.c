#include "wireless_control.h"


wireless_control_state_t g_wireless_control_state = {0};

static float wireless_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float wireless_scale_axis_to_limit(int16 axis_value, float output_limit)
{
    return ((float)axis_value * output_limit) / 1000.0f;
}

static void wireless_clear_targets(void)
{
    uint8 index;

    g_wireless_control_state.forward_speed = 0;
    g_wireless_control_state.strafe_speed = 0;
    g_wireless_control_state.rotate_speed = 0.0f;

    for(index = 0U; index < WIRELESS_CONTROL_WHEEL_COUNT; index++)
    {
        g_wireless_control_state.wheel_target[index] = 0;
        g_wireless_control_state.wheel_pwm[index] = 0;
    }
}

static void wireless_force_estop(void)
{
    g_wireless_control_state.control_enabled = 0U;
    g_wireless_control_state.emergency_stop_active = 1U;
    g_wireless_control_state.remote_mode_requested = 0U;
    g_wireless_control_state.uwb_follow_requested = 0U;
    g_wireless_control_state.mode_request_valid = 0U;
    wireless_clear_targets();
}

static void wireless_snapshot_sbus(const sbus_state_t *sbus)
{
    uint8 index;

    for(index = 0U; index < WIRELESS_CONTROL_CHANNEL_COUNT; index++)
    {
        g_wireless_control_state.raw_channel[index] = sbus->raw_channel[index];
        g_wireless_control_state.std_channel[index] = sbus->std_channel[index];
    }
}

void wireless_control_init(void)
{
    wireless_force_estop();
}

void wireless_control_update_25HZ(void)
{
    const sbus_state_t *sbus;
    uint8 ch5_enabled;
    uint8 ch6_remote_mode;
    uint8 ch6_uwb_mode;
    float manual_rotate_speed;

    sbus = sbus_get_state();
    wireless_snapshot_sbus(sbus);
    g_wireless_control_state.receiver_online = sbus->receiver_online;

    ch5_enabled = ((0U != sbus->channel_valid[SBUS_CH5]) &&
                   (SBUS_STD_SWITCH_DOWN == sbus->std_channel[SBUS_CH5])) ? 1U : 0U;
    ch6_remote_mode = ((0U != sbus->channel_valid[SBUS_CH6]) &&
                       (SBUS_STD_SWITCH_UP == sbus->std_channel[SBUS_CH6])) ? 1U : 0U;
    ch6_uwb_mode = ((0U != sbus->channel_valid[SBUS_CH6]) &&
                    (SBUS_STD_SWITCH_DOWN == sbus->std_channel[SBUS_CH6])) ? 1U : 0U;

    if((0U == g_wireless_control_state.receiver_online) ||
       (0U == ch5_enabled) ||
       ((0U == ch6_remote_mode) && (0U == ch6_uwb_mode)))
    {
        wireless_force_estop();
        return;
    }

    g_wireless_control_state.control_enabled = 1U;
    g_wireless_control_state.emergency_stop_active = 0U;
    g_wireless_control_state.remote_mode_requested = ch6_remote_mode;
    g_wireless_control_state.uwb_follow_requested = ch6_uwb_mode;
    g_wireless_control_state.mode_request_valid = 1U;

    if(0U != ch6_uwb_mode)
    {
        wireless_clear_targets();
        return;
    }

    g_wireless_control_state.forward_speed =
        (int16)wireless_scale_axis_to_limit(sbus->std_channel[SBUS_CH2], (float)MAX_CONTROL_SPEED);
    g_wireless_control_state.strafe_speed =
        (int16)wireless_scale_axis_to_limit(sbus->std_channel[SBUS_CH1], (float)MAX_CONTROL_SPEED);
    manual_rotate_speed =
        wireless_scale_axis_to_limit(sbus->std_channel[SBUS_CH4], MAX_ANGULAR_SPEED);

    if(wireless_absf(manual_rotate_speed) < 0.001f)
    {
        manual_rotate_speed = 0.0f;
    }

    g_wireless_control_state.rotate_speed = manual_rotate_speed;
}

const wireless_control_state_t *wireless_control_get_state(void)
{
    return &g_wireless_control_state;
}
