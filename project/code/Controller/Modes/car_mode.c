#include "car_mode.h"


car_mode_state_t g_car_mode_state = {0};

static float s_yaw_angle_target = 0.0f;
static uint8_t s_last_rotate_active = 0U;
static uint8_t s_yaw_angle_hold_active = 0U;

static void car_mode_clear_targets(void)
{
    g_car_mode_state.forward_target = 0.0f;
    g_car_mode_state.strafe_target = 0.0f;
    g_car_mode_state.rotate_target = 0.0f;
}

static void car_mode_reset_yaw_hold(void)
{
    s_last_rotate_active = 0U;
    s_yaw_angle_hold_active = 0U;
    s_yaw_angle_target = control_get_current_yaw_angle();
}

static void car_mode_update_yaw_hold_25HZ(void)
{
    if(0.0f != g_car_mode_state.rotate_target)
    {
        s_yaw_angle_target = control_get_current_yaw_angle();
        s_yaw_angle_hold_active = 0U;
    }
    else
    {
        if((0U == s_yaw_angle_hold_active) || (0U != s_last_rotate_active))
        {
            s_yaw_angle_target = control_get_current_yaw_angle();
            control_yaw_angle_loop_reset();
            s_yaw_angle_hold_active = 1U;
        }
        control_yaw_angle_loop_update_25HZ(s_yaw_angle_target);
    }

    s_last_rotate_active = (0.0f != g_car_mode_state.rotate_target) ? 1U : 0U;
}

void car_mode_init(void)
{
    car_mode_reset();
}

void car_mode_reset(void)
{
    g_car_mode_state.mode = CAR_MODE_REMOTE;
    g_car_mode_state.last_mode = CAR_MODE_REMOTE;
    g_car_mode_state.mode_changed = 0U;
    g_car_mode_state.control_enabled = 0U;
    g_car_mode_state.emergency_stop_active = 1U;
    car_mode_clear_targets();
    car_mode_reset_yaw_hold();
}

uint8_t car_mode_get(void)
{
    return g_car_mode_state.mode;
}

void car_mode_update_25HZ(uint32 now_ms)
{
    const wireless_control_state_t *remote;
    uint8_t next_mode;

    remote = wireless_control_get_state();
    next_mode = CAR_MODE_REMOTE;

    if((0U != remote->control_enabled) &&
       (0U != remote->mode_request_valid) &&
       (0U != remote->uwb_follow_requested))
    {
        next_mode = CAR_MODE_UWB_FOLLOW;
    }

    g_car_mode_state.last_mode = g_car_mode_state.mode;
    g_car_mode_state.mode = next_mode;
    g_car_mode_state.mode_changed =
        (g_car_mode_state.last_mode != g_car_mode_state.mode) ? 1U : 0U;
    g_car_mode_state.control_enabled = remote->control_enabled;
    g_car_mode_state.emergency_stop_active = remote->emergency_stop_active;

    if(0U != g_car_mode_state.mode_changed)
    {
        control_cascade_reset();
        uwb_follow_reset();
        target_follow_reset();
        car_mode_reset_yaw_hold();
    }

    if(0U == remote->control_enabled)
    {
        car_mode_clear_targets();
        uwb_follow_reset();
        target_follow_reset();
        car_mode_reset_yaw_hold();
        return;
    }

    if(CAR_MODE_UWB_FOLLOW == g_car_mode_state.mode)
    {
        target_follow_update_25HZ(now_ms);
        g_car_mode_state.forward_target = (0U != g_target_follow_state.output_valid) ?
                                          g_target_follow_state.forward_target : 0.0f;
        g_car_mode_state.strafe_target = (0U != g_target_follow_state.output_valid) ?
                                         g_target_follow_state.strafe_target : 0.0f;
        g_car_mode_state.rotate_target = 0.0f;
        car_mode_update_yaw_hold_25HZ();
        return;
    }

    g_car_mode_state.forward_target = (float)remote->forward_speed;
    g_car_mode_state.strafe_target = (float)remote->strafe_speed;
    g_car_mode_state.rotate_target = remote->rotate_speed;
    car_mode_update_yaw_hold_25HZ();
}
