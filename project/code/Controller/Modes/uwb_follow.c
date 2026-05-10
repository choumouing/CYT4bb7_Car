#include "uwb_follow.h"

#include "Controller/PID/pid.h"
#include "Menu/menu_config.h"
#include "HW_Drivers/UWB/ALX_AOA.h"

uwb_follow_state_t g_uwb_follow_state = {0};

static PositionalPID s_uwb_follow_x_pid;
static PositionalPID s_uwb_follow_y_pid;

static float uwb_follow_deadband(float value, float deadband)
{
    if((value > -deadband) && (value < deadband))
    {
        return 0.0f;
    }

    return value;
}

static float uwb_follow_limit(float value, float limit)
{
    if(value > limit)
    {
        return limit;
    }

    if(value < -limit)
    {
        return -limit;
    }

    return value;
}

static void uwb_follow_apply_vector_limit(void)
{
    float abs_forward;
    float abs_strafe;
    float max_value;
    float scale;

    abs_forward = (g_uwb_follow_state.forward_target >= 0.0f) ?
                  g_uwb_follow_state.forward_target :
                  -g_uwb_follow_state.forward_target;
    abs_strafe = (g_uwb_follow_state.strafe_target >= 0.0f) ?
                 g_uwb_follow_state.strafe_target :
                 -g_uwb_follow_state.strafe_target;
    max_value = (abs_forward > abs_strafe) ? abs_forward : abs_strafe;

    if(max_value <= uwb_follow_output_limit)
    {
        return;
    }

    scale = uwb_follow_output_limit / max_value;
    g_uwb_follow_state.forward_target *= scale;
    g_uwb_follow_state.strafe_target *= scale;
}

static void uwb_follow_pid_init(void)
{
    PositionalPID_Init(&s_uwb_follow_x_pid,
                       0.0f,
                       uwb_follow_x_kp,
                       uwb_follow_x_ki,
                       uwb_follow_x_kd,
                       uwb_follow_i_limit,
                       uwb_follow_output_limit);
    PositionalPID_Init(&s_uwb_follow_y_pid,
                       0.0f,
                       uwb_follow_y_kp,
                       uwb_follow_y_ki,
                       uwb_follow_y_kd,
                       uwb_follow_i_limit,
                       uwb_follow_output_limit);
}

static void uwb_follow_pid_apply_params(void)
{
    s_uwb_follow_x_pid.kp_2 = 0.0f;
    s_uwb_follow_x_pid.kp_1 = uwb_follow_x_kp;
    s_uwb_follow_x_pid.ki = uwb_follow_x_ki;
    s_uwb_follow_x_pid.kd = uwb_follow_x_kd;
    s_uwb_follow_x_pid.i_limit = uwb_follow_i_limit;
    s_uwb_follow_x_pid.output_limit = uwb_follow_output_limit;

    s_uwb_follow_y_pid.kp_2 = 0.0f;
    s_uwb_follow_y_pid.kp_1 = uwb_follow_y_kp;
    s_uwb_follow_y_pid.ki = uwb_follow_y_ki;
    s_uwb_follow_y_pid.kd = uwb_follow_y_kd;
    s_uwb_follow_y_pid.i_limit = uwb_follow_i_limit;
    s_uwb_follow_y_pid.output_limit = uwb_follow_output_limit;
}

void uwb_follow_init(void)
{
    uwb_follow_reset();
}

void uwb_follow_reset(void)
{
    uwb_follow_pid_init();

    g_uwb_follow_state.error_x_cm = 0.0f;
    g_uwb_follow_state.error_y_cm = 0.0f;
    g_uwb_follow_state.x_pid_p_term = 0.0f;
    g_uwb_follow_state.x_pid_i_term = 0.0f;
    g_uwb_follow_state.x_pid_d_term = 0.0f;
    g_uwb_follow_state.y_pid_p_term = 0.0f;
    g_uwb_follow_state.y_pid_i_term = 0.0f;
    g_uwb_follow_state.y_pid_d_term = 0.0f;
    g_uwb_follow_state.raw_x_cm = 0.0f;
    g_uwb_follow_state.raw_y_cm = 0.0f;
    g_uwb_follow_state.filt_x_cm = 0.0f;
    g_uwb_follow_state.filt_y_cm = 0.0f;
    g_uwb_follow_state.forward_target = 0.0f;
    g_uwb_follow_state.strafe_target = 0.0f;
    g_uwb_follow_state.tag_online = 0U;
    g_uwb_follow_state.output_valid = 0U;
}

void uwb_follow_update(uint32 now_ms)
{
    ALX_AOA_Position_t position;

    g_uwb_follow_state.output_valid = 0U;
    g_uwb_follow_state.tag_online = ALX_AOA_IsTagOnline(now_ms, UWB_FOLLOW_TIMEOUT_MS);

    if((0U == g_uwb_follow_state.tag_online) ||
       (0U == ALX_AOA_GetLatest(&position)) ||
       (0U == ALX_AOA_GetFilteredXY(&g_uwb_follow_state.filt_x_cm,
                                    &g_uwb_follow_state.filt_y_cm)))
    {
        uwb_follow_reset();
        return;
    }

    uwb_follow_pid_apply_params();
    g_uwb_follow_state.raw_x_cm = (float)position.x_cm;
    g_uwb_follow_state.raw_y_cm = (float)position.y_cm;
    g_uwb_follow_state.error_x_cm =
        uwb_follow_deadband(g_uwb_follow_state.filt_x_cm, uwb_follow_deadband_x_cm);
    g_uwb_follow_state.error_y_cm =
        uwb_follow_deadband(g_uwb_follow_state.filt_y_cm, uwb_follow_deadband_y_cm);

    g_uwb_follow_state.strafe_target =
        -uwb_follow_limit(PositionalPID_Update(&s_uwb_follow_x_pid,
                                               g_uwb_follow_state.error_x_cm,
                                               0.0f),
                          uwb_follow_output_limit);
    g_uwb_follow_state.forward_target =
        uwb_follow_limit(PositionalPID_Update(&s_uwb_follow_y_pid,
                                              g_uwb_follow_state.error_y_cm,
                                              0.0f),
                         uwb_follow_output_limit);
    g_uwb_follow_state.x_pid_p_term = s_uwb_follow_x_pid.p_term;
    g_uwb_follow_state.x_pid_i_term = s_uwb_follow_x_pid.i_term;
    g_uwb_follow_state.x_pid_d_term = s_uwb_follow_x_pid.d_term;
    g_uwb_follow_state.y_pid_p_term = s_uwb_follow_y_pid.p_term;
    g_uwb_follow_state.y_pid_i_term = s_uwb_follow_y_pid.i_term;
    g_uwb_follow_state.y_pid_d_term = s_uwb_follow_y_pid.d_term;

    uwb_follow_apply_vector_limit();
    g_uwb_follow_state.output_valid = 1U;
}
