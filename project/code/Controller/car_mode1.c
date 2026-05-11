#include "car_mode.h"


car_mode1_state_t g_car_mode1_state = {0};

static PositionalPID s_car_mode1_x_pid;
static PositionalPID s_car_mode1_y_pid;

static float car_mode1_deadband(float value, float deadband)
{
    if((value > -deadband) && (value < deadband))
    {
        return 0.0f;
    }

    return value;
}

static float car_mode1_limit(float value, float limit)
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

static void car_mode1_apply_vector_limit(void)
{
    float abs_forward;
    float abs_strafe;
    float max_value;
    float scale;

    abs_forward = (g_car_mode1_state.forward_target >= 0.0f) ?
                  g_car_mode1_state.forward_target :
                  -g_car_mode1_state.forward_target;
    abs_strafe = (g_car_mode1_state.strafe_target >= 0.0f) ?
                 g_car_mode1_state.strafe_target :
                 -g_car_mode1_state.strafe_target;
    max_value = (abs_forward > abs_strafe) ? abs_forward : abs_strafe;

    if(max_value <= uwb_follow_output_limit)
    {
        return;
    }

    scale = uwb_follow_output_limit / max_value;
    g_car_mode1_state.forward_target *= scale;
    g_car_mode1_state.strafe_target *= scale;
}

static void car_mode1_pid_init(void)
{
    PositionalPID_Init(&s_car_mode1_x_pid,
                       0.0f,
                       uwb_follow_x_kp,
                       uwb_follow_x_ki,
                       uwb_follow_x_kd,
                       uwb_follow_i_limit,
                       uwb_follow_output_limit);
    PositionalPID_Init(&s_car_mode1_y_pid,
                       0.0f,
                       uwb_follow_y_kp,
                       uwb_follow_y_ki,
                       uwb_follow_y_kd,
                       uwb_follow_i_limit,
                       uwb_follow_output_limit);
}

static void car_mode1_pid_apply_params(void)
{
    s_car_mode1_x_pid.kp_2 = 0.0f;
    s_car_mode1_x_pid.kp_1 = uwb_follow_x_kp;
    s_car_mode1_x_pid.ki = uwb_follow_x_ki;
    s_car_mode1_x_pid.kd = uwb_follow_x_kd;
    s_car_mode1_x_pid.i_limit = uwb_follow_i_limit;
    s_car_mode1_x_pid.output_limit = uwb_follow_output_limit;

    s_car_mode1_y_pid.kp_2 = 0.0f;
    s_car_mode1_y_pid.kp_1 = uwb_follow_y_kp;
    s_car_mode1_y_pid.ki = uwb_follow_y_ki;
    s_car_mode1_y_pid.kd = uwb_follow_y_kd;
    s_car_mode1_y_pid.i_limit = uwb_follow_i_limit;
    s_car_mode1_y_pid.output_limit = uwb_follow_output_limit;
}

void car_mode1_init(void)
{
    car_mode1_reset();
}

void car_mode1_reset(void)
{
    car_mode1_pid_init();

    g_car_mode1_state.error_x_cm = 0.0f;
    g_car_mode1_state.error_y_cm = 0.0f;
    g_car_mode1_state.x_pid_p_term = 0.0f;
    g_car_mode1_state.x_pid_i_term = 0.0f;
    g_car_mode1_state.x_pid_d_term = 0.0f;
    g_car_mode1_state.y_pid_p_term = 0.0f;
    g_car_mode1_state.y_pid_i_term = 0.0f;
    g_car_mode1_state.y_pid_d_term = 0.0f;
    g_car_mode1_state.raw_x_cm = 0.0f;
    g_car_mode1_state.raw_y_cm = 0.0f;
    g_car_mode1_state.filt_x_cm = 0.0f;
    g_car_mode1_state.filt_y_cm = 0.0f;
    g_car_mode1_state.forward_target = 0.0f;
    g_car_mode1_state.strafe_target = 0.0f;
    g_car_mode1_state.tag_online = 0U;
    g_car_mode1_state.output_valid = 0U;
}

void car_mode1_update_25HZ(uint32 now_ms)
{
    ALX_AOA_Position_t position;

    g_car_mode1_state.output_valid = 0U;
    g_car_mode1_state.tag_online = ALX_AOA_IsTagOnline(now_ms, UWB_FOLLOW_TIMEOUT_MS);

    if((0U == g_car_mode1_state.tag_online) ||
       (0U == ALX_AOA_GetLatest(&position)) ||
       (0U == ALX_AOA_GetFilteredXY(&g_car_mode1_state.filt_x_cm,
                                    &g_car_mode1_state.filt_y_cm)))
    {
        car_mode1_reset();
        return;
    }

    car_mode1_pid_apply_params();
    g_car_mode1_state.raw_x_cm = (float)position.x_cm;
    g_car_mode1_state.raw_y_cm = (float)position.y_cm;
    g_car_mode1_state.error_x_cm =
        car_mode1_deadband(g_car_mode1_state.filt_x_cm, uwb_follow_deadband_x_cm);
    g_car_mode1_state.error_y_cm =
        car_mode1_deadband(g_car_mode1_state.filt_y_cm, uwb_follow_deadband_y_cm);

    g_car_mode1_state.strafe_target =
        -car_mode1_limit(PositionalPID_Update(&s_car_mode1_x_pid,
                                              g_car_mode1_state.error_x_cm,
                                              0.0f),
                         uwb_follow_output_limit);
    g_car_mode1_state.forward_target =
        car_mode1_limit(PositionalPID_Update(&s_car_mode1_y_pid,
                                             g_car_mode1_state.error_y_cm,
                                             0.0f),
                        uwb_follow_output_limit);
    g_car_mode1_state.x_pid_p_term = s_car_mode1_x_pid.p_term;
    g_car_mode1_state.x_pid_i_term = s_car_mode1_x_pid.i_term;
    g_car_mode1_state.x_pid_d_term = s_car_mode1_x_pid.d_term;
    g_car_mode1_state.y_pid_p_term = s_car_mode1_y_pid.p_term;
    g_car_mode1_state.y_pid_i_term = s_car_mode1_y_pid.i_term;
    g_car_mode1_state.y_pid_d_term = s_car_mode1_y_pid.d_term;

    car_mode1_apply_vector_limit();
    g_car_mode1_state.output_valid = 1U;
    car_forward_target = g_car_mode1_state.forward_target;
    car_strafe_target = g_car_mode1_state.strafe_target;
    car_rotate_target = 0.0f;
}
