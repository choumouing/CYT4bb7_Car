/* Mode2: image beacon tracking.
 *
 * The tracked beacon delta is expressed relative to the center camera image.
 * The car is allowed to move only when the center camera sees the long lamp
 * board near the center image point.
 */
#include "car_mode.h"
#include "car_loop.h"
#include "Common/car_math.h"
#include "Estimation/Image/beacon_fusion.h"

#define MODE2_CENTER_CAMERA_INDEX          (BEACON_FUSION_CAMERA_CENTER)
#define MODE2_CENTER_X                     (94.0f)
#define MODE2_CENTER_Y                     (60.0f)
#define MODE2_CAR_POSITION_WINDOW_HALF     (20.0f)
#define MODE2_IMAGE_PID_OUTPUT_DEADBAND    (0.0f)

car_mode2_state_t g_car_mode2_state = {0};

static PositionalPID s_mode2_forward_pid;
static PositionalPID s_mode2_strafe_pid;

static void car_mode2_pid_init(void)
{
    PositionalPID_Init(&s_mode2_forward_pid,
                       0.0f,
                       mode2_image_forward_kp,
                       mode2_image_forward_ki,
                       mode2_image_forward_kd,
                       mode2_image_i_limit,
                       mode2_image_output_limit);
    PositionalPID_Init(&s_mode2_strafe_pid,
                       0.0f,
                       mode2_image_strafe_kp,
                       mode2_image_strafe_ki,
                       mode2_image_strafe_kd,
                       mode2_image_i_limit,
                       mode2_image_output_limit);
}

static void car_mode2_pid_apply_params(void)
{
    s_mode2_forward_pid.kp_2 = 0.0f;
    s_mode2_forward_pid.kp_1 = mode2_image_forward_kp;
    s_mode2_forward_pid.ki = mode2_image_forward_ki;
    s_mode2_forward_pid.kd = mode2_image_forward_kd;
    s_mode2_forward_pid.i_limit = mode2_image_i_limit;
    s_mode2_forward_pid.output_limit = mode2_image_output_limit;

    s_mode2_strafe_pid.kp_2 = 0.0f;
    s_mode2_strafe_pid.kp_1 = mode2_image_strafe_kp;
    s_mode2_strafe_pid.ki = mode2_image_strafe_ki;
    s_mode2_strafe_pid.kd = mode2_image_strafe_kd;
    s_mode2_strafe_pid.i_limit = mode2_image_i_limit;
    s_mode2_strafe_pid.output_limit = mode2_image_output_limit;
}

static float car_mode2_apply_pid_output_deadband(float output)
{
    if(output == 0.0f)
    {
        return 0.0f;
    }

    if(car_math_absf(output) >= MODE2_IMAGE_PID_OUTPUT_DEADBAND)
    {
        return output;
    }

    return (output > 0.0f) ? MODE2_IMAGE_PID_OUTPUT_DEADBAND : -MODE2_IMAGE_PID_OUTPUT_DEADBAND;
}

static void car_mode2_clear_output(void)
{
    g_car_mode2_state.forward_pid_output = 0.0f;
    g_car_mode2_state.strafe_pid_output = 0.0f;
    g_car_mode2_state.forward_target = 0.0f;
    g_car_mode2_state.strafe_target = 0.0f;
    g_car_mode2_state.forward_pid_p_term = 0.0f;
    g_car_mode2_state.forward_pid_i_term = 0.0f;
    g_car_mode2_state.forward_pid_d_term = 0.0f;
    g_car_mode2_state.strafe_pid_p_term = 0.0f;
    g_car_mode2_state.strafe_pid_i_term = 0.0f;
    g_car_mode2_state.strafe_pid_d_term = 0.0f;
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
}

static uint8 car_mode2_car_position_allowed(void)
{
    volatile car_image_spi_car_lamp_t *lamp =
        &g_image_spi.board[MODE2_CENTER_CAMERA_INDEX].car_lamp;
    float dx;
    float dy;

    g_car_mode2_state.car_position_valid = lamp->valid;
    g_car_mode2_state.car_position_x = lamp->cx;
    g_car_mode2_state.car_position_y = lamp->cy;

    if(lamp->valid == 0U)
    {
        g_car_mode2_state.car_position_in_center_window = 0U;
        return 0U;
    }

    dx = lamp->cx - MODE2_CENTER_X;
    dy = lamp->cy - MODE2_CENTER_Y;
    if((car_math_absf(dx) <= MODE2_CAR_POSITION_WINDOW_HALF) &&
       (car_math_absf(dy) <= MODE2_CAR_POSITION_WINDOW_HALF))
    {
        g_car_mode2_state.car_position_in_center_window = 1U;
        return 1U;
    }

    g_car_mode2_state.car_position_in_center_window = 0U;
    return 0U;
}

void car_mode2_init(void)
{
    car_mode2_reset();
}

void car_mode2_reset(void)
{
    memset(&g_car_mode2_state, 0, sizeof(g_car_mode2_state));
    car_mode2_pid_init();
    car_mode2_clear_output();
    g_car_mode2_state.output_valid = 0U;
}

void car_mode2_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode2_update_100HZ(uint32 now_ms)
{
    uint8 car_position_allowed;
    float delta_x;
    float delta_y;

    (void)now_ms;

    car_mode2_pid_apply_params();

    g_car_mode2_state.target_valid =
        (g_beacon_fusion.valid != 0U) &&
        (g_beacon_fusion.center_delta_valid != 0U);
    g_car_mode2_state.target_delta_x = g_beacon_fusion.center_delta_x;
    g_car_mode2_state.target_delta_y = g_beacon_fusion.center_delta_y;
    car_position_allowed = car_mode2_car_position_allowed();

    if((g_car_mode2_state.target_valid == 0U) ||
       (car_position_allowed == 0U))
    {
        car_mode2_pid_init();
        car_mode2_clear_output();
        g_car_mode2_state.output_valid = 1U;
        return;
    }

    delta_x = car_math_deadband(g_car_mode2_state.target_delta_x, mode2_image_target_deadband_px);
    delta_y = car_math_deadband(g_car_mode2_state.target_delta_y, mode2_image_target_deadband_px);

    g_car_mode2_state.forward_pid_output =
        car_mode2_apply_pid_output_deadband(
            PositionalPID_Update(&s_mode2_forward_pid, 0.0f, delta_y));
    g_car_mode2_state.strafe_pid_output =
        car_mode2_apply_pid_output_deadband(
            PositionalPID_Update(&s_mode2_strafe_pid, 0.0f, delta_x));

    g_car_mode2_state.forward_target =
        car_math_limit_absf(g_car_mode2_state.forward_pid_output, mode2_image_output_limit);
    g_car_mode2_state.strafe_target =
        car_math_limit_absf(g_car_mode2_state.strafe_pid_output, mode2_image_output_limit);

    g_car_mode2_state.forward_pid_p_term = s_mode2_forward_pid.p_term;
    g_car_mode2_state.forward_pid_i_term = s_mode2_forward_pid.i_term;
    g_car_mode2_state.forward_pid_d_term = s_mode2_forward_pid.d_term;
    g_car_mode2_state.strafe_pid_p_term = s_mode2_strafe_pid.p_term;
    g_car_mode2_state.strafe_pid_i_term = s_mode2_strafe_pid.i_term;
    g_car_mode2_state.strafe_pid_d_term = s_mode2_strafe_pid.d_term;
    g_car_mode2_state.output_valid = 1U;

    car_forward_target = g_car_mode2_state.forward_target;
    car_strafe_target = g_car_mode2_state.strafe_target;
}
