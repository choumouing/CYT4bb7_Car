/* Mode2: image beacon tracking driven by AIR-side fusion result. */
#include "car_mode.h"
#include "car_loop.h"

#include <math.h>
#include <string.h>

#define MODE2_CAR_LAMP_OFFSET_PX      (13.0f)
#define MODE2_DEG_TO_RAD              (0.017453292519943295f)
#define MODE2_CENTER_X                (94.0f)
#define MODE2_CENTER_Y                (60.0f)
#define MODE2_CAR_POSITION_SMALL_WINDOW_RADIUS (20.0f)
#define MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS (25.0f)
#define MODE2_DIRECT_ALLOW_COS        (0.70710678118f)
#define MODE2_REVERSE_SMALL_WINDOW_COS (-0.96592582629f)
#define MODE2_VECTOR_NORM2_MIN        (1.0e-6f)
#define MODE2_ZONE_STOP               (0U)
#define MODE2_ZONE_NEAR               (1U)
#define MODE2_ZONE_MID                (2U)
#define MODE2_ZONE_FAR                (3U)
#define MODE2_PID_VECTOR_NORM_MIN     (0.001f)

car_mode2_state_t g_car_mode2_state = {0};

static PositionalPID s_mode2_forward_pid;
static PositionalPID s_mode2_strafe_pid;

static float car_mode2_abs_limit_param(float value, float fallback)
{
    if(value < 0.0f)
    {
        return fallback;
    }

    return value;
}

static void car_mode2_pid_init(void)
{
    PositionalPID_Init(&s_mode2_forward_pid,
                       0.0f,
                       mode2_image_forward_kp,
                       mode2_image_forward_ki,
                       mode2_image_forward_kd,
                       mode2_image_i_limit,
                       mode2_image_pid_output_limit);
    PositionalPID_Init(&s_mode2_strafe_pid,
                       0.0f,
                       mode2_image_strafe_kp,
                       mode2_image_strafe_ki,
                       mode2_image_strafe_kd,
                       mode2_image_i_limit,
                       mode2_image_pid_output_limit);
}

static void car_mode2_pid_apply_params(void)
{
    s_mode2_forward_pid.kp_2 = 0.0f;
    s_mode2_forward_pid.kp_1 = mode2_image_forward_kp;
    s_mode2_forward_pid.ki = mode2_image_forward_ki;
    s_mode2_forward_pid.kd = mode2_image_forward_kd;
    s_mode2_forward_pid.i_limit = mode2_image_i_limit;
    s_mode2_forward_pid.output_limit = mode2_image_pid_output_limit;

    s_mode2_strafe_pid.kp_2 = 0.0f;
    s_mode2_strafe_pid.kp_1 = mode2_image_strafe_kp;
    s_mode2_strafe_pid.ki = mode2_image_strafe_ki;
    s_mode2_strafe_pid.kd = mode2_image_strafe_kd;
    s_mode2_strafe_pid.i_limit = mode2_image_i_limit;
    s_mode2_strafe_pid.output_limit = mode2_image_pid_output_limit;
}

static void car_mode2_clear_output(void)
{
    g_car_mode2_state.distance_zone = MODE2_ZONE_STOP;
    g_car_mode2_state.target_distance_px = 0.0f;
    g_car_mode2_state.target_speed_mps = 0.0f;
    g_car_mode2_state.forward_pid_output = 0.0f;
    g_car_mode2_state.strafe_pid_output = 0.0f;
    g_car_mode2_state.forward_ratio = 0.0f;
    g_car_mode2_state.strafe_ratio = 0.0f;
    g_car_mode2_state.target_forward_mps = 0.0f;
    g_car_mode2_state.target_strafe_mps = 0.0f;
    g_car_mode2_state.feedback_forward_mps = 0.0f;
    g_car_mode2_state.feedback_strafe_mps = 0.0f;
    g_car_mode2_state.limited_forward_mps = 0.0f;
    g_car_mode2_state.limited_strafe_mps = 0.0f;
    g_car_mode2_state.forward_command = 0.0f;
    g_car_mode2_state.strafe_command = 0.0f;
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
}

static void car_mode2_rotate_air_to_car(float air_x,
                                        float air_y,
                                        float angle_deg,
                                        float *car_x,
                                        float *car_y)
{
    const float angle_rad = angle_deg * MODE2_DEG_TO_RAD;
    const float cos_angle = cosf(angle_rad);
    const float sin_angle = sinf(angle_rad);

    if((car_x == NULL) || (car_y == NULL))
    {
        return;
    }

    *car_x = (cos_angle * air_x) + (sin_angle * air_y);
    *car_y = (-sin_angle * air_x) + (cos_angle * air_y);
}

static void car_mode2_lamp_ref_from_center(float cx,
                                           float cy,
                                           float angle_deg,
                                           float *ref_x,
                                           float *ref_y)
{
    const float angle_rad = angle_deg * MODE2_DEG_TO_RAD;
    const float normal_x = -sinf(angle_rad);
    const float normal_y = cosf(angle_rad);

    if((ref_x == NULL) || (ref_y == NULL))
    {
        return;
    }

    *ref_x = cx + (MODE2_CAR_LAMP_OFFSET_PX * normal_x);
    *ref_y = cy + (MODE2_CAR_LAMP_OFFSET_PX * normal_y);
}

static uint8 car_mode2_angle_position_allowed(float ref_x, float ref_y)
{
    float target_x;
    float target_y;
    float center_x;
    float center_y;
    float target_norm2;
    float center_norm2;
    float dot;
    float cos_angle;
    float range_radius;

    target_x = g_air_mode2_target_x;
    target_y = g_air_mode2_target_y;
    center_x = MODE2_CENTER_X - ref_x;
    center_y = MODE2_CENTER_Y - ref_y;
    target_norm2 = (target_x * target_x) + (target_y * target_y);
    center_norm2 = (center_x * center_x) + (center_y * center_y);

    if(center_norm2 <= MODE2_VECTOR_NORM2_MIN)
    {
        g_car_mode2_state.car_position_in_center_window = 1U;
        return 1U;
    }
    if(target_norm2 <= MODE2_VECTOR_NORM2_MIN)
    {
        g_car_mode2_state.car_position_in_center_window = 0U;
        return 0U;
    }

    dot = (target_x * center_x) + (target_y * center_y);
    cos_angle = dot / sqrtf(target_norm2 * center_norm2);
    if(cos_angle > 1.0f)
    {
        cos_angle = 1.0f;
    }
    else if(cos_angle < -1.0f)
    {
        cos_angle = -1.0f;
    }

    if(cos_angle >= MODE2_DIRECT_ALLOW_COS)
    {
        g_car_mode2_state.car_position_in_center_window =
            (center_norm2 <=
             (MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS *
              MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS)) ? 1U : 0U;
        return 1U;
    }

    range_radius = (cos_angle <= MODE2_REVERSE_SMALL_WINDOW_COS) ?
                   MODE2_CAR_POSITION_SMALL_WINDOW_RADIUS :
                   MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS;
    g_car_mode2_state.car_position_in_center_window =
        (center_norm2 <= (range_radius * range_radius)) ? 1U : 0U;
    return g_car_mode2_state.car_position_in_center_window;
}

static uint8 car_mode2_car_position_allowed(void)
{
    float ref_x;
    float ref_y;

    if(g_air_mode2_car_lamp_valid <= 0.5f)
    {
        g_car_mode2_state.car_position_valid = 0U;
        g_car_mode2_state.car_position_x = g_air_mode2_car_lamp_cx;
        g_car_mode2_state.car_position_y = g_air_mode2_car_lamp_cy;
        g_car_mode2_state.car_position_in_center_window = 0U;
        return 0U;
    }

    car_mode2_lamp_ref_from_center(g_air_mode2_car_lamp_cx,
                                   g_air_mode2_car_lamp_cy,
                                   g_air_mode2_lamp_angle_deg,
                                   &ref_x,
                                   &ref_y);
    g_car_mode2_state.car_position_valid = 1U;
    g_car_mode2_state.car_position_x = ref_x;
    g_car_mode2_state.car_position_y = ref_y;

    if(g_air_mode2_target_valid <= 0.5f)
    {
        g_car_mode2_state.car_position_in_center_window = 0U;
        return 0U;
    }

    return car_mode2_angle_position_allowed(ref_x, ref_y);
}

static float car_mode2_distance_speed(float distance_px, uint8 *zone)
{
    float mid_threshold;
    float far_threshold;
    float speed;

    if(zone != NULL)
    {
        *zone = MODE2_ZONE_STOP;
    }

    if(distance_px <= mode2_image_target_deadband_px)
    {
        return 0.0f;
    }

    mid_threshold = car_mode2_abs_limit_param(mode2_distance_mid_threshold_px, 0.0f);
    far_threshold = car_mode2_abs_limit_param(mode2_distance_far_threshold_px, mid_threshold);
    if(far_threshold < mid_threshold)
    {
        far_threshold = mid_threshold;
    }

    if(distance_px > far_threshold)
    {
        speed = car_mode2_abs_limit_param(mode2_distance_far_speed_mps, 0.0f);
        if(zone != NULL)
        {
            *zone = MODE2_ZONE_FAR;
        }
    }
    else if(distance_px > mid_threshold)
    {
        speed = car_mode2_abs_limit_param(mode2_distance_mid_speed_mps, 0.0f);
        if(zone != NULL)
        {
            *zone = MODE2_ZONE_MID;
        }
    }
    else
    {
        speed = car_mode2_abs_limit_param(mode2_distance_near_speed_mps, 0.0f);
        if(zone != NULL)
        {
            *zone = MODE2_ZONE_NEAR;
        }
    }

    return speed;
}

static float car_mode2_limit_speed_by_accel(float current_mps, float target_mps)
{
    float max_accel = mode2_max_accel_mps2;
    float max_step;
    float current_speed;
    float target_speed;
    float limited_speed;

    if(max_accel < 0.0f)
    {
        max_accel = 0.0f;
    }

    max_step = max_accel * ODOMETER_UPDATE_DT_S;
    current_speed = fabsf(current_mps);
    target_speed = fabsf(target_mps);

    if((current_mps * target_mps) < 0.0f)
    {
        if(target_speed > max_step)
        {
            return (target_mps > 0.0f) ? max_step : -max_step;
        }

        return target_mps;
    }

    if(target_speed <= current_speed)
    {
        return target_mps;
    }

    if((target_speed - current_speed) > max_step)
    {
        limited_speed = current_speed + max_step;
        return (target_mps >= 0.0f) ? limited_speed : -limited_speed;
    }

    return target_mps;
}

static void car_mode2_update_limited_speed(void)
{
    g_car_mode2_state.limited_forward_mps =
        car_mode2_limit_speed_by_accel(g_car_mode2_state.limited_forward_mps,
                                       g_car_mode2_state.target_forward_mps);
    g_car_mode2_state.limited_strafe_mps =
        car_mode2_limit_speed_by_accel(g_car_mode2_state.limited_strafe_mps,
                                       g_car_mode2_state.target_strafe_mps);
}

static void car_mode2_publish_command_from_mps(void)
{
    g_car_mode2_state.forward_command =
        g_car_mode2_state.limited_forward_mps *
        ODOMETER_FORWARD_COUNT_PER_METER *
        ODOMETER_UPDATE_DT_S;
    g_car_mode2_state.strafe_command =
        -g_car_mode2_state.limited_strafe_mps *
        ODOMETER_STRAFE_COUNT_PER_METER_ABS *
        ODOMETER_UPDATE_DT_S;

    car_forward_target = g_car_mode2_state.forward_command;
    car_strafe_target = g_car_mode2_state.strafe_command;
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
    float pid_norm;

    (void)now_ms;

    car_mode2_pid_apply_params();

    g_car_mode2_state.target_valid = (g_air_mode2_target_valid > 0.5f) ? 1U : 0U;
    car_mode2_rotate_air_to_car(g_air_mode2_target_x,
                                g_air_mode2_target_y,
                                g_air_mode2_lamp_angle_deg,
                                &g_car_mode2_state.target_delta_x,
                                &g_car_mode2_state.target_delta_y);
    car_position_allowed = car_mode2_car_position_allowed();
    g_car_mode2_state.feedback_forward_mps = g_odometer.vel[y];
    g_car_mode2_state.feedback_strafe_mps = g_odometer.vel[x];

    if((g_car_mode2_state.target_valid == 0U) ||
       (car_position_allowed == 0U))
    {
        car_mode2_pid_init();
        g_car_mode2_state.distance_zone = MODE2_ZONE_STOP;
        g_car_mode2_state.target_distance_px = 0.0f;
        g_car_mode2_state.target_speed_mps = 0.0f;
        g_car_mode2_state.forward_pid_output = 0.0f;
        g_car_mode2_state.strafe_pid_output = 0.0f;
        g_car_mode2_state.forward_ratio = 0.0f;
        g_car_mode2_state.strafe_ratio = 0.0f;
        g_car_mode2_state.target_forward_mps = 0.0f;
        g_car_mode2_state.target_strafe_mps = 0.0f;
        car_mode2_update_limited_speed();
        car_mode2_publish_command_from_mps();
        g_car_mode2_state.output_valid = 1U;
        return;
    }

    delta_x = g_car_mode2_state.target_delta_x;
    delta_y = g_car_mode2_state.target_delta_y;
    g_car_mode2_state.target_distance_px = sqrtf((delta_x * delta_x) + (delta_y * delta_y));

    g_car_mode2_state.target_speed_mps =
        car_mode2_distance_speed(g_car_mode2_state.target_distance_px,
                                 &g_car_mode2_state.distance_zone);

    g_car_mode2_state.forward_pid_output =
        PositionalPID_Update(&s_mode2_forward_pid, 0.0f, delta_y);
    g_car_mode2_state.strafe_pid_output =
        PositionalPID_Update(&s_mode2_strafe_pid, 0.0f, delta_x);

    pid_norm = sqrtf((g_car_mode2_state.forward_pid_output *
                      g_car_mode2_state.forward_pid_output) +
                     (g_car_mode2_state.strafe_pid_output *
                      g_car_mode2_state.strafe_pid_output));
    if((g_car_mode2_state.target_speed_mps <= 0.0f) ||
       (pid_norm <= MODE2_PID_VECTOR_NORM_MIN))
    {
        g_car_mode2_state.forward_ratio = 0.0f;
        g_car_mode2_state.strafe_ratio = 0.0f;
        g_car_mode2_state.target_forward_mps = 0.0f;
        g_car_mode2_state.target_strafe_mps = 0.0f;
    }
    else
    {
        g_car_mode2_state.forward_ratio =
            g_car_mode2_state.forward_pid_output / pid_norm;
        g_car_mode2_state.strafe_ratio =
            -g_car_mode2_state.strafe_pid_output / pid_norm;
        g_car_mode2_state.target_forward_mps =
            g_car_mode2_state.target_speed_mps *
            g_car_mode2_state.forward_ratio;
        g_car_mode2_state.target_strafe_mps =
            g_car_mode2_state.target_speed_mps *
            g_car_mode2_state.strafe_ratio;
    }

    car_mode2_update_limited_speed();
    car_mode2_publish_command_from_mps();
    g_car_mode2_state.output_valid = 1U;
}
