/* Mode2: beacon tracking via AIR-side 3-camera fusion */
#include "car_mode.h"
#include "car_loop.h"
#include <math.h>
#include <string.h>

#define MODE2_CAR_LAMP_OFFSET_PX      (13.0f)
#define MODE2_DEG_TO_RAD              (0.017453292519943295f)
#define MODE2_CENTER_X                (94.0f)
#define MODE2_CENTER_Y                (60.0f)
#define MODE2_CAR_POSITION_SMALL_WINDOW_RADIUS (25.0f)
#define MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS (30.0f)
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

static float mode2_abs_limit(float value, float fallback)
{
    return (value < 0.0f) ? fallback : value;
}

static void mode2_pid_init(void)
{
    PositionalPID_Init(&s_mode2_forward_pid, 0.0f,
                       mode2_image_forward_kp, mode2_image_forward_ki,
                       mode2_image_forward_kd, mode2_image_i_limit,
                       mode2_image_pid_output_limit);
    PositionalPID_Init(&s_mode2_strafe_pid, 0.0f,
                       mode2_image_strafe_kp, mode2_image_strafe_ki,
                       mode2_image_strafe_kd, mode2_image_i_limit,
                       mode2_image_pid_output_limit);
}

static void mode2_pid_apply_params(void)
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

static void mode2_clear_output(void)
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
    g_car_mode2_state.limited_forward_mps = 0.0f;
    g_car_mode2_state.limited_strafe_mps = 0.0f;
    g_car_mode2_state.forward_command = 0.0f;
    g_car_mode2_state.strafe_command = 0.0f;
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
}

static void mode2_rotate_air_to_car(float air_x, float air_y, float angle_deg,
                                    float *car_x, float *car_y)
{
    float rad = angle_deg * MODE2_DEG_TO_RAD;
    float c = cosf(rad);
    float s = sinf(rad);

    *car_x = c * air_x + s * air_y;
    *car_y = -s * air_x + c * air_y;
}

static uint8 mode2_position_allowed(float camera_angle_deg)
{
    float ref_x, ref_y;
    float rad, nx, ny;
    float tx, ty, cx, cy;
    float tn2, cn2, dot, cos_a, range_r;

    if(g_air_mode2_car_lamp_valid <= 0.5f)
    {
        g_car_mode2_state.car_position_valid = 0U;
        g_car_mode2_state.car_position_x = g_air_mode2_car_lamp_cx;
        g_car_mode2_state.car_position_y = g_air_mode2_car_lamp_cy;
        g_car_mode2_state.car_position_in_center_window = 0U;
        return 0U;
    }

    rad = camera_angle_deg * MODE2_DEG_TO_RAD;
    nx = -sinf(rad);
    ny = cosf(rad);
    ref_x = g_air_mode2_car_lamp_cx + MODE2_CAR_LAMP_OFFSET_PX * nx;
    ref_y = g_air_mode2_car_lamp_cy + MODE2_CAR_LAMP_OFFSET_PX * ny;
    g_car_mode2_state.car_position_valid = 1U;
    g_car_mode2_state.car_position_x = ref_x;
    g_car_mode2_state.car_position_y = ref_y;

    if(g_air_mode2_target_valid <= 0.5f)
    {
        g_car_mode2_state.car_position_in_center_window = 0U;
        return 0U;
    }

    tx = g_air_mode2_target_x;
    ty = g_air_mode2_target_y;
    cx = MODE2_CENTER_X - ref_x;
    cy = MODE2_CENTER_Y - ref_y;
    tn2 = tx * tx + ty * ty;
    cn2 = cx * cx + cy * cy;

    if(cn2 <= MODE2_VECTOR_NORM2_MIN)
    {
        g_car_mode2_state.car_position_in_center_window = 1U;
        return 1U;
    }
    if(tn2 <= MODE2_VECTOR_NORM2_MIN)
    {
        g_car_mode2_state.car_position_in_center_window = 0U;
        return 0U;
    }

    dot = tx * cx + ty * cy;
    cos_a = dot / sqrtf(tn2 * cn2);
    if(cos_a > 1.0f) { cos_a = 1.0f; }
    if(cos_a < -1.0f) { cos_a = -1.0f; }

    if(cos_a >= MODE2_DIRECT_ALLOW_COS)
    {
        g_car_mode2_state.car_position_in_center_window =
            (cn2 <= MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS * MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS) ? 1U : 0U;
        return 1U;
    }

    range_r = (cos_a <= MODE2_REVERSE_SMALL_WINDOW_COS)
            ? MODE2_CAR_POSITION_SMALL_WINDOW_RADIUS
            : MODE2_CAR_POSITION_LARGE_WINDOW_RADIUS;
    g_car_mode2_state.car_position_in_center_window =
        (cn2 <= range_r * range_r) ? 1U : 0U;
    return g_car_mode2_state.car_position_in_center_window;
}

static float mode2_distance_speed(float dist, uint8 *zone)
{
    float mid_th, far_th;

    if(zone != NULL) { *zone = MODE2_ZONE_STOP; }
    if(dist <= mode2_image_target_deadband_px) { return 0.0f; }

    mid_th = mode2_abs_limit(mode2_distance_mid_threshold_px, 0.0f);
    far_th = mode2_abs_limit(mode2_distance_far_threshold_px, mid_th);
    if(far_th < mid_th) { far_th = mid_th; }

    if(dist > far_th)
    {
        if(zone != NULL) { *zone = MODE2_ZONE_FAR; }
        return mode2_abs_limit(mode2_distance_far_speed_mps, 0.0f);
    }
    if(dist > mid_th)
    {
        if(zone != NULL) { *zone = MODE2_ZONE_MID; }
        return mode2_abs_limit(mode2_distance_mid_speed_mps, 0.0f);
    }
    if(zone != NULL) { *zone = MODE2_ZONE_NEAR; }
    return mode2_abs_limit(mode2_distance_near_speed_mps, 0.0f);
}

static float mode2_accel_limit(float current, float target)
{
    float max_step = mode2_abs_limit(mode2_max_accel_mps2, 0.0f) * ODOMETER_UPDATE_DT_S;
    float ac = fabsf(current);
    float at = fabsf(target);
    float lim;

    if(current * target < 0.0f)
    {
        return (at > max_step) ? ((target > 0.0f) ? max_step : -max_step) : target;
    }
    if(at <= ac) { return target; }
    if((at - ac) > max_step)
    {
        lim = ac + max_step;
        return (target >= 0.0f) ? lim : -lim;
    }
    return target;
}

static void mode2_publish_command(void)
{
    g_car_mode2_state.limited_forward_mps =
        mode2_accel_limit(g_car_mode2_state.limited_forward_mps,
                          g_car_mode2_state.target_forward_mps);
    g_car_mode2_state.limited_strafe_mps =
        mode2_accel_limit(g_car_mode2_state.limited_strafe_mps,
                          g_car_mode2_state.target_strafe_mps);
    g_car_mode2_state.forward_command =
        g_car_mode2_state.limited_forward_mps *
        ODOMETER_FORWARD_COUNT_PER_METER * ODOMETER_UPDATE_DT_S;
    g_car_mode2_state.strafe_command =
        -g_car_mode2_state.limited_strafe_mps *
        ODOMETER_STRAFE_COUNT_PER_METER_ABS * ODOMETER_UPDATE_DT_S;
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
    mode2_pid_init();
    mode2_clear_output();
}

void car_mode2_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode2_update_100HZ(uint32 now_ms)
{
    float cam_angle, dx, dy, pid_norm;
    uint8 pos_ok;

    (void)now_ms;
    mode2_pid_apply_params();

    cam_angle = -g_air_euler_yaw;
    while(cam_angle >= 180.0f) { cam_angle -= 360.0f; }
    while(cam_angle < -180.0f) { cam_angle += 360.0f; }

    g_car_mode2_state.target_valid = (g_air_mode2_target_valid > 0.5f) ? 1U : 0U;
    mode2_rotate_air_to_car(g_air_mode2_target_x, g_air_mode2_target_y, cam_angle,
                            &g_car_mode2_state.target_delta_x,
                            &g_car_mode2_state.target_delta_y);
    pos_ok = mode2_position_allowed(cam_angle);
    g_car_mode2_state.feedback_forward_mps = g_odometer.vel[y];
    g_car_mode2_state.feedback_strafe_mps = g_odometer.vel[x];

    if((g_car_mode2_state.target_valid == 0U) || (pos_ok == 0U))
    {
        mode2_pid_init();
        g_car_mode2_state.distance_zone = MODE2_ZONE_STOP;
        g_car_mode2_state.target_distance_px = 0.0f;
        g_car_mode2_state.target_speed_mps = 0.0f;
        g_car_mode2_state.forward_pid_output = 0.0f;
        g_car_mode2_state.strafe_pid_output = 0.0f;
        g_car_mode2_state.forward_ratio = 0.0f;
        g_car_mode2_state.strafe_ratio = 0.0f;
        g_car_mode2_state.target_forward_mps = 0.0f;
        g_car_mode2_state.target_strafe_mps = 0.0f;
        mode2_publish_command();
        g_car_mode2_state.output_valid = 1U;
        return;
    }

    dx = g_car_mode2_state.target_delta_x;
    dy = g_car_mode2_state.target_delta_y;
    g_car_mode2_state.target_distance_px = sqrtf(dx * dx + dy * dy);
    g_car_mode2_state.target_speed_mps =
        mode2_distance_speed(g_car_mode2_state.target_distance_px,
                             &g_car_mode2_state.distance_zone);

    g_car_mode2_state.forward_pid_output =
        PositionalPID_Update(&s_mode2_forward_pid, 0.0f, dy);
    g_car_mode2_state.strafe_pid_output =
        PositionalPID_Update(&s_mode2_strafe_pid, 0.0f, dx);

    pid_norm = sqrtf(g_car_mode2_state.forward_pid_output *
                     g_car_mode2_state.forward_pid_output +
                     g_car_mode2_state.strafe_pid_output *
                     g_car_mode2_state.strafe_pid_output);

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
            g_car_mode2_state.target_speed_mps * g_car_mode2_state.forward_ratio;
        g_car_mode2_state.target_strafe_mps =
            g_car_mode2_state.target_speed_mps * g_car_mode2_state.strafe_ratio;
    }

    mode2_publish_command();
    g_car_mode2_state.output_valid = 1U;
}
