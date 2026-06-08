/* Mode2: image beacon tracking.
 *
 * 信标追踪只在本模式内使用像素距离分段固定速度，速度单位统一为 m/s。
 * 进入底层 Control_100Hz 前再转换为现有麦轮编码器计数命令。
 */
#include "car_mode.h"
#include "car_loop.h"
#include "Common/car_math.h"
#include "Estimation/Image/beacon_fusion.h"
#include "Estimation/Position/odometer.h"

#define MODE2_CENTER_CAMERA_INDEX          (BEACON_FUSION_CAMERA_CENTER)
#define MODE2_CENTER_X                     (94.0f)
#define MODE2_CENTER_Y                     (60.0f)
#define MODE2_CAR_POSITION_WINDOW_HALF     (20.0f)

#define MODE2_ZONE_STOP                    (0U)
#define MODE2_ZONE_NEAR                    (1U)
#define MODE2_ZONE_MID                     (2U)
#define MODE2_ZONE_FAR                     (3U)

car_mode2_state_t g_car_mode2_state = {0};

static float car_mode2_abs_limit_param(float value, float fallback)
{
    if(value < 0.0f)
    {
        return fallback;
    }

    return value;
}

static void car_mode2_clear_output(void)
{
    g_car_mode2_state.forward_zone = MODE2_ZONE_STOP;
    g_car_mode2_state.strafe_zone = MODE2_ZONE_STOP;
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

static float car_mode2_segment_speed(float delta_px,
                                     float mid_threshold_px,
                                     float far_threshold_px,
                                     float near_speed_mps,
                                     float mid_speed_mps,
                                     float far_speed_mps,
                                     uint8 *zone)
{
    float abs_delta;
    float mid_threshold;
    float far_threshold;
    float speed;

    if(zone != NULL)
    {
        *zone = MODE2_ZONE_STOP;
    }

    if(car_math_absf(delta_px) <= mode2_image_target_deadband_px)
    {
        return 0.0f;
    }

    abs_delta = car_math_absf(delta_px);
    mid_threshold = car_mode2_abs_limit_param(mid_threshold_px, 0.0f);
    far_threshold = car_mode2_abs_limit_param(far_threshold_px, mid_threshold);
    if(far_threshold < mid_threshold)
    {
        far_threshold = mid_threshold;
    }

    if(abs_delta > far_threshold)
    {
        speed = car_mode2_abs_limit_param(far_speed_mps, 0.0f);
        if(zone != NULL)
        {
            *zone = MODE2_ZONE_FAR;
        }
    }
    else if(abs_delta > mid_threshold)
    {
        speed = car_mode2_abs_limit_param(mid_speed_mps, 0.0f);
        if(zone != NULL)
        {
            *zone = MODE2_ZONE_MID;
        }
    }
    else
    {
        speed = car_mode2_abs_limit_param(near_speed_mps, 0.0f);
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
    float error;

    if(max_accel < 0.0f)
    {
        max_accel = 0.0f;
    }

    max_step = max_accel * ODOMETER_UPDATE_DT_S;
    error = target_mps - current_mps;

    if(error > max_step)
    {
        return current_mps + max_step;
    }
    if(error < -max_step)
    {
        return current_mps - max_step;
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
    float forward_speed_mps;
    float strafe_speed_mps;

    (void)now_ms;

    g_car_mode2_state.target_valid =
        (g_beacon_fusion.valid != 0U) &&
        (g_beacon_fusion.center_delta_valid != 0U);
    g_car_mode2_state.target_delta_x = g_beacon_fusion.center_delta_x;
    g_car_mode2_state.target_delta_y = g_beacon_fusion.center_delta_y;
    car_position_allowed = car_mode2_car_position_allowed();
    g_car_mode2_state.feedback_forward_mps = g_odometer.vel[y];
    g_car_mode2_state.feedback_strafe_mps = g_odometer.vel[x];

    if((g_car_mode2_state.target_valid == 0U) ||
       (car_position_allowed == 0U))
    {
        g_car_mode2_state.forward_zone = MODE2_ZONE_STOP;
        g_car_mode2_state.strafe_zone = MODE2_ZONE_STOP;
        g_car_mode2_state.target_forward_mps = 0.0f;
        g_car_mode2_state.target_strafe_mps = 0.0f;
        car_mode2_update_limited_speed();
        car_mode2_publish_command_from_mps();
        g_car_mode2_state.output_valid = 1U;
        return;
    }

    delta_x = car_math_deadband(g_car_mode2_state.target_delta_x, mode2_image_target_deadband_px);
    delta_y = car_math_deadband(g_car_mode2_state.target_delta_y, mode2_image_target_deadband_px);

    forward_speed_mps =
        car_mode2_segment_speed(delta_y,
                                mode2_forward_mid_threshold_px,
                                mode2_forward_far_threshold_px,
                                mode2_forward_near_speed_mps,
                                mode2_forward_mid_speed_mps,
                                mode2_forward_far_speed_mps,
                                &g_car_mode2_state.forward_zone);
    strafe_speed_mps =
        car_mode2_segment_speed(delta_x,
                                mode2_strafe_mid_threshold_px,
                                mode2_strafe_far_threshold_px,
                                mode2_strafe_near_speed_mps,
                                mode2_strafe_mid_speed_mps,
                                mode2_strafe_far_speed_mps,
                                &g_car_mode2_state.strafe_zone);

    g_car_mode2_state.target_forward_mps =
        (delta_y > 0.0f) ? -forward_speed_mps : forward_speed_mps;
    g_car_mode2_state.target_strafe_mps =
        (delta_x > 0.0f) ? strafe_speed_mps : -strafe_speed_mps;

    if(delta_y == 0.0f)
    {
        g_car_mode2_state.target_forward_mps = 0.0f;
    }
    if(delta_x == 0.0f)
    {
        g_car_mode2_state.target_strafe_mps = 0.0f;
    }

    car_mode2_update_limited_speed();
    car_mode2_publish_command_from_mps();
    g_car_mode2_state.output_valid = 1U;
}
