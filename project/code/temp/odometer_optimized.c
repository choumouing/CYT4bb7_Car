#include "odometer_optimized.h"


#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_STARTUP_HOLD_TICKS         (50U)
#define ODOMETER_FORWARD_COUNT_PER_METER    (11287.0f)
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (12100.0f)

#define ODOMETER_DEG_TO_RAD                 (0.017453292519943295f)
#define ODOMETER_PI                         (3.14159265358979323846f)
#define ODOMETER_TWO_PI                     (6.28318530717958647692f)

/* 保守版离线标定参数: 带符号轴向尺度 + 速度段补偿。 */
#define ODOMETER_YAW_INTEGRAL_GAIN          (-0.45f)
#define ODOMETER_SPEED_BREAK_MPS            (0.40f)
#define ODOMETER_SPEED_BLEND_RANGE_MPS      (1.20f)

#define ODOMETER_FWD_POS_GAIN               (0.89704637f)
#define ODOMETER_FWD_NEG_GAIN               (0.98483632f)
#define ODOMETER_FWD_STRAFE_POS_COUPLE      (-0.01096042f)
#define ODOMETER_FWD_STRAFE_NEG_COUPLE      (-0.03801268f)
#define ODOMETER_FWD_POS_FAST_GAIN          (0.03226033f)
#define ODOMETER_FWD_NEG_FAST_GAIN          (-0.22742763f)
#define ODOMETER_FWD_STRAFE_POS_FAST_COUPLE (0.21097528f)
#define ODOMETER_FWD_STRAFE_NEG_FAST_COUPLE (0.21411205f)

#define ODOMETER_STRAFE_FWD_POS_COUPLE      (0.08807285f)
#define ODOMETER_STRAFE_FWD_NEG_COUPLE      (-0.14035020f)
#define ODOMETER_STRAFE_POS_GAIN            (0.80397033f)
#define ODOMETER_STRAFE_NEG_GAIN            (1.03638767f)
#define ODOMETER_STRAFE_FWD_POS_FAST_COUPLE (-0.23835485f)
#define ODOMETER_STRAFE_FWD_NEG_FAST_COUPLE (0.17059638f)
#define ODOMETER_STRAFE_POS_FAST_GAIN       (0.13346644f)
#define ODOMETER_STRAFE_NEG_FAST_GAIN       (-0.22465882f)

#define ODOMETER_STILL_SPEED_MPS            (0.020f)
#define ODOMETER_STILL_GYRO_DPS             (2.50f)
#define ODOMETER_STILL_ACCEL_NORM_MIN_G     (0.96f)
#define ODOMETER_STILL_ACCEL_NORM_MAX_G     (1.04f)
#define ODOMETER_STILL_HOLD_TICKS           (20U)

typedef struct
{
    float forward;
    float strafe;
} odometer_vec2_t;

typedef struct
{
    float yaw_delta_rad;
    float gyro_sum_dps[3];
    uint16_t gyro_sample_count;
    uint16_t startup_hold_ticks;
    uint16_t still_ticks;
} odometer_filter_state_t;

odometer_data_t g_odometer = {0.0f, 0.0f, 0.0f};

static odometer_filter_state_t g_odometer_filter;

static float odometer_vec_norm(odometer_vec2_t value)
{
    return sqrtf((value.forward * value.forward) + (value.strafe * value.strafe));
}

static float odometer_vec3_norm(float x, float y, float z)
{
    return sqrtf((x * x) + (y * y) + (z * z));
}

static float odometer_normalize_angle(float angle)
{
    while(angle > ODOMETER_PI)
    {
        angle -= ODOMETER_TWO_PI;
    }

    while(angle < -ODOMETER_PI)
    {
        angle += ODOMETER_TWO_PI;
    }

    return angle;
}

static float odometer_positive_part(float value)
{
    return (value > 0.0f) ? value : 0.0f;
}

static float odometer_negative_part(float value)
{
    return (value < 0.0f) ? value : 0.0f;
}

static float odometer_speed_weight(float speed_abs_mps)
{
    return car_math_clampf((speed_abs_mps - ODOMETER_SPEED_BREAK_MPS) /
                           ODOMETER_SPEED_BLEND_RANGE_MPS,
                           0.0f,
                           1.0f);
}

static odometer_vec2_t odometer_get_encoder_delta_count(void)
{
    odometer_vec2_t count;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    count.forward = (left_front + right_front + left_rear + right_rear) * 0.25f;
    count.strafe = (-left_front + right_front + left_rear - right_rear) * 0.25f;
    return count;
}

static odometer_vec2_t odometer_get_encoder_velocity(odometer_vec2_t delta_count)
{
    odometer_vec2_t velocity;

    velocity.forward = delta_count.forward / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S;
    velocity.strafe = delta_count.strafe / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S;
    return velocity;
}

static odometer_vec2_t odometer_compensate_encoder_velocity(odometer_vec2_t velocity)
{
    odometer_vec2_t compensated;
    float fp;
    float fn;
    float sp;
    float sn;
    float fw;
    float sw;

    fp = odometer_positive_part(velocity.forward);
    fn = odometer_negative_part(velocity.forward);
    sp = odometer_positive_part(velocity.strafe);
    sn = odometer_negative_part(velocity.strafe);
    fw = odometer_speed_weight(car_math_absf(velocity.forward));
    sw = odometer_speed_weight(car_math_absf(velocity.strafe));

    compensated.forward =
        (ODOMETER_FWD_POS_GAIN * fp) +
        (ODOMETER_FWD_NEG_GAIN * fn) +
        (ODOMETER_FWD_STRAFE_POS_COUPLE * sp) +
        (ODOMETER_FWD_STRAFE_NEG_COUPLE * sn) +
        (ODOMETER_FWD_POS_FAST_GAIN * fp * fw) +
        (ODOMETER_FWD_NEG_FAST_GAIN * fn * fw) +
        (ODOMETER_FWD_STRAFE_POS_FAST_COUPLE * sp * sw) +
        (ODOMETER_FWD_STRAFE_NEG_FAST_COUPLE * sn * sw);

    compensated.strafe =
        (ODOMETER_STRAFE_FWD_POS_COUPLE * fp) +
        (ODOMETER_STRAFE_FWD_NEG_COUPLE * fn) +
        (ODOMETER_STRAFE_POS_GAIN * sp) +
        (ODOMETER_STRAFE_NEG_GAIN * sn) +
        (ODOMETER_STRAFE_FWD_POS_FAST_COUPLE * fp * fw) +
        (ODOMETER_STRAFE_FWD_NEG_FAST_COUPLE * fn * fw) +
        (ODOMETER_STRAFE_POS_FAST_GAIN * sp * sw) +
        (ODOMETER_STRAFE_NEG_FAST_GAIN * sn * sw);

    return compensated;
}

static void odometer_clear_gyro_average(void)
{
    g_odometer_filter.gyro_sum_dps[0] = 0.0f;
    g_odometer_filter.gyro_sum_dps[1] = 0.0f;
    g_odometer_filter.gyro_sum_dps[2] = 0.0f;
    g_odometer_filter.gyro_sample_count = 0U;
}

static void odometer_get_gyro_average_dps(float gyro_avg_dps[3])
{
    float inv_count;

    if((gyro_avg_dps == 0) || (0U == g_odometer_filter.gyro_sample_count))
    {
        if(gyro_avg_dps != 0)
        {
            gyro_avg_dps[0] = 0.0f;
            gyro_avg_dps[1] = 0.0f;
            gyro_avg_dps[2] = 0.0f;
        }
        return;
    }

    inv_count = 1.0f / (float)g_odometer_filter.gyro_sample_count;
    gyro_avg_dps[0] = g_odometer_filter.gyro_sum_dps[0] * inv_count;
    gyro_avg_dps[1] = g_odometer_filter.gyro_sum_dps[1] * inv_count;
    gyro_avg_dps[2] = g_odometer_filter.gyro_sum_dps[2] * inv_count;
}

static uint8_t odometer_is_still(odometer_vec2_t raw_velocity, float gyro_avg_dps[3])
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float speed_norm;
    float gyro_norm;
    float accel_norm;

    accel_x_g = 0.0f;
    accel_y_g = 0.0f;
    accel_z_g = -1.0f;
    AccelCalibration_GetCorrectedSpecificForceG(&accel_x_g, &accel_y_g, &accel_z_g);

    speed_norm = odometer_vec_norm(raw_velocity);
    gyro_norm = odometer_vec3_norm(gyro_avg_dps[0], gyro_avg_dps[1], gyro_avg_dps[2]);
    accel_norm = odometer_vec3_norm(accel_x_g, accel_y_g, accel_z_g);

    return ((speed_norm < ODOMETER_STILL_SPEED_MPS) &&
            (gyro_norm < ODOMETER_STILL_GYRO_DPS) &&
            (accel_norm >= ODOMETER_STILL_ACCEL_NORM_MIN_G) &&
            (accel_norm <= ODOMETER_STILL_ACCEL_NORM_MAX_G)) ? 1U : 0U;
}

void odometer_init(void)
{
    odometer_reset();
}

void odometer_reset(void)
{
    g_odometer.forward_distance = 0.0f;
    g_odometer.strafe_distance = 0.0f;
    g_odometer.travel_distance = 0.0f;

    g_odometer_filter.yaw_delta_rad = 0.0f;
    g_odometer_filter.startup_hold_ticks = ODOMETER_STARTUP_HOLD_TICKS;
    g_odometer_filter.still_ticks = 0U;
    odometer_clear_gyro_average();
}

void odometer_update_1000HZ(void)
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    g_odometer_filter.gyro_sum_dps[0] += gyro_x_dps;
    g_odometer_filter.gyro_sum_dps[1] += gyro_y_dps;
    g_odometer_filter.gyro_sum_dps[2] += gyro_z_dps;
    if(g_odometer_filter.gyro_sample_count < 1000U)
    {
        g_odometer_filter.gyro_sample_count++;
    }
}

void odometer_update_100HZ(void)
{
    odometer_vec2_t delta_count;
    odometer_vec2_t raw_velocity;
    odometer_vec2_t compensated_velocity;
    odometer_vec2_t distance_delta;
    float gyro_avg_dps[3];
    float yaw_step_rad;
    float yaw_for_projection;
    float cos_yaw;
    float sin_yaw;

    delta_count = odometer_get_encoder_delta_count();
    raw_velocity = odometer_get_encoder_velocity(delta_count);

    odometer_get_gyro_average_dps(gyro_avg_dps);
    odometer_clear_gyro_average();

    compensated_velocity = odometer_compensate_encoder_velocity(raw_velocity);

    if(g_odometer_filter.startup_hold_ticks > 0U)
    {
        g_odometer.forward_distance = 0.0f;
        g_odometer.strafe_distance = 0.0f;
        g_odometer.travel_distance = 0.0f;
        g_odometer_filter.yaw_delta_rad = 0.0f;
        g_odometer_filter.still_ticks = 0U;
        g_odometer_filter.startup_hold_ticks--;
        return;
    }

    if(odometer_is_still(raw_velocity, gyro_avg_dps))
    {
        if(g_odometer_filter.still_ticks < ODOMETER_STILL_HOLD_TICKS)
        {
            g_odometer_filter.still_ticks++;
        }
    }
    else
    {
        g_odometer_filter.still_ticks = 0U;
    }

    if(g_odometer_filter.still_ticks >= ODOMETER_STILL_HOLD_TICKS)
    {
        compensated_velocity.forward = 0.0f;
        compensated_velocity.strafe = 0.0f;
    }

    yaw_step_rad = gyro_avg_dps[2] * ODOMETER_DEG_TO_RAD * ODOMETER_UPDATE_DT_S;
    g_odometer_filter.yaw_delta_rad =
        odometer_normalize_angle(g_odometer_filter.yaw_delta_rad + yaw_step_rad);

    yaw_for_projection = g_odometer_filter.yaw_delta_rad * ODOMETER_YAW_INTEGRAL_GAIN;
    cos_yaw = cosf(yaw_for_projection);
    sin_yaw = sinf(yaw_for_projection);
    distance_delta.forward = ((cos_yaw * compensated_velocity.forward) -
                              (sin_yaw * compensated_velocity.strafe)) *
                             ODOMETER_UPDATE_DT_S;
    distance_delta.strafe = ((sin_yaw * compensated_velocity.forward) +
                             (cos_yaw * compensated_velocity.strafe)) *
                            ODOMETER_UPDATE_DT_S;

    g_odometer.forward_distance += distance_delta.forward;
    g_odometer.strafe_distance += distance_delta.strafe;
    g_odometer.travel_distance += odometer_vec_norm(distance_delta);
}
