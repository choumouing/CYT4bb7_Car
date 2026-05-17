#include "odometer_high_freedom.h"


#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_STARTUP_HOLD_TICKS         (50U)
#define ODOMETER_FORWARD_COUNT_PER_METER    (11287.0f)
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (12100.0f)

#define ODOMETER_DEG_TO_RAD                 (0.017453292519943295f)
#define ODOMETER_PI                         (3.14159265358979323846f)
#define ODOMETER_TWO_PI                     (6.28318530717958647692f)
#define ODOMETER_EPSILON                    (1.0e-6f)

/*
 * 高自由度离线拟合版本。
 * 说明: 该版本在当前 11 组数据上误差最低，但参数自由度较高，正式使用前需要新增数据交叉验证。
 */
#define ODOMETER_YAW_INTEGRAL_GAIN          (0.10f)
#define ODOMETER_AXIS_SPEED_BREAK_MPS       (0.35f)
#define ODOMETER_AXIS_SPEED_RANGE_MPS       (1.00f)
#define ODOMETER_DUAL_RATIO_BREAK           (0.15f)
#define ODOMETER_DUAL_RATIO_RANGE           (0.50f)
#define ODOMETER_YAW_RATE_REF_DPS           (80.0f)

#define ODOMETER_FWD_FP_BASE                (0.87647697f)
#define ODOMETER_FWD_FN_BASE                (0.90167164f)
#define ODOMETER_FWD_SP_BASE                (-0.09383003f)
#define ODOMETER_FWD_SN_BASE                (-0.05782720f)
#define ODOMETER_FWD_FP_AXIS                (-0.06856631f)
#define ODOMETER_FWD_FN_AXIS                (-0.00171054f)
#define ODOMETER_FWD_SP_AXIS                (0.33128398f)
#define ODOMETER_FWD_SN_AXIS                (0.10364321f)
#define ODOMETER_FWD_FP_DUAL                (-0.06949153f)
#define ODOMETER_FWD_FN_DUAL                (0.23064157f)
#define ODOMETER_FWD_SP_DUAL                (0.09346553f)
#define ODOMETER_FWD_SN_DUAL                (-0.09552139f)
#define ODOMETER_FWD_FP_YAWRATE             (1.44317809f)
#define ODOMETER_FWD_FN_YAWRATE             (-1.01732248f)
#define ODOMETER_FWD_SP_YAWRATE             (-0.51637106f)
#define ODOMETER_FWD_SN_YAWRATE             (1.10436543f)

#define ODOMETER_STRAFE_FP_BASE             (0.26635523f)
#define ODOMETER_STRAFE_FN_BASE             (-0.08793618f)
#define ODOMETER_STRAFE_SP_BASE             (0.67135251f)
#define ODOMETER_STRAFE_SN_BASE             (1.16741196f)
#define ODOMETER_STRAFE_FP_AXIS             (-0.41095207f)
#define ODOMETER_STRAFE_FN_AXIS             (0.13405844f)
#define ODOMETER_STRAFE_SP_AXIS             (0.42152586f)
#define ODOMETER_STRAFE_SN_AXIS             (-0.21177676f)
#define ODOMETER_STRAFE_FP_DUAL             (0.18528411f)
#define ODOMETER_STRAFE_FN_DUAL             (-0.47628442f)
#define ODOMETER_STRAFE_SP_DUAL             (-0.39473401f)
#define ODOMETER_STRAFE_SN_DUAL             (0.24036549f)
#define ODOMETER_STRAFE_FP_YAWRATE          (-0.10017021f)
#define ODOMETER_STRAFE_FN_YAWRATE          (0.23634134f)
#define ODOMETER_STRAFE_SP_YAWRATE          (-0.86158350f)
#define ODOMETER_STRAFE_SN_YAWRATE          (-1.64313940f)

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
} odometer_filter_state_t;

odometer_data_t g_odometer = {0.0f, 0.0f, 0.0f};

static odometer_filter_state_t g_odometer_filter;

static float odometer_vec_norm(odometer_vec2_t value)
{
    return sqrtf((value.forward * value.forward) + (value.strafe * value.strafe));
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

static float odometer_axis_speed_weight(float speed_abs_mps)
{
    return car_math_clampf((speed_abs_mps - ODOMETER_AXIS_SPEED_BREAK_MPS) /
                           ODOMETER_AXIS_SPEED_RANGE_MPS,
                           0.0f,
                           1.0f);
}

static float odometer_dual_axis_weight(odometer_vec2_t velocity)
{
    float speed_norm;
    float dual_ratio;

    speed_norm = odometer_vec_norm(velocity);
    dual_ratio = car_math_minf(car_math_absf(velocity.forward),
                               car_math_absf(velocity.strafe)) /
                 (speed_norm + ODOMETER_EPSILON);
    return car_math_clampf((dual_ratio - ODOMETER_DUAL_RATIO_BREAK) /
                           ODOMETER_DUAL_RATIO_RANGE,
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

static odometer_vec2_t odometer_compensate_encoder_velocity(odometer_vec2_t velocity,
                                                           float yaw_rate_abs_dps)
{
    odometer_vec2_t compensated;
    float fp;
    float fn;
    float sp;
    float sn;
    float fw;
    float sw;
    float dw;
    float yw;

    fp = odometer_positive_part(velocity.forward);
    fn = odometer_negative_part(velocity.forward);
    sp = odometer_positive_part(velocity.strafe);
    sn = odometer_negative_part(velocity.strafe);
    fw = odometer_axis_speed_weight(car_math_absf(velocity.forward));
    sw = odometer_axis_speed_weight(car_math_absf(velocity.strafe));
    dw = odometer_dual_axis_weight(velocity);
    yw = car_math_clampf(yaw_rate_abs_dps / ODOMETER_YAW_RATE_REF_DPS, 0.0f, 1.0f);

    compensated.forward =
        (ODOMETER_FWD_FP_BASE * fp) +
        (ODOMETER_FWD_FN_BASE * fn) +
        (ODOMETER_FWD_SP_BASE * sp) +
        (ODOMETER_FWD_SN_BASE * sn) +
        (ODOMETER_FWD_FP_AXIS * fp * fw) +
        (ODOMETER_FWD_FN_AXIS * fn * fw) +
        (ODOMETER_FWD_SP_AXIS * sp * sw) +
        (ODOMETER_FWD_SN_AXIS * sn * sw) +
        (ODOMETER_FWD_FP_DUAL * fp * dw) +
        (ODOMETER_FWD_FN_DUAL * fn * dw) +
        (ODOMETER_FWD_SP_DUAL * sp * dw) +
        (ODOMETER_FWD_SN_DUAL * sn * dw) +
        (ODOMETER_FWD_FP_YAWRATE * fp * yw) +
        (ODOMETER_FWD_FN_YAWRATE * fn * yw) +
        (ODOMETER_FWD_SP_YAWRATE * sp * yw) +
        (ODOMETER_FWD_SN_YAWRATE * sn * yw);

    compensated.strafe =
        (ODOMETER_STRAFE_FP_BASE * fp) +
        (ODOMETER_STRAFE_FN_BASE * fn) +
        (ODOMETER_STRAFE_SP_BASE * sp) +
        (ODOMETER_STRAFE_SN_BASE * sn) +
        (ODOMETER_STRAFE_FP_AXIS * fp * fw) +
        (ODOMETER_STRAFE_FN_AXIS * fn * fw) +
        (ODOMETER_STRAFE_SP_AXIS * sp * sw) +
        (ODOMETER_STRAFE_SN_AXIS * sn * sw) +
        (ODOMETER_STRAFE_FP_DUAL * fp * dw) +
        (ODOMETER_STRAFE_FN_DUAL * fn * dw) +
        (ODOMETER_STRAFE_SP_DUAL * sp * dw) +
        (ODOMETER_STRAFE_SN_DUAL * sn * dw) +
        (ODOMETER_STRAFE_FP_YAWRATE * fp * yw) +
        (ODOMETER_STRAFE_FN_YAWRATE * fn * yw) +
        (ODOMETER_STRAFE_SP_YAWRATE * sp * yw) +
        (ODOMETER_STRAFE_SN_YAWRATE * sn * yw);

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

    compensated_velocity =
        odometer_compensate_encoder_velocity(raw_velocity, car_math_absf(gyro_avg_dps[2]));

    if(g_odometer_filter.startup_hold_ticks > 0U)
    {
        g_odometer.forward_distance = 0.0f;
        g_odometer.strafe_distance = 0.0f;
        g_odometer.travel_distance = 0.0f;
        g_odometer_filter.yaw_delta_rad = 0.0f;
        g_odometer_filter.startup_hold_ticks--;
        return;
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
