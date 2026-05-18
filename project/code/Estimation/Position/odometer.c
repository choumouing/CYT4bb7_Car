#include "odometer.h"


#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_STARTUP_HOLD_TICKS         (50U)
#define ODOMETER_FORWARD_COUNT_PER_METER    (11287.0f)
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (12100.0f)

#define ODOMETER_DEG_TO_RAD                 (0.017453292519943295f)
#define ODOMETER_PI                         (3.14159265358979323846f)
#define ODOMETER_TWO_PI                     (6.28318530717958647692f)
#define ODOMETER_EPSILON                    (1.0e-6f)

#define ODOMETER_FAST_SPEED_START_MPS       (0.35f)
#define ODOMETER_FAST_SPEED_RANGE_MPS       (1.00f)
#define ODOMETER_DUAL_RATIO_START           (0.15f)
#define ODOMETER_DUAL_RATIO_RANGE           (0.50f)
#define ODOMETER_YAW_RATE_REF_DPS           (80.0f)
#define ODOMETER_YAW_PROJECTION_GAIN        (-0.30f)

#define ODOMETER_CF_FP                      (1.30232242f)
#define ODOMETER_CF_FN                      (0.74862410f)
#define ODOMETER_CF_SP                      (-0.49652230f)
#define ODOMETER_CF_SN                      (0.25794964f)
#define ODOMETER_CF_FP_FAST                 (-0.81804909f)
#define ODOMETER_CF_FN_FAST                 (0.23802903f)
#define ODOMETER_CF_SP_FAST                 (0.78750857f)
#define ODOMETER_CF_SN_FAST                 (-0.38733112f)
#define ODOMETER_CF_FP_DUAL                 (0.49952085f)
#define ODOMETER_CF_FN_DUAL                 (-0.78582360f)
#define ODOMETER_CF_SP_DUAL                 (-0.55221557f)
#define ODOMETER_CF_SN_DUAL                 (0.59020688f)
#define ODOMETER_CF_FP_YAW                  (3.54574689f)
#define ODOMETER_CF_FN_YAW                  (1.25208046f)
#define ODOMETER_CF_SP_YAW                  (0.26910336f)
#define ODOMETER_CF_SN_YAW                  (0.72310716f)

#define ODOMETER_CS_FP                      (0.12188119f)
#define ODOMETER_CS_FN                      (-0.05250789f)
#define ODOMETER_CS_SP                      (0.82308289f)
#define ODOMETER_CS_SN                      (1.08350193f)
#define ODOMETER_CS_FP_FAST                 (-0.13035869f)
#define ODOMETER_CS_FN_FAST                 (0.08995032f)
#define ODOMETER_CS_SP_FAST                 (0.22171650f)
#define ODOMETER_CS_SN_FAST                 (-0.07503716f)
#define ODOMETER_CS_FP_DUAL                 (0.07562612f)
#define ODOMETER_CS_FN_DUAL                 (-0.25891603f)
#define ODOMETER_CS_SP_DUAL                 (-0.20408599f)
#define ODOMETER_CS_SN_DUAL                 (0.16322013f)
#define ODOMETER_CS_FP_YAW                  (-1.05012010f)
#define ODOMETER_CS_FN_YAW                  (-0.40064650f)
#define ODOMETER_CS_SP_YAW                  (-1.02145046f)
#define ODOMETER_CS_SN_YAW                  (-1.83732820f)

typedef struct
{
    float forward;
    float strafe;
} odometer_vec2_t;

typedef struct
{
    float yaw_rad;
    float last_gyro_z_dps;
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

static uint8_t odometer_is_finite(float value)
{
    if(value != value)
    {
        return 0U;
    }

    if((value > 1000000.0f) || (value < -1000000.0f))
    {
        return 0U;
    }

    return 1U;
}

static float odometer_read_gyro_z_dps(void)
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    if(0U == odometer_is_finite(gyro_z_dps))
    {
        gyro_z_dps = g_odometer_filter.last_gyro_z_dps;
    }

    g_odometer_filter.last_gyro_z_dps = gyro_z_dps;
    return gyro_z_dps;
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

    velocity.forward = delta_count.forward /
                       ODOMETER_FORWARD_COUNT_PER_METER /
                       ODOMETER_UPDATE_DT_S;
    velocity.strafe = delta_count.strafe /
                      ODOMETER_STRAFE_COUNT_PER_METER_ABS /
                      ODOMETER_UPDATE_DT_S;
    return velocity;
}

static odometer_vec2_t odometer_compensate_velocity(odometer_vec2_t raw_velocity,
                                                    float gyro_z_dps)
{
    odometer_vec2_t compensated;
    float fp;
    float fn;
    float sp;
    float sn;
    float forward_fast_weight;
    float strafe_fast_weight;
    float speed_norm;
    float dual_ratio;
    float dual_weight;
    float yawrate_weight;

    fp = (raw_velocity.forward > 0.0f) ? raw_velocity.forward : 0.0f;
    fn = (raw_velocity.forward < 0.0f) ? raw_velocity.forward : 0.0f;
    sp = (raw_velocity.strafe > 0.0f) ? raw_velocity.strafe : 0.0f;
    sn = (raw_velocity.strafe < 0.0f) ? raw_velocity.strafe : 0.0f;

    forward_fast_weight = car_math_clampf((car_math_absf(raw_velocity.forward) -
                                           ODOMETER_FAST_SPEED_START_MPS) /
                                              ODOMETER_FAST_SPEED_RANGE_MPS,
                                          0.0f,
                                          1.0f);
    strafe_fast_weight = car_math_clampf((car_math_absf(raw_velocity.strafe) -
                                          ODOMETER_FAST_SPEED_START_MPS) /
                                             ODOMETER_FAST_SPEED_RANGE_MPS,
                                         0.0f,
                                         1.0f);
    speed_norm = odometer_vec_norm(raw_velocity);
    dual_ratio = car_math_minf(car_math_absf(raw_velocity.forward),
                               car_math_absf(raw_velocity.strafe)) /
                 (speed_norm + ODOMETER_EPSILON);
    dual_weight = car_math_clampf((dual_ratio - ODOMETER_DUAL_RATIO_START) /
                                      ODOMETER_DUAL_RATIO_RANGE,
                                  0.0f,
                                  1.0f);
    yawrate_weight = car_math_clampf(car_math_absf(gyro_z_dps) / ODOMETER_YAW_RATE_REF_DPS,
                                     0.0f,
                                     1.0f);

    compensated.forward =
        (ODOMETER_CF_FP * fp) + (ODOMETER_CF_FN * fn) +
        (ODOMETER_CF_SP * sp) + (ODOMETER_CF_SN * sn) +
        (ODOMETER_CF_FP_FAST * fp * forward_fast_weight) +
        (ODOMETER_CF_FN_FAST * fn * forward_fast_weight) +
        (ODOMETER_CF_SP_FAST * sp * strafe_fast_weight) +
        (ODOMETER_CF_SN_FAST * sn * strafe_fast_weight) +
        (ODOMETER_CF_FP_DUAL * fp * dual_weight) +
        (ODOMETER_CF_FN_DUAL * fn * dual_weight) +
        (ODOMETER_CF_SP_DUAL * sp * dual_weight) +
        (ODOMETER_CF_SN_DUAL * sn * dual_weight) +
        (ODOMETER_CF_FP_YAW * fp * yawrate_weight) +
        (ODOMETER_CF_FN_YAW * fn * yawrate_weight) +
        (ODOMETER_CF_SP_YAW * sp * yawrate_weight) +
        (ODOMETER_CF_SN_YAW * sn * yawrate_weight);

    compensated.strafe =
        (ODOMETER_CS_FP * fp) + (ODOMETER_CS_FN * fn) +
        (ODOMETER_CS_SP * sp) + (ODOMETER_CS_SN * sn) +
        (ODOMETER_CS_FP_FAST * fp * forward_fast_weight) +
        (ODOMETER_CS_FN_FAST * fn * forward_fast_weight) +
        (ODOMETER_CS_SP_FAST * sp * strafe_fast_weight) +
        (ODOMETER_CS_SN_FAST * sn * strafe_fast_weight) +
        (ODOMETER_CS_FP_DUAL * fp * dual_weight) +
        (ODOMETER_CS_FN_DUAL * fn * dual_weight) +
        (ODOMETER_CS_SP_DUAL * sp * dual_weight) +
        (ODOMETER_CS_SN_DUAL * sn * dual_weight) +
        (ODOMETER_CS_FP_YAW * fp * yawrate_weight) +
        (ODOMETER_CS_FN_YAW * fn * yawrate_weight) +
        (ODOMETER_CS_SP_YAW * sp * yawrate_weight) +
        (ODOMETER_CS_SN_YAW * sn * yawrate_weight);

    return compensated;
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

    g_odometer_filter.yaw_rad = 0.0f;
    g_odometer_filter.last_gyro_z_dps = 0.0f;
    g_odometer_filter.startup_hold_ticks = ODOMETER_STARTUP_HOLD_TICKS;
}

void odometer_update_100HZ(void)
{
    odometer_vec2_t delta_count;
    odometer_vec2_t raw_velocity;
    odometer_vec2_t compensated_velocity;
    odometer_vec2_t distance_delta;
    float gyro_z_dps;
    float yaw_for_projection;
    float cos_yaw;
    float sin_yaw;

    delta_count = odometer_get_encoder_delta_count();
    raw_velocity = odometer_get_encoder_velocity(delta_count);
    gyro_z_dps = odometer_read_gyro_z_dps();

    if(g_odometer_filter.startup_hold_ticks > 0U)
    {
        g_odometer.forward_distance = 0.0f;
        g_odometer.strafe_distance = 0.0f;
        g_odometer.travel_distance = 0.0f;
        g_odometer_filter.yaw_rad = 0.0f;
        g_odometer_filter.startup_hold_ticks--;
        return;
    }

    compensated_velocity = odometer_compensate_velocity(raw_velocity, gyro_z_dps);
    g_odometer_filter.yaw_rad =
        odometer_normalize_angle(g_odometer_filter.yaw_rad +
                                 (gyro_z_dps * ODOMETER_DEG_TO_RAD * ODOMETER_UPDATE_DT_S));

    yaw_for_projection = g_odometer_filter.yaw_rad * ODOMETER_YAW_PROJECTION_GAIN;
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
