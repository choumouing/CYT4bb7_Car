#include "odometer.h"

#include "../Attitude/Accel_Calibration.h"
#include "../Attitude/IMU_TOP.h"
#include "../encoder/encoder_control.h"
#include <math.h>

#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_FORWARD_COUNT_PER_METER    (11287.0f)
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (12100.0f)

#define ODOMETER_KX_SPEED                   (0.00529850743f)
#define ODOMETER_KY_SPEED                   (0.0f)
#define ODOMETER_KX_RESIDUAL                (0.488792866f)
#define ODOMETER_KY_RESIDUAL                (0.00697509527f)
#define ODOMETER_KX_REVERSE                 (1.00619317f)
#define ODOMETER_KY_REVERSE                 (0.00000105576f)
#define ODOMETER_KX_YAW                     (0.68f)
#define ODOMETER_KY_YAW                     (0.80f)
#define ODOMETER_K_DUAL_AXIS                (0.0653664524f)
#define ODOMETER_V_REF_MPS                  (1.66942520f)
#define ODOMETER_RESIDUAL_REF_MPS2          (8.33671585f)
#define ODOMETER_RISK_GAMMA                 (7.68239773f)
#define ODOMETER_YAW_GAIN                   (0.72f)
#define ODOMETER_DEAD_FORWARD_MPS           (0.000000424f)
#define ODOMETER_DEAD_STRAFE_MPS            (0.00496915618f)
#define ODOMETER_SPEED_POWER_X              (2.51620747f)
#define ODOMETER_SPEED_POWER_Y              (1.26317978f)
#define ODOMETER_CROSS_AXIS                 (1.83240615f)
#define ODOMETER_REVERSE_REF_MPS            (0.0759208411f)
#define ODOMETER_YAW_RATE_REF_RPS           (0.936060514f)

#define ODOMETER_ALPHA_MAX                  (0.03f)
#define ODOMETER_ALPHA_TAU_S                (0.06f)
#define ODOMETER_DEG_TO_RAD                 (0.017453292519943295f)
#define ODOMETER_PI                         (3.14159265358979323846f)
#define ODOMETER_TWO_PI                     (6.28318530717958647692f)
#define ODOMETER_EPSILON                    (1.0e-6f)

typedef struct
{
    float forward;
    float strafe;
} odometer_vec2_t;

typedef struct
{
    odometer_vec2_t velocity_mps;
    odometer_vec2_t prev_encoder_velocity_mps;
    odometer_vec2_t accel_bias_mps2;
    float yaw_zero_rad;
    float prev_yaw_delta_rad;
    float alpha;
    uint8_t yaw_ready;
} odometer_filter_state_t;

odometer_data_t g_odometer = {0.0f, 0.0f, 0.0f};

static odometer_filter_state_t g_odometer_filter;

static float odometer_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float odometer_clampf(float value, float low, float high)
{
    if(value < low)
    {
        return low;
    }

    if(value > high)
    {
        return high;
    }

    return value;
}

static float odometer_satf(float value)
{
    return odometer_clampf(value, 0.0f, 1.0f);
}

static float odometer_minf(float a, float b)
{
    return (a < b) ? a : b;
}

static float odometer_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float odometer_lpf_scalar(float previous, float raw, float dt, float tau)
{
    float beta;

    if(tau <= ODOMETER_EPSILON)
    {
        return raw;
    }

    beta = dt / (tau + dt);
    return previous + (beta * (raw - previous));
}

static float odometer_soft_deadband(float value, float deadband)
{
    float magnitude;

    magnitude = odometer_absf(value) - deadband;
    if(magnitude <= 0.0f)
    {
        return 0.0f;
    }

    return (value >= 0.0f) ? magnitude : -magnitude;
}

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
    velocity.strafe = delta_count.strafe / (-ODOMETER_STRAFE_COUNT_PER_METER_ABS) / ODOMETER_UPDATE_DT_S;
    return velocity;
}

static void odometer_update_accel_bias(odometer_vec2_t encoder_velocity,
                                       odometer_vec2_t accel_mps2)
{
    float speed;

    speed = odometer_vec_norm(encoder_velocity);
    if(speed < 0.08f)
    {
        g_odometer_filter.accel_bias_mps2.forward =
            odometer_lpf_scalar(g_odometer_filter.accel_bias_mps2.forward,
                                accel_mps2.forward,
                                ODOMETER_UPDATE_DT_S,
                                1.2f);
        g_odometer_filter.accel_bias_mps2.strafe =
            odometer_lpf_scalar(g_odometer_filter.accel_bias_mps2.strafe,
                                accel_mps2.strafe,
                                ODOMETER_UPDATE_DT_S,
                                1.2f);
    }
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

    g_odometer_filter.velocity_mps.forward = 0.0f;
    g_odometer_filter.velocity_mps.strafe = 0.0f;
    g_odometer_filter.prev_encoder_velocity_mps.forward = 0.0f;
    g_odometer_filter.prev_encoder_velocity_mps.strafe = 0.0f;
    g_odometer_filter.accel_bias_mps2.forward = 0.0f;
    g_odometer_filter.accel_bias_mps2.strafe = 0.0f;
    g_odometer_filter.yaw_zero_rad = 0.0f;
    g_odometer_filter.prev_yaw_delta_rad = 0.0f;
    g_odometer_filter.alpha = 0.0f;
    g_odometer_filter.yaw_ready = 0U;
}

void odometer_update(void)
{
    odometer_vec2_t delta_count;
    odometer_vec2_t encoder_velocity;
    odometer_vec2_t raw_encoder_velocity;
    odometer_vec2_t encoder_accel;
    odometer_vec2_t accel_raw;
    odometer_vec2_t accel_corrected;
    odometer_vec2_t distance_delta;
    float accel_z_mps2;
    float qvx;
    float qvy;
    float qrx;
    float qry;
    float reverse_forward;
    float reverse_strafe;
    float yaw_now_rad;
    float yaw_delta_rad;
    float yaw_step_rad;
    float yaw_rate_abs;
    float yaw_risk;
    float dual_axis;
    float scale_forward;
    float scale_strafe;
    float risk;
    float alpha_raw;
    float predicted_forward;
    float predicted_strafe;
    float fused_forward;
    float fused_strafe;
    float yaw_for_projection;
    float cos_yaw;
    float sin_yaw;

    delta_count = odometer_get_encoder_delta_count();
    encoder_velocity = odometer_get_encoder_velocity(delta_count);
    raw_encoder_velocity = encoder_velocity;

    accel_raw.forward = 0.0f;
    accel_raw.strafe = 0.0f;
    accel_z_mps2 = 0.0f;
    AccelCalibration_GetBodyAccelMps2(&accel_raw.forward, &accel_raw.strafe, &accel_z_mps2);
    odometer_update_accel_bias(encoder_velocity, accel_raw);
    accel_corrected.forward = accel_raw.forward - g_odometer_filter.accel_bias_mps2.forward;
    accel_corrected.strafe = accel_raw.strafe - g_odometer_filter.accel_bias_mps2.strafe;

    encoder_accel.forward = (encoder_velocity.forward -
                             g_odometer_filter.prev_encoder_velocity_mps.forward) / ODOMETER_UPDATE_DT_S;
    encoder_accel.strafe = (encoder_velocity.strafe -
                            g_odometer_filter.prev_encoder_velocity_mps.strafe) / ODOMETER_UPDATE_DT_S;

    yaw_now_rad = g_euler.yaw * ODOMETER_DEG_TO_RAD;
    if(0U == g_odometer_filter.yaw_ready)
    {
        g_odometer_filter.yaw_zero_rad = yaw_now_rad;
        g_odometer_filter.prev_yaw_delta_rad = 0.0f;
        g_odometer_filter.yaw_ready = 1U;
    }
    yaw_delta_rad = odometer_normalize_angle(yaw_now_rad - g_odometer_filter.yaw_zero_rad);
    yaw_step_rad = odometer_normalize_angle(yaw_delta_rad - g_odometer_filter.prev_yaw_delta_rad);
    yaw_rate_abs = odometer_absf(yaw_step_rad) / ODOMETER_UPDATE_DT_S;

    qvx = odometer_clampf(odometer_absf(encoder_velocity.forward) / ODOMETER_V_REF_MPS, 0.0f, 2.0f);
    qvy = odometer_clampf(odometer_absf(encoder_velocity.strafe) / ODOMETER_V_REF_MPS, 0.0f, 2.0f);
    qrx = odometer_clampf(odometer_absf(encoder_accel.forward - accel_corrected.forward) /
                          ODOMETER_RESIDUAL_REF_MPS2,
                          0.0f,
                          2.0f);
    qry = odometer_clampf(odometer_absf(encoder_accel.strafe - accel_corrected.strafe) /
                          ODOMETER_RESIDUAL_REF_MPS2,
                          0.0f,
                          2.0f);
    reverse_forward = odometer_clampf(-(encoder_velocity.forward *
                                        g_odometer_filter.prev_encoder_velocity_mps.forward) /
                                      (ODOMETER_REVERSE_REF_MPS * ODOMETER_REVERSE_REF_MPS),
                                      0.0f,
                                      2.0f);
    reverse_strafe = odometer_clampf(-(encoder_velocity.strafe *
                                       g_odometer_filter.prev_encoder_velocity_mps.strafe) /
                                     (ODOMETER_REVERSE_REF_MPS * ODOMETER_REVERSE_REF_MPS),
                                     0.0f,
                                     2.0f);
    yaw_risk = odometer_clampf(yaw_rate_abs / ODOMETER_YAW_RATE_REF_RPS, 0.0f, 2.0f);
    dual_axis = odometer_clampf(odometer_minf(odometer_absf(encoder_velocity.forward),
                                             odometer_absf(encoder_velocity.strafe)) / ODOMETER_V_REF_MPS,
                                0.0f,
                                1.5f);

    scale_forward = 1.0f /
                    (1.0f +
                     (ODOMETER_KX_SPEED * powf(qvx, ODOMETER_SPEED_POWER_X)) +
                     (ODOMETER_KX_RESIDUAL * powf(qrx, ODOMETER_RISK_GAMMA)) +
                     (ODOMETER_KX_REVERSE * reverse_forward) +
                     (ODOMETER_KX_YAW * yaw_risk) +
                     (ODOMETER_K_DUAL_AXIS * dual_axis) +
                     (ODOMETER_CROSS_AXIS * powf(qvy, ODOMETER_SPEED_POWER_Y) *
                      odometer_satf(qrx)));
    scale_strafe = 1.0f /
                   (1.0f +
                    (ODOMETER_KY_SPEED * powf(qvy, ODOMETER_SPEED_POWER_Y)) +
                    (ODOMETER_KY_RESIDUAL * powf(qry, ODOMETER_RISK_GAMMA)) +
                    (ODOMETER_KY_REVERSE * reverse_strafe) +
                    (ODOMETER_KY_YAW * yaw_risk) +
                    (ODOMETER_K_DUAL_AXIS * dual_axis) +
                    (ODOMETER_CROSS_AXIS * powf(qvx, ODOMETER_SPEED_POWER_X) *
                     odometer_satf(qry)));

    encoder_velocity.forward = odometer_soft_deadband(encoder_velocity.forward * scale_forward,
                                                      ODOMETER_DEAD_FORWARD_MPS);
    encoder_velocity.strafe = odometer_soft_deadband(encoder_velocity.strafe * scale_strafe,
                                                     ODOMETER_DEAD_STRAFE_MPS);

    risk = odometer_clampf((0.30f * odometer_maxf(qvx, qvy)) +
                           (0.45f * odometer_maxf(qrx, qry)) +
                           (0.15f * odometer_maxf(reverse_forward, reverse_strafe)) +
                           (0.10f * yaw_risk),
                           0.0f,
                           1.0f);
    alpha_raw = ODOMETER_ALPHA_MAX * risk;
    g_odometer_filter.alpha = odometer_lpf_scalar(g_odometer_filter.alpha,
                                                  alpha_raw,
                                                  ODOMETER_UPDATE_DT_S,
                                                  ODOMETER_ALPHA_TAU_S);

    predicted_forward = g_odometer_filter.velocity_mps.forward +
                        (accel_corrected.forward * ODOMETER_UPDATE_DT_S);
    predicted_strafe = g_odometer_filter.velocity_mps.strafe +
                       (accel_corrected.strafe * ODOMETER_UPDATE_DT_S);
    fused_forward = (g_odometer_filter.alpha * predicted_forward) +
                    ((1.0f - g_odometer_filter.alpha) * encoder_velocity.forward);
    fused_strafe = (g_odometer_filter.alpha * predicted_strafe) +
                   ((1.0f - g_odometer_filter.alpha) * encoder_velocity.strafe);

    yaw_for_projection = yaw_delta_rad * ODOMETER_YAW_GAIN;
    cos_yaw = cosf(yaw_for_projection);
    sin_yaw = sinf(yaw_for_projection);
    distance_delta.forward = ((cos_yaw * fused_forward) - (sin_yaw * fused_strafe)) *
                             ODOMETER_UPDATE_DT_S;
    distance_delta.strafe = ((sin_yaw * fused_forward) + (cos_yaw * fused_strafe)) *
                            ODOMETER_UPDATE_DT_S;

    g_odometer.forward_distance += distance_delta.forward;
    g_odometer.strafe_distance += distance_delta.strafe;
    g_odometer.travel_distance += odometer_vec_norm(distance_delta);

    g_odometer_filter.velocity_mps.forward = fused_forward;
    g_odometer_filter.velocity_mps.strafe = fused_strafe;
    g_odometer_filter.prev_encoder_velocity_mps = raw_encoder_velocity;
    g_odometer_filter.prev_yaw_delta_rad = yaw_delta_rad;
}
