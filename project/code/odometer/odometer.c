#include "odometer.h"
#include "../encoder/encoder_control.h"
#include "../Attitude/Accel_Calibration.h"
#include <math.h>

/*
 * 方案3实时因果里程计。
 * 参数来源：project/code/temp/causal_realtime_optimizer.py
 * 运行时不使用文件名先验、终点闭合或位置回拉。
 */
#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_FORWARD_COUNT_PER_METER    (12224.0697365f)
#define ODOMETER_STRAFE_COUNT_PER_METER     (-10763.8382733f)

#define ODOMETER_GAIN_FORWARD_0             (0.986f)
#define ODOMETER_GAIN_STRAFE_0              (0.820f)
#define ODOMETER_GAIN_FORWARD_V             (0.0f)
#define ODOMETER_GAIN_STRAFE_V              (0.0f)
#define ODOMETER_V_REF_MPS                  (1.20f)
#define ODOMETER_DEAD_FORWARD_MPS           (0.0f)
#define ODOMETER_DEAD_STRAFE_MPS            (0.0f)
#define ODOMETER_ENCODER_TAU_S              (0.0f)
#define ODOMETER_AXIS_RATIO                 (2.20f)
#define ODOMETER_CROSS_DAMP                 (0.80f)

#define ODOMETER_ACC_BIAS_FORWARD           (0.0f)
#define ODOMETER_ACC_BIAS_STRAFE            (0.0f)
#define ODOMETER_ACC_SCALE_FORWARD          (1.0f)
#define ODOMETER_ACC_SCALE_STRAFE           (1.0f)
#define ODOMETER_ACC_TAU_S                  (0.04f)
#define ODOMETER_JERK_CLIP_MPS3             (25.0f)
#define ODOMETER_IMU_TO_ENCODER_BETA        (0.10f)

#define ODOMETER_ALPHA_MIN                  (0.0f)
#define ODOMETER_ALPHA_MAX                  (0.015f)
#define ODOMETER_ACC_REF_MPS2               (6.0f)
#define ODOMETER_JERK_REF_MPS3              (80.0f)
#define ODOMETER_RESIDUAL_REF_MPS2          (40.0f)
#define ODOMETER_RISK_GAMMA                 (1.8f)
#define ODOMETER_ALPHA_TAU_S                (0.08f)
#define ODOMETER_EPSILON                    (1.0e-6f)

typedef struct
{
    float forward;
    float strafe;
} odometer_vec2_t;

typedef struct
{
    odometer_vec2_t velocity_mps;
    odometer_vec2_t encoder_velocity_mps;
    odometer_vec2_t encoder_velocity_lpf_mps;
    odometer_vec2_t accel_lpf_mps2;
    odometer_vec2_t accel_imu_mps2;
    float alpha;
} odometer_filter_state_t;

odometer_data_t g_odometer = {0.0f, 0.0f, 0.0f};

static odometer_filter_state_t g_odometer_filter;
static odometer_vec2_t g_odometer_raw_position_m;

static void odometer_update_with_accel(float accel_forward_mps2, float accel_strafe_mps2);

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

static float odometer_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float odometer_signf(float value)
{
    if(value > 0.0f)
    {
        return 1.0f;
    }

    if(value < 0.0f)
    {
        return -1.0f;
    }

    return 0.0f;
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

static odometer_vec2_t odometer_vec_lpf(odometer_vec2_t previous,
                                        odometer_vec2_t raw,
                                        float dt,
                                        float tau)
{
    odometer_vec2_t out;

    out.forward = odometer_lpf_scalar(previous.forward, raw.forward, dt, tau);
    out.strafe = odometer_lpf_scalar(previous.strafe, raw.strafe, dt, tau);
    return out;
}

static float odometer_vec_norm(odometer_vec2_t value)
{
    return sqrtf((value.forward * value.forward) + (value.strafe * value.strafe));
}

static odometer_vec2_t odometer_axis_constrain(odometer_vec2_t velocity)
{
    float forward_abs;
    float strafe_abs;
    float q;

    forward_abs = odometer_absf(velocity.forward);
    strafe_abs = odometer_absf(velocity.strafe);

    if(forward_abs > (ODOMETER_AXIS_RATIO * strafe_abs))
    {
        q = odometer_satf(((forward_abs / (strafe_abs + ODOMETER_EPSILON)) -
                           ODOMETER_AXIS_RATIO) / ODOMETER_AXIS_RATIO);
        velocity.strafe *= 1.0f - (ODOMETER_CROSS_DAMP * q);
    }

    if(strafe_abs > (ODOMETER_AXIS_RATIO * forward_abs))
    {
        q = odometer_satf(((strafe_abs / (forward_abs + ODOMETER_EPSILON)) -
                           ODOMETER_AXIS_RATIO) / ODOMETER_AXIS_RATIO);
        velocity.forward *= 1.0f - (ODOMETER_CROSS_DAMP * q);
    }

    return velocity;
}

static odometer_vec2_t odometer_correct_encoder_velocity(odometer_vec2_t raw_velocity)
{
    odometer_vec2_t velocity;
    float q_forward;
    float q_strafe;
    float gain_forward;
    float gain_strafe;
    float forward_abs;
    float strafe_abs;

    q_forward = odometer_satf(odometer_absf(raw_velocity.forward) / ODOMETER_V_REF_MPS);
    q_strafe = odometer_satf(odometer_absf(raw_velocity.strafe) / ODOMETER_V_REF_MPS);
    gain_forward = ODOMETER_GAIN_FORWARD_0 + (ODOMETER_GAIN_FORWARD_V * q_forward);
    gain_strafe = ODOMETER_GAIN_STRAFE_0 + (ODOMETER_GAIN_STRAFE_V * q_strafe);

    velocity.forward = gain_forward * raw_velocity.forward;
    velocity.strafe = gain_strafe * raw_velocity.strafe;

    forward_abs = odometer_absf(velocity.forward) - ODOMETER_DEAD_FORWARD_MPS;
    strafe_abs = odometer_absf(velocity.strafe) - ODOMETER_DEAD_STRAFE_MPS;
    velocity.forward = odometer_signf(velocity.forward) * ((forward_abs > 0.0f) ? forward_abs : 0.0f);
    velocity.strafe = odometer_signf(velocity.strafe) * ((strafe_abs > 0.0f) ? strafe_abs : 0.0f);

    return odometer_axis_constrain(velocity);
}

static odometer_vec2_t odometer_get_encoder_raw_velocity(void)
{
    odometer_vec2_t velocity;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;
    float forward_count;
    float strafe_count;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    forward_count = (left_front + right_front + left_rear + right_rear) * 0.25f;
    strafe_count = (-left_front + right_front + left_rear - right_rear) * 0.25f;

    velocity.forward = forward_count / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S;
    velocity.strafe = strafe_count / ODOMETER_STRAFE_COUNT_PER_METER / ODOMETER_UPDATE_DT_S;
    return velocity;
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
    g_odometer_raw_position_m.forward = 0.0f;
    g_odometer_raw_position_m.strafe = 0.0f;

    g_odometer_filter.velocity_mps.forward = 0.0f;
    g_odometer_filter.velocity_mps.strafe = 0.0f;
    g_odometer_filter.encoder_velocity_mps.forward = 0.0f;
    g_odometer_filter.encoder_velocity_mps.strafe = 0.0f;
    g_odometer_filter.encoder_velocity_lpf_mps.forward = 0.0f;
    g_odometer_filter.encoder_velocity_lpf_mps.strafe = 0.0f;
    g_odometer_filter.accel_lpf_mps2.forward = 0.0f;
    g_odometer_filter.accel_lpf_mps2.strafe = 0.0f;
    g_odometer_filter.accel_imu_mps2.forward = 0.0f;
    g_odometer_filter.accel_imu_mps2.strafe = 0.0f;
    g_odometer_filter.alpha = ODOMETER_ALPHA_MIN;
}

void odometer_update(void)
{
    float accel_forward_mps2;
    float accel_strafe_mps2;
    float accel_z_mps2;

    accel_forward_mps2 = 0.0f;
    accel_strafe_mps2 = 0.0f;
    accel_z_mps2 = 0.0f;
    AccelCalibration_GetBodyAccelMps2(&accel_forward_mps2, &accel_strafe_mps2, &accel_z_mps2);
    odometer_update_with_accel(accel_forward_mps2, accel_strafe_mps2);
}

static void odometer_update_with_accel(float accel_forward_mps2, float accel_strafe_mps2)
{
    odometer_vec2_t raw_encoder_velocity;
    odometer_vec2_t encoder_velocity;
    odometer_vec2_t encoder_accel;
    odometer_vec2_t raw_accel;
    odometer_vec2_t limited_accel;
    odometer_vec2_t accel_delta;
    odometer_vec2_t residual_vec;
    odometer_vec2_t predicted_velocity;
    odometer_vec2_t fused_velocity;
    odometer_vec2_t distance_delta;
    odometer_vec2_t corrected_accel;
    float max_accel_delta;
    float residual;
    float jerk;
    float risk0;
    float blend;
    float risk;
    float alpha_raw;

    raw_encoder_velocity = odometer_get_encoder_raw_velocity();
    g_odometer_raw_position_m.forward += raw_encoder_velocity.forward * ODOMETER_UPDATE_DT_S;
    g_odometer_raw_position_m.strafe += raw_encoder_velocity.strafe * ODOMETER_UPDATE_DT_S;

    encoder_velocity = odometer_correct_encoder_velocity(raw_encoder_velocity);
    g_odometer_filter.encoder_velocity_lpf_mps = odometer_vec_lpf(g_odometer_filter.encoder_velocity_lpf_mps,
                                                                  encoder_velocity,
                                                                  ODOMETER_UPDATE_DT_S,
                                                                  ODOMETER_ENCODER_TAU_S);
    encoder_velocity = g_odometer_filter.encoder_velocity_lpf_mps;

    encoder_accel.forward = (encoder_velocity.forward -
                             g_odometer_filter.encoder_velocity_mps.forward) / ODOMETER_UPDATE_DT_S;
    encoder_accel.strafe = (encoder_velocity.strafe -
                            g_odometer_filter.encoder_velocity_mps.strafe) / ODOMETER_UPDATE_DT_S;

    raw_accel.forward = ODOMETER_ACC_SCALE_FORWARD *
                        (accel_forward_mps2 - ODOMETER_ACC_BIAS_FORWARD);
    raw_accel.strafe = ODOMETER_ACC_SCALE_STRAFE *
                       (accel_strafe_mps2 - ODOMETER_ACC_BIAS_STRAFE);
    g_odometer_filter.accel_lpf_mps2 = odometer_vec_lpf(g_odometer_filter.accel_lpf_mps2,
                                                        raw_accel,
                                                        ODOMETER_UPDATE_DT_S,
                                                        ODOMETER_ACC_TAU_S);

    max_accel_delta = ODOMETER_JERK_CLIP_MPS3 * ODOMETER_UPDATE_DT_S;
    accel_delta.forward = odometer_clampf(g_odometer_filter.accel_lpf_mps2.forward -
                                          g_odometer_filter.accel_imu_mps2.forward,
                                          -max_accel_delta,
                                          max_accel_delta);
    accel_delta.strafe = odometer_clampf(g_odometer_filter.accel_lpf_mps2.strafe -
                                         g_odometer_filter.accel_imu_mps2.strafe,
                                         -max_accel_delta,
                                         max_accel_delta);
    limited_accel.forward = g_odometer_filter.accel_imu_mps2.forward + accel_delta.forward;
    limited_accel.strafe = g_odometer_filter.accel_imu_mps2.strafe + accel_delta.strafe;

    residual_vec.forward = encoder_accel.forward - limited_accel.forward;
    residual_vec.strafe = encoder_accel.strafe - limited_accel.strafe;
    residual = odometer_vec_norm(residual_vec);
    jerk = odometer_vec_norm(accel_delta) / ODOMETER_UPDATE_DT_S;

    risk0 = (0.20f * odometer_satf(odometer_vec_norm(encoder_velocity) / ODOMETER_V_REF_MPS)) +
            (0.22f * odometer_satf(odometer_vec_norm(limited_accel) / ODOMETER_ACC_REF_MPS2)) +
            (0.18f * odometer_satf(jerk / ODOMETER_JERK_REF_MPS3)) +
            (0.40f * odometer_satf(residual / ODOMETER_RESIDUAL_REF_MPS2));
    blend = ODOMETER_IMU_TO_ENCODER_BETA * (1.0f - risk0);

    corrected_accel.forward = ((1.0f - blend) * limited_accel.forward) +
                              (blend * encoder_accel.forward);
    corrected_accel.strafe = ((1.0f - blend) * limited_accel.strafe) +
                             (blend * encoder_accel.strafe);

    risk = powf(odometer_satf(risk0), ODOMETER_RISK_GAMMA);
    alpha_raw = ODOMETER_ALPHA_MIN + ((ODOMETER_ALPHA_MAX - ODOMETER_ALPHA_MIN) * risk);
    g_odometer_filter.alpha = odometer_lpf_scalar(g_odometer_filter.alpha,
                                                  alpha_raw,
                                                  ODOMETER_UPDATE_DT_S,
                                                  ODOMETER_ALPHA_TAU_S);

    predicted_velocity.forward = g_odometer_filter.velocity_mps.forward +
                                 (corrected_accel.forward * ODOMETER_UPDATE_DT_S);
    predicted_velocity.strafe = g_odometer_filter.velocity_mps.strafe +
                                (corrected_accel.strafe * ODOMETER_UPDATE_DT_S);

    fused_velocity.forward = (g_odometer_filter.alpha * predicted_velocity.forward) +
                             ((1.0f - g_odometer_filter.alpha) * encoder_velocity.forward);
    fused_velocity.strafe = (g_odometer_filter.alpha * predicted_velocity.strafe) +
                            ((1.0f - g_odometer_filter.alpha) * encoder_velocity.strafe);

    distance_delta.forward = fused_velocity.forward * ODOMETER_UPDATE_DT_S;
    distance_delta.strafe = fused_velocity.strafe * ODOMETER_UPDATE_DT_S;

    g_odometer.forward_distance += distance_delta.forward;
    g_odometer.strafe_distance += distance_delta.strafe;
    g_odometer.travel_distance += odometer_vec_norm(distance_delta);

    g_odometer_filter.velocity_mps = fused_velocity;
    g_odometer_filter.encoder_velocity_mps = encoder_velocity;
    g_odometer_filter.accel_imu_mps2 = limited_accel;
}

float odometer_get_forward_distance(void)
{
    return g_odometer.forward_distance;
}

float odometer_get_strafe_distance(void)
{
    return g_odometer.strafe_distance;
}

float odometer_get_travel_distance(void)
{
    return g_odometer.travel_distance;
}

float odometer_get_raw_forward_distance(void)
{
    return g_odometer_raw_position_m.forward;
}

float odometer_get_raw_strafe_distance(void)
{
    return g_odometer_raw_position_m.strafe;
}

void odometer_get_data(odometer_data_t *data)
{
    if(NULL == data)
    {
        return;
    }

    *data = g_odometer;
}
