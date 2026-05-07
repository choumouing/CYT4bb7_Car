#include <math.h>
#include <stddef.h>

typedef struct
{
    float forward_m;
    float strafe_m;
    float travel_m;
} adaptive_odometer_output_t;

typedef struct
{
    float velocity_forward_mps;
    float velocity_strafe_mps;
    float prev_encoder_forward_mps;
    float prev_encoder_strafe_mps;
    float prev_forward_delta_count;
    float prev_strafe_delta_count;
    float accel_bias_forward_mps2;
    float accel_bias_strafe_mps2;
    float alpha;
} adaptive_odometer_state_t;

#define ODOM_FWD_COUNT_PER_M        (11287.0f)
#define ODOM_STRAFE_COUNT_PER_M_ABS (12100.0f)
#define ODOM_DT_S                   (0.01f)

#define ODOM_KX_SPEED               (0.00529850743f)
#define ODOM_KY_SPEED               (0.0f)
#define ODOM_KX_RESIDUAL            (0.488792866f)
#define ODOM_KY_RESIDUAL            (0.00697509527f)
#define ODOM_KX_REVERSE             (1.00619317f)
#define ODOM_KY_REVERSE             (0.00000105576f)
#define ODOM_KX_YAW                 (0.68f)
#define ODOM_KY_YAW                 (0.80f)
#define ODOM_K_DUAL_AXIS            (0.0653664524f)
#define ODOM_V_REF_MPS              (1.66942520f)
#define ODOM_RESIDUAL_REF_MPS2      (8.33671585f)
#define ODOM_RISK_GAMMA             (7.68239773f)
#define ODOM_YAW_GAIN               (0.72f)
#define ODOM_DEAD_FORWARD_MPS       (0.000000424f)
#define ODOM_DEAD_STRAFE_MPS        (0.00496915618f)
#define ODOM_SPEED_POWER_X          (2.51620747f)
#define ODOM_SPEED_POWER_Y          (1.26317978f)
#define ODOM_CROSS_AXIS             (1.83240615f)
#define ODOM_REVERSE_REF_MPS        (0.0759208411f)
#define ODOM_YAW_RATE_REF_RPS       (0.936060514f)

#define ODOM_ALPHA_MAX              (0.03f)
#define ODOM_ALPHA_TAU_S            (0.06f)
#define ODOM_ALPHA_RISK_REF         (1.0f)

static float odom_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float odom_clampf(float value, float low, float high)
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

static float odom_soft_deadband(float value, float deadband)
{
    float mag = odom_absf(value) - deadband;
    if(mag <= 0.0f)
    {
        return 0.0f;
    }
    return (value >= 0.0f) ? mag : -mag;
}

void adaptive_odometer_reset(adaptive_odometer_state_t *state,
                             adaptive_odometer_output_t *output)
{
    if(state != NULL)
    {
        state->velocity_forward_mps = 0.0f;
        state->velocity_strafe_mps = 0.0f;
        state->prev_encoder_forward_mps = 0.0f;
        state->prev_encoder_strafe_mps = 0.0f;
        state->prev_forward_delta_count = 0.0f;
        state->prev_strafe_delta_count = 0.0f;
        state->accel_bias_forward_mps2 = 0.0f;
        state->accel_bias_strafe_mps2 = 0.0f;
        state->alpha = 0.0f;
    }

    if(output != NULL)
    {
        output->forward_m = 0.0f;
        output->strafe_m = 0.0f;
        output->travel_m = 0.0f;
    }
}

void adaptive_odometer_update(adaptive_odometer_state_t *state,
                              adaptive_odometer_output_t *output,
                              float forward_delta_count,
                              float strafe_delta_count,
                              float accel_forward_mps2,
                              float accel_strafe_mps2,
                              float yaw_delta_rad,
                              float yaw_rate_abs_rps)
{
    float enc_forward;
    float enc_strafe;
    float enc_accel_forward;
    float enc_accel_strafe;
    float accel_forward;
    float accel_strafe;
    float qvx;
    float qvy;
    float qrx;
    float qry;
    float reverse_forward;
    float reverse_strafe;
    float yaw_risk;
    float dual_axis;
    float scale_forward;
    float scale_strafe;
    float risk;
    float alpha_target;
    float alpha_beta;
    float pred_forward;
    float pred_strafe;
    float yaw;
    float cos_yaw;
    float sin_yaw;
    float dx;
    float dy;

    if((state == NULL) || (output == NULL))
    {
        return;
    }

    enc_forward = forward_delta_count / ODOM_FWD_COUNT_PER_M / ODOM_DT_S;
    enc_strafe = strafe_delta_count / (-ODOM_STRAFE_COUNT_PER_M_ABS) / ODOM_DT_S;

    enc_accel_forward = (enc_forward - state->prev_encoder_forward_mps) / ODOM_DT_S;
    enc_accel_strafe = (enc_strafe - state->prev_encoder_strafe_mps) / ODOM_DT_S;

    accel_forward = accel_forward_mps2 - state->accel_bias_forward_mps2;
    accel_strafe = accel_strafe_mps2 - state->accel_bias_strafe_mps2;

    qvx = odom_clampf(odom_absf(enc_forward) / ODOM_V_REF_MPS, 0.0f, 2.0f);
    qvy = odom_clampf(odom_absf(enc_strafe) / ODOM_V_REF_MPS, 0.0f, 2.0f);
    qrx = odom_clampf(odom_absf(enc_accel_forward - accel_forward) / ODOM_RESIDUAL_REF_MPS2, 0.0f, 2.0f);
    qry = odom_clampf(odom_absf(enc_accel_strafe - accel_strafe) / ODOM_RESIDUAL_REF_MPS2, 0.0f, 2.0f);

    reverse_forward = odom_clampf(-(enc_forward * state->prev_encoder_forward_mps) /
                                  (ODOM_REVERSE_REF_MPS * ODOM_REVERSE_REF_MPS),
                                  0.0f,
                                  2.0f);
    reverse_strafe = odom_clampf(-(enc_strafe * state->prev_encoder_strafe_mps) /
                                 (ODOM_REVERSE_REF_MPS * ODOM_REVERSE_REF_MPS),
                                 0.0f,
                                 2.0f);
    yaw_risk = odom_clampf(yaw_rate_abs_rps / ODOM_YAW_RATE_REF_RPS, 0.0f, 2.0f);
    dual_axis = odom_clampf(fminf(odom_absf(enc_forward), odom_absf(enc_strafe)) / ODOM_V_REF_MPS,
                            0.0f,
                            1.5f);

    scale_forward = 1.0f /
                    (1.0f +
                     (ODOM_KX_SPEED * powf(qvx, ODOM_SPEED_POWER_X)) +
                     (ODOM_KX_RESIDUAL * powf(qrx, ODOM_RISK_GAMMA)) +
                     (ODOM_KX_REVERSE * reverse_forward) +
                     (ODOM_KX_YAW * yaw_risk) +
                     (ODOM_K_DUAL_AXIS * dual_axis) +
                     (ODOM_CROSS_AXIS * powf(qvy, ODOM_SPEED_POWER_Y) * odom_clampf(qrx, 0.0f, 1.0f)));
    scale_strafe = 1.0f /
                   (1.0f +
                    (ODOM_KY_SPEED * powf(qvy, ODOM_SPEED_POWER_Y)) +
                    (ODOM_KY_RESIDUAL * powf(qry, ODOM_RISK_GAMMA)) +
                    (ODOM_KY_REVERSE * reverse_strafe) +
                    (ODOM_KY_YAW * yaw_risk) +
                    (ODOM_K_DUAL_AXIS * dual_axis) +
                    (ODOM_CROSS_AXIS * powf(qvx, ODOM_SPEED_POWER_X) * odom_clampf(qry, 0.0f, 1.0f)));

    enc_forward = odom_soft_deadband(enc_forward * scale_forward, ODOM_DEAD_FORWARD_MPS);
    enc_strafe = odom_soft_deadband(enc_strafe * scale_strafe, ODOM_DEAD_STRAFE_MPS);

    risk = odom_clampf((0.30f * fmaxf(qvx, qvy)) +
                       (0.45f * fmaxf(qrx, qry)) +
                       (0.15f * fmaxf(reverse_forward, reverse_strafe)) +
                       (0.10f * yaw_risk),
                       0.0f,
                       1.0f);
    alpha_target = ODOM_ALPHA_MAX * risk / ODOM_ALPHA_RISK_REF;
    alpha_beta = ODOM_DT_S / (ODOM_ALPHA_TAU_S + ODOM_DT_S);
    state->alpha += alpha_beta * (alpha_target - state->alpha);

    pred_forward = state->velocity_forward_mps + (accel_forward * ODOM_DT_S);
    pred_strafe = state->velocity_strafe_mps + (accel_strafe * ODOM_DT_S);
    state->velocity_forward_mps = (state->alpha * pred_forward) + ((1.0f - state->alpha) * enc_forward);
    state->velocity_strafe_mps = (state->alpha * pred_strafe) + ((1.0f - state->alpha) * enc_strafe);

    yaw = yaw_delta_rad * ODOM_YAW_GAIN;
    cos_yaw = cosf(yaw);
    sin_yaw = sinf(yaw);
    dx = ((cos_yaw * state->velocity_forward_mps) - (sin_yaw * state->velocity_strafe_mps)) * ODOM_DT_S;
    dy = ((sin_yaw * state->velocity_forward_mps) + (cos_yaw * state->velocity_strafe_mps)) * ODOM_DT_S;

    output->forward_m += dx;
    output->strafe_m += dy;
    output->travel_m += sqrtf((state->velocity_forward_mps * ODOM_DT_S * state->velocity_forward_mps * ODOM_DT_S) +
                              (state->velocity_strafe_mps * ODOM_DT_S * state->velocity_strafe_mps * ODOM_DT_S));

    state->prev_encoder_forward_mps = enc_forward;
    state->prev_encoder_strafe_mps = enc_strafe;
    state->prev_forward_delta_count = forward_delta_count;
    state->prev_strafe_delta_count = strafe_delta_count;
}
