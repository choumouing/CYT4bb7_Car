#include "odometer_high_freedom.h"

#include <math.h>
#include <stddef.h>

#define ODOM_HF_UPDATE_DT_S                (0.01f)
#define ODOM_HF_STARTUP_HOLD_TICKS         (50U)
#define ODOM_HF_FORWARD_COUNT_PER_METER    (11287.0f)
#define ODOM_HF_STRAFE_COUNT_PER_METER_ABS (12100.0f)

#define ODOM_HF_DEG_TO_RAD                 (0.017453292519943295f)
#define ODOM_HF_PI                         (3.14159265358979323846f)
#define ODOM_HF_TWO_PI                     (6.28318530717958647692f)
#define ODOM_HF_EPSILON                    (1.0e-6f)

#define ODOM_HF_FAST_SPEED_START_MPS       (0.35f)
#define ODOM_HF_FAST_SPEED_RANGE_MPS       (1.00f)
#define ODOM_HF_DUAL_RATIO_START           (0.15f)
#define ODOM_HF_DUAL_RATIO_RANGE           (0.50f)
#define ODOM_HF_YAW_RATE_REF_DPS           (80.0f)
#define ODOM_HF_YAW_PROJECTION_GAIN        (-0.30f)

#define ODOM_HF_CF_FP                      (1.30232242f)
#define ODOM_HF_CF_FN                      (0.74862410f)
#define ODOM_HF_CF_SP                      (-0.49652230f)
#define ODOM_HF_CF_SN                      (0.25794964f)
#define ODOM_HF_CF_FP_FAST                 (-0.81804909f)
#define ODOM_HF_CF_FN_FAST                 (0.23802903f)
#define ODOM_HF_CF_SP_FAST                 (0.78750857f)
#define ODOM_HF_CF_SN_FAST                 (-0.38733112f)
#define ODOM_HF_CF_FP_DUAL                 (0.49952085f)
#define ODOM_HF_CF_FN_DUAL                 (-0.78582360f)
#define ODOM_HF_CF_SP_DUAL                 (-0.55221557f)
#define ODOM_HF_CF_SN_DUAL                 (0.59020688f)
#define ODOM_HF_CF_FP_YAW                  (3.54574689f)
#define ODOM_HF_CF_FN_YAW                  (1.25208046f)
#define ODOM_HF_CF_SP_YAW                  (0.26910336f)
#define ODOM_HF_CF_SN_YAW                  (0.72310716f)

#define ODOM_HF_CS_FP                      (0.12188119f)
#define ODOM_HF_CS_FN                      (-0.05250789f)
#define ODOM_HF_CS_SP                      (0.82308289f)
#define ODOM_HF_CS_SN                      (1.08350193f)
#define ODOM_HF_CS_FP_FAST                 (-0.13035869f)
#define ODOM_HF_CS_FN_FAST                 (0.08995032f)
#define ODOM_HF_CS_SP_FAST                 (0.22171650f)
#define ODOM_HF_CS_SN_FAST                 (-0.07503716f)
#define ODOM_HF_CS_FP_DUAL                 (0.07562612f)
#define ODOM_HF_CS_FN_DUAL                 (-0.25891603f)
#define ODOM_HF_CS_SP_DUAL                 (-0.20408599f)
#define ODOM_HF_CS_SN_DUAL                 (0.16322013f)
#define ODOM_HF_CS_FP_YAW                  (-1.05012010f)
#define ODOM_HF_CS_FN_YAW                  (-0.40064650f)
#define ODOM_HF_CS_SP_YAW                  (-1.02145046f)
#define ODOM_HF_CS_SN_YAW                  (-1.83732820f)

static float odom_hf_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float odom_hf_minf(float a, float b)
{
    return (a < b) ? a : b;
}

static float odom_hf_clampf(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

static float odom_hf_normalize_angle(float angle)
{
    while(angle > ODOM_HF_PI)
    {
        angle -= ODOM_HF_TWO_PI;
    }

    while(angle < -ODOM_HF_PI)
    {
        angle += ODOM_HF_TWO_PI;
    }

    return angle;
}

static uint8_t odom_hf_is_finite(float value)
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

static float odom_hf_choose_gyro_z_dps(odometer_high_freedom_t *odometer,
                                       float fallback_gyro_z_dps)
{
    float gyro_z_dps;

    if(odometer->gyro_z_sample_count > 0U)
    {
        gyro_z_dps = odometer->gyro_z_sum_dps / (float)odometer->gyro_z_sample_count;
    }
    else
    {
        gyro_z_dps = fallback_gyro_z_dps;
    }

    odometer->gyro_z_sum_dps = 0.0f;
    odometer->gyro_z_sample_count = 0U;

    if(0U == odom_hf_is_finite(gyro_z_dps))
    {
        gyro_z_dps = odometer->last_gyro_z_dps;
    }

    odometer->last_gyro_z_dps = gyro_z_dps;
    return gyro_z_dps;
}

void odometer_high_freedom_init(odometer_high_freedom_t *odometer)
{
    odometer_high_freedom_reset(odometer);
}

void odometer_high_freedom_reset(odometer_high_freedom_t *odometer)
{
    if(odometer == NULL)
    {
        return;
    }

    odometer->output.forward_distance_m = 0.0f;
    odometer->output.strafe_distance_m = 0.0f;
    odometer->output.travel_distance_m = 0.0f;
    odometer->yaw_rad = 0.0f;
    odometer->gyro_z_sum_dps = 0.0f;
    odometer->last_gyro_z_dps = 0.0f;
    odometer->gyro_z_sample_count = 0U;
    odometer->startup_hold_ticks = ODOM_HF_STARTUP_HOLD_TICKS;
}

void odometer_high_freedom_accumulate_gyro_z_1000hz(odometer_high_freedom_t *odometer,
                                                    float gyro_z_dps)
{
    if((odometer == NULL) || (0U == odom_hf_is_finite(gyro_z_dps)))
    {
        return;
    }

    odometer->gyro_z_sum_dps += gyro_z_dps;
    if(odometer->gyro_z_sample_count < 1000U)
    {
        odometer->gyro_z_sample_count++;
    }
}

void odometer_high_freedom_update_100hz(odometer_high_freedom_t *odometer,
                                        float left_front_count,
                                        float right_front_count,
                                        float left_rear_count,
                                        float right_rear_count,
                                        float gyro_z_dps)
{
    float raw_forward_mps;
    float raw_strafe_mps;
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
    float corrected_forward_mps;
    float corrected_strafe_mps;
    float yaw_for_projection;
    float cos_yaw;
    float sin_yaw;
    float forward_delta_m;
    float strafe_delta_m;

    if(odometer == NULL)
    {
        return;
    }

    gyro_z_dps = odom_hf_choose_gyro_z_dps(odometer, gyro_z_dps);

    if(odometer->startup_hold_ticks > 0U)
    {
        odometer->output.forward_distance_m = 0.0f;
        odometer->output.strafe_distance_m = 0.0f;
        odometer->output.travel_distance_m = 0.0f;
        odometer->yaw_rad = 0.0f;
        odometer->startup_hold_ticks--;
        return;
    }

    raw_forward_mps = (left_front_count + right_front_count + left_rear_count + right_rear_count) *
                      0.25f / ODOM_HF_FORWARD_COUNT_PER_METER / ODOM_HF_UPDATE_DT_S;
    raw_strafe_mps = (-left_front_count + right_front_count + left_rear_count - right_rear_count) *
                     0.25f / ODOM_HF_STRAFE_COUNT_PER_METER_ABS / ODOM_HF_UPDATE_DT_S;

    odometer->yaw_rad = odom_hf_normalize_angle(odometer->yaw_rad +
                                                gyro_z_dps * ODOM_HF_DEG_TO_RAD *
                                                ODOM_HF_UPDATE_DT_S);

    fp = (raw_forward_mps > 0.0f) ? raw_forward_mps : 0.0f;
    fn = (raw_forward_mps < 0.0f) ? raw_forward_mps : 0.0f;
    sp = (raw_strafe_mps > 0.0f) ? raw_strafe_mps : 0.0f;
    sn = (raw_strafe_mps < 0.0f) ? raw_strafe_mps : 0.0f;

    forward_fast_weight = odom_hf_clampf((odom_hf_absf(raw_forward_mps) -
                                          ODOM_HF_FAST_SPEED_START_MPS) /
                                             ODOM_HF_FAST_SPEED_RANGE_MPS,
                                         0.0f,
                                         1.0f);
    strafe_fast_weight = odom_hf_clampf((odom_hf_absf(raw_strafe_mps) -
                                         ODOM_HF_FAST_SPEED_START_MPS) /
                                            ODOM_HF_FAST_SPEED_RANGE_MPS,
                                        0.0f,
                                        1.0f);
    speed_norm = sqrtf((raw_forward_mps * raw_forward_mps) +
                       (raw_strafe_mps * raw_strafe_mps));
    dual_ratio = odom_hf_minf(odom_hf_absf(raw_forward_mps),
                              odom_hf_absf(raw_strafe_mps)) /
                 (speed_norm + ODOM_HF_EPSILON);
    dual_weight = odom_hf_clampf((dual_ratio - ODOM_HF_DUAL_RATIO_START) /
                                     ODOM_HF_DUAL_RATIO_RANGE,
                                 0.0f,
                                 1.0f);
    yawrate_weight = odom_hf_clampf(odom_hf_absf(gyro_z_dps) / ODOM_HF_YAW_RATE_REF_DPS,
                                    0.0f,
                                    1.0f);

    corrected_forward_mps =
        (ODOM_HF_CF_FP * fp) + (ODOM_HF_CF_FN * fn) +
        (ODOM_HF_CF_SP * sp) + (ODOM_HF_CF_SN * sn) +
        (ODOM_HF_CF_FP_FAST * fp * forward_fast_weight) +
        (ODOM_HF_CF_FN_FAST * fn * forward_fast_weight) +
        (ODOM_HF_CF_SP_FAST * sp * strafe_fast_weight) +
        (ODOM_HF_CF_SN_FAST * sn * strafe_fast_weight) +
        (ODOM_HF_CF_FP_DUAL * fp * dual_weight) +
        (ODOM_HF_CF_FN_DUAL * fn * dual_weight) +
        (ODOM_HF_CF_SP_DUAL * sp * dual_weight) +
        (ODOM_HF_CF_SN_DUAL * sn * dual_weight) +
        (ODOM_HF_CF_FP_YAW * fp * yawrate_weight) +
        (ODOM_HF_CF_FN_YAW * fn * yawrate_weight) +
        (ODOM_HF_CF_SP_YAW * sp * yawrate_weight) +
        (ODOM_HF_CF_SN_YAW * sn * yawrate_weight);

    corrected_strafe_mps =
        (ODOM_HF_CS_FP * fp) + (ODOM_HF_CS_FN * fn) +
        (ODOM_HF_CS_SP * sp) + (ODOM_HF_CS_SN * sn) +
        (ODOM_HF_CS_FP_FAST * fp * forward_fast_weight) +
        (ODOM_HF_CS_FN_FAST * fn * forward_fast_weight) +
        (ODOM_HF_CS_SP_FAST * sp * strafe_fast_weight) +
        (ODOM_HF_CS_SN_FAST * sn * strafe_fast_weight) +
        (ODOM_HF_CS_FP_DUAL * fp * dual_weight) +
        (ODOM_HF_CS_FN_DUAL * fn * dual_weight) +
        (ODOM_HF_CS_SP_DUAL * sp * dual_weight) +
        (ODOM_HF_CS_SN_DUAL * sn * dual_weight) +
        (ODOM_HF_CS_FP_YAW * fp * yawrate_weight) +
        (ODOM_HF_CS_FN_YAW * fn * yawrate_weight) +
        (ODOM_HF_CS_SP_YAW * sp * yawrate_weight) +
        (ODOM_HF_CS_SN_YAW * sn * yawrate_weight);

    yaw_for_projection = odometer->yaw_rad * ODOM_HF_YAW_PROJECTION_GAIN;
    cos_yaw = cosf(yaw_for_projection);
    sin_yaw = sinf(yaw_for_projection);
    forward_delta_m = ((cos_yaw * corrected_forward_mps) -
                       (sin_yaw * corrected_strafe_mps)) *
                      ODOM_HF_UPDATE_DT_S;
    strafe_delta_m = ((sin_yaw * corrected_forward_mps) +
                      (cos_yaw * corrected_strafe_mps)) *
                     ODOM_HF_UPDATE_DT_S;

    odometer->output.forward_distance_m += forward_delta_m;
    odometer->output.strafe_distance_m += strafe_delta_m;
    odometer->output.travel_distance_m += sqrtf((forward_delta_m * forward_delta_m) +
                                                (strafe_delta_m * strafe_delta_m));
}

const odometer_high_freedom_output_t *odometer_high_freedom_get_output(
    const odometer_high_freedom_t *odometer)
{
    if(odometer == NULL)
    {
        return NULL;
    }

    return &odometer->output;
}
