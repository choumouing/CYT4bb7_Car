#include "beacon_detection.h"


#define BEACON_DETECTION_DT_S                  (0.01f)
#define BEACON_DETECTION_FORWARD_COUNT_PER_M   (11287.0f)
#define BEACON_DETECTION_STRAFE_COUNT_PER_M    (12100.0f)
#define BEACON_DETECTION_HISTORY_SIZE          (21U)
#define BEACON_DETECTION_SHORT_WINDOW          (11U)
#define BEACON_DETECTION_WHEEL_HPF_TAU_S       (0.08f)
#define BEACON_DETECTION_STARTUP_HOLD_TICKS    (100U)
#define BEACON_DETECTION_EVENT_HOLD_TICKS      (70U)
#define BEACON_DETECTION_EVENT_COOLDOWN_TICKS  (90U)
#define BEACON_DETECTION_EPSILON               (1.0e-6f)

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float tilt_deg;
    float tilt_rate_dps;
    float gyro_xy_dps;
    float gyro_z_abs_dps;
    float accel_norm_error_g;
    float speed_mps;
    float forward_velocity_mps;
    float strafe_velocity_mps;
    float wheel_highpass_count;
    float wheel_highpass[4];
} beacon_detection_sample_t;

typedef struct
{
    beacon_detection_sample_t history[BEACON_DETECTION_HISTORY_SIZE];
    float roll_zero_deg;
    float pitch_zero_deg;
    float prev_roll_deg;
    float prev_pitch_deg;
    float wheel_lpf[4];
    uint16_t startup_hold_ticks;
    uint16_t cooldown_ticks;
    uint8_t history_index;
    uint8_t history_count;
    uint8_t tilt_ready;
} beacon_detection_filter_t;

beacon_detection_state_t g_beacon_detection;

static beacon_detection_filter_t g_beacon_detection_filter;

static float beacon_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float beacon_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float beacon_minf(float a, float b)
{
    return (a < b) ? a : b;
}

static uint8_t beacon_minu8(uint8_t a, uint8_t b)
{
    return (a < b) ? a : b;
}

static float beacon_clampf(float value, float low, float high)
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

static float beacon_vec2_norm(float x, float y)
{
    return sqrtf((x * x) + (y * y));
}

static float beacon_vec3_norm(float x, float y, float z)
{
    return sqrtf((x * x) + (y * y) + (z * z));
}

static void beacon_detection_history_clear(void)
{
    uint8_t i;
    uint8_t j;

    for(i = 0U; i < BEACON_DETECTION_HISTORY_SIZE; i++)
    {
        g_beacon_detection_filter.history[i].roll_deg = 0.0f;
        g_beacon_detection_filter.history[i].pitch_deg = 0.0f;
        g_beacon_detection_filter.history[i].tilt_deg = 0.0f;
        g_beacon_detection_filter.history[i].tilt_rate_dps = 0.0f;
        g_beacon_detection_filter.history[i].gyro_xy_dps = 0.0f;
        g_beacon_detection_filter.history[i].gyro_z_abs_dps = 0.0f;
        g_beacon_detection_filter.history[i].accel_norm_error_g = 0.0f;
        g_beacon_detection_filter.history[i].speed_mps = 0.0f;
        g_beacon_detection_filter.history[i].forward_velocity_mps = 0.0f;
        g_beacon_detection_filter.history[i].strafe_velocity_mps = 0.0f;
        g_beacon_detection_filter.history[i].wheel_highpass_count = 0.0f;

        for(j = 0U; j < 4U; j++)
        {
            g_beacon_detection_filter.history[i].wheel_highpass[j] = 0.0f;
        }
    }

    g_beacon_detection_filter.history_index = 0U;
    g_beacon_detection_filter.history_count = 0U;
}

static void beacon_detection_push_sample(const beacon_detection_sample_t *sample)
{
    g_beacon_detection_filter.history[g_beacon_detection_filter.history_index] = *sample;
    g_beacon_detection_filter.history_index++;
    if(g_beacon_detection_filter.history_index >= BEACON_DETECTION_HISTORY_SIZE)
    {
        g_beacon_detection_filter.history_index = 0U;
    }

    if(g_beacon_detection_filter.history_count < BEACON_DETECTION_HISTORY_SIZE)
    {
        g_beacon_detection_filter.history_count++;
    }
}

static const beacon_detection_sample_t *beacon_detection_history_at(uint8_t age)
{
    uint8_t index;

    if(age >= g_beacon_detection_filter.history_count)
    {
        return 0;
    }

    index = g_beacon_detection_filter.history_index;
    if(index < (uint8_t)(age + 1U))
    {
        index = (uint8_t)(index + BEACON_DETECTION_HISTORY_SIZE);
    }
    index = (uint8_t)(index - age - 1U);
    return &g_beacon_detection_filter.history[index];
}

static float beacon_detection_window_max(uint8_t window_size,
                                         float (*selector)(const beacon_detection_sample_t *sample))
{
    float value_max;
    uint8_t i;
    uint8_t count;
    const beacon_detection_sample_t *sample;

    value_max = 0.0f;
    count = beacon_minu8(window_size, g_beacon_detection_filter.history_count);
    for(i = 0U; i < count; i++)
    {
        sample = beacon_detection_history_at(i);
        if(sample != 0)
        {
            value_max = beacon_maxf(value_max, selector(sample));
        }
    }

    return value_max;
}

static float beacon_select_tilt(const beacon_detection_sample_t *sample)
{
    return sample->tilt_deg;
}

static float beacon_select_tilt_rate(const beacon_detection_sample_t *sample)
{
    return sample->tilt_rate_dps;
}

static float beacon_select_gyro_xy(const beacon_detection_sample_t *sample)
{
    return sample->gyro_xy_dps;
}

static float beacon_select_gyro_z_abs(const beacon_detection_sample_t *sample)
{
    return sample->gyro_z_abs_dps;
}

static float beacon_select_accel_norm_error(const beacon_detection_sample_t *sample)
{
    return sample->accel_norm_error_g;
}

static float beacon_select_speed(const beacon_detection_sample_t *sample)
{
    return sample->speed_mps;
}

static float beacon_select_wheel_highpass(const beacon_detection_sample_t *sample)
{
    return sample->wheel_highpass_count;
}

static beacon_detection_sample_t beacon_detection_get_sample(void)
{
    beacon_detection_sample_t sample;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float accel_norm_g;
    float roll_delta;
    float pitch_delta;
    float forward_count;
    float strafe_count;
    float forward_velocity;
    float strafe_velocity;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;
    float beta;

    accel_x_g = 0.0f;
    accel_y_g = 0.0f;
    accel_z_g = 0.0f;
    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;

    AccelCalibration_GetCorrectedSpecificForceG(&accel_x_g, &accel_y_g, &accel_z_g);
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    sample.roll_deg = g_euler.roll;
    sample.pitch_deg = g_euler.pitch;
    if(0U == g_beacon_detection_filter.tilt_ready)
    {
        g_beacon_detection_filter.roll_zero_deg = sample.roll_deg;
        g_beacon_detection_filter.pitch_zero_deg = sample.pitch_deg;
        g_beacon_detection_filter.prev_roll_deg = sample.roll_deg;
        g_beacon_detection_filter.prev_pitch_deg = sample.pitch_deg;
        g_beacon_detection_filter.tilt_ready = 1U;
    }

    roll_delta = sample.roll_deg - g_beacon_detection_filter.roll_zero_deg;
    pitch_delta = sample.pitch_deg - g_beacon_detection_filter.pitch_zero_deg;
    sample.tilt_deg = beacon_vec2_norm(roll_delta, pitch_delta);
    sample.tilt_rate_dps =
        beacon_vec2_norm(sample.roll_deg - g_beacon_detection_filter.prev_roll_deg,
                         sample.pitch_deg - g_beacon_detection_filter.prev_pitch_deg) /
        BEACON_DETECTION_DT_S;
    g_beacon_detection_filter.prev_roll_deg = sample.roll_deg;
    g_beacon_detection_filter.prev_pitch_deg = sample.pitch_deg;

    sample.gyro_xy_dps = beacon_vec2_norm(gyro_x_dps, gyro_y_dps);
    sample.gyro_z_abs_dps = beacon_absf(gyro_z_dps);
    accel_norm_g = beacon_vec3_norm(accel_x_g, accel_y_g, accel_z_g);
    sample.accel_norm_error_g = beacon_absf(accel_norm_g - 1.0f);

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    forward_count = (left_front + right_front + left_rear + right_rear) * 0.25f;
    strafe_count = (-left_front + right_front + left_rear - right_rear) * 0.25f;
    forward_velocity = forward_count / BEACON_DETECTION_FORWARD_COUNT_PER_M / BEACON_DETECTION_DT_S;
    strafe_velocity = strafe_count / BEACON_DETECTION_STRAFE_COUNT_PER_M / BEACON_DETECTION_DT_S;
    sample.forward_velocity_mps = forward_velocity;
    sample.strafe_velocity_mps = strafe_velocity;
    sample.speed_mps = beacon_vec2_norm(forward_velocity, strafe_velocity);

    beta = BEACON_DETECTION_DT_S / (BEACON_DETECTION_WHEEL_HPF_TAU_S + BEACON_DETECTION_DT_S);
    g_beacon_detection_filter.wheel_lpf[0] += beta * (left_front - g_beacon_detection_filter.wheel_lpf[0]);
    g_beacon_detection_filter.wheel_lpf[1] += beta * (right_front - g_beacon_detection_filter.wheel_lpf[1]);
    g_beacon_detection_filter.wheel_lpf[2] += beta * (left_rear - g_beacon_detection_filter.wheel_lpf[2]);
    g_beacon_detection_filter.wheel_lpf[3] += beta * (right_rear - g_beacon_detection_filter.wheel_lpf[3]);

    sample.wheel_highpass[0] = left_front - g_beacon_detection_filter.wheel_lpf[0];
    sample.wheel_highpass[1] = right_front - g_beacon_detection_filter.wheel_lpf[1];
    sample.wheel_highpass[2] = left_rear - g_beacon_detection_filter.wheel_lpf[2];
    sample.wheel_highpass[3] = right_rear - g_beacon_detection_filter.wheel_lpf[3];
    sample.wheel_highpass_count = beacon_absf(sample.wheel_highpass[0]);
    sample.wheel_highpass_count =
        beacon_maxf(sample.wheel_highpass_count, beacon_absf(sample.wheel_highpass[1]));
    sample.wheel_highpass_count =
        beacon_maxf(sample.wheel_highpass_count, beacon_absf(sample.wheel_highpass[2]));
    sample.wheel_highpass_count =
        beacon_maxf(sample.wheel_highpass_count, beacon_absf(sample.wheel_highpass[3]));

    return sample;
}

static void beacon_detection_get_peak_wheel(float wheel_peak[4])
{
    float value_max;
    uint8_t i;
    uint8_t j;
    uint8_t count;
    const beacon_detection_sample_t *sample;

    wheel_peak[0] = 0.0f;
    wheel_peak[1] = 0.0f;
    wheel_peak[2] = 0.0f;
    wheel_peak[3] = 0.0f;
    value_max = 0.0f;
    count = beacon_minu8(BEACON_DETECTION_SHORT_WINDOW,
                         g_beacon_detection_filter.history_count);

    for(i = 0U; i < count; i++)
    {
        sample = beacon_detection_history_at(i);
        if((sample != 0) && (sample->wheel_highpass_count >= value_max))
        {
            value_max = sample->wheel_highpass_count;
            for(j = 0U; j < 4U; j++)
            {
                wheel_peak[j] = sample->wheel_highpass[j];
            }
        }
    }
}

static uint8_t beacon_detection_wheel_mask_from_peak(const float wheel_abs[4],
                                                     float max_abs)
{
    uint8_t mask;
    float threshold;

    mask = 0U;
    threshold = beacon_maxf(max_abs * 0.45f, 8.0f);
    if(wheel_abs[0] >= threshold)
    {
        mask |= BEACON_BUMP_WHEEL_LF_MASK;
    }
    if(wheel_abs[1] >= threshold)
    {
        mask |= BEACON_BUMP_WHEEL_RF_MASK;
    }
    if(wheel_abs[2] >= threshold)
    {
        mask |= BEACON_BUMP_WHEEL_LR_MASK;
    }
    if(wheel_abs[3] >= threshold)
    {
        mask |= BEACON_BUMP_WHEEL_RR_MASK;
    }

    return mask;
}

static uint8_t beacon_detection_wheel_mask_from_location(beacon_bump_location_t location)
{
    uint8_t mask;

    mask = 0U;
    switch(location)
    {
        case BEACON_BUMP_LOCATION_FRONT:
            mask = BEACON_BUMP_WHEEL_LF_MASK | BEACON_BUMP_WHEEL_RF_MASK;
            break;
        case BEACON_BUMP_LOCATION_REAR:
            mask = BEACON_BUMP_WHEEL_LR_MASK | BEACON_BUMP_WHEEL_RR_MASK;
            break;
        case BEACON_BUMP_LOCATION_LEFT:
            mask = BEACON_BUMP_WHEEL_LF_MASK | BEACON_BUMP_WHEEL_LR_MASK;
            break;
        case BEACON_BUMP_LOCATION_RIGHT:
            mask = BEACON_BUMP_WHEEL_RF_MASK | BEACON_BUMP_WHEEL_RR_MASK;
            break;
        case BEACON_BUMP_LOCATION_LEFT_FRONT:
            mask = BEACON_BUMP_WHEEL_LF_MASK;
            break;
        case BEACON_BUMP_LOCATION_RIGHT_FRONT:
            mask = BEACON_BUMP_WHEEL_RF_MASK;
            break;
        case BEACON_BUMP_LOCATION_LEFT_REAR:
            mask = BEACON_BUMP_WHEEL_LR_MASK;
            break;
        case BEACON_BUMP_LOCATION_RIGHT_REAR:
            mask = BEACON_BUMP_WHEEL_RR_MASK;
            break;
        case BEACON_BUMP_LOCATION_DIAGONAL_LF_RR:
            mask = BEACON_BUMP_WHEEL_LF_MASK | BEACON_BUMP_WHEEL_RR_MASK;
            break;
        case BEACON_BUMP_LOCATION_DIAGONAL_RF_LR:
            mask = BEACON_BUMP_WHEEL_RF_MASK | BEACON_BUMP_WHEEL_LR_MASK;
            break;
        default:
            mask = 0U;
            break;
    }

    return mask;
}

static beacon_bump_location_t beacon_detection_location_from_motion(float forward_velocity,
                                                                    float strafe_velocity)
{
    float forward_abs;
    float strafe_abs;
    beacon_bump_location_t location;

    forward_abs = beacon_absf(forward_velocity);
    strafe_abs = beacon_absf(strafe_velocity);
    location = BEACON_BUMP_LOCATION_UNKNOWN;

    if((forward_abs > 0.25f) && (strafe_abs > 0.25f))
    {
        if((forward_velocity >= 0.0f) && (strafe_velocity >= 0.0f))
        {
            location = BEACON_BUMP_LOCATION_LEFT_FRONT;
        }
        else if((forward_velocity >= 0.0f) && (strafe_velocity < 0.0f))
        {
            location = BEACON_BUMP_LOCATION_RIGHT_FRONT;
        }
        else if((forward_velocity < 0.0f) && (strafe_velocity >= 0.0f))
        {
            location = BEACON_BUMP_LOCATION_LEFT_REAR;
        }
        else
        {
            location = BEACON_BUMP_LOCATION_RIGHT_REAR;
        }
    }
    else if(forward_abs > (strafe_abs * 1.35f))
    {
        location = (forward_velocity >= 0.0f) ?
                   BEACON_BUMP_LOCATION_FRONT :
                   BEACON_BUMP_LOCATION_REAR;
    }
    else if(strafe_abs > (forward_abs * 1.35f))
    {
        location = (strafe_velocity >= 0.0f) ?
                   BEACON_BUMP_LOCATION_LEFT :
                   BEACON_BUMP_LOCATION_RIGHT;
    }

    return location;
}

static beacon_bump_location_t beacon_detection_location_from_wheels(const float wheel_peak[4],
                                                                    uint8_t *partial_bump,
                                                                    uint8_t *wheel_mask)
{
    float wheel_abs[4];
    float total;
    float front;
    float rear;
    float left;
    float right;
    float diag_lf_rr;
    float diag_rf_lr;
    float max_group;
    float max_wheel;
    beacon_bump_location_t location;

    wheel_abs[0] = beacon_absf(wheel_peak[0]);
    wheel_abs[1] = beacon_absf(wheel_peak[1]);
    wheel_abs[2] = beacon_absf(wheel_peak[2]);
    wheel_abs[3] = beacon_absf(wheel_peak[3]);
    total = wheel_abs[0] + wheel_abs[1] + wheel_abs[2] + wheel_abs[3] + BEACON_DETECTION_EPSILON;

    max_wheel = beacon_maxf(beacon_maxf(wheel_abs[0], wheel_abs[1]),
                            beacon_maxf(wheel_abs[2], wheel_abs[3]));
    *wheel_mask = beacon_detection_wheel_mask_from_peak(wheel_abs, max_wheel);

    front = wheel_abs[0] + wheel_abs[1];
    rear = wheel_abs[2] + wheel_abs[3];
    left = wheel_abs[0] + wheel_abs[2];
    right = wheel_abs[1] + wheel_abs[3];
    diag_lf_rr = wheel_abs[0] + wheel_abs[3];
    diag_rf_lr = wheel_abs[1] + wheel_abs[2];

    location = BEACON_BUMP_LOCATION_FRONT;
    max_group = front;
    if(rear > max_group)
    {
        max_group = rear;
        location = BEACON_BUMP_LOCATION_REAR;
    }
    if(left > max_group)
    {
        max_group = left;
        location = BEACON_BUMP_LOCATION_LEFT;
    }
    if(right > max_group)
    {
        max_group = right;
        location = BEACON_BUMP_LOCATION_RIGHT;
    }
    if(diag_lf_rr > max_group)
    {
        max_group = diag_lf_rr;
        location = BEACON_BUMP_LOCATION_DIAGONAL_LF_RR;
    }
    if(diag_rf_lr > max_group)
    {
        max_group = diag_rf_lr;
        location = BEACON_BUMP_LOCATION_DIAGONAL_RF_LR;
    }

    if((max_group / total) < 0.58f)
    {
        location = BEACON_BUMP_LOCATION_UNKNOWN;
    }

    *partial_bump = ((max_group / total) >= 0.58f) ? 1U : 0U;
    return location;
}

static void beacon_detection_latch_event(float score,
                                         beacon_bump_confidence_t confidence,
                                         float gyro_xy_dps,
                                         float tilt_rate_dps,
                                         float tilt_deg,
                                         float accel_norm_error_g,
                                         float speed_mps,
                                         float forward_velocity_mps,
                                         float strafe_velocity_mps,
                                         float wheel_highpass_count)
{
    float wheel_peak[4];
    beacon_bump_location_t motion_location;

    beacon_detection_get_peak_wheel(wheel_peak);

    g_beacon_detection.bump_detected = 1U;
    g_beacon_detection.confidence = confidence;
    g_beacon_detection.score = score;
    g_beacon_detection.gyro_xy_dps = gyro_xy_dps;
    g_beacon_detection.tilt_rate_dps = tilt_rate_dps;
    g_beacon_detection.tilt_deg = tilt_deg;
    g_beacon_detection.accel_norm_error_g = accel_norm_error_g;
    g_beacon_detection.speed_mps = speed_mps;
    g_beacon_detection.forward_velocity_mps = forward_velocity_mps;
    g_beacon_detection.strafe_velocity_mps = strafe_velocity_mps;
    g_beacon_detection.wheel_highpass_count = wheel_highpass_count;
    g_beacon_detection.location =
        beacon_detection_location_from_wheels(wheel_peak,
                                              &g_beacon_detection.partial_bump,
                                              &g_beacon_detection.wheel_mask);
    motion_location = beacon_detection_location_from_motion(forward_velocity_mps,
                                                            strafe_velocity_mps);
    if((speed_mps > 0.25f) && (motion_location != BEACON_BUMP_LOCATION_UNKNOWN))
    {
        g_beacon_detection.location = motion_location;
        g_beacon_detection.wheel_mask =
            beacon_detection_wheel_mask_from_location(motion_location);
        g_beacon_detection.partial_bump = 1U;
    }

    g_beacon_detection.event_count++;
    g_beacon_detection.hold_ticks = BEACON_DETECTION_EVENT_HOLD_TICKS;
    g_beacon_detection_filter.cooldown_ticks = BEACON_DETECTION_EVENT_COOLDOWN_TICKS;
}

void beacon_detection_init(void)
{
    beacon_detection_reset();
}

void beacon_detection_reset(void)
{
    uint8_t i;

    g_beacon_detection.bump_detected = 0U;
    g_beacon_detection.partial_bump = 0U;
    g_beacon_detection.wheel_mask = 0U;
    g_beacon_detection.location = BEACON_BUMP_LOCATION_UNKNOWN;
    g_beacon_detection.confidence = BEACON_BUMP_CONFIDENCE_NONE;
    g_beacon_detection.event_count = 0U;
    g_beacon_detection.hold_ticks = 0U;
    g_beacon_detection.score = 0.0f;
    g_beacon_detection.speed_mps = 0.0f;
    g_beacon_detection.forward_velocity_mps = 0.0f;
    g_beacon_detection.strafe_velocity_mps = 0.0f;
    g_beacon_detection.gyro_xy_dps = 0.0f;
    g_beacon_detection.tilt_rate_dps = 0.0f;
    g_beacon_detection.tilt_deg = 0.0f;
    g_beacon_detection.accel_norm_error_g = 0.0f;
    g_beacon_detection.wheel_highpass_count = 0.0f;

    g_beacon_detection_filter.roll_zero_deg = 0.0f;
    g_beacon_detection_filter.pitch_zero_deg = 0.0f;
    g_beacon_detection_filter.prev_roll_deg = 0.0f;
    g_beacon_detection_filter.prev_pitch_deg = 0.0f;
    g_beacon_detection_filter.startup_hold_ticks = BEACON_DETECTION_STARTUP_HOLD_TICKS;
    g_beacon_detection_filter.cooldown_ticks = 0U;
    g_beacon_detection_filter.tilt_ready = 0U;
    for(i = 0U; i < 4U; i++)
    {
        g_beacon_detection_filter.wheel_lpf[i] = 0.0f;
    }
    beacon_detection_history_clear();
}

void beacon_detection_update_100HZ(void)
{
    beacon_detection_sample_t sample;
    float gyro_xy_window;
    float gyro_z_window;
    float tilt_rate_window;
    float tilt_window;
    float accel_window;
    float speed_window;
    float wheel_window;
    float score;
    uint8_t classic_bump;
    uint8_t axis_bump;
    uint8_t edge_bump;
    uint8_t strong_partial_bump;
    uint8_t hand_push_bump;

    sample = beacon_detection_get_sample();
    beacon_detection_push_sample(&sample);

    if(g_beacon_detection_filter.startup_hold_ticks > 0U)
    {
        g_beacon_detection_filter.startup_hold_ticks--;
        return;
    }

    if(g_beacon_detection.hold_ticks > 0U)
    {
        g_beacon_detection.hold_ticks--;
        if(0U == g_beacon_detection.hold_ticks)
        {
            g_beacon_detection.bump_detected = 0U;
            g_beacon_detection.confidence = BEACON_BUMP_CONFIDENCE_NONE;
        }
    }

    if(g_beacon_detection_filter.cooldown_ticks > 0U)
    {
        g_beacon_detection_filter.cooldown_ticks--;
        return;
    }

    gyro_xy_window = beacon_detection_window_max(BEACON_DETECTION_SHORT_WINDOW,
                                                 beacon_select_gyro_xy);
    gyro_z_window = beacon_detection_window_max(BEACON_DETECTION_SHORT_WINDOW,
                                                beacon_select_gyro_z_abs);
    tilt_rate_window = beacon_detection_window_max(BEACON_DETECTION_SHORT_WINDOW,
                                                   beacon_select_tilt_rate);
    tilt_window = beacon_detection_window_max(BEACON_DETECTION_HISTORY_SIZE,
                                              beacon_select_tilt);
    accel_window = beacon_detection_window_max(BEACON_DETECTION_SHORT_WINDOW,
                                               beacon_select_accel_norm_error);
    speed_window = beacon_detection_window_max(BEACON_DETECTION_SHORT_WINDOW,
                                               beacon_select_speed);
    wheel_window = beacon_detection_window_max(BEACON_DETECTION_SHORT_WINDOW,
                                               beacon_select_wheel_highpass);

    classic_bump = ((speed_window > 0.08f) &&
                    (gyro_z_window < 35.0f) &&
                    (gyro_xy_window > 80.0f) &&
                    (tilt_rate_window > 70.0f) &&
                    (tilt_window > 4.45f) &&
                    (accel_window > 0.20f)) ? 1U : 0U;
    axis_bump = ((speed_window > 0.08f) &&
                 (gyro_z_window < 30.0f) &&
                 (gyro_xy_window > 90.0f) &&
                 (tilt_rate_window > 80.0f) &&
                 (tilt_window > 3.75f) &&
                 (accel_window > 0.24f)) ? 1U : 0U;
    edge_bump = ((speed_window > 0.45f) &&
                 (gyro_z_window < 35.0f) &&
                 (gyro_xy_window > 75.0f) &&
                 (tilt_rate_window > 75.0f) &&
                 (tilt_window > 5.20f) &&
                 (accel_window > 0.25f)) ? 1U : 0U;
    strong_partial_bump = ((speed_window > 0.45f) &&
                           (gyro_xy_window > 115.0f) &&
                           (tilt_rate_window > 95.0f) &&
                           (tilt_window > 6.0f) &&
                           (accel_window > 0.25f) &&
                           (wheel_window > 30.0f)) ? 1U : 0U;
    hand_push_bump = ((speed_window > 0.03f) &&
                      (speed_window < 0.24f) &&
                      (gyro_xy_window > 18.0f) &&
                      (tilt_rate_window > 22.0f) &&
                      (tilt_window > 3.60f) &&
                      (accel_window > 0.055f) &&
                      (wheel_window > 10.0f)) ? 1U : 0U;

    if((classic_bump != 0U) || (axis_bump != 0U) ||
       (edge_bump != 0U) || (strong_partial_bump != 0U) ||
       (hand_push_bump != 0U))
    {
        score = beacon_minf(gyro_xy_window / 80.0f, tilt_rate_window / 70.0f);
        score = beacon_minf(score, beacon_clampf(tilt_window / 4.45f, 0.0f, 4.0f));
        score = beacon_minf(score, beacon_clampf(accel_window / 0.20f, 0.0f, 4.0f));
        if(strong_partial_bump != 0U)
        {
            score = beacon_maxf(score, beacon_minf(gyro_xy_window / 115.0f,
                                                   wheel_window / 30.0f));
        }

        beacon_detection_latch_event(score,
                                     (hand_push_bump != 0U) ?
                                     BEACON_BUMP_CONFIDENCE_LOW :
                                     BEACON_BUMP_CONFIDENCE_HIGH,
                                     gyro_xy_window,
                                     tilt_rate_window,
                                     tilt_window,
                                     accel_window,
                                     speed_window,
                                     sample.forward_velocity_mps,
                                     sample.strafe_velocity_mps,
                                     wheel_window);
    }
}

const beacon_detection_state_t *beacon_detection_get_state(void)
{
    return &g_beacon_detection;
}
