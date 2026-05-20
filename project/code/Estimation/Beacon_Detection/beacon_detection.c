#include "beacon_detection.h"

#define BEACON_DETECTION_IMU_DT_S                 (0.001f)
#define BEACON_DETECTION_IMU_WINDOW_SIZE          (32U)
#define BEACON_DETECTION_WHEEL_HPF_TAU_S          (0.08f)
#define BEACON_DETECTION_STARTUP_TICKS            (1000U)
#define BEACON_DETECTION_EVENT_HOLD_TICKS         (700U)
#define BEACON_DETECTION_EVENT_COOLDOWN_TICKS     (900U)
#define BEACON_DETECTION_EPSILON                  (1.0e-6f)

typedef struct
{
    float tilt_deg;
    float tilt_rate_dps;
    float gyro_xy_dps;
    float gyro_z_abs_dps;
    float accel_norm_error_g;
} beacon_detection_imu_sample_t;

beacon_detection_data_t g_beacon_detection;

static beacon_detection_imu_sample_t s_imu_history[BEACON_DETECTION_IMU_WINDOW_SIZE];
static float s_roll_zero_deg;
static float s_pitch_zero_deg;
static float s_prev_roll_deg;
static float s_prev_pitch_deg;
static float s_wheel_lpf[4];
static float s_wheel_highpass[4];
static uint16_t s_startup_ticks;
static uint16_t s_cooldown_ticks;
static uint8_t s_imu_history_index;
static uint8_t s_imu_history_count;
static uint8_t s_tilt_ready;

static float beacon_detection_vec2_norm(float x_value, float y_value)
{
    return sqrtf((x_value * x_value) + (y_value * y_value));
}

static float beacon_detection_vec3_norm(float x_value, float y_value, float z_value)
{
    return sqrtf((x_value * x_value) + (y_value * y_value) + (z_value * z_value));
}

static void beacon_detection_clear_history(void)
{
    memset(s_imu_history, 0, sizeof(s_imu_history));
    s_imu_history_index = 0U;
    s_imu_history_count = 0U;
}

static void beacon_detection_push_imu_sample(const beacon_detection_imu_sample_t *sample)
{
    s_imu_history[s_imu_history_index] = *sample;
    s_imu_history_index++;
    if(s_imu_history_index >= BEACON_DETECTION_IMU_WINDOW_SIZE)
    {
        s_imu_history_index = 0U;
    }

    if(s_imu_history_count < BEACON_DETECTION_IMU_WINDOW_SIZE)
    {
        s_imu_history_count++;
    }
}

static void beacon_detection_get_imu_window_max(beacon_detection_imu_sample_t *window)
{
    uint8_t i;

    window->tilt_deg = 0.0f;
    window->tilt_rate_dps = 0.0f;
    window->gyro_xy_dps = 0.0f;
    window->gyro_z_abs_dps = 0.0f;
    window->accel_norm_error_g = 0.0f;

    for(i = 0U; i < s_imu_history_count; i++)
    {
        window->tilt_deg = car_math_maxf(window->tilt_deg, s_imu_history[i].tilt_deg);
        window->tilt_rate_dps = car_math_maxf(window->tilt_rate_dps, s_imu_history[i].tilt_rate_dps);
        window->gyro_xy_dps = car_math_maxf(window->gyro_xy_dps, s_imu_history[i].gyro_xy_dps);
        window->gyro_z_abs_dps = car_math_maxf(window->gyro_z_abs_dps, s_imu_history[i].gyro_z_abs_dps);
        window->accel_norm_error_g =
            car_math_maxf(window->accel_norm_error_g, s_imu_history[i].accel_norm_error_g);
    }
}

static uint8_t beacon_detection_wheel_mask_from_peak(const float wheel_abs[4], float max_abs)
{
    uint8_t mask;
    float threshold;

    mask = 0U;
    threshold = car_math_maxf(max_abs * 0.45f, 8.0f);
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

    forward_abs = car_math_absf(forward_velocity);
    strafe_abs = car_math_absf(strafe_velocity);
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
        location = (forward_velocity >= 0.0f) ? BEACON_BUMP_LOCATION_FRONT : BEACON_BUMP_LOCATION_REAR;
    }
    else if(strafe_abs > (forward_abs * 1.35f))
    {
        location = (strafe_velocity >= 0.0f) ? BEACON_BUMP_LOCATION_LEFT : BEACON_BUMP_LOCATION_RIGHT;
    }

    return location;
}

static beacon_bump_location_t beacon_detection_location_from_wheels(uint8_t *wheel_mask)
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

    wheel_abs[0] = car_math_absf(s_wheel_highpass[0]);
    wheel_abs[1] = car_math_absf(s_wheel_highpass[1]);
    wheel_abs[2] = car_math_absf(s_wheel_highpass[2]);
    wheel_abs[3] = car_math_absf(s_wheel_highpass[3]);
    total = wheel_abs[0] + wheel_abs[1] + wheel_abs[2] + wheel_abs[3] + BEACON_DETECTION_EPSILON;
    max_wheel = car_math_maxf(car_math_maxf(wheel_abs[0], wheel_abs[1]),
                              car_math_maxf(wheel_abs[2], wheel_abs[3]));
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

    return location;
}

static void beacon_detection_latch_event(const beacon_detection_imu_sample_t *window,
                                         float score,
                                         beacon_bump_confidence_t confidence)
{
    beacon_bump_location_t motion_location;

    g_beacon_detection.bump_detected = 1U;
    g_beacon_detection.confidence = confidence;
    g_beacon_detection.score = score;
    g_beacon_detection.gyro_xy_dps = window->gyro_xy_dps;
    g_beacon_detection.gyro_z_abs_dps = window->gyro_z_abs_dps;
    g_beacon_detection.tilt_rate_dps = window->tilt_rate_dps;
    g_beacon_detection.tilt_deg = window->tilt_deg;
    g_beacon_detection.accel_norm_error_g = window->accel_norm_error_g;
    g_beacon_detection.location = beacon_detection_location_from_wheels(&g_beacon_detection.wheel_mask);

    motion_location = beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                                            g_beacon_detection.vel[1]);
    if((g_beacon_detection.speed_mps > 0.25f) &&
       (motion_location != BEACON_BUMP_LOCATION_UNKNOWN))
    {
        g_beacon_detection.location = motion_location;
        g_beacon_detection.wheel_mask = beacon_detection_wheel_mask_from_location(motion_location);
    }

    g_beacon_detection.event_count++;
    g_beacon_detection.hold_ticks = BEACON_DETECTION_EVENT_HOLD_TICKS;
    s_cooldown_ticks = BEACON_DETECTION_EVENT_COOLDOWN_TICKS;
}

void beacon_detection_reset(void)
{
    uint8_t i;

    memset(&g_beacon_detection, 0, sizeof(g_beacon_detection));
    g_beacon_detection.location = BEACON_BUMP_LOCATION_UNKNOWN;
    g_beacon_detection.confidence = BEACON_BUMP_CONFIDENCE_NONE;

    s_roll_zero_deg = 0.0f;
    s_pitch_zero_deg = 0.0f;
    s_prev_roll_deg = 0.0f;
    s_prev_pitch_deg = 0.0f;
    s_startup_ticks = BEACON_DETECTION_STARTUP_TICKS;
    s_cooldown_ticks = 0U;
    s_tilt_ready = 0U;
    for(i = 0U; i < 4U; i++)
    {
        s_wheel_lpf[i] = 0.0f;
        s_wheel_highpass[i] = 0.0f;
    }
    beacon_detection_clear_history();
}

void beacon_detection_update_1000HZ(void)
{
    beacon_detection_imu_sample_t sample;
    beacon_detection_imu_sample_t window;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float roll_delta;
    float pitch_delta;
    float accel_norm_g;
    float score;
    uint8_t classic_bump;
    uint8_t axis_bump;
    uint8_t edge_bump;
    uint8_t strong_partial_bump;
    uint8_t hand_push_bump;

    accel_x_g = 0.0f;
    accel_y_g = 0.0f;
    accel_z_g = 0.0f;
    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;

    AccelCalibration_GetCorrectedSpecificForceG(&accel_x_g, &accel_y_g, &accel_z_g);
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    roll_deg = g_euler.roll;
    pitch_deg = g_euler.pitch;
    if(0U == s_tilt_ready)
    {
        s_roll_zero_deg = roll_deg;
        s_pitch_zero_deg = pitch_deg;
        s_prev_roll_deg = roll_deg;
        s_prev_pitch_deg = pitch_deg;
        s_tilt_ready = 1U;
    }

    roll_delta = roll_deg - s_roll_zero_deg;
    pitch_delta = pitch_deg - s_pitch_zero_deg;
    sample.tilt_deg = beacon_detection_vec2_norm(roll_delta, pitch_delta);
    sample.tilt_rate_dps =
        beacon_detection_vec2_norm(roll_deg - s_prev_roll_deg,
                                   pitch_deg - s_prev_pitch_deg) /
        BEACON_DETECTION_IMU_DT_S;
    s_prev_roll_deg = roll_deg;
    s_prev_pitch_deg = pitch_deg;

    sample.gyro_xy_dps = beacon_detection_vec2_norm(gyro_x_dps, gyro_y_dps);
    sample.gyro_z_abs_dps = car_math_absf(gyro_z_dps);
    accel_norm_g = beacon_detection_vec3_norm(accel_x_g, accel_y_g, accel_z_g);
    sample.accel_norm_error_g = car_math_absf(accel_norm_g - 1.0f);

    g_beacon_detection.gyro_xy_dps = sample.gyro_xy_dps;
    g_beacon_detection.gyro_z_abs_dps = sample.gyro_z_abs_dps;
    g_beacon_detection.tilt_rate_dps = sample.tilt_rate_dps;
    g_beacon_detection.tilt_deg = sample.tilt_deg;
    g_beacon_detection.accel_norm_error_g = sample.accel_norm_error_g;

    beacon_detection_push_imu_sample(&sample);
    beacon_detection_get_imu_window_max(&window);

    if(s_startup_ticks > 0U)
    {
        s_startup_ticks--;
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

    if(s_cooldown_ticks > 0U)
    {
        s_cooldown_ticks--;
        return;
    }

    classic_bump = ((g_beacon_detection.speed_mps > 0.08f) &&
                    (window.gyro_z_abs_dps < 35.0f) &&
                    (window.gyro_xy_dps > 80.0f) &&
                    (window.tilt_rate_dps > 70.0f) &&
                    (window.tilt_deg > 4.45f) &&
                    (window.accel_norm_error_g > 0.20f)) ? 1U : 0U;
    axis_bump = ((g_beacon_detection.speed_mps > 0.08f) &&
                 (window.gyro_z_abs_dps < 30.0f) &&
                 (window.gyro_xy_dps > 90.0f) &&
                 (window.tilt_rate_dps > 80.0f) &&
                 (window.tilt_deg > 3.75f) &&
                 (window.accel_norm_error_g > 0.24f)) ? 1U : 0U;
    edge_bump = ((g_beacon_detection.speed_mps > 0.45f) &&
                 (window.gyro_z_abs_dps < 35.0f) &&
                 (window.gyro_xy_dps > 75.0f) &&
                 (window.tilt_rate_dps > 75.0f) &&
                 (window.tilt_deg > 5.20f) &&
                 (window.accel_norm_error_g > 0.25f)) ? 1U : 0U;
    strong_partial_bump = ((g_beacon_detection.speed_mps > 0.45f) &&
                           (window.gyro_xy_dps > 115.0f) &&
                           (window.tilt_rate_dps > 95.0f) &&
                           (window.tilt_deg > 6.0f) &&
                           (window.accel_norm_error_g > 0.25f) &&
                           (g_beacon_detection.wheel_highpass_count > 30.0f)) ? 1U : 0U;
    hand_push_bump = ((g_beacon_detection.speed_mps > 0.03f) &&
                      (g_beacon_detection.speed_mps < 0.24f) &&
                      (window.gyro_xy_dps > 18.0f) &&
                      (window.tilt_rate_dps > 22.0f) &&
                      (window.tilt_deg > 3.60f) &&
                      (window.accel_norm_error_g > 0.055f) &&
                      (g_beacon_detection.wheel_highpass_count > 10.0f)) ? 1U : 0U;

    if((classic_bump != 0U) || (axis_bump != 0U) ||
       (edge_bump != 0U) || (strong_partial_bump != 0U) ||
       (hand_push_bump != 0U))
    {
        score = car_math_minf(window.gyro_xy_dps / 80.0f,
                              window.tilt_rate_dps / 70.0f);
        score = car_math_minf(score, car_math_clampf(window.tilt_deg / 4.45f, 0.0f, 4.0f));
        score = car_math_minf(score, car_math_clampf(window.accel_norm_error_g / 0.20f, 0.0f, 4.0f));
        if(strong_partial_bump != 0U)
        {
            score = car_math_maxf(score,
                                  car_math_minf(window.gyro_xy_dps / 115.0f,
                                                g_beacon_detection.wheel_highpass_count / 30.0f));
        }

        beacon_detection_latch_event(&window,
                                     score,
                                     (hand_push_bump != 0U) ?
                                     BEACON_BUMP_CONFIDENCE_LOW :
                                     BEACON_BUMP_CONFIDENCE_HIGH);
    }
    float left_front = encoder_get_left_front_filtered_count();
    float right_front = encoder_get_right_front_filtered_count();
    float left_rear = encoder_get_left_rear_filtered_count();
    float right_rear = encoder_get_right_rear_filtered_count();


    wifi_justfloat(
      tick_1000us_cnt,

      ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z,
      ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z,

      g_imufilter_1000hz.accx, g_imufilter_1000hz.accy, g_imufilter_1000hz.accz,
      g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,

      g_euler.roll, g_euler.pitch, g_euler.yaw,

      left_front, right_front, left_rear, right_rear,

      accel_x_g, accel_y_g, accel_z_g,
      gyro_x_dps, gyro_y_dps, gyro_z_dps,

      sample.tilt_deg,

      g_beacon_detection.bump_detected,
      g_beacon_detection.confidence,
      g_beacon_detection.location,
      g_beacon_detection.wheel_mask,
      g_beacon_detection.score
  );
}

void beacon_detection_update_100HZ(void)
{
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;
    float beta;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    yaw_rad = g_euler.yaw * 0.017453292519943295f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    g_beacon_detection.vel[0] =  cos_yaw * g_odometer.vel[x] + sin_yaw * g_odometer.vel[y];
    g_beacon_detection.vel[1] = -sin_yaw * g_odometer.vel[x] + cos_yaw * g_odometer.vel[y];
    g_beacon_detection.speed_mps =
        beacon_detection_vec2_norm(g_beacon_detection.vel[0],
                                   g_beacon_detection.vel[1]);

    beta = ODOMETER_UPDATE_DT_S / (BEACON_DETECTION_WHEEL_HPF_TAU_S + ODOMETER_UPDATE_DT_S);
    s_wheel_lpf[0] += beta * (left_front - s_wheel_lpf[0]);
    s_wheel_lpf[1] += beta * (right_front - s_wheel_lpf[1]);
    s_wheel_lpf[2] += beta * (left_rear - s_wheel_lpf[2]);
    s_wheel_lpf[3] += beta * (right_rear - s_wheel_lpf[3]);

    s_wheel_highpass[0] = left_front - s_wheel_lpf[0];
    s_wheel_highpass[1] = right_front - s_wheel_lpf[1];
    s_wheel_highpass[2] = left_rear - s_wheel_lpf[2];
    s_wheel_highpass[3] = right_rear - s_wheel_lpf[3];

    g_beacon_detection.wheel_highpass_count = car_math_absf(s_wheel_highpass[0]);
    g_beacon_detection.wheel_highpass_count =
        car_math_maxf(g_beacon_detection.wheel_highpass_count, car_math_absf(s_wheel_highpass[1]));
    g_beacon_detection.wheel_highpass_count =
        car_math_maxf(g_beacon_detection.wheel_highpass_count, car_math_absf(s_wheel_highpass[2]));
    g_beacon_detection.wheel_highpass_count =
        car_math_maxf(g_beacon_detection.wheel_highpass_count, car_math_absf(s_wheel_highpass[3]));
}
