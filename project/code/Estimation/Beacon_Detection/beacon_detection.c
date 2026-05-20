#include "beacon_detection.h"

#define BEACON_DETECTION_IMU_DT_S                 (0.001f)
#define BEACON_DETECTION_IMU_WINDOW_SIZE          (32U)
#define BEACON_DETECTION_WHEEL_HPF_TAU_S          (0.08f)
#define BEACON_DETECTION_STARTUP_TICKS            (1000U)
#define BEACON_DETECTION_EVENT_HOLD_TICKS         (120U)
#define BEACON_DETECTION_ENTER_CONFIRM_TICKS      (1100U)
#define BEACON_DETECTION_ENTER_SEGMENT_GAP_TICKS  (450U)
#define BEACON_DETECTION_PENDING_ENTER_MAX        (4U)
#define BEACON_DETECTION_POST_SPEED_START_TICKS   (120U)
#define BEACON_DETECTION_TAIL_ACCEL_START_TICKS   (200U)
#define BEACON_DETECTION_ON_MIN_TICKS             (280U)
#define BEACON_DETECTION_EXIT_SEARCH_TICKS        (2800U)
#define BEACON_DETECTION_BASELINE_ALPHA           (0.001f)
#define BEACON_DETECTION_BASELINE_FLOOR           (0.05f)
#define BEACON_DETECTION_MOTION_REF_DECAY         (0.995f)
#define BEACON_DETECTION_EPSILON                  (1.0e-6f)

typedef struct
{
    float tilt_deg;
    float tilt_rate_dps;
    float gyro_xy_dps;
    float gyro_z_abs_dps;
    float accel_norm_error_g;
    float impact_score;
} beacon_detection_imu_sample_t;

typedef struct
{
    uint8_t valid;
    uint8_t suppressed;
    uint16_t age_ticks;
    uint16_t quiet_ticks;

    beacon_detection_imu_sample_t peak;
    beacon_bump_location_t peak_location;
    beacon_bump_location_t pre_location;
    beacon_bump_location_t post_location;
    float peak_z;
    float pre_speed_mps;
    float post_min_speed_mps;
    float post_speed_average_mps;
    float tail_accel_max_g;
    float wheel_highpass_sum_max;

} beacon_detection_enter_candidate_t;

typedef struct
{
    uint8_t is_enter_event;
    uint8_t on_beacon_state;
    beacon_detection_imu_sample_t sample;
    beacon_bump_location_t location;
} beacon_detection_event_item_t;

beacon_detection_data_t g_beacon_detection;

static beacon_detection_imu_sample_t s_imu_history[BEACON_DETECTION_IMU_WINDOW_SIZE];
static beacon_detection_enter_candidate_t s_active_enter;
static beacon_detection_enter_candidate_t s_pending_enter[BEACON_DETECTION_PENDING_ENTER_MAX];
static beacon_detection_event_item_t s_event_queue[BEACON_DETECTION_PENDING_ENTER_MAX];
static beacon_detection_imu_sample_t s_exit_peak;
static beacon_detection_imu_sample_t s_exit_min_speed_sample;
static beacon_bump_location_t s_exit_peak_location;
static beacon_bump_location_t s_exit_min_speed_location;
static float s_exit_min_speed_mps;
static uint16_t s_exit_age_ticks;
static uint8_t s_exit_track_active;
static uint8_t s_exit_peak_valid;
static uint8_t s_exit_min_speed_valid;
static beacon_detection_imu_sample_t s_deferred_exit_sample;
static beacon_bump_location_t s_deferred_exit_location;
static uint8_t s_deferred_exit_valid;
static float s_motion_reference_vel[2];
static float s_motion_reference_speed_mps;
static float s_roll_zero_deg;
static float s_pitch_zero_deg;
static float s_prev_roll_deg;
static float s_prev_pitch_deg;
static float s_wheel_lpf[4];
static float s_wheel_highpass[4];
static float s_impact_baseline;
static float s_impact_deviation;
static uint16_t s_startup_ticks;
static uint16_t s_event_ticks;
static uint8_t s_event_queue_head;
static uint8_t s_event_queue_tail;
static uint8_t s_event_queue_count;
static uint8_t s_on_beacon_state;
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

static float beacon_detection_ratio(float value, float threshold)
{
    return value / (threshold + BEACON_DETECTION_EPSILON);
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

    memset(window, 0, sizeof(*window));
    for(i = 0U; i < s_imu_history_count; i++)
    {
        window->tilt_deg = car_math_maxf(window->tilt_deg, s_imu_history[i].tilt_deg);
        window->tilt_rate_dps = car_math_maxf(window->tilt_rate_dps, s_imu_history[i].tilt_rate_dps);
        window->gyro_xy_dps = car_math_maxf(window->gyro_xy_dps, s_imu_history[i].gyro_xy_dps);
        window->gyro_z_abs_dps = car_math_maxf(window->gyro_z_abs_dps, s_imu_history[i].gyro_z_abs_dps);
        window->accel_norm_error_g =
            car_math_maxf(window->accel_norm_error_g, s_imu_history[i].accel_norm_error_g);
        window->impact_score = car_math_maxf(window->impact_score, s_imu_history[i].impact_score);
    }
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
        case BEACON_BUMP_LOCATION_RIGHT:
            mask = BEACON_BUMP_WHEEL_RF_MASK | BEACON_BUMP_WHEEL_RR_MASK;
            break;
        case BEACON_BUMP_LOCATION_LEFT:
            mask = BEACON_BUMP_WHEEL_LF_MASK | BEACON_BUMP_WHEEL_LR_MASK;
            break;
        case BEACON_BUMP_LOCATION_REAR:
            mask = BEACON_BUMP_WHEEL_LR_MASK | BEACON_BUMP_WHEEL_RR_MASK;
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

    if((forward_abs < 0.08f) && (strafe_abs < 0.08f))
    {
        return BEACON_BUMP_LOCATION_UNKNOWN;
    }

    if(forward_abs >= strafe_abs)
    {
        location = (forward_velocity >= 0.0f) ?
                   BEACON_BUMP_LOCATION_FRONT :
                   BEACON_BUMP_LOCATION_REAR;
    }
    else
    {
        location = (strafe_velocity >= 0.0f) ?
                   BEACON_BUMP_LOCATION_LEFT :
                   BEACON_BUMP_LOCATION_RIGHT;
    }

    return location;
}

static beacon_bump_location_t beacon_detection_location_from_reference(void)
{
    return beacon_detection_location_from_motion(s_motion_reference_vel[0],
                                                s_motion_reference_vel[1]);
}

static uint8_t beacon_detection_locations_match(beacon_bump_location_t first,
                                                beacon_bump_location_t second)
{
    if((first == BEACON_BUMP_LOCATION_UNKNOWN) || (second == BEACON_BUMP_LOCATION_UNKNOWN))
    {
        return 0U;
    }

    return (first == second) ? 1U : 0U;
}

static float beacon_detection_wheel_highpass_sum_abs(void)
{
    return car_math_absf(s_wheel_highpass[0]) +
           car_math_absf(s_wheel_highpass[1]) +
           car_math_absf(s_wheel_highpass[2]) +
           car_math_absf(s_wheel_highpass[3]);
}

static void beacon_detection_clear_enter_candidate(beacon_detection_enter_candidate_t *candidate)
{
    memset(candidate, 0, sizeof(*candidate));
    candidate->peak_location = BEACON_BUMP_LOCATION_UNKNOWN;
    candidate->pre_location = BEACON_BUMP_LOCATION_UNKNOWN;
    candidate->post_location = BEACON_BUMP_LOCATION_UNKNOWN;
}

static void beacon_detection_init_enter_candidate(beacon_detection_enter_candidate_t *candidate,
                                                  const beacon_detection_imu_sample_t *window)
{
    beacon_detection_clear_enter_candidate(candidate);
    candidate->valid = 1U;
    candidate->peak = *window;
    candidate->peak_location =
        beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                              g_beacon_detection.vel[1]);
    candidate->pre_location = beacon_detection_location_from_reference();
    candidate->post_location = candidate->peak_location;
    candidate->peak_z = g_beacon_detection.impact_robust_z;
    candidate->pre_speed_mps = car_math_maxf(g_beacon_detection.speed_mps,
                                             s_motion_reference_speed_mps);
    candidate->post_min_speed_mps = g_beacon_detection.speed_mps;
    candidate->post_speed_average_mps = g_beacon_detection.speed_mps;
    candidate->tail_accel_max_g = 0.0f;
    candidate->wheel_highpass_sum_max = beacon_detection_wheel_highpass_sum_abs();
}

static void beacon_detection_observe_enter_candidate(beacon_detection_enter_candidate_t *candidate,
                                                     const beacon_detection_imu_sample_t *window)
{
    beacon_bump_location_t location_now;
    float wheel_highpass_sum;

    if(candidate->valid == 0U)
    {
        return;
    }

    candidate->age_ticks++;
    candidate->quiet_ticks++;
    location_now = beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                                         g_beacon_detection.vel[1]);
    if(window->impact_score > candidate->peak.impact_score)
    {
        candidate->peak = *window;
        candidate->peak_location = location_now;
        candidate->peak_z = g_beacon_detection.impact_robust_z;
        candidate->age_ticks = 0U;
        candidate->quiet_ticks = 0U;
        candidate->pre_location = beacon_detection_location_from_reference();
        candidate->post_location = location_now;
        candidate->pre_speed_mps = car_math_maxf(g_beacon_detection.speed_mps,
                                                 s_motion_reference_speed_mps);
        candidate->post_min_speed_mps = g_beacon_detection.speed_mps;
        candidate->post_speed_average_mps = g_beacon_detection.speed_mps;
        candidate->tail_accel_max_g = 0.0f;
        candidate->wheel_highpass_sum_max = beacon_detection_wheel_highpass_sum_abs();
    }

    if(candidate->age_ticks >= BEACON_DETECTION_POST_SPEED_START_TICKS)
    {
        if(g_beacon_detection.speed_mps < candidate->post_min_speed_mps)
        {
            candidate->post_min_speed_mps = g_beacon_detection.speed_mps;
        }
        candidate->post_speed_average_mps +=
            0.02f * (g_beacon_detection.speed_mps - candidate->post_speed_average_mps);

        if(location_now != BEACON_BUMP_LOCATION_UNKNOWN)
        {
            candidate->post_location = location_now;
        }
    }

    if(candidate->age_ticks >= BEACON_DETECTION_TAIL_ACCEL_START_TICKS)
    {
        candidate->tail_accel_max_g =
            car_math_maxf(candidate->tail_accel_max_g, window->accel_norm_error_g);
    }

    wheel_highpass_sum = beacon_detection_wheel_highpass_sum_abs();
    candidate->wheel_highpass_sum_max =
        car_math_maxf(candidate->wheel_highpass_sum_max, wheel_highpass_sum);
}

static void beacon_detection_update_adaptive_baseline(float impact_score)
{
    float error;

    error = impact_score - s_impact_baseline;
    s_impact_baseline += BEACON_DETECTION_BASELINE_ALPHA * error;
    s_impact_deviation += BEACON_DETECTION_BASELINE_ALPHA *
                          (car_math_absf(error) - s_impact_deviation);
    if(s_impact_deviation < BEACON_DETECTION_BASELINE_FLOOR)
    {
        s_impact_deviation = BEACON_DETECTION_BASELINE_FLOOR;
    }

    g_beacon_detection.impact_baseline = s_impact_baseline;
    g_beacon_detection.impact_robust_z =
        (impact_score - s_impact_baseline) / s_impact_deviation;
}

static uint8_t beacon_detection_is_main_enter_seed(const beacon_detection_imu_sample_t *window)
{
    uint8_t wheel_stop_candidate;
    uint8_t front_fast_candidate;

    wheel_stop_candidate = ((beacon_detection_wheel_highpass_sum_abs() >= 600.0f) &&
                            (s_motion_reference_speed_mps >= 1.40f)) ? 1U : 0U;
    front_fast_candidate = ((window->impact_score >= 1.45f) &&
                            (g_beacon_detection.impact_robust_z >= 4.0f) &&
                            (beacon_detection_wheel_highpass_sum_abs() < 180.0f)) ? 1U : 0U;

    return ((window->impact_score >= 0.75f) ||
            (wheel_stop_candidate != 0U) ||
            (front_fast_candidate != 0U)) ? 1U : 0U;
}

static uint8_t beacon_detection_is_weak_enter_seed(const beacon_detection_imu_sample_t *window)
{
    return ((window->impact_score >= 0.35f) &&
            (window->impact_score < 0.75f)) ? 1U : 0U;
}

static uint8_t beacon_detection_is_enter_valid(const beacon_detection_enter_candidate_t *candidate,
                                               beacon_bump_location_t *location)
{
    uint8_t adaptive_hit;
    uint8_t low_speed_hit;
    uint8_t strong_hit;
    uint8_t weak_low_speed_hit;
    uint8_t front_fast_hit;
    uint8_t all_wheel_stop_hit;
    uint8_t unstable_duplicate;
    uint8_t hard_stop_noise;
    uint8_t spin_noise;

    *location = candidate->peak_location;
    if((*location == BEACON_BUMP_LOCATION_UNKNOWN) &&
       (candidate->pre_location != BEACON_BUMP_LOCATION_UNKNOWN))
    {
        *location = candidate->pre_location;
    }

    adaptive_hit = ((candidate->peak.impact_score >= 1.10f) &&
                    (candidate->pre_speed_mps <= 1.20f) &&
                    (candidate->peak_z >= 2.0f)) ? 1U : 0U;
    low_speed_hit = ((candidate->peak.impact_score >= 1.25f) &&
                     (candidate->post_speed_average_mps <= 0.25f) &&
                     (candidate->peak.gyro_z_abs_dps <= 45.0f)) ? 1U : 0U;
    strong_hit = ((candidate->peak.impact_score >= 1.90f) &&
                  (candidate->peak.gyro_z_abs_dps <= 25.0f)) ? 1U : 0U;
    weak_low_speed_hit = ((candidate->peak.impact_score >= 0.70f) &&
                          (candidate->peak_z >= 2.0f) &&
                          (candidate->peak.gyro_z_abs_dps <= 5.0f) &&
                          (candidate->pre_speed_mps <= 1.00f) &&
                          (candidate->post_speed_average_mps <= 0.80f) &&
                          (candidate->post_min_speed_mps >= 0.05f) &&
                          (candidate->post_min_speed_mps <= 0.80f) &&
                          (candidate->tail_accel_max_g <= 0.18f)) ? 1U : 0U;
    front_fast_hit = ((candidate->peak.impact_score >= 1.45f) &&
                      (candidate->peak_z >= 4.0f) &&
                      (candidate->peak.gyro_z_abs_dps <= 45.0f) &&
                      (candidate->peak.accel_norm_error_g <= 0.20f) &&
                      (candidate->pre_speed_mps >= 1.20f) &&
                      (candidate->post_speed_average_mps <= 0.90f) &&
                      (candidate->post_min_speed_mps <= 0.35f) &&
                      (candidate->tail_accel_max_g <= 0.25f) &&
                      (beacon_detection_locations_match(candidate->peak_location,
                                                        BEACON_BUMP_LOCATION_FRONT) != 0U) &&
                      (beacon_detection_locations_match(candidate->pre_location,
                                                        BEACON_BUMP_LOCATION_FRONT) != 0U) &&
                      (candidate->wheel_highpass_sum_max < 180.0f)) ? 1U : 0U;
    all_wheel_stop_hit = ((candidate->peak.impact_score >= 1.20f) &&
                          (candidate->peak_z >= 4.0f) &&
                          (candidate->peak.gyro_z_abs_dps <= 8.0f) &&
                          (candidate->pre_speed_mps >= 1.40f) &&
                          (candidate->post_speed_average_mps <= 0.10f) &&
                          (candidate->post_min_speed_mps <= 0.05f) &&
                          (candidate->tail_accel_max_g <= 0.08f) &&
                          (candidate->wheel_highpass_sum_max >= 600.0f)) ? 1U : 0U;
    unstable_duplicate = ((candidate->peak.impact_score < 1.30f) &&
                          (candidate->tail_accel_max_g >= 0.24f) &&
                          (candidate->post_min_speed_mps >= 0.60f) &&
                          (beacon_detection_locations_match(candidate->peak_location,
                                                            candidate->pre_location) == 0U) &&
                          (beacon_detection_locations_match(candidate->peak_location,
                                                            candidate->post_location) == 0U) &&
                          (beacon_detection_locations_match(candidate->pre_location,
                                                            candidate->post_location) == 0U)) ? 1U : 0U;

    hard_stop_noise = ((candidate->pre_speed_mps > 1.80f) &&
                       (candidate->post_speed_average_mps < 0.20f) &&
                       (candidate->peak.impact_score < 3.0f)) ? 1U : 0U;
    spin_noise = ((candidate->peak.gyro_z_abs_dps > 40.0f) &&
                  (candidate->peak.impact_score < 1.30f)) ? 1U : 0U;

    if(all_wheel_stop_hit != 0U)
    {
        if(candidate->pre_location != BEACON_BUMP_LOCATION_UNKNOWN)
        {
            *location = candidate->pre_location;
        }
    }
    else if(front_fast_hit != 0U)
    {
        *location = BEACON_BUMP_LOCATION_FRONT;
    }

    return (((adaptive_hit != 0U) ||
             (low_speed_hit != 0U) ||
             (strong_hit != 0U) ||
             (weak_low_speed_hit != 0U) ||
             (front_fast_hit != 0U) ||
             (all_wheel_stop_hit != 0U)) &&
            ((hard_stop_noise == 0U) || (all_wheel_stop_hit != 0U)) &&
            (unstable_duplicate == 0U) &&
            (spin_noise == 0U)) ? 1U : 0U;
}

static uint8_t beacon_detection_is_exit_hit(const beacon_detection_imu_sample_t *window)
{
    return ((window->impact_score >= 0.82f) &&
            (g_beacon_detection.speed_mps > 0.05f)) ? 1U : 0U;
}

static void beacon_detection_latch_event(const beacon_detection_imu_sample_t *window,
                                         uint8_t is_enter_event,
                                         uint8_t on_beacon_state,
                                         beacon_bump_location_t location)
{
    g_beacon_detection.bump_detected = 1U;
    g_beacon_detection.enter_event = is_enter_event;
    g_beacon_detection.exit_event = (is_enter_event == 0U) ? 1U : 0U;
    g_beacon_detection.on_beacon = on_beacon_state;
    g_beacon_detection.confidence = (window->impact_score >= 1.25f) ?
                                    BEACON_BUMP_CONFIDENCE_HIGH :
                                    BEACON_BUMP_CONFIDENCE_LOW;
    g_beacon_detection.location = location;
    g_beacon_detection.wheel_mask = beacon_detection_wheel_mask_from_location(location);
    g_beacon_detection.score = window->impact_score;
    g_beacon_detection.gyro_xy_dps = window->gyro_xy_dps;
    g_beacon_detection.gyro_z_abs_dps = window->gyro_z_abs_dps;
    g_beacon_detection.tilt_rate_dps = window->tilt_rate_dps;
    g_beacon_detection.tilt_deg = window->tilt_deg;
    g_beacon_detection.accel_norm_error_g = window->accel_norm_error_g;
    g_beacon_detection.hold_ticks = BEACON_DETECTION_EVENT_HOLD_TICKS;
    s_event_ticks = BEACON_DETECTION_EVENT_HOLD_TICKS;

    if(is_enter_event != 0U)
    {
        g_beacon_detection.enter_count++;
    }
    else
    {
        g_beacon_detection.exit_count++;
    }
    g_beacon_detection.event_count++;
}

static void beacon_detection_push_event(const beacon_detection_imu_sample_t *window,
                                        uint8_t is_enter_event,
                                        uint8_t on_beacon_state,
                                        beacon_bump_location_t location)
{
    beacon_detection_event_item_t *item;

    if(s_event_queue_count >= BEACON_DETECTION_PENDING_ENTER_MAX)
    {
        item = &s_event_queue[s_event_queue_tail];
    }
    else
    {
        item = &s_event_queue[s_event_queue_tail];
        s_event_queue_tail++;
        if(s_event_queue_tail >= BEACON_DETECTION_PENDING_ENTER_MAX)
        {
            s_event_queue_tail = 0U;
        }
        s_event_queue_count++;
    }

    item->is_enter_event = is_enter_event;
    item->on_beacon_state = on_beacon_state;
    item->sample = *window;
    item->location = location;
}

static void beacon_detection_dispatch_queued_event(void)
{
    beacon_detection_event_item_t *item;

    if((s_event_ticks != 0U) || (s_event_queue_count == 0U))
    {
        return;
    }

    item = &s_event_queue[s_event_queue_head];
    beacon_detection_latch_event(&item->sample,
                                 item->is_enter_event,
                                 item->on_beacon_state,
                                 item->location);
    memset(item, 0, sizeof(*item));
    s_event_queue_head++;
    if(s_event_queue_head >= BEACON_DETECTION_PENDING_ENTER_MAX)
    {
        s_event_queue_head = 0U;
    }
    s_event_queue_count--;
}

static float beacon_detection_candidate_quality(const beacon_detection_enter_candidate_t *candidate)
{
    float quality;

    quality = candidate->peak.impact_score + (0.05f * candidate->peak_z);
    if(candidate->wheel_highpass_sum_max >= 600.0f)
    {
        quality += 0.50f;
    }
    if((candidate->peak.impact_score >= 1.45f) &&
       (candidate->wheel_highpass_sum_max < 180.0f))
    {
        quality += 0.25f;
    }

    return quality;
}

static uint8_t beacon_detection_candidate_is_weak_only(const beacon_detection_enter_candidate_t *candidate)
{
    return ((candidate->peak.impact_score >= 0.70f) &&
            (candidate->peak.impact_score < 1.10f) &&
            (candidate->peak_z >= 2.0f) &&
            (candidate->peak.gyro_z_abs_dps <= 5.0f)) ? 1U : 0U;
}

static void beacon_detection_pending_add(const beacon_detection_enter_candidate_t *candidate)
{
    uint8_t i;
    uint8_t best_index;
    float best_quality;
    float current_quality;

    if(candidate->valid == 0U)
    {
        return;
    }

    for(i = 0U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        if(s_pending_enter[i].valid == 0U)
        {
            s_pending_enter[i] = *candidate;
            return;
        }
    }

    best_index = 0U;
    best_quality = beacon_detection_candidate_quality(&s_pending_enter[0]);
    for(i = 1U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        current_quality = beacon_detection_candidate_quality(&s_pending_enter[i]);
        if(current_quality < best_quality)
        {
            best_quality = current_quality;
            best_index = i;
        }
    }

    if(beacon_detection_candidate_quality(candidate) > best_quality)
    {
        s_pending_enter[best_index] = *candidate;
    }
}

static void beacon_detection_reset_exit_track(void)
{
    memset(&s_exit_peak, 0, sizeof(s_exit_peak));
    memset(&s_exit_min_speed_sample, 0, sizeof(s_exit_min_speed_sample));
    s_exit_peak_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_exit_min_speed_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_exit_min_speed_mps = 999.0f;
    s_exit_age_ticks = 0U;
    s_exit_track_active = 0U;
    s_exit_peak_valid = 0U;
    s_exit_min_speed_valid = 0U;
}

static void beacon_detection_start_exit_track(void)
{
    beacon_detection_reset_exit_track();
    s_exit_track_active = 1U;
}

static void beacon_detection_update_exit_track(const beacon_detection_imu_sample_t *window)
{
    beacon_bump_location_t location_now;

    if(s_exit_track_active == 0U)
    {
        return;
    }

    s_exit_age_ticks++;
    if(s_exit_age_ticks < BEACON_DETECTION_ON_MIN_TICKS)
    {
        return;
    }

    location_now = beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                                         g_beacon_detection.vel[1]);
    if((s_exit_min_speed_valid == 0U) ||
       (g_beacon_detection.speed_mps < s_exit_min_speed_mps))
    {
        s_exit_min_speed_valid = 1U;
        s_exit_min_speed_mps = g_beacon_detection.speed_mps;
        s_exit_min_speed_sample = *window;
        s_exit_min_speed_location = location_now;
    }

    if((beacon_detection_is_exit_hit(window) != 0U) &&
       ((s_exit_peak_valid == 0U) || (window->impact_score > s_exit_peak.impact_score)))
    {
        s_exit_peak_valid = 1U;
        s_exit_peak = *window;
        s_exit_peak_location = location_now;
    }
}

static void beacon_detection_capture_deferred_exit(void)
{
    if(s_exit_track_active == 0U)
    {
        return;
    }

    if(s_exit_peak_valid != 0U)
    {
        s_deferred_exit_sample = s_exit_peak;
        s_deferred_exit_location = s_exit_peak_location;
        s_deferred_exit_valid = 1U;
    }
    else if(s_exit_min_speed_valid != 0U)
    {
        s_deferred_exit_sample = s_exit_min_speed_sample;
        s_deferred_exit_location = s_exit_min_speed_location;
        s_deferred_exit_valid = 1U;
    }
    else
    {
        memset(&s_deferred_exit_sample, 0, sizeof(s_deferred_exit_sample));
        s_deferred_exit_location = BEACON_BUMP_LOCATION_UNKNOWN;
        s_deferred_exit_valid = 1U;
    }
}

static void beacon_detection_finish_exit_if_due(void)
{
    if((s_exit_track_active != 0U) &&
       (s_exit_age_ticks >= BEACON_DETECTION_EXIT_SEARCH_TICKS))
    {
        beacon_detection_capture_deferred_exit();
        beacon_detection_push_event(&s_deferred_exit_sample, 0U, 0U, s_deferred_exit_location);
        s_deferred_exit_valid = 0U;
        beacon_detection_reset_exit_track();
        s_on_beacon_state = 0U;
        g_beacon_detection.on_beacon = s_on_beacon_state;
    }
}

static void beacon_detection_accept_enter(const beacon_detection_enter_candidate_t *candidate,
                                          beacon_bump_location_t location)
{
    if((s_on_beacon_state != 0U) || (s_exit_track_active != 0U))
    {
        beacon_detection_capture_deferred_exit();
        if(s_deferred_exit_valid != 0U)
        {
            beacon_detection_push_event(&s_deferred_exit_sample, 0U, 0U, s_deferred_exit_location);
            s_deferred_exit_valid = 0U;
        }
        beacon_detection_reset_exit_track();
        s_on_beacon_state = 0U;
    }

    beacon_detection_push_event(&candidate->peak, 1U, 1U, location);
    s_on_beacon_state = 1U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
    beacon_detection_start_exit_track();
}

static void beacon_detection_process_pending_enters(void)
{
    uint8_t i;
    uint8_t j;
    beacon_bump_location_t location;

    for(i = 0U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        if(s_pending_enter[i].valid == 0U)
        {
            continue;
        }
        s_pending_enter[i].age_ticks++;

        if(s_pending_enter[i].age_ticks < BEACON_DETECTION_ENTER_CONFIRM_TICKS)
        {
            continue;
        }

        if(s_pending_enter[i].suppressed == 0U)
        {
            for(j = 0U; j < BEACON_DETECTION_PENDING_ENTER_MAX; j++)
            {
                if((i != j) &&
                   (s_pending_enter[j].valid != 0U) &&
                   (s_pending_enter[j].age_ticks < s_pending_enter[i].age_ticks) &&
                   (s_pending_enter[j].peak.impact_score >= 1.10f) &&
                   (beacon_detection_candidate_is_weak_only(&s_pending_enter[i]) != 0U))
                {
                    s_pending_enter[i].suppressed = 1U;
                }
            }
            if((s_active_enter.valid != 0U) &&
               (s_active_enter.peak.impact_score >= 1.10f) &&
               (beacon_detection_candidate_is_weak_only(&s_pending_enter[i]) != 0U))
            {
                s_pending_enter[i].suppressed = 1U;
            }
        }

        if((s_pending_enter[i].suppressed == 0U) &&
           (beacon_detection_is_enter_valid(&s_pending_enter[i], &location) != 0U))
        {
            beacon_detection_accept_enter(&s_pending_enter[i], location);
        }

        beacon_detection_clear_enter_candidate(&s_pending_enter[i]);
    }
}

static void beacon_detection_update_active_enter(const beacon_detection_imu_sample_t *window)
{
    uint8_t main_seed;
    uint8_t weak_seed;
    uint8_t active_main;

    main_seed = beacon_detection_is_main_enter_seed(window);
    weak_seed = beacon_detection_is_weak_enter_seed(window);
    if(s_active_enter.valid == 0U)
    {
        if((main_seed != 0U) || (weak_seed != 0U))
        {
            beacon_detection_init_enter_candidate(&s_active_enter, window);
        }
        return;
    }

    active_main = (s_active_enter.peak.impact_score >= 0.75f) ? 1U : 0U;
    if((active_main == 0U) && (main_seed != 0U))
    {
        beacon_detection_pending_add(&s_active_enter);
        beacon_detection_init_enter_candidate(&s_active_enter, window);
        return;
    }

    beacon_detection_observe_enter_candidate(&s_active_enter, window);
    active_main = (s_active_enter.peak.impact_score >= 0.75f) ? 1U : 0U;
    if(((active_main != 0U) && (main_seed != 0U)) ||
       ((active_main == 0U) && (weak_seed != 0U)))
    {
        s_active_enter.quiet_ticks = 0U;
    }

    if(s_active_enter.quiet_ticks >= BEACON_DETECTION_ENTER_SEGMENT_GAP_TICKS)
    {
        beacon_detection_pending_add(&s_active_enter);
        beacon_detection_clear_enter_candidate(&s_active_enter);
    }
}

static void beacon_detection_advance_realtime(const beacon_detection_imu_sample_t *window)
{
    beacon_detection_update_active_enter(window);
    beacon_detection_update_exit_track(window);
    beacon_detection_finish_exit_if_due();
    beacon_detection_process_pending_enters();
}

void beacon_detection_reset(void)
{
    uint8_t i;

    memset(&g_beacon_detection, 0, sizeof(g_beacon_detection));
    g_beacon_detection.location = BEACON_BUMP_LOCATION_UNKNOWN;
    g_beacon_detection.confidence = BEACON_BUMP_CONFIDENCE_NONE;

    beacon_detection_clear_enter_candidate(&s_active_enter);
    for(i = 0U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        beacon_detection_clear_enter_candidate(&s_pending_enter[i]);
        memset(&s_event_queue[i], 0, sizeof(s_event_queue[i]));
    }
    beacon_detection_reset_exit_track();
    memset(&s_deferred_exit_sample, 0, sizeof(s_deferred_exit_sample));
    s_deferred_exit_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_deferred_exit_valid = 0U;
    s_motion_reference_vel[0] = 0.0f;
    s_motion_reference_vel[1] = 0.0f;
    s_motion_reference_speed_mps = 0.0f;
    s_roll_zero_deg = 0.0f;
    s_pitch_zero_deg = 0.0f;
    s_prev_roll_deg = 0.0f;
    s_prev_pitch_deg = 0.0f;
    s_impact_baseline = 0.20f;
    s_impact_deviation = 0.20f;
    s_startup_ticks = BEACON_DETECTION_STARTUP_TICKS;
    s_event_ticks = 0U;
    s_event_queue_head = 0U;
    s_event_queue_tail = 0U;
    s_event_queue_count = 0U;
    s_on_beacon_state = 0U;
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
    float gyro_score;
    float tilt_rate_score;
    float accel_score;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;

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

    gyro_score = beacon_detection_ratio(sample.gyro_xy_dps, 45.0f);
    tilt_rate_score = beacon_detection_ratio(sample.tilt_rate_dps, 45.0f);
    accel_score = beacon_detection_ratio(sample.accel_norm_error_g, 0.12f);
    sample.impact_score = car_math_minf(car_math_minf(gyro_score, tilt_rate_score), accel_score);

    g_beacon_detection.gyro_xy_dps = sample.gyro_xy_dps;
    g_beacon_detection.gyro_z_abs_dps = sample.gyro_z_abs_dps;
    g_beacon_detection.tilt_rate_dps = sample.tilt_rate_dps;
    g_beacon_detection.tilt_deg = sample.tilt_deg;
    g_beacon_detection.accel_norm_error_g = sample.accel_norm_error_g;

    beacon_detection_push_imu_sample(&sample);
    beacon_detection_get_imu_window_max(&window);
    beacon_detection_update_adaptive_baseline(window.impact_score);

    if(s_startup_ticks > 0U)
    {
        s_startup_ticks--;
    }
    else
    {
        beacon_detection_advance_realtime(&window);
    }

    if(s_event_ticks > 0U)
    {
        s_event_ticks--;
        g_beacon_detection.hold_ticks = s_event_ticks;
        if(0U == s_event_ticks)
        {
            g_beacon_detection.bump_detected = 0U;
            g_beacon_detection.enter_event = 0U;
            g_beacon_detection.exit_event = 0U;
            g_beacon_detection.confidence = BEACON_BUMP_CONFIDENCE_NONE;
            g_beacon_detection.on_beacon = s_on_beacon_state;
            beacon_detection_dispatch_queued_event();
        }
    }
    else
    {
        beacon_detection_dispatch_queued_event();
    }

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

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
    float wheel_highpass_sum;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    g_beacon_detection.vel[0] =
        (left_front + right_front + left_rear + right_rear) *
        (0.25f / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S);
    g_beacon_detection.vel[1] =
        (-left_front + right_front + left_rear - right_rear) *
        (0.25f / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S);
    g_beacon_detection.speed_mps =
        beacon_detection_vec2_norm(g_beacon_detection.vel[0],
                                   g_beacon_detection.vel[1]);
    if(g_beacon_detection.speed_mps > 0.12f)
    {
        s_motion_reference_vel[0] = g_beacon_detection.vel[0];
        s_motion_reference_vel[1] = g_beacon_detection.vel[1];
        s_motion_reference_speed_mps = g_beacon_detection.speed_mps;
    }
    else
    {
        s_motion_reference_vel[0] *= BEACON_DETECTION_MOTION_REF_DECAY;
        s_motion_reference_vel[1] *= BEACON_DETECTION_MOTION_REF_DECAY;
        s_motion_reference_speed_mps *= BEACON_DETECTION_MOTION_REF_DECAY;
    }

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
    wheel_highpass_sum = beacon_detection_wheel_highpass_sum_abs();
    if((s_active_enter.valid != 0U) &&
       (wheel_highpass_sum > s_active_enter.wheel_highpass_sum_max))
    {
        s_active_enter.wheel_highpass_sum_max = wheel_highpass_sum;
    }
}
