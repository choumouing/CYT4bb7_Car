#include "beacon_edge_state.h"

#define BEACON_EDGE_IMU_DT_S                 (0.001f)
#define BEACON_EDGE_ODOMETER_DT_S            (0.01f)
#define BEACON_EDGE_FORWARD_COUNT_PER_M      (14750.0f)
#define BEACON_EDGE_STRAFE_COUNT_PER_M       (14000.0f)
#define BEACON_EDGE_WHEEL_HPF_TAU_S          (0.08f)

#define BEACON_EDGE_WINDOW_SIZE              (41U)
#define BEACON_EDGE_WINDOW_HALF_SIZE         (20U)
#define BEACON_EDGE_HISTORY_SIZE             (3072U)
#define BEACON_EDGE_STARTUP_TICKS            (250U)
#define BEACON_EDGE_EVENT_HOLD_TICKS         (80U)
#define BEACON_EDGE_PEAK_CLOSE_TICKS         (24U)
#define BEACON_EDGE_GROUP_GAP_TICKS          (300U)
#define BEACON_EDGE_QUEUE_SIZE               (4U)
#define BEACON_EDGE_BASE_PRE_TICKS           (220U)
#define BEACON_EDGE_BASE_GAP_TICKS           (60U)
#define BEACON_EDGE_VALIDATE_PRE_TICKS       (180U)
#define BEACON_EDGE_VALIDATE_POST_TICKS      (220U)
#define BEACON_EDGE_PEAK_MERGE_TICKS         (130U)
#define BEACON_EDGE_ENTER_BACKDATE_TICKS     (350U)
#define BEACON_EDGE_EXIT_EXTEND_TICKS        (500U)

#define BEACON_EDGE_MIN_SPEED_MPS            (0.18f)
#define BEACON_EDGE_PEAK_SCORE_MIN           (1.18f)
#define BEACON_EDGE_PROJECTED_GYRO_SCALE     (34.0f)
#define BEACON_EDGE_GYRO_XY_SCALE            (70.0f)
#define BEACON_EDGE_ACCEL_NORM_SCALE         (0.14f)
#define BEACON_EDGE_ACCEL_XY_LIMIT           (0.72f)
#define BEACON_EDGE_ACCEL_ALONG_LIMIT        (0.66f)
#define BEACON_EDGE_EPSILON                  (1.0e-6f)

typedef struct
{
    float projected_gyro_dps;
    float gyro_xy_dps;
    float gyro_z_abs_dps;
    float accel_norm_error_g;
    float accel_horizontal_g;
    float accel_along_g;
    float score;
} beacon_edge_feature_sample_t;

typedef struct
{
    uint8_t valid;
    uint32_t tick_ms;
    uint8_t active;
    beacon_edge_feature_sample_t feature;
    float roll_deg;
    float pitch_deg;
    float vel[2];
    float speed_mps;
    float wheel_highpass_count;
} beacon_edge_peak_t;

typedef struct
{
    uint8_t valid;
    uint32_t tick_ms;
    float roll_deg;
    float pitch_deg;
    float vel[2];
    float speed_mps;
    float wheel_highpass_count;
} beacon_edge_history_sample_t;

typedef struct
{
    uint8_t active;
    uint8_t edge_count;
    uint8_t enter_emitted;
    uint32_t first_tick_ms;
    uint32_t last_tick_ms;
    beacon_edge_location_t location;

    float base_roll_deg;
    float base_pitch_deg;
    float max_score;
    float max_projected_gyro_dps;
    float max_gyro_xy_dps;
    float max_gyro_z_abs_dps;
    float max_tilt_deg;
    float max_accel_norm_error_g;
    float max_accel_horizontal_g;
    float max_accel_along_abs_g;
    float max_wheel_highpass_count;
    float distance_m;
    float vel_sum[2];
} beacon_edge_group_t;

typedef struct
{
    uint8_t is_enter_event;
    uint8_t on_beacon_state;
    uint32_t event_tick_ms;
    beacon_edge_location_t location;
    beacon_edge_confidence_t confidence;
    float score;
    float tilt_deg;
    float distance_m;
} beacon_edge_event_item_t;

beacon_edge_state_t g_beacon_edge_state;

static beacon_edge_feature_sample_t s_window[BEACON_EDGE_WINDOW_SIZE];
static beacon_edge_history_sample_t s_history[BEACON_EDGE_HISTORY_SIZE];
static beacon_edge_peak_t s_peak;
static beacon_edge_peak_t s_pending_peak;
static beacon_edge_group_t s_group;
static beacon_edge_event_item_t s_event_queue[BEACON_EDGE_QUEUE_SIZE];

static float s_wheel_lpf[4];
static float s_wheel_highpass[4];
static uint32_t s_tick_ms;
static uint16_t s_startup_ticks;
static uint16_t s_peak_below_ticks;
static uint8_t s_window_index;
static uint8_t s_window_count;
static uint16_t s_history_index;
static uint16_t s_history_count;
static uint8_t s_event_head;
static uint8_t s_event_tail;
static uint8_t s_event_count;

static float beacon_edge_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float beacon_edge_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float beacon_edge_clampf(float value, float min_value, float max_value)
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

static float beacon_edge_norm2(float x_value, float y_value)
{
    return sqrtf((x_value * x_value) + (y_value * y_value));
}

static float beacon_edge_norm3(float x_value, float y_value, float z_value)
{
    return sqrtf((x_value * x_value) + (y_value * y_value) + (z_value * z_value));
}

static uint32_t beacon_edge_backdate_tick(uint32_t tick_ms, uint32_t backdate_ticks)
{
    return (tick_ms > backdate_ticks) ? (tick_ms - backdate_ticks) : 0U;
}

static beacon_edge_location_t beacon_edge_location_from_velocity(float forward_mps,
                                                                 float strafe_mps)
{
    if((beacon_edge_absf(forward_mps) < BEACON_EDGE_MIN_SPEED_MPS) &&
       (beacon_edge_absf(strafe_mps) < BEACON_EDGE_MIN_SPEED_MPS))
    {
        return BEACON_EDGE_LOCATION_UNKNOWN;
    }

    if(beacon_edge_absf(forward_mps) >= beacon_edge_absf(strafe_mps))
    {
        return (forward_mps >= 0.0f) ?
               BEACON_EDGE_LOCATION_FRONT :
               BEACON_EDGE_LOCATION_REAR;
    }

    return (strafe_mps >= 0.0f) ?
           BEACON_EDGE_LOCATION_LEFT :
           BEACON_EDGE_LOCATION_RIGHT;
}

static void beacon_edge_window_push(const beacon_edge_feature_sample_t *sample)
{
    s_window[s_window_index] = *sample;
    s_window_index++;
    if(s_window_index >= BEACON_EDGE_WINDOW_SIZE)
    {
        s_window_index = 0U;
    }

    if(s_window_count < BEACON_EDGE_WINDOW_SIZE)
    {
        s_window_count++;
    }
}

static void beacon_edge_window_max(beacon_edge_feature_sample_t *output)
{
    uint8_t i;

    memset(output, 0, sizeof(*output));
    for(i = 0U; i < s_window_count; i++)
    {
        output->projected_gyro_dps =
            beacon_edge_maxf(output->projected_gyro_dps, s_window[i].projected_gyro_dps);
        output->gyro_xy_dps =
            beacon_edge_maxf(output->gyro_xy_dps, s_window[i].gyro_xy_dps);
        output->gyro_z_abs_dps =
            beacon_edge_maxf(output->gyro_z_abs_dps, s_window[i].gyro_z_abs_dps);
        output->accel_norm_error_g =
            beacon_edge_maxf(output->accel_norm_error_g, s_window[i].accel_norm_error_g);
        output->accel_horizontal_g =
            beacon_edge_maxf(output->accel_horizontal_g, s_window[i].accel_horizontal_g);
        output->accel_along_g =
            beacon_edge_maxf(output->accel_along_g, beacon_edge_absf(s_window[i].accel_along_g));
    }
}

static uint16_t beacon_edge_history_prev_index(uint16_t index)
{
    return (index == 0U) ? (BEACON_EDGE_HISTORY_SIZE - 1U) : (uint16_t)(index - 1U);
}

static uint16_t beacon_edge_history_index_by_age(uint16_t age)
{
    uint16_t index;

    index = s_history_index;
    do
    {
        index = beacon_edge_history_prev_index(index);
        if(age == 0U)
        {
            break;
        }
        age--;
    } while(1);

    return index;
}

static void beacon_edge_history_push(const beacon_edge_history_sample_t *sample)
{
    s_history[s_history_index] = *sample;
    s_history_index++;
    if(s_history_index >= BEACON_EDGE_HISTORY_SIZE)
    {
        s_history_index = 0U;
    }

    if(s_history_count < BEACON_EDGE_HISTORY_SIZE)
    {
        s_history_count++;
    }
}

static const beacon_edge_history_sample_t *beacon_edge_history_get_by_age(uint16_t age)
{
    if(age >= s_history_count)
    {
        return 0;
    }

    return &s_history[beacon_edge_history_index_by_age(age)];
}

static void beacon_edge_history_window_stats(uint32_t start_tick_ms,
                                             uint32_t end_tick_ms,
                                             float *base_roll_deg,
                                             float *base_pitch_deg,
                                             float *max_tilt_deg,
                                             float *distance_m,
                                             float *max_wheel_highpass_count)
{
    const beacon_edge_history_sample_t *sample;
    uint16_t age;
    uint16_t base_count;
    uint16_t validate_count;
    uint32_t base_start_tick_ms;
    uint32_t base_end_tick_ms;
    uint32_t validate_start_tick_ms;
    uint32_t validate_end_tick_ms;
    float roll_sum;
    float pitch_sum;
    float tilt_deg;

    base_start_tick_ms = (start_tick_ms > BEACON_EDGE_BASE_PRE_TICKS) ?
                         (start_tick_ms - BEACON_EDGE_BASE_PRE_TICKS) : 0U;
    base_end_tick_ms = (start_tick_ms > BEACON_EDGE_BASE_GAP_TICKS) ?
                       (start_tick_ms - BEACON_EDGE_BASE_GAP_TICKS) : start_tick_ms;
    validate_start_tick_ms = (start_tick_ms > BEACON_EDGE_VALIDATE_PRE_TICKS) ?
                             (start_tick_ms - BEACON_EDGE_VALIDATE_PRE_TICKS) : 0U;
    validate_end_tick_ms = end_tick_ms + BEACON_EDGE_VALIDATE_POST_TICKS;

    roll_sum = 0.0f;
    pitch_sum = 0.0f;
    base_count = 0U;

    for(age = 0U; age < s_history_count; age++)
    {
        sample = beacon_edge_history_get_by_age(age);
        if((sample == 0) || (sample->valid == 0U))
        {
            continue;
        }

        if(sample->tick_ms < base_start_tick_ms)
        {
            break;
        }

        if((sample->tick_ms >= base_start_tick_ms) &&
           (sample->tick_ms < base_end_tick_ms))
        {
            roll_sum += sample->roll_deg;
            pitch_sum += sample->pitch_deg;
            base_count++;
        }
    }

    if(base_count != 0U)
    {
        *base_roll_deg = roll_sum / (float)base_count;
        *base_pitch_deg = pitch_sum / (float)base_count;
    }

    *max_tilt_deg = 0.0f;
    *distance_m = 0.0f;
    *max_wheel_highpass_count = 0.0f;
    validate_count = 0U;

    for(age = 0U; age < s_history_count; age++)
    {
        sample = beacon_edge_history_get_by_age(age);
        if((sample == 0) || (sample->valid == 0U))
        {
            continue;
        }

        if(sample->tick_ms < validate_start_tick_ms)
        {
            break;
        }

        if((sample->tick_ms >= validate_start_tick_ms) &&
           (sample->tick_ms <= validate_end_tick_ms))
        {
            tilt_deg = beacon_edge_norm2(sample->roll_deg - *base_roll_deg,
                                         sample->pitch_deg - *base_pitch_deg);
            *max_tilt_deg = beacon_edge_maxf(*max_tilt_deg, tilt_deg);
            *distance_m += sample->speed_mps * BEACON_EDGE_IMU_DT_S;
            *max_wheel_highpass_count =
                beacon_edge_maxf(*max_wheel_highpass_count,
                                 sample->wheel_highpass_count);
            validate_count++;
        }
    }

    if(validate_count == 0U)
    {
        *distance_m = 0.0f;
    }
}

static float beacon_edge_score(const beacon_edge_feature_sample_t *window)
{
    float projected_score;
    float gyro_score;

    projected_score =
        beacon_edge_clampf(window->projected_gyro_dps / BEACON_EDGE_PROJECTED_GYRO_SCALE, 0.0f, 3.0f) +
        0.48f * beacon_edge_clampf(window->accel_norm_error_g / BEACON_EDGE_ACCEL_NORM_SCALE, 0.0f, 3.0f);
    gyro_score =
        beacon_edge_clampf(window->gyro_xy_dps / BEACON_EDGE_GYRO_XY_SCALE, 0.0f, 3.0f) +
        0.35f * beacon_edge_clampf(window->accel_norm_error_g / 0.18f, 0.0f, 3.0f);

    return beacon_edge_maxf(projected_score, gyro_score);
}

static uint8_t beacon_edge_peak_is_allowed(const beacon_edge_peak_t *peak)
{
    if(peak->speed_mps < BEACON_EDGE_MIN_SPEED_MPS)
    {
        return 0U;
    }

    if((peak->feature.gyro_z_abs_dps > 125.0f) &&
       (peak->feature.score < 3.2f))
    {
        return 0U;
    }

    if((beacon_edge_absf(peak->feature.accel_along_g) > BEACON_EDGE_ACCEL_ALONG_LIMIT) &&
       (peak->feature.score < 3.4f))
    {
        return 0U;
    }

    if((peak->feature.accel_horizontal_g > BEACON_EDGE_ACCEL_XY_LIMIT) &&
       (peak->feature.score < 3.2f))
    {
        return 0U;
    }

    return 1U;
}

static void beacon_edge_add_peak_to_group(const beacon_edge_peak_t *peak);

static void beacon_edge_push_event(uint8_t is_enter_event,
                                   uint8_t on_beacon_state,
                                   uint32_t event_tick_ms,
                                   beacon_edge_location_t location,
                                   beacon_edge_confidence_t confidence,
                                   float score,
                                   float tilt_deg,
                                   float distance_m)
{
    beacon_edge_event_item_t *item;

    if(s_event_count >= BEACON_EDGE_QUEUE_SIZE)
    {
        item = &s_event_queue[s_event_tail];
    }
    else
    {
        item = &s_event_queue[s_event_tail];
        s_event_tail++;
        if(s_event_tail >= BEACON_EDGE_QUEUE_SIZE)
        {
            s_event_tail = 0U;
        }
        s_event_count++;
    }

    item->is_enter_event = is_enter_event;
    item->on_beacon_state = on_beacon_state;
    item->event_tick_ms = event_tick_ms;
    item->location = location;
    item->confidence = confidence;
    item->score = score;
    item->tilt_deg = tilt_deg;
    item->distance_m = distance_m;
}

static uint8_t beacon_edge_pending_peak_is_close(const beacon_edge_peak_t *peak)
{
    if(s_pending_peak.valid == 0U)
    {
        return 0U;
    }

    if(peak->tick_ms >= s_pending_peak.tick_ms)
    {
        return ((peak->tick_ms - s_pending_peak.tick_ms) < BEACON_EDGE_PEAK_MERGE_TICKS) ?
               1U : 0U;
    }

    return ((s_pending_peak.tick_ms - peak->tick_ms) < BEACON_EDGE_PEAK_MERGE_TICKS) ?
           1U : 0U;
}

static void beacon_edge_flush_pending_peak(void)
{
    if(s_pending_peak.valid == 0U)
    {
        return;
    }

    beacon_edge_add_peak_to_group(&s_pending_peak);
    memset(&s_pending_peak, 0, sizeof(s_pending_peak));
}

static void beacon_edge_update_group_stats(void)
{
    float base_roll_deg;
    float base_pitch_deg;
    float max_tilt_deg;
    float distance_m;
    float max_wheel_highpass_count;

    if(s_group.active == 0U)
    {
        return;
    }

    base_roll_deg = s_group.base_roll_deg;
    base_pitch_deg = s_group.base_pitch_deg;
    beacon_edge_history_window_stats(s_group.first_tick_ms,
                                     s_group.last_tick_ms,
                                     &base_roll_deg,
                                     &base_pitch_deg,
                                     &max_tilt_deg,
                                     &distance_m,
                                     &max_wheel_highpass_count);
    s_group.base_roll_deg = base_roll_deg;
    s_group.base_pitch_deg = base_pitch_deg;
    s_group.max_tilt_deg = max_tilt_deg;
    s_group.distance_m = distance_m;
    s_group.max_wheel_highpass_count = max_wheel_highpass_count;
}

static void beacon_edge_latch_event(const beacon_edge_event_item_t *item)
{
    g_beacon_edge_state.bump_detected = 1U;
    g_beacon_edge_state.enter_event = item->is_enter_event;
    g_beacon_edge_state.exit_event = (item->is_enter_event == 0U) ? 1U : 0U;
    g_beacon_edge_state.on_beacon = item->on_beacon_state;
    g_beacon_edge_state.location = item->location;
    g_beacon_edge_state.confidence = item->confidence;
    g_beacon_edge_state.event_tick_ms = item->event_tick_ms;
    g_beacon_edge_state.score = item->score;
    g_beacon_edge_state.tilt_deg = item->tilt_deg;
    g_beacon_edge_state.group_distance_m = item->distance_m;
    g_beacon_edge_state.hold_ticks = BEACON_EDGE_EVENT_HOLD_TICKS;

    if(item->is_enter_event != 0U)
    {
        g_beacon_edge_state.enter_count++;
        g_beacon_edge_state.enter_tick_ms = item->event_tick_ms;
    }
    else
    {
        g_beacon_edge_state.exit_count++;
        g_beacon_edge_state.exit_tick_ms = item->event_tick_ms;
    }
    g_beacon_edge_state.event_count++;
}

static void beacon_edge_dispatch_event(void)
{
    beacon_edge_event_item_t item;

    if(s_event_count == 0U)
    {
        return;
    }

    if((g_beacon_edge_state.hold_ticks != 0U) &&
       (s_event_queue[s_event_head].is_enter_event == 0U))
    {
        return;
    }

    item = s_event_queue[s_event_head];
    memset(&s_event_queue[s_event_head], 0, sizeof(s_event_queue[s_event_head]));
    s_event_head++;
    if(s_event_head >= BEACON_EDGE_QUEUE_SIZE)
    {
        s_event_head = 0U;
    }
    s_event_count--;

    beacon_edge_latch_event(&item);
}

static void beacon_edge_clear_group(void)
{
    memset(&s_group, 0, sizeof(s_group));
    s_group.location = BEACON_EDGE_LOCATION_UNKNOWN;
}

static uint8_t beacon_edge_group_is_valid(const beacon_edge_group_t *group)
{
    uint8_t valid;

    valid = 0U;
    if((group->max_tilt_deg >= 2.55f) &&
       (group->max_score >= 2.0f) &&
       (group->max_accel_along_abs_g <= 0.68f) &&
       (group->max_accel_horizontal_g <= 0.72f))
    {
        valid = 1U;
    }

    if((group->max_tilt_deg >= 3.10f) &&
       (group->max_score >= 1.45f) &&
       (group->edge_count >= 2U) &&
       (group->max_accel_along_abs_g <= 0.68f) &&
       (group->max_accel_horizontal_g <= 0.72f))
    {
        valid = 1U;
    }

    if((group->max_tilt_deg >= 2.35f) &&
       (group->max_score >= 3.35f) &&
       (group->max_accel_along_abs_g <= 0.70f) &&
       (group->max_accel_horizontal_g <= 0.75f))
    {
        valid = 1U;
    }

    if((group->max_tilt_deg >= 5.50f) &&
       (group->max_score >= 1.70f) &&
       (group->max_accel_along_abs_g <= 0.20f) &&
       (group->max_accel_horizontal_g <= 0.20f))
    {
        valid = 1U;
    }

    if((group->max_tilt_deg >= 4.80f) &&
       (group->max_score >= 4.10f) &&
       (group->max_accel_along_abs_g <= 0.95f) &&
       (group->max_accel_horizontal_g <= 1.05f))
    {
        valid = 1U;
    }

    if((group->distance_m > 2.8f) && (group->max_score < 3.4f))
    {
        valid = 0U;
    }

    if(group->distance_m > 6.0f)
    {
        valid = 0U;
    }

    if((group->max_wheel_highpass_count > 260.0f) &&
       (group->max_score < 3.4f))
    {
        valid = 0U;
    }

    if(group->first_tick_ms <= BEACON_EDGE_STARTUP_TICKS)
    {
        valid = 0U;
    }

    return valid;
}

static void beacon_edge_try_emit_enter(void)
{
    beacon_edge_confidence_t confidence;
    uint32_t enter_tick_ms;

    if((s_group.active == 0U) || (s_group.enter_emitted != 0U))
    {
        return;
    }

    beacon_edge_update_group_stats();
    if(beacon_edge_group_is_valid(&s_group) == 0U)
    {
        return;
    }

    confidence = (s_group.max_score >= 3.0f) ?
                 BEACON_EDGE_CONFIDENCE_HIGH :
                 BEACON_EDGE_CONFIDENCE_LOW;
    enter_tick_ms = beacon_edge_backdate_tick(s_group.first_tick_ms,
                                              BEACON_EDGE_ENTER_BACKDATE_TICKS);
    beacon_edge_push_event(1U, 1U, enter_tick_ms,
                           s_group.location, confidence,
                           s_group.max_score, s_group.max_tilt_deg,
                           s_group.distance_m);
    s_group.enter_emitted = 1U;
}

static void beacon_edge_finish_group(void)
{
    beacon_edge_confidence_t confidence;
    uint8_t valid;
    uint32_t enter_tick_ms;

    if(s_group.active == 0U)
    {
        return;
    }

    beacon_edge_update_group_stats();
    valid = beacon_edge_group_is_valid(&s_group);

    if((valid != 0U) || (s_group.enter_emitted != 0U))
    {
        confidence = (s_group.max_score >= 3.0f) ?
                     BEACON_EDGE_CONFIDENCE_HIGH :
                     BEACON_EDGE_CONFIDENCE_LOW;

        if(s_group.enter_emitted == 0U)
        {
            enter_tick_ms = beacon_edge_backdate_tick(s_group.first_tick_ms,
                                                      BEACON_EDGE_ENTER_BACKDATE_TICKS);
            beacon_edge_push_event(1U, 1U, enter_tick_ms,
                                   s_group.location, confidence,
                                   s_group.max_score, s_group.max_tilt_deg,
                                   s_group.distance_m);
        }

        beacon_edge_push_event(0U, 0U,
                               s_group.last_tick_ms + BEACON_EDGE_EXIT_EXTEND_TICKS,
                               s_group.location, confidence,
                               s_group.max_score, s_group.max_tilt_deg,
                               s_group.distance_m);
    }

    beacon_edge_clear_group();
}

static void beacon_edge_start_group(const beacon_edge_peak_t *peak)
{
    beacon_edge_clear_group();
    s_group.active = 1U;
    s_group.first_tick_ms = peak->tick_ms;
    s_group.last_tick_ms = peak->tick_ms;
    s_group.base_roll_deg = peak->roll_deg;
    s_group.base_pitch_deg = peak->pitch_deg;
    s_group.location = beacon_edge_location_from_velocity(peak->vel[0], peak->vel[1]);
}

static void beacon_edge_add_peak_to_group(const beacon_edge_peak_t *peak)
{
    if((s_group.active != 0U) &&
       ((peak->tick_ms - s_group.last_tick_ms) > BEACON_EDGE_GROUP_GAP_TICKS))
    {
        beacon_edge_finish_group();
    }

    if(s_group.active == 0U)
    {
        beacon_edge_start_group(peak);
    }

    s_group.edge_count++;
    s_group.last_tick_ms = peak->tick_ms;
    s_group.max_score = beacon_edge_maxf(s_group.max_score, peak->feature.score);
    s_group.max_projected_gyro_dps =
        beacon_edge_maxf(s_group.max_projected_gyro_dps, peak->feature.projected_gyro_dps);
    s_group.max_gyro_xy_dps =
        beacon_edge_maxf(s_group.max_gyro_xy_dps, peak->feature.gyro_xy_dps);
    s_group.max_gyro_z_abs_dps =
        beacon_edge_maxf(s_group.max_gyro_z_abs_dps, peak->feature.gyro_z_abs_dps);
    s_group.max_accel_norm_error_g =
        beacon_edge_maxf(s_group.max_accel_norm_error_g, peak->feature.accel_norm_error_g);
    s_group.max_accel_horizontal_g =
        beacon_edge_maxf(s_group.max_accel_horizontal_g, peak->feature.accel_horizontal_g);
    s_group.max_accel_along_abs_g =
        beacon_edge_maxf(s_group.max_accel_along_abs_g,
                         beacon_edge_absf(peak->feature.accel_along_g));
    s_group.vel_sum[0] += peak->vel[0];
    s_group.vel_sum[1] += peak->vel[1];
    s_group.location =
        beacon_edge_location_from_velocity(s_group.vel_sum[0], s_group.vel_sum[1]);
    beacon_edge_try_emit_enter();
}

static void beacon_edge_submit_peak(void)
{
    if(s_peak.active == 0U)
    {
        return;
    }

    if(beacon_edge_peak_is_allowed(&s_peak) != 0U)
    {
        if(beacon_edge_pending_peak_is_close(&s_peak) != 0U)
        {
            if(s_peak.feature.score > s_pending_peak.feature.score)
            {
                s_pending_peak = s_peak;
            }
        }
        else
        {
            beacon_edge_flush_pending_peak();
            s_pending_peak = s_peak;
            s_pending_peak.valid = 1U;
        }
    }

    memset(&s_peak, 0, sizeof(s_peak));
    s_peak_below_ticks = 0U;
}

static void beacon_edge_update_peak_tracker(const beacon_edge_history_sample_t *center,
                                            const beacon_edge_feature_sample_t *window)
{
    if(window->score >= BEACON_EDGE_PEAK_SCORE_MIN)
    {
        s_peak_below_ticks = 0U;
        if((s_peak.active == 0U) ||
           (window->score > s_peak.feature.score))
        {
            s_peak.active = 1U;
            s_peak.valid = 1U;
            s_peak.tick_ms = center->tick_ms;
            s_peak.feature = *window;
            s_peak.roll_deg = center->roll_deg;
            s_peak.pitch_deg = center->pitch_deg;
            s_peak.vel[0] = center->vel[0];
            s_peak.vel[1] = center->vel[1];
            s_peak.speed_mps = center->speed_mps;
            s_peak.wheel_highpass_count = center->wheel_highpass_count;
        }
        return;
    }

    if(s_peak.active != 0U)
    {
        s_peak_below_ticks++;
        if(s_peak_below_ticks >= BEACON_EDGE_PEAK_CLOSE_TICKS)
        {
            beacon_edge_submit_peak();
        }
    }
}

void beacon_edge_state_reset(void)
{
    uint8_t i;

    memset(&g_beacon_edge_state, 0, sizeof(g_beacon_edge_state));
    memset(s_window, 0, sizeof(s_window));
    memset(s_history, 0, sizeof(s_history));
    memset(&s_peak, 0, sizeof(s_peak));
    memset(&s_pending_peak, 0, sizeof(s_pending_peak));
    memset(s_event_queue, 0, sizeof(s_event_queue));
    beacon_edge_clear_group();

    for(i = 0U; i < 4U; i++)
    {
        s_wheel_lpf[i] = 0.0f;
        s_wheel_highpass[i] = 0.0f;
    }

    s_tick_ms = 0U;
    s_startup_ticks = BEACON_EDGE_STARTUP_TICKS;
    s_peak_below_ticks = 0U;
    s_window_index = 0U;
    s_window_count = 0U;
    s_history_index = 0U;
    s_history_count = 0U;
    s_event_head = 0U;
    s_event_tail = 0U;
    s_event_count = 0U;
}

void beacon_edge_state_update_100HZ(float left_front_count,
                                    float right_front_count,
                                    float left_rear_count,
                                    float right_rear_count)
{
    float beta;

    g_beacon_edge_state.vel[0] =
        (left_front_count + right_front_count + left_rear_count + right_rear_count) *
        (0.25f / BEACON_EDGE_FORWARD_COUNT_PER_M / BEACON_EDGE_ODOMETER_DT_S);
    g_beacon_edge_state.vel[1] =
        (-left_front_count + right_front_count + left_rear_count - right_rear_count) *
        (0.25f / BEACON_EDGE_STRAFE_COUNT_PER_M / BEACON_EDGE_ODOMETER_DT_S);
    g_beacon_edge_state.speed_mps =
        beacon_edge_norm2(g_beacon_edge_state.vel[0], g_beacon_edge_state.vel[1]);

    beta = BEACON_EDGE_ODOMETER_DT_S /
           (BEACON_EDGE_WHEEL_HPF_TAU_S + BEACON_EDGE_ODOMETER_DT_S);
    s_wheel_lpf[0] += beta * (left_front_count - s_wheel_lpf[0]);
    s_wheel_lpf[1] += beta * (right_front_count - s_wheel_lpf[1]);
    s_wheel_lpf[2] += beta * (left_rear_count - s_wheel_lpf[2]);
    s_wheel_lpf[3] += beta * (right_rear_count - s_wheel_lpf[3]);

    s_wheel_highpass[0] = left_front_count - s_wheel_lpf[0];
    s_wheel_highpass[1] = right_front_count - s_wheel_lpf[1];
    s_wheel_highpass[2] = left_rear_count - s_wheel_lpf[2];
    s_wheel_highpass[3] = right_rear_count - s_wheel_lpf[3];

    g_beacon_edge_state.wheel_highpass_count =
        beacon_edge_maxf(beacon_edge_absf(s_wheel_highpass[0]),
        beacon_edge_maxf(beacon_edge_absf(s_wheel_highpass[1]),
        beacon_edge_maxf(beacon_edge_absf(s_wheel_highpass[2]),
                         beacon_edge_absf(s_wheel_highpass[3]))));
}

void beacon_edge_state_update_1000HZ(float accel_x_g,
                                     float accel_y_g,
                                     float accel_z_g,
                                     float gyro_x_dps,
                                     float gyro_y_dps,
                                     float gyro_z_dps,
                                     float roll_deg,
                                     float pitch_deg)
{
    beacon_edge_feature_sample_t sample;
    beacon_edge_feature_sample_t window;
    beacon_edge_history_sample_t history_sample;
    const beacon_edge_history_sample_t *center;
    float speed_safe;
    float forward_weight;
    float strafe_weight;

    s_tick_ms++;
    if(s_startup_ticks > 0U)
    {
        s_startup_ticks--;
    }

    speed_safe = g_beacon_edge_state.speed_mps + BEACON_EDGE_EPSILON;
    forward_weight = beacon_edge_absf(g_beacon_edge_state.vel[0]) / speed_safe;
    strafe_weight = beacon_edge_absf(g_beacon_edge_state.vel[1]) / speed_safe;

    memset(&sample, 0, sizeof(sample));
    sample.projected_gyro_dps =
        beacon_edge_norm2(gyro_y_dps * forward_weight,
                          gyro_x_dps * strafe_weight);
    sample.gyro_xy_dps = beacon_edge_norm2(gyro_x_dps, gyro_y_dps);
    sample.gyro_z_abs_dps = beacon_edge_absf(gyro_z_dps);
    sample.accel_norm_error_g =
        beacon_edge_absf(beacon_edge_norm3(accel_x_g, accel_y_g, accel_z_g) - 1.0f);
    sample.accel_horizontal_g = beacon_edge_norm2(accel_x_g, accel_y_g);
    sample.accel_along_g =
        ((accel_x_g * g_beacon_edge_state.vel[0]) +
         (accel_y_g * g_beacon_edge_state.vel[1])) / speed_safe;

    beacon_edge_window_push(&sample);

    memset(&history_sample, 0, sizeof(history_sample));
    history_sample.valid = 1U;
    history_sample.tick_ms = s_tick_ms;
    history_sample.roll_deg = roll_deg;
    history_sample.pitch_deg = pitch_deg;
    history_sample.vel[0] = g_beacon_edge_state.vel[0];
    history_sample.vel[1] = g_beacon_edge_state.vel[1];
    history_sample.speed_mps = g_beacon_edge_state.speed_mps;
    history_sample.wheel_highpass_count = g_beacon_edge_state.wheel_highpass_count;
    beacon_edge_history_push(&history_sample);

    beacon_edge_window_max(&window);
    window.score = beacon_edge_score(&window);
    center = beacon_edge_history_get_by_age(BEACON_EDGE_WINDOW_HALF_SIZE);

    g_beacon_edge_state.projected_gyro_dps = window.projected_gyro_dps;
    g_beacon_edge_state.gyro_xy_dps = window.gyro_xy_dps;
    g_beacon_edge_state.gyro_z_abs_dps = window.gyro_z_abs_dps;
    g_beacon_edge_state.accel_norm_error_g = window.accel_norm_error_g;
    g_beacon_edge_state.accel_horizontal_g = window.accel_horizontal_g;
    g_beacon_edge_state.accel_along_g = sample.accel_along_g;
    g_beacon_edge_state.score = window.score;

    if((s_startup_ticks == 0U) &&
       (center != 0) &&
       (center->valid != 0U))
    {
        beacon_edge_update_peak_tracker(center, &window);
        if((s_pending_peak.valid != 0U) &&
           ((s_tick_ms - s_pending_peak.tick_ms) >
            (BEACON_EDGE_PEAK_MERGE_TICKS + BEACON_EDGE_WINDOW_HALF_SIZE)))
        {
            beacon_edge_flush_pending_peak();
        }
        beacon_edge_try_emit_enter();
        if((s_group.active != 0U) &&
           ((s_tick_ms - s_group.last_tick_ms) >
            (BEACON_EDGE_GROUP_GAP_TICKS + BEACON_EDGE_VALIDATE_POST_TICKS +
             BEACON_EDGE_WINDOW_HALF_SIZE)))
        {
            beacon_edge_flush_pending_peak();
            if((s_group.active != 0U) &&
               ((s_tick_ms - s_group.last_tick_ms) >
                (BEACON_EDGE_GROUP_GAP_TICKS + BEACON_EDGE_VALIDATE_POST_TICKS +
                 BEACON_EDGE_WINDOW_HALF_SIZE)))
            {
                beacon_edge_finish_group();
            }
        }
    }

    if(g_beacon_edge_state.hold_ticks > 0U)
    {
        g_beacon_edge_state.hold_ticks--;
        if(g_beacon_edge_state.hold_ticks == 0U)
        {
            g_beacon_edge_state.bump_detected = 0U;
            g_beacon_edge_state.enter_event = 0U;
            g_beacon_edge_state.exit_event = 0U;
            g_beacon_edge_state.confidence = BEACON_EDGE_CONFIDENCE_NONE;
        }
    }

    beacon_edge_dispatch_event();
}

void beacon_edge_state_update_from_project_100HZ(void)
{
    beacon_edge_state_update_100HZ(encoder_get_left_front_filtered_count(),
                                   encoder_get_right_front_filtered_count(),
                                   encoder_get_left_rear_filtered_count(),
                                   encoder_get_right_rear_filtered_count());
}

void beacon_edge_state_update_from_project_1000HZ(void)
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    accel_x_g = 0.0f;
    accel_y_g = 0.0f;
    accel_z_g = 0.0f;
    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;

    AccelCalibration_GetCorrectedSpecificForceG(&accel_x_g, &accel_y_g, &accel_z_g);
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);
    beacon_edge_state_update_1000HZ(accel_x_g, accel_y_g, accel_z_g,
                                    gyro_x_dps, gyro_y_dps, gyro_z_dps,
                                    g_euler.roll, g_euler.pitch);
}

const beacon_edge_state_t *beacon_edge_state_get(void)
{
    return &g_beacon_edge_state;
}
