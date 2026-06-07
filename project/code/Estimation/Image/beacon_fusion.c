#include "beacon_fusion.h"

#include "Common/car_filter.h"
#include "camera_crop_boundary_mapping.h"

#define BEACON_FUSION_IMAGE_CENTER_X          (94.0f)
#define BEACON_FUSION_IMAGE_CENTER_Y          (60.0f)
#define BEACON_FUSION_MIN_TARGET_AREA         (1.0f)
#define TRACK_CENTER_START_AREA_BONUS         (5.0f)
#define TRACK_MATCH_RADIUS                    (15.0f)
#define TRACK_LOST_FRAME_LIMIT                (5U)
#define TRACK_BOUNDARY_SWITCH_Y_EPS           (5.0f)
#define TRACK_CENTER_SEARCH_RADIUS            (20.0f)
#define BEACON_FUSION_CAR_LAMP_OFFSET_Y       (10.0f)
#define BEACON_FUSION_CAR_LAMP_HOLD_FRAMES    (5U)
#define BEACON_FUSION_CAR_LAMP_LPF_ALPHA      (0.2f)

typedef struct
{
    int camera_index;
    uint8 target_index;
    float image_x;
    float image_y;
    float area;
    uint8 valid;
} beacon_fusion_candidate_t;

typedef struct
{
    int camera_index;
    uint16 frame_id;
    float image_x;
    float image_y;
    float area;
    uint8 valid;
} beacon_fusion_point_t;

beacon_fusion_state_t g_beacon_fusion;

static beacon_fusion_point_t s_last_point;
static uint8 s_missing_frame_count;
static uint8 s_auto_max_count = BEACON_FUSION_CAMERA_TARGETS;
static uint16 s_frame_id;
static float s_car_lamp_ref_x;
static float s_car_lamp_ref_y;
static uint8 s_car_lamp_ref_valid;
static uint8 s_car_lamp_missing_count;
static car_filter_lpf1_t s_car_lamp_filter_x;
static car_filter_lpf1_t s_car_lamp_filter_y;

static float beacon_fusion_square(float value)
{
    return value * value;
}

static float beacon_fusion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float beacon_fusion_distance2(float lhs_x, float lhs_y, float rhs_x, float rhs_y)
{
    const float dx = lhs_x - rhs_x;
    const float dy = lhs_y - rhs_y;

    return (dx * dx) + (dy * dy);
}

static float beacon_fusion_candidate_score(const beacon_fusion_candidate_t *candidate)
{
    return candidate->area;
}

static float beacon_fusion_start_candidate_score(const beacon_fusion_candidate_t *candidate)
{
    float score;

    score = beacon_fusion_candidate_score(candidate);
    if(candidate->camera_index == (int)BEACON_FUSION_CAMERA_CENTER)
    {
        score += TRACK_CENTER_START_AREA_BONUS;
    }

    return score;
}

static uint8 beacon_fusion_target_limit(void)
{
    if(s_auto_max_count > BEACON_FUSION_CAMERA_TARGETS)
    {
        return BEACON_FUSION_CAMERA_TARGETS;
    }

    return s_auto_max_count;
}

static void beacon_fusion_clear_point(beacon_fusion_point_t *point)
{
    point->camera_index = -1;
    point->frame_id = 0U;
    point->image_x = 0.0f;
    point->image_y = 0.0f;
    point->area = 0.0f;
    point->valid = 0U;
}

static void beacon_fusion_clear_track(void)
{
    beacon_fusion_clear_point(&s_last_point);
    s_missing_frame_count = 0U;
}

static void beacon_fusion_reset_state(void)
{
    memset(&g_beacon_fusion, 0, sizeof(g_beacon_fusion));
    beacon_fusion_clear_track();
    s_frame_id = 0U;
    s_car_lamp_ref_x = BEACON_FUSION_IMAGE_CENTER_X;
    s_car_lamp_ref_y = BEACON_FUSION_IMAGE_CENTER_Y;
    s_car_lamp_ref_valid = 0U;
    s_car_lamp_missing_count = BEACON_FUSION_CAR_LAMP_HOLD_FRAMES;
    s_car_lamp_filter_x.ready = 0U;
    s_car_lamp_filter_y.ready = 0U;
}

static beacon_fusion_candidate_t beacon_fusion_candidate_from_target(const beacon_fusion_target_t *target,
                                                                     uint8 camera_index,
                                                                     uint8 target_index)
{
    beacon_fusion_candidate_t candidate;

    candidate.camera_index = (int)camera_index;
    candidate.target_index = target_index;
    candidate.image_x = target->x;
    candidate.image_y = target->y;
    candidate.area = target->area;
    candidate.valid = target->valid;

    if((candidate.valid == 0U) ||
       (beacon_fusion_candidate_score(&candidate) < BEACON_FUSION_MIN_TARGET_AREA))
    {
        candidate.valid = 0U;
    }

    return candidate;
}

static void beacon_fusion_point_from_candidate(const beacon_fusion_candidate_t *candidate,
                                               uint16 frame_id,
                                               beacon_fusion_point_t *point)
{
    if((candidate == NULL) || (point == NULL) || (candidate->valid == 0U))
    {
        return;
    }

    point->camera_index = candidate->camera_index;
    point->frame_id = frame_id;
    point->image_x = candidate->image_x;
    point->image_y = candidate->image_y;
    point->area = candidate->area;
    point->valid = 1U;
}

static beacon_fusion_point_t beacon_fusion_last_point(void)
{
    return s_last_point;
}

static void beacon_fusion_store_last_point(const beacon_fusion_point_t *point)
{
    if((point == NULL) || (point->valid == 0U))
    {
        return;
    }

    s_last_point = *point;
}

static uint8 beacon_fusion_calc_center_delta(const beacon_fusion_point_t *point,
                                             float *delta_x,
                                             float *delta_y)
{
    float center_x;
    float center_y;

    if((point == NULL) || (delta_x == NULL) || (delta_y == NULL) || (point->valid == 0U))
    {
        return 0U;
    }

    if(point->camera_index == (int)BEACON_FUSION_CAMERA_FRONT)
    {
        center_x = camera_front_boundary_center_x(point->image_x);
        center_y = camera_front_boundary_center_y(point->image_x) +
                   (point->image_y - CAMERA_CROP_BOUNDARY_BOTTOM_ROW_Y);
    }
    else if(point->camera_index == (int)BEACON_FUSION_CAMERA_CENTER)
    {
        center_x = point->image_x;
        center_y = point->image_y;
    }
    else if(point->camera_index == (int)BEACON_FUSION_CAMERA_REAR)
    {
        center_x = camera_rear_boundary_center_x(point->image_x);
        center_y = camera_rear_boundary_center_y(point->image_x) +
                   (CAMERA_CROP_BOUNDARY_BOTTOM_ROW_Y - point->image_y);
    }
    else
    {
        return 0U;
    }

    *delta_x = center_x - s_car_lamp_ref_x;
    *delta_y = center_y - s_car_lamp_ref_y;
    return 1U;
}

static void beacon_fusion_publish_point(const beacon_fusion_point_t *point)
{
    float center_delta_x;
    float center_delta_y;

    if((point == NULL) || (point->valid == 0U))
    {
        g_beacon_fusion.active = 0U;
        g_beacon_fusion.valid = 0U;
        g_beacon_fusion.camera_id = 0U;
        g_beacon_fusion.frame_id = s_frame_id;
        g_beacon_fusion.image_x = 0.0f;
        g_beacon_fusion.image_y = 0.0f;
        g_beacon_fusion.center_delta_valid = 0U;
        g_beacon_fusion.center_delta_x = 0.0f;
        g_beacon_fusion.center_delta_y = 0.0f;
        g_beacon_fusion.area = 0.0f;
        g_beacon_fusion.missing_frame_count = s_missing_frame_count;
        return;
    }

    g_beacon_fusion.active = 1U;
    g_beacon_fusion.valid = 1U;
    g_beacon_fusion.camera_id = (uint8)point->camera_index;
    g_beacon_fusion.frame_id = point->frame_id;
    g_beacon_fusion.image_x = point->image_x;
    g_beacon_fusion.image_y = point->image_y;
    center_delta_x = 0.0f;
    center_delta_y = 0.0f;
    g_beacon_fusion.center_delta_valid =
        beacon_fusion_calc_center_delta(point, &center_delta_x, &center_delta_y);
    g_beacon_fusion.center_delta_x = center_delta_x;
    g_beacon_fusion.center_delta_y = center_delta_y;
    g_beacon_fusion.area = point->area;
    g_beacon_fusion.missing_frame_count = s_missing_frame_count;
}

static uint8 beacon_fusion_find_largest_candidate(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                                  beacon_fusion_candidate_t *best_candidate)
{
    uint8 camera_index;
    uint8 target_index;
    uint8 target_limit;
    float best_score;

    best_score = 0.0f;
    target_limit = beacon_fusion_target_limit();
    best_candidate->valid = 0U;

    for(camera_index = 0U; camera_index < BEACON_FUSION_CAMERA_COUNT; camera_index++)
    {
        for(target_index = 0U; target_index < target_limit; target_index++)
        {
            const beacon_fusion_candidate_t candidate =
                beacon_fusion_candidate_from_target(&camera[camera_index].target[target_index],
                                                    camera_index,
                                                    target_index);
            const float score = beacon_fusion_start_candidate_score(&candidate);

            if(candidate.valid == 0U)
            {
                continue;
            }

            if((best_candidate->valid == 0U) || (score > best_score))
            {
                *best_candidate = candidate;
                best_score = score;
            }
        }
    }

    return best_candidate->valid;
}

static uint8 beacon_fusion_find_nearest_candidate(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                                  uint8 camera_index,
                                                  float search_x,
                                                  float search_y,
                                                  float search_radius,
                                                  beacon_fusion_candidate_t *best_candidate)
{
    uint8 target_index;
    uint8 target_limit;
    float best_distance;
    const float max_distance2 = beacon_fusion_square(search_radius);

    if((camera_index >= BEACON_FUSION_CAMERA_COUNT) || (best_candidate == NULL))
    {
        return 0U;
    }

    target_limit = beacon_fusion_target_limit();
    best_distance = 0.0f;
    best_candidate->valid = 0U;

    for(target_index = 0U; target_index < target_limit; target_index++)
    {
        const beacon_fusion_candidate_t candidate =
            beacon_fusion_candidate_from_target(&camera[camera_index].target[target_index],
                                                camera_index,
                                                target_index);
        const float distance2 = beacon_fusion_distance2(candidate.image_x,
                                                        candidate.image_y,
                                                        search_x,
                                                        search_y);

        if(candidate.valid == 0U)
        {
            continue;
        }
        if(distance2 > max_distance2)
        {
            continue;
        }
        if((best_candidate->valid == 0U) || (distance2 < best_distance))
        {
            *best_candidate = candidate;
            best_distance = distance2;
        }
    }

    return best_candidate->valid;
}

static uint8 beacon_fusion_start_from_largest(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                              uint16 frame_id,
                                              beacon_fusion_point_t *next_point)
{
    beacon_fusion_candidate_t start_candidate;

    beacon_fusion_clear_point(next_point);
    if(beacon_fusion_find_largest_candidate(camera, &start_candidate) == 0U)
    {
        return 0U;
    }

    beacon_fusion_clear_track();
    beacon_fusion_point_from_candidate(&start_candidate, frame_id, next_point);
    beacon_fusion_store_last_point(next_point);
    return 1U;
}

static uint8 beacon_fusion_update_current_camera(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                                 uint16 frame_id,
                                                 beacon_fusion_point_t *next_point)
{
    uint8 current_camera;
    beacon_fusion_candidate_t best_candidate;
    const beacon_fusion_point_t last_point = beacon_fusion_last_point();

    beacon_fusion_clear_point(next_point);
    if((last_point.valid == 0U) ||
       (last_point.camera_index < 0) ||
       (last_point.camera_index >= (int)BEACON_FUSION_CAMERA_COUNT))
    {
        return 0U;
    }

    current_camera = (uint8)last_point.camera_index;
    if(beacon_fusion_find_nearest_candidate(camera,
                                            current_camera,
                                            last_point.image_x,
                                            last_point.image_y,
                                            TRACK_MATCH_RADIUS,
                                            &best_candidate) == 0U)
    {
        return 0U;
    }

    beacon_fusion_point_from_candidate(&best_candidate, frame_id, next_point);
    return 1U;
}

static uint8 beacon_fusion_calc_boundary_center_point(const beacon_fusion_point_t *point,
                                                      float *center_x,
                                                      float *center_y)
{
    float boundary_y;

    if((point == NULL) || (center_x == NULL) || (center_y == NULL) || (point->valid == 0U))
    {
        return 0U;
    }

    if(point->camera_index == (int)BEACON_FUSION_CAMERA_FRONT)
    {
        camera_front_boundary_point_to_center(point->image_x, &boundary_y, center_x, center_y);
    }
    else if(point->camera_index == (int)BEACON_FUSION_CAMERA_REAR)
    {
        camera_rear_boundary_point_to_center(point->image_x, &boundary_y, center_x, center_y);
    }
    else
    {
        return 0U;
    }

    if(beacon_fusion_abs(point->image_y - boundary_y) >= TRACK_BOUNDARY_SWITCH_Y_EPS)
    {
        return 0U;
    }

    return 1U;
}

static uint8 beacon_fusion_try_switch_to_center(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                                const beacon_fusion_point_t *source_point,
                                                uint16 frame_id,
                                                beacon_fusion_point_t *next_point)
{
    float center_x;
    float center_y;
    beacon_fusion_candidate_t best_candidate;

    beacon_fusion_clear_point(next_point);
    if(beacon_fusion_calc_boundary_center_point(source_point, &center_x, &center_y) == 0U)
    {
        return 0U;
    }

    if(beacon_fusion_find_nearest_candidate(camera,
                                            BEACON_FUSION_CAMERA_CENTER,
                                            center_x,
                                            center_y,
                                            TRACK_CENTER_SEARCH_RADIUS,
                                            &best_candidate) == 0U)
    {
        return 0U;
    }

    beacon_fusion_point_from_candidate(&best_candidate, frame_id, next_point);
    return 1U;
}

void beacon_fusion_init(void)
{
    beacon_fusion_reset_state();
}

void beacon_fusion_set_auto_max_count(uint8 max_count)
{
    s_auto_max_count = (max_count > BEACON_FUSION_CAMERA_TARGETS) ?
                       BEACON_FUSION_CAMERA_TARGETS :
                       max_count;
}

void beacon_fusion_set_center_car_lamp(uint8 valid, float cx, float cy)
{
    if(valid != 0U)
    {
        s_car_lamp_ref_x =
            car_filter_lpf1_update(&s_car_lamp_filter_x, cx, BEACON_FUSION_CAR_LAMP_LPF_ALPHA);
        s_car_lamp_ref_y =
            car_filter_lpf1_update(&s_car_lamp_filter_y, cy, BEACON_FUSION_CAR_LAMP_LPF_ALPHA) +
            BEACON_FUSION_CAR_LAMP_OFFSET_Y;
        s_car_lamp_ref_valid = 1U;
        s_car_lamp_missing_count = 0U;
    }
    else
    {
        if(s_car_lamp_missing_count < BEACON_FUSION_CAR_LAMP_HOLD_FRAMES)
        {
            s_car_lamp_missing_count++;
        }
        else
        {
            s_car_lamp_ref_x = BEACON_FUSION_IMAGE_CENTER_X;
            s_car_lamp_ref_y = BEACON_FUSION_IMAGE_CENTER_Y;
            s_car_lamp_ref_valid = 0U;
            s_car_lamp_filter_x.ready = 0U;
            s_car_lamp_filter_y.ready = 0U;
        }
    }
}

uint8 beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT])
{
    beacon_fusion_point_t next_point;

    s_frame_id++;
    if(camera == NULL)
    {
        beacon_fusion_reset_state();
        return 0U;
    }

    if(g_beacon_fusion.active == 0U)
    {
        if(beacon_fusion_start_from_largest(camera, s_frame_id, &next_point) == 0U)
        {
            beacon_fusion_publish_point(NULL);
            return 0U;
        }

        s_missing_frame_count = 0U;
        beacon_fusion_publish_point(&next_point);
        return 1U;
    }

    if(beacon_fusion_update_current_camera(camera, s_frame_id, &next_point) != 0U)
    {
        beacon_fusion_point_t center_point;

        if(beacon_fusion_try_switch_to_center(camera, &next_point, s_frame_id, &center_point) != 0U)
        {
            next_point = center_point;
        }

        beacon_fusion_store_last_point(&next_point);
        s_missing_frame_count = 0U;
        beacon_fusion_publish_point(&next_point);
        return 1U;
    }

    if(beacon_fusion_try_switch_to_center(camera, &s_last_point, s_frame_id, &next_point) != 0U)
    {
        beacon_fusion_store_last_point(&next_point);
        s_missing_frame_count = 0U;
        beacon_fusion_publish_point(&next_point);
        return 1U;
    }

    if(s_last_point.valid == 0U)
    {
        beacon_fusion_publish_point(NULL);
        beacon_fusion_clear_track();
        return 0U;
    }

    s_missing_frame_count++;
    if(s_missing_frame_count <= TRACK_LOST_FRAME_LIMIT)
    {
        next_point = beacon_fusion_last_point();
        beacon_fusion_publish_point(&next_point);
        return 1U;
    }

    beacon_fusion_publish_point(NULL);
    beacon_fusion_clear_track();
    return 0U;
}
