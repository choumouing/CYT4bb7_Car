#include "beacon_fusion.h"

#include "camera_crop_boundary_mapping.h"

#define BEACON_FUSION_IMAGE_CENTER_X          (94.0f)
#define BEACON_FUSION_IMAGE_CENTER_Y          (60.0f)
#define BEACON_FUSION_HISTORY_FRAME_LIMIT     (5U)
#define BEACON_FUSION_MIN_TARGET_AREA         (1.0f)
#define BEACON_FUSION_BASE_MATCH_DISTANCE     (14.0f)
#define BEACON_FUSION_CROSS_CAMERA_DISTANCE   (30.0f)
#define BEACON_FUSION_MAX_MISSING_FRAMES      (3U)
#define BEACON_FUSION_FRONT_STITCH_LAST_ROW   (52.0f)
#define BEACON_FUSION_FRONT_STITCH_ROWS       (BEACON_FUSION_FRONT_STITCH_LAST_ROW + 1.0f)
#define BEACON_FUSION_CENTER_STITCH_ROWS      (120.0f)
#define BEACON_FUSION_REAR_STITCH_LAST_ROW    (41.0f)
#define BEACON_FUSION_REAR_STITCH_X0_COLUMN   (119.0f)
#define BEACON_FUSION_STITCH_CENTER_Y         (BEACON_FUSION_FRONT_STITCH_ROWS + BEACON_FUSION_IMAGE_CENTER_Y)

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

static beacon_fusion_point_t s_history[BEACON_FUSION_HISTORY_FRAME_LIMIT];
static beacon_fusion_point_t s_predicted_point;
static uint8 s_history_count;
static uint8 s_missing_frame_count;
static uint8 s_auto_max_count = BEACON_FUSION_CAMERA_TARGETS;
static uint16 s_frame_id;

static float beacon_fusion_square(float value)
{
    return value * value;
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
    uint8 i;

    for(i = 0U; i < BEACON_FUSION_HISTORY_FRAME_LIMIT; i++)
    {
        beacon_fusion_clear_point(&s_history[i]);
    }
    beacon_fusion_clear_point(&s_predicted_point);
    s_history_count = 0U;
    s_missing_frame_count = 0U;
}

static void beacon_fusion_reset_state(void)
{
    memset(&g_beacon_fusion, 0, sizeof(g_beacon_fusion));
    beacon_fusion_clear_track();
    s_frame_id = 0U;
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

static beacon_fusion_point_t beacon_fusion_history_last(void)
{
    if(s_history_count == 0U)
    {
        beacon_fusion_point_t empty;

        beacon_fusion_clear_point(&empty);
        return empty;
    }

    return s_history[s_history_count - 1U];
}

static void beacon_fusion_append_history(const beacon_fusion_point_t *point)
{
    uint8 i;

    if(point->valid == 0U)
    {
        return;
    }

    if(s_history_count < BEACON_FUSION_HISTORY_FRAME_LIMIT)
    {
        s_history[s_history_count] = *point;
        s_history_count++;
        return;
    }

    for(i = 1U; i < BEACON_FUSION_HISTORY_FRAME_LIMIT; i++)
    {
        s_history[i - 1U] = s_history[i];
    }
    s_history[BEACON_FUSION_HISTORY_FRAME_LIMIT - 1U] = *point;
}

static uint8 beacon_fusion_calc_center_delta(const beacon_fusion_point_t *point,
                                             float *delta_x,
                                             float *delta_y)
{
    float stitched_x;
    float stitched_y;

    if((point == NULL) || (delta_x == NULL) || (delta_y == NULL) || (point->valid == 0U))
    {
        return 0U;
    }

    if(point->camera_index == (int)BEACON_FUSION_CAMERA_FRONT)
    {
        if((point->image_y < 0.0f) || (point->image_y > BEACON_FUSION_FRONT_STITCH_LAST_ROW))
        {
            return 0U;
        }

        stitched_x = point->image_x;
        stitched_y = point->image_y;
    }
    else if(point->camera_index == (int)BEACON_FUSION_CAMERA_CENTER)
    {
        stitched_x = point->image_x;
        stitched_y = BEACON_FUSION_FRONT_STITCH_ROWS + point->image_y;
    }
    else if(point->camera_index == (int)BEACON_FUSION_CAMERA_REAR)
    {
        if((point->image_y < 0.0f) || (point->image_y > BEACON_FUSION_REAR_STITCH_LAST_ROW))
        {
            return 0U;
        }

        stitched_x = BEACON_FUSION_REAR_STITCH_X0_COLUMN - point->image_x;
        stitched_y = BEACON_FUSION_FRONT_STITCH_ROWS +
                     BEACON_FUSION_CENTER_STITCH_ROWS +
                     (BEACON_FUSION_REAR_STITCH_LAST_ROW - point->image_y);
    }
    else
    {
        return 0U;
    }

    *delta_x = stitched_x - BEACON_FUSION_IMAGE_CENTER_X;
    *delta_y = stitched_y - BEACON_FUSION_STITCH_CENTER_Y;
    return 1U;
}

static void beacon_fusion_publish_point(const beacon_fusion_point_t *point, uint8 predicted)
{
    float center_delta_x;
    float center_delta_y;

    if((point == NULL) || (point->valid == 0U))
    {
        g_beacon_fusion.active = 0U;
        g_beacon_fusion.valid = 0U;
        g_beacon_fusion.predicted = 0U;
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
    g_beacon_fusion.predicted = predicted;
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
            const float score = beacon_fusion_candidate_score(&candidate);

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

static void beacon_fusion_predict_xy(uint16 frame_id, float *image_x, float *image_y)
{
    uint8 i;
    uint8 first_index;
    uint8 velocity_count;
    float velocity_x;
    float velocity_y;
    beacon_fusion_point_t last_point;

    last_point = beacon_fusion_history_last();
    if(last_point.valid == 0U)
    {
        *image_x = 0.0f;
        *image_y = 0.0f;
        return;
    }

    velocity_x = 0.0f;
    velocity_y = 0.0f;
    velocity_count = 0U;
    first_index = (s_history_count > BEACON_FUSION_HISTORY_FRAME_LIMIT) ?
                  (uint8)(s_history_count - BEACON_FUSION_HISTORY_FRAME_LIMIT) :
                  1U;

    for(i = first_index; i < s_history_count; i++)
    {
        const beacon_fusion_point_t *previous = &s_history[i - 1U];
        const beacon_fusion_point_t *current = &s_history[i];
        const uint16 frame_delta = (uint16)(current->frame_id - previous->frame_id);

        if(previous->camera_index != current->camera_index)
        {
            continue;
        }
        if(frame_delta == 0U)
        {
            continue;
        }

        velocity_x += (current->image_x - previous->image_x) / (float)frame_delta;
        velocity_y += (current->image_y - previous->image_y) / (float)frame_delta;
        velocity_count++;
    }

    if(velocity_count > 0U)
    {
        velocity_x /= (float)velocity_count;
        velocity_y /= (float)velocity_count;
    }

    *image_x = last_point.image_x + (velocity_x * (float)(frame_id - last_point.frame_id));
    *image_y = last_point.image_y + (velocity_y * (float)(frame_id - last_point.frame_id));
}

static uint8 beacon_fusion_update_current_camera(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                                 uint16 frame_id,
                                                 beacon_fusion_point_t *next_point)
{
    uint8 target_index;
    uint8 target_limit;
    uint8 current_camera;
    float predict_x;
    float predict_y;
    float match_distance;
    float best_distance;
    beacon_fusion_candidate_t best_candidate;
    const beacon_fusion_point_t last_point = beacon_fusion_history_last();

    beacon_fusion_clear_point(next_point);
    if((last_point.valid == 0U) ||
       (last_point.camera_index < 0) ||
       (last_point.camera_index >= (int)BEACON_FUSION_CAMERA_COUNT))
    {
        return 0U;
    }

    current_camera = (uint8)last_point.camera_index;
    target_limit = beacon_fusion_target_limit();
    best_distance = 0.0f;
    best_candidate.valid = 0U;
    beacon_fusion_predict_xy(frame_id, &predict_x, &predict_y);
    match_distance = BEACON_FUSION_BASE_MATCH_DISTANCE;

    for(target_index = 0U; target_index < target_limit; target_index++)
    {
        const beacon_fusion_candidate_t candidate =
            beacon_fusion_candidate_from_target(&camera[current_camera].target[target_index],
                                                current_camera,
                                                target_index);
        const float distance2 = beacon_fusion_distance2(candidate.image_x,
                                                        candidate.image_y,
                                                        predict_x,
                                                        predict_y);

        if(candidate.valid == 0U)
        {
            continue;
        }
        if(distance2 > beacon_fusion_square(match_distance))
        {
            continue;
        }
        if((best_candidate.valid == 0U) || (distance2 < best_distance))
        {
            best_candidate = candidate;
            best_distance = distance2;
        }
    }

    if(best_candidate.valid == 0U)
    {
        return 0U;
    }

    next_point->camera_index = best_candidate.camera_index;
    next_point->frame_id = frame_id;
    next_point->image_x = best_candidate.image_x;
    next_point->image_y = best_candidate.image_y;
    next_point->area = best_candidate.area;
    next_point->valid = 1U;
    return 1U;
}

static uint8 beacon_fusion_map_to_center(const beacon_fusion_point_t *point,
                                         float *center_x,
                                         float *center_y)
{
    if((point == NULL) || (center_x == NULL) || (center_y == NULL))
    {
        return 0U;
    }

    if(point->camera_index == (int)BEACON_FUSION_CAMERA_FRONT)
    {
        camera_front_boundary_center_xy(point->image_x, center_x, center_y);
        return 1U;
    }

    if(point->camera_index == (int)BEACON_FUSION_CAMERA_REAR)
    {
        camera_rear_boundary_center_xy(point->image_x, center_x, center_y);
        return 1U;
    }

    return 0U;
}

static uint8 beacon_fusion_update_across_camera(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                                uint16 frame_id,
                                                beacon_fusion_point_t *next_point)
{
    uint8 target_index;
    uint8 target_limit;
    float mapped_x;
    float mapped_y;
    float best_distance;
    beacon_fusion_candidate_t best_candidate;
    const beacon_fusion_point_t last_point = beacon_fusion_history_last();

    beacon_fusion_clear_point(next_point);
    if(last_point.valid == 0U)
    {
        return 0U;
    }
    if(beacon_fusion_map_to_center(&last_point, &mapped_x, &mapped_y) == 0U)
    {
        return 0U;
    }

    target_limit = beacon_fusion_target_limit();
    best_distance = 0.0f;
    best_candidate.valid = 0U;

    for(target_index = 0U; target_index < target_limit; target_index++)
    {
        const beacon_fusion_candidate_t candidate =
            beacon_fusion_candidate_from_target(&camera[BEACON_FUSION_CAMERA_CENTER].target[target_index],
                                                BEACON_FUSION_CAMERA_CENTER,
                                                target_index);
        const float distance2 = beacon_fusion_distance2(candidate.image_x,
                                                        candidate.image_y,
                                                        mapped_x,
                                                        mapped_y);

        if(candidate.valid == 0U)
        {
            continue;
        }
        if(distance2 > beacon_fusion_square(BEACON_FUSION_CROSS_CAMERA_DISTANCE))
        {
            continue;
        }
        if((best_candidate.valid == 0U) || (distance2 < best_distance))
        {
            best_candidate = candidate;
            best_distance = distance2;
        }
    }

    if(best_candidate.valid == 0U)
    {
        return 0U;
    }

    next_point->camera_index = best_candidate.camera_index;
    next_point->frame_id = frame_id;
    next_point->image_x = best_candidate.image_x;
    next_point->image_y = best_candidate.image_y;
    next_point->area = best_candidate.area;
    next_point->valid = 1U;
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

uint8 beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT])
{
    beacon_fusion_candidate_t start_candidate;
    beacon_fusion_point_t next_point;

    s_frame_id++;
    if(camera == NULL)
    {
        beacon_fusion_reset_state();
        return 0U;
    }

    if(g_beacon_fusion.active == 0U)
    {
        if(beacon_fusion_find_largest_candidate(camera, &start_candidate) == 0U)
        {
            beacon_fusion_publish_point(NULL, 0U);
            return 0U;
        }

        beacon_fusion_clear_track();
        next_point.camera_index = start_candidate.camera_index;
        next_point.frame_id = s_frame_id;
        next_point.image_x = start_candidate.image_x;
        next_point.image_y = start_candidate.image_y;
        next_point.area = start_candidate.area;
        next_point.valid = 1U;
        beacon_fusion_append_history(&next_point);
        beacon_fusion_clear_point(&s_predicted_point);
        s_missing_frame_count = 0U;
        beacon_fusion_publish_point(&next_point, 0U);
        return 1U;
    }

    if(beacon_fusion_update_current_camera(camera, s_frame_id, &next_point) != 0U)
    {
        beacon_fusion_append_history(&next_point);
        beacon_fusion_clear_point(&s_predicted_point);
        s_missing_frame_count = 0U;
        beacon_fusion_publish_point(&next_point, 0U);
        return 1U;
    }

    if(beacon_fusion_update_across_camera(camera, s_frame_id, &next_point) != 0U)
    {
        beacon_fusion_append_history(&next_point);
        beacon_fusion_clear_point(&s_predicted_point);
        s_missing_frame_count = 0U;
        beacon_fusion_publish_point(&next_point, 0U);
        return 1U;
    }

    if(s_missing_frame_count < BEACON_FUSION_MAX_MISSING_FRAMES)
    {
        s_predicted_point = beacon_fusion_history_last();
        s_predicted_point.frame_id = s_frame_id;
        beacon_fusion_predict_xy(s_frame_id,
                                 &s_predicted_point.image_x,
                                 &s_predicted_point.image_y);
        s_predicted_point.valid = 1U;
        s_missing_frame_count++;
        beacon_fusion_publish_point(&s_predicted_point, 1U);
        return 1U;
    }

    beacon_fusion_clear_point(&s_predicted_point);
    g_beacon_fusion.active = 0U;
    g_beacon_fusion.valid = 0U;
    g_beacon_fusion.predicted = 0U;
    g_beacon_fusion.missing_frame_count = s_missing_frame_count;
    return 0U;
}
