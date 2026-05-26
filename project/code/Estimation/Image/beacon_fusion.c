#include "beacon_fusion.h"

#include <math.h>
#include <string.h>

#define BEACON_FUSION_PI                         (3.1415926f)
#define BEACON_FUSION_HALF_DIAGONAL_PX           (111.5168f)
#define BEACON_FUSION_MAX_IMAGE_ANGLE_RAD        (1.48353f)
#define BEACON_FUSION_IMAGE_WIDTH_PX             (188.0f)
#define BEACON_FUSION_IMAGE_HEIGHT_PX            (120.0f)
#define BEACON_FUSION_IMAGE_CENTER_X_PX          (94.0f)
#define BEACON_FUSION_IMAGE_CENTER_Y_PX          (60.0f)
#define BEACON_FUSION_ORIGIN_PROJ_X_PX           (94.0f)
#define BEACON_FUSION_ORIGIN_PROJ_Y_PX           (100.0f)
#define BEACON_FUSION_MIN_RADIUS_PX              (1.0f)
#define BEACON_FUSION_SECONDARY_MIN_RADIUS_PX    (1.35f)
#define BEACON_FUSION_MIN_GROUND_RAY_Z           (0.02f)
#define BEACON_FUSION_FALLBACK_RANGE_MM          (900.0f)
#define BEACON_FUSION_ASSOC_DISTANCE_MM          (1050.0f)
#define BEACON_FUSION_ASSOC_ANGLE_DEG            (45.0f)
#define BEACON_FUSION_ASSOC_SCORE_MAX            (1700.0f)
#define BEACON_FUSION_SINGLE_RECOVER_ANGLE_DEG   (25.0f)
#define BEACON_FUSION_SINGLE_RECOVER_RAW_MIN     (4U)
#define BEACON_FUSION_SINGLE_RECOVER_GROUP_MIN   (3U)
#define BEACON_FUSION_MERGE_ANGLE_WEIGHT_MM      (25.0f)
#define BEACON_FUSION_NO_OBSERVATION_CLEAR_TICKS (30U)
#define BEACON_FUSION_FILTER_ALPHA               (0.35f)
#define BEACON_FUSION_ASSIGN_UNUSED              (0xFFU)

typedef struct
{
    float x;
    float y;
    float z;
} beacon_fusion_vec3_t;

typedef struct
{
    beacon_fusion_vec3_t forward;
    beacon_fusion_vec3_t right;
    beacon_fusion_vec3_t down;
} beacon_fusion_camera_model_t;

typedef struct
{
    uint8 valid;
    uint8 camera_id;
    uint8 target_id;
    uint8 source_camera_mask;
    float image_x;
    float image_y;
    float radius;
    float bearing_deg;
    float range_proxy;
    float x_body;
    float y_body;
    float weight;
} beacon_fusion_observation_t;

typedef struct
{
    uint8 valid;
    uint8 source_camera_mask;
    uint8 observation_count;
    float sum_x;
    float sum_y;
    float sum_image_x;
    float sum_image_y;
    float sum_sin;
    float sum_cos;
    float sum_weight;
    float radius_max;
} beacon_fusion_group_t;

typedef struct
{
    uint8 observation_index[BEACON_FUSION_CAMERA_TARGETS];
    uint8 count;
    float radius_sum;
} beacon_fusion_camera_bucket_t;

typedef struct
{
    uint8 assignment[BEACON_FUSION_CAMERA_TARGETS];
    uint8 used_group[BEACON_FUSION_MAX_BEACONS];
    uint8 match_count;
    float score_sum;
} beacon_fusion_assignment_t;

beacon_fusion_result_t g_beacon_fusion_result;

static uint8 s_fused_count_state;
static uint8 s_no_observation_ticks;

static const beacon_fusion_camera_model_t s_camera_model[BEACON_FUSION_CAMERA_COUNT] =
{
    {
        { 0.6229665f, -0.3596699f, -0.6946584f },
        { -0.5000000f, -0.8660254f, 0.0f },
        { -0.6015918f, 0.3473292f, -0.7193398f }
    },
    {
        { 0.0f, 0.7193398f, -0.6946584f },
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, -0.6946584f, -0.7193398f }
    },
    {
        { -0.6229665f, -0.3596699f, -0.6946584f },
        { -0.5000000f, 0.8660254f, 0.0f },
        { 0.6015918f, 0.3473292f, -0.7193398f }
    }
};

static float beacon_fusion_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float beacon_fusion_clampf(float value, float min_value, float max_value)
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

static float beacon_fusion_wrap_deg(float angle_deg)
{
    while(angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float beacon_fusion_angle_diff_abs_deg(float a_deg, float b_deg)
{
    return beacon_fusion_absf(beacon_fusion_wrap_deg(a_deg - b_deg));
}

static float beacon_fusion_deg_to_rad(float angle_deg)
{
    return angle_deg * (BEACON_FUSION_PI / 180.0f);
}

static float beacon_fusion_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / BEACON_FUSION_PI);
}

static uint8 beacon_fusion_float_valid(float value)
{
    return (value == value) &&
           (value < 1000000.0f) &&
           (value > -1000000.0f);
}

static float beacon_fusion_vec3_len(const beacon_fusion_vec3_t *vec)
{
    return sqrtf((vec->x * vec->x) + (vec->y * vec->y) + (vec->z * vec->z));
}

static beacon_fusion_vec3_t beacon_fusion_vec3_scale(const beacon_fusion_vec3_t *vec, float scale)
{
    beacon_fusion_vec3_t out;

    out.x = vec->x * scale;
    out.y = vec->y * scale;
    out.z = vec->z * scale;
    return out;
}

static beacon_fusion_vec3_t beacon_fusion_vec3_add(const beacon_fusion_vec3_t *a,
                                                   const beacon_fusion_vec3_t *b)
{
    beacon_fusion_vec3_t out;

    out.x = a->x + b->x;
    out.y = a->y + b->y;
    out.z = a->z + b->z;
    return out;
}

static beacon_fusion_vec3_t beacon_fusion_vec3_normalize(const beacon_fusion_vec3_t *vec)
{
    beacon_fusion_vec3_t out = *vec;
    float len = beacon_fusion_vec3_len(vec);

    if(len > 0.000001f)
    {
        out.x /= len;
        out.y /= len;
        out.z /= len;
    }
    return out;
}

static beacon_fusion_vec3_t beacon_fusion_image_to_ray(uint8 camera_id, float target_x, float target_y)
{
    const beacon_fusion_camera_model_t *camera = &s_camera_model[camera_id];
    beacon_fusion_vec3_t ray;
    float image_x = target_x - BEACON_FUSION_IMAGE_CENTER_X_PX;
    float image_y = target_y - BEACON_FUSION_IMAGE_CENTER_Y_PX;
    float radius_px = sqrtf((image_x * image_x) + (image_y * image_y));

    ray = camera->forward;
    if(radius_px > 0.001f)
    {
        float theta = (radius_px / BEACON_FUSION_HALF_DIAGONAL_PX) *
                      BEACON_FUSION_MAX_IMAGE_ANGLE_RAD;
        float tx = image_x / radius_px;
        float ty = image_y / radius_px;
        beacon_fusion_vec3_t right_part = beacon_fusion_vec3_scale(&camera->right, tx);
        beacon_fusion_vec3_t down_part = beacon_fusion_vec3_scale(&camera->down, ty);
        beacon_fusion_vec3_t tangent = beacon_fusion_vec3_add(&right_part, &down_part);
        beacon_fusion_vec3_t forward_part;
        beacon_fusion_vec3_t tangent_part;

        theta = beacon_fusion_clampf(theta, 0.0f, BEACON_FUSION_MAX_IMAGE_ANGLE_RAD);
        tangent = beacon_fusion_vec3_normalize(&tangent);
        forward_part = beacon_fusion_vec3_scale(&camera->forward, cosf(theta));
        tangent_part = beacon_fusion_vec3_scale(&tangent, sinf(theta));
        ray = beacon_fusion_vec3_add(&forward_part, &tangent_part);
    }

    return beacon_fusion_vec3_normalize(&ray);
}

static uint8 beacon_fusion_image_to_origin_delta(uint8 camera_id,
                                                 float target_x,
                                                 float target_y,
                                                 float *x_body,
                                                 float *y_body)
{
    beacon_fusion_vec3_t target_ray = beacon_fusion_image_to_ray(camera_id, target_x, target_y);
    beacon_fusion_vec3_t origin_ray = beacon_fusion_image_to_ray(camera_id,
                                                                 BEACON_FUSION_ORIGIN_PROJ_X_PX,
                                                                 BEACON_FUSION_ORIGIN_PROJ_Y_PX);
    float target_scale;
    float origin_scale;

    if((target_ray.z > -BEACON_FUSION_MIN_GROUND_RAY_Z) ||
       (origin_ray.z > -BEACON_FUSION_MIN_GROUND_RAY_Z))
    {
        return 0U;
    }

    target_scale = BEACON_FUSION_FALLBACK_RANGE_MM / -target_ray.z;
    origin_scale = BEACON_FUSION_FALLBACK_RANGE_MM / -origin_ray.z;
    *x_body = (target_ray.x * target_scale) - (origin_ray.x * origin_scale);
    *y_body = (target_ray.y * target_scale) - (origin_ray.y * origin_scale);
    return 1U;
}

static float beacon_fusion_target_weight(float radius, uint8 target_id)
{
    float weight = beacon_fusion_clampf(radius / 4.0f, 0.25f, 2.0f);

    if(target_id > 0U)
    {
        weight *= 0.8f;
    }
    return weight;
}

static uint8 beacon_fusion_target_valid(const beacon_fusion_camera_target_t *target, uint8 target_id)
{
    if(target->valid == 0U)
    {
        return 0U;
    }
    if(target->radius < BEACON_FUSION_MIN_RADIUS_PX)
    {
        return 0U;
    }
    if((beacon_fusion_float_valid(target->x) == 0U) ||
       (beacon_fusion_float_valid(target->y) == 0U) ||
       (beacon_fusion_float_valid(target->radius) == 0U))
    {
        return 0U;
    }
    if((beacon_fusion_absf(target->x) < 0.001f) &&
       (beacon_fusion_absf(target->y) < 0.001f) &&
       (beacon_fusion_absf(target->radius) < 0.001f))
    {
        return 0U;
    }
    if((target->x < 0.0f) || (target->x >= BEACON_FUSION_IMAGE_WIDTH_PX) ||
       (target->y < 0.0f) || (target->y >= BEACON_FUSION_IMAGE_HEIGHT_PX))
    {
        return 0U;
    }
    if((target_id > 0U) && (target->radius < BEACON_FUSION_SECONDARY_MIN_RADIUS_PX))
    {
        return 0U;
    }
    return 1U;
}

static void beacon_fusion_observation_from_target(
    uint8 camera_id,
    uint8 target_id,
    const beacon_fusion_camera_target_t *target,
    beacon_fusion_observation_t *observation)
{
    beacon_fusion_vec3_t ray = beacon_fusion_image_to_ray(camera_id, target->x, target->y);
    float bearing_rad;
    float delta_x;
    float delta_y;

    memset(observation, 0, sizeof(*observation));
    observation->valid = 1U;
    observation->camera_id = camera_id;
    observation->target_id = target_id;
    observation->source_camera_mask = (uint8)(1U << camera_id);
    observation->image_x = target->x;
    observation->image_y = target->y;
    observation->radius = target->radius;

    if(beacon_fusion_image_to_origin_delta(camera_id, target->x, target->y, &delta_x, &delta_y) != 0U)
    {
        observation->x_body = delta_x;
        observation->y_body = delta_y;
        observation->range_proxy = sqrtf((delta_x * delta_x) + (delta_y * delta_y));
        if(observation->range_proxy > 0.001f)
        {
            observation->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(delta_x,
                                                                                               delta_y)));
        }
    }
    else
    {
        observation->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(ray.x, ray.y)));
        bearing_rad = beacon_fusion_deg_to_rad(observation->bearing_deg);
        observation->range_proxy = BEACON_FUSION_FALLBACK_RANGE_MM;
        observation->x_body = sinf(bearing_rad) * BEACON_FUSION_FALLBACK_RANGE_MM;
        observation->y_body = cosf(bearing_rad) * BEACON_FUSION_FALLBACK_RANGE_MM;
    }
    observation->weight = beacon_fusion_target_weight(target->radius, target_id);
}

static uint8 beacon_fusion_collect_observations(
    const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
    beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS])
{
    uint8 camera_id;
    uint8 count = 0U;

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        uint8 target_id;

        for(target_id = 0U; target_id < BEACON_FUSION_CAMERA_TARGETS; target_id++)
        {
            const beacon_fusion_camera_target_t *target = &camera[camera_id].target[target_id];

            if(beacon_fusion_target_valid(target, target_id) == 0U)
            {
                continue;
            }
            if(count >= BEACON_FUSION_MAX_OBSERVATIONS)
            {
                break;
            }

            beacon_fusion_observation_from_target(camera_id,
                                                  target_id,
                                                  target,
                                                  &observation[count]);
            count++;
        }
    }

    return count;
}

static uint8 beacon_fusion_update_count_state(uint8 detected_count)
{
    if(detected_count == 0U)
    {
        if(s_no_observation_ticks < 255U)
        {
            s_no_observation_ticks++;
        }
        if(s_no_observation_ticks >= BEACON_FUSION_NO_OBSERVATION_CLEAR_TICKS)
        {
            s_fused_count_state = 0U;
        }
        return s_fused_count_state;
    }

    s_no_observation_ticks = 0U;
    if(detected_count > BEACON_FUSION_MAX_BEACONS)
    {
        detected_count = BEACON_FUSION_MAX_BEACONS;
    }

    s_fused_count_state = detected_count;
    return s_fused_count_state;
}

static void beacon_fusion_group_reset(beacon_fusion_group_t *group)
{
    memset(group, 0, sizeof(*group));
}

static float beacon_fusion_group_x(const beacon_fusion_group_t *group)
{
    return group->sum_x / group->sum_weight;
}

static float beacon_fusion_group_y(const beacon_fusion_group_t *group)
{
    return group->sum_y / group->sum_weight;
}

static float beacon_fusion_group_image_x(const beacon_fusion_group_t *group)
{
    return group->sum_image_x / group->sum_weight;
}

static float beacon_fusion_group_image_y(const beacon_fusion_group_t *group)
{
    return group->sum_image_y / group->sum_weight;
}

static float beacon_fusion_group_bearing_deg(const beacon_fusion_group_t *group)
{
    return beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(group->sum_sin,
                                                                  group->sum_cos)));
}

static void beacon_fusion_group_add_values(beacon_fusion_group_t *group,
                                           float x_body,
                                           float y_body,
                                           float image_x,
                                           float image_y,
                                           float bearing_deg,
                                           float weight,
                                           float radius,
                                           uint8 source_camera_mask,
                                           uint8 observation_count)
{
    float bearing_rad = beacon_fusion_deg_to_rad(bearing_deg);

    group->valid = 1U;
    group->source_camera_mask |= source_camera_mask;
    group->observation_count += observation_count;
    group->sum_x += x_body * weight;
    group->sum_y += y_body * weight;
    group->sum_image_x += image_x * weight;
    group->sum_image_y += image_y * weight;
    group->sum_sin += sinf(bearing_rad) * weight;
    group->sum_cos += cosf(bearing_rad) * weight;
    group->sum_weight += weight;
    if(radius > group->radius_max)
    {
        group->radius_max = radius;
    }
}

static void beacon_fusion_group_add_observation(beacon_fusion_group_t *group,
                                                const beacon_fusion_observation_t *observation)
{
    beacon_fusion_group_add_values(group,
                                   observation->x_body,
                                   observation->y_body,
                                   observation->image_x,
                                   observation->image_y,
                                   observation->bearing_deg,
                                   observation->weight,
                                   observation->radius,
                                   observation->source_camera_mask,
                                   1U);
}

static float beacon_fusion_group_observation_score(const beacon_fusion_group_t *group,
                                                   const beacon_fusion_observation_t *observation)
{
    float dx = observation->x_body - beacon_fusion_group_x(group);
    float dy = observation->y_body - beacon_fusion_group_y(group);
    float distance = sqrtf((dx * dx) + (dy * dy));
    float angle = beacon_fusion_angle_diff_abs_deg(observation->bearing_deg,
                                                   beacon_fusion_group_bearing_deg(group));

    return distance + (angle * BEACON_FUSION_MERGE_ANGLE_WEIGHT_MM);
}

static uint8 beacon_fusion_observation_passes_gate(const beacon_fusion_group_t *group,
                                                   const beacon_fusion_observation_t *observation,
                                                   float *score_out)
{
    if((group->source_camera_mask & observation->source_camera_mask) != 0U)
    {
        return 0U;
    }

    if(group->valid != 0U)
    {
        float score = beacon_fusion_group_observation_score(group, observation);
        float dx = observation->x_body - beacon_fusion_group_x(group);
        float dy = observation->y_body - beacon_fusion_group_y(group);
        float distance = sqrtf((dx * dx) + (dy * dy));
        float angle = beacon_fusion_angle_diff_abs_deg(observation->bearing_deg,
                                                       beacon_fusion_group_bearing_deg(group));

        if((distance <= BEACON_FUSION_ASSOC_DISTANCE_MM) &&
           (angle <= BEACON_FUSION_ASSOC_ANGLE_DEG) &&
           (score <= BEACON_FUSION_ASSOC_SCORE_MAX))
        {
            if(score_out != 0)
            {
                *score_out = score;
            }
            return 1U;
        }
    }

    return 0U;
}

static void beacon_fusion_build_camera_buckets(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 observation_count,
    beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT])
{
    uint8 i;

    memset(bucket, 0, sizeof(beacon_fusion_camera_bucket_t) * BEACON_FUSION_CAMERA_COUNT);
    for(i = 0U; i < observation_count; i++)
    {
        uint8 camera_id;

        if(observation[i].valid == 0U)
        {
            continue;
        }

        camera_id = observation[i].camera_id;
        if((camera_id >= BEACON_FUSION_CAMERA_COUNT) ||
           (bucket[camera_id].count >= BEACON_FUSION_CAMERA_TARGETS))
        {
            continue;
        }

        bucket[camera_id].observation_index[bucket[camera_id].count] = i;
        bucket[camera_id].count++;
        bucket[camera_id].radius_sum += observation[i].radius;
    }
}

static uint8 beacon_fusion_choose_anchor_camera(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT])
{
    uint8 camera_id;
    uint8 best = 0U;

    for(camera_id = 1U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        if((bucket[camera_id].count > bucket[best].count) ||
           ((bucket[camera_id].count == bucket[best].count) &&
            (bucket[camera_id].radius_sum > bucket[best].radius_sum)))
        {
            best = camera_id;
        }
    }

    return best;
}

static uint8 beacon_fusion_assignment_better(const beacon_fusion_assignment_t *candidate,
                                             const beacon_fusion_assignment_t *best)
{
    if(candidate->match_count > best->match_count)
    {
        return 1U;
    }
    if((candidate->match_count == best->match_count) &&
       (candidate->score_sum < best->score_sum))
    {
        return 1U;
    }
    return 0U;
}

static void beacon_fusion_assign_camera_recursive(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    const beacon_fusion_camera_bucket_t *bucket,
    uint8 target_pos,
    const beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS],
    uint8 group_count,
    beacon_fusion_assignment_t *current,
    beacon_fusion_assignment_t *best)
{
    uint8 group_id;

    if(target_pos >= bucket->count)
    {
        if(beacon_fusion_assignment_better(current, best) != 0U)
        {
            *best = *current;
        }
        return;
    }

    current->assignment[target_pos] = BEACON_FUSION_ASSIGN_UNUSED;
    beacon_fusion_assign_camera_recursive(observation,
                                          bucket,
                                          (uint8)(target_pos + 1U),
                                          group,
                                          group_count,
                                          current,
                                          best);

    for(group_id = 0U; group_id < group_count; group_id++)
    {
        float score;
        uint8 observation_index;

        if(current->used_group[group_id] != 0U)
        {
            continue;
        }

        observation_index = bucket->observation_index[target_pos];
        if(beacon_fusion_observation_passes_gate(&group[group_id],
                                                 &observation[observation_index],
                                                 &score) == 0U)
        {
            continue;
        }

        current->assignment[target_pos] = group_id;
        current->used_group[group_id] = 1U;
        current->match_count++;
        current->score_sum += score;

        beacon_fusion_assign_camera_recursive(observation,
                                              bucket,
                                              (uint8)(target_pos + 1U),
                                              group,
                                              group_count,
                                              current,
                                              best);

        current->score_sum -= score;
        current->match_count--;
        current->used_group[group_id] = 0U;
        current->assignment[target_pos] = BEACON_FUSION_ASSIGN_UNUSED;
    }
}

static void beacon_fusion_assign_camera_observations(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    const beacon_fusion_camera_bucket_t *bucket,
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS],
    uint8 group_count,
    uint8 unmatched[BEACON_FUSION_CAMERA_TARGETS])
{
    beacon_fusion_assignment_t current;
    beacon_fusion_assignment_t best;
    uint8 i;

    memset(&current, 0, sizeof(current));
    memset(&best, 0, sizeof(best));
    best.score_sum = 100000000.0f;
    for(i = 0U; i < BEACON_FUSION_CAMERA_TARGETS; i++)
    {
        current.assignment[i] = BEACON_FUSION_ASSIGN_UNUSED;
        best.assignment[i] = BEACON_FUSION_ASSIGN_UNUSED;
        unmatched[i] = BEACON_FUSION_ASSIGN_UNUSED;
    }

    beacon_fusion_assign_camera_recursive(observation,
                                          bucket,
                                          0U,
                                          group,
                                          group_count,
                                          &current,
                                          &best);

    for(i = 0U; i < bucket->count; i++)
    {
        uint8 observation_index = bucket->observation_index[i];

        if(best.assignment[i] < BEACON_FUSION_MAX_BEACONS)
        {
            beacon_fusion_group_add_observation(&group[best.assignment[i]],
                                                &observation[observation_index]);
        }
        else
        {
            unmatched[i] = observation_index;
        }
    }
}

static uint8 beacon_fusion_observations_match(
    const beacon_fusion_observation_t *a,
    const beacon_fusion_observation_t *b)
{
    beacon_fusion_group_t temp_group;
    float score;

    if(a->camera_id == b->camera_id)
    {
        return 0U;
    }

    beacon_fusion_group_reset(&temp_group);
    beacon_fusion_group_add_observation(&temp_group, a);
    return beacon_fusion_observation_passes_gate(&temp_group, b, &score);
}

static void beacon_fusion_add_unmatched_pairs(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    const uint8 unmatched[BEACON_FUSION_CAMERA_COUNT][BEACON_FUSION_CAMERA_TARGETS],
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT],
    uint8 anchor_camera,
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS],
    uint8 *group_count)
{
    uint8 camera_a;

    for(camera_a = 0U; camera_a < BEACON_FUSION_CAMERA_COUNT; camera_a++)
    {
        uint8 pos_a;

        if(camera_a == anchor_camera)
        {
            continue;
        }

        for(pos_a = 0U; pos_a < bucket[camera_a].count; pos_a++)
        {
            uint8 obs_a = unmatched[camera_a][pos_a];
            uint8 camera_b;

            if(obs_a >= BEACON_FUSION_MAX_OBSERVATIONS)
            {
                continue;
            }
            if(*group_count >= BEACON_FUSION_MAX_BEACONS)
            {
                return;
            }

            for(camera_b = (uint8)(camera_a + 1U); camera_b < BEACON_FUSION_CAMERA_COUNT; camera_b++)
            {
                uint8 pos_b;

                if(camera_b == anchor_camera)
                {
                    continue;
                }

                for(pos_b = 0U; pos_b < bucket[camera_b].count; pos_b++)
                {
                    uint8 obs_b = unmatched[camera_b][pos_b];

                    if(obs_b >= BEACON_FUSION_MAX_OBSERVATIONS)
                    {
                        continue;
                    }
                    if(beacon_fusion_observations_match(&observation[obs_a],
                                                        &observation[obs_b]) == 0U)
                    {
                        continue;
                    }

                    beacon_fusion_group_add_observation(&group[*group_count],
                                                        &observation[obs_a]);
                    beacon_fusion_group_add_observation(&group[*group_count],
                                                        &observation[obs_b]);
                    (*group_count)++;
                    return;
                }
            }
        }
    }
}

static uint8 beacon_fusion_observation_separate_from_groups(
    const beacon_fusion_observation_t *observation,
    const beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS],
    uint8 group_count)
{
    uint8 i;

    for(i = 0U; i < group_count; i++)
    {
        if(beacon_fusion_angle_diff_abs_deg(observation->bearing_deg,
                                            beacon_fusion_group_bearing_deg(&group[i])) <
           BEACON_FUSION_SINGLE_RECOVER_ANGLE_DEG)
        {
            return 0U;
        }
    }

    return 1U;
}

static void beacon_fusion_add_unmatched_single(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    const uint8 unmatched[BEACON_FUSION_CAMERA_COUNT][BEACON_FUSION_CAMERA_TARGETS],
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT],
    uint8 anchor_camera,
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS],
    uint8 *group_count)
{
    uint8 camera_id;
    uint8 best_observation = BEACON_FUSION_MAX_OBSERVATIONS;
    float best_radius = -1.0f;

    if(*group_count >= BEACON_FUSION_SINGLE_RECOVER_GROUP_MIN)
    {
        return;
    }

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        uint8 pos;

        if(camera_id == anchor_camera)
        {
            continue;
        }

        for(pos = 0U; pos < bucket[camera_id].count; pos++)
        {
            uint8 observation_index = unmatched[camera_id][pos];

            if(observation_index >= BEACON_FUSION_MAX_OBSERVATIONS)
            {
                continue;
            }
            if(beacon_fusion_observation_separate_from_groups(&observation[observation_index],
                                                              group,
                                                              *group_count) == 0U)
            {
                continue;
            }
            if(observation[observation_index].radius > best_radius)
            {
                best_radius = observation[observation_index].radius;
                best_observation = observation_index;
            }
        }
    }

    if(best_observation < BEACON_FUSION_MAX_OBSERVATIONS)
    {
        beacon_fusion_group_add_observation(&group[*group_count],
                                            &observation[best_observation]);
        (*group_count)++;
    }
}

static uint8 beacon_fusion_build_groups(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 observation_count,
    uint8 group_limit,
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS])
{
    beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT];
    uint8 unmatched[BEACON_FUSION_CAMERA_COUNT][BEACON_FUSION_CAMERA_TARGETS];
    uint8 anchor_camera;
    uint8 camera_id;
    uint8 i;
    uint8 raw_count = 0U;
    uint8 group_count = 0U;

    for(i = 0U; i < BEACON_FUSION_MAX_BEACONS; i++)
    {
        beacon_fusion_group_reset(&group[i]);
    }
    memset(unmatched, BEACON_FUSION_ASSIGN_UNUSED, sizeof(unmatched));

    beacon_fusion_build_camera_buckets(observation, observation_count, bucket);
    anchor_camera = beacon_fusion_choose_anchor_camera(bucket);
    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        raw_count += bucket[camera_id].count;
    }

    for(i = 0U; (i < bucket[anchor_camera].count) && (group_count < group_limit); i++)
    {
        uint8 observation_index = bucket[anchor_camera].observation_index[i];

        beacon_fusion_group_add_observation(&group[group_count],
                                            &observation[observation_index]);
        group_count++;
    }

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        if(camera_id == anchor_camera)
        {
            continue;
        }

        beacon_fusion_assign_camera_observations(observation,
                                                 &bucket[camera_id],
                                                 group,
                                                 group_count,
                                                 unmatched[camera_id]);
    }

    if(group_count == 0U)
    {
        for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
        {
            for(i = 0U; (i < bucket[camera_id].count) && (group_count < group_limit); i++)
            {
                uint8 observation_index = bucket[camera_id].observation_index[i];

                beacon_fusion_group_add_observation(&group[group_count],
                                                    &observation[observation_index]);
                group_count++;
            }
        }
    }
    else if(group_count < group_limit)
    {
        beacon_fusion_add_unmatched_pairs(observation,
                                          unmatched,
                                          bucket,
                                          anchor_camera,
                                          group,
                                          &group_count);
        if((group_count < BEACON_FUSION_SINGLE_RECOVER_GROUP_MIN) &&
           (raw_count >= BEACON_FUSION_SINGLE_RECOVER_RAW_MIN))
        {
            beacon_fusion_add_unmatched_single(observation,
                                               unmatched,
                                               bucket,
                                               anchor_camera,
                                               group,
                                               &group_count);
        }
    }

    return group_count;
}

static void beacon_fusion_group_to_beacon(const beacon_fusion_group_t *group,
                                          beacon_fusion_beacon_t *beacon)
{
    memset(beacon, 0, sizeof(*beacon));
    if((group->valid == 0U) || (group->sum_weight <= 0.0f))
    {
        return;
    }

    beacon->valid = 1U;
    beacon->source_camera_mask = group->source_camera_mask;
    beacon->observation_count = group->observation_count;
    beacon->x_body = beacon_fusion_group_x(group);
    beacon->y_body = beacon_fusion_group_y(group);
    beacon->control_x = BEACON_FUSION_ORIGIN_PROJ_X_PX - beacon_fusion_group_image_x(group);
    beacon->control_y = beacon_fusion_group_image_y(group) - BEACON_FUSION_ORIGIN_PROJ_Y_PX;
    beacon->range_proxy = sqrtf((beacon->x_body * beacon->x_body) + (beacon->y_body * beacon->y_body));
    beacon->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(beacon->x_body,
                                                                                  beacon->y_body)));

    beacon->confidence = 0.25f +
                         (group->radius_max * 0.08f) +
                         ((float)group->observation_count * 0.08f);
    if((group->source_camera_mask == 0x03U) ||
       (group->source_camera_mask == 0x05U) ||
       (group->source_camera_mask == 0x06U) ||
       (group->source_camera_mask == 0x07U))
    {
        beacon->confidence += 0.1f;
    }
    beacon->confidence = beacon_fusion_clampf(beacon->confidence, 0.0f, 1.0f);
}

static uint8 beacon_fusion_find_previous_match(const beacon_fusion_beacon_t *beacon,
                                               const beacon_fusion_result_t *previous,
                                               uint8 used[BEACON_FUSION_MAX_BEACONS])
{
    uint8 i;
    uint8 best = BEACON_FUSION_MAX_BEACONS;
    float best_score = 100000000.0f;

    for(i = 0U; i < previous->beacon_count; i++)
    {
        float dx;
        float dy;
        float distance;
        float angle;
        float score;

        if((used[i] != 0U) || (previous->beacon[i].valid == 0U))
        {
            continue;
        }

        dx = beacon->x_body - previous->beacon[i].x_body;
        dy = beacon->y_body - previous->beacon[i].y_body;
        distance = sqrtf((dx * dx) + (dy * dy));
        angle = beacon_fusion_angle_diff_abs_deg(beacon->bearing_deg,
                                                 previous->beacon[i].bearing_deg);
        score = distance + (angle * BEACON_FUSION_MERGE_ANGLE_WEIGHT_MM);
        if(score < best_score)
        {
            best_score = score;
            best = i;
        }
    }

    return best;
}

static void beacon_fusion_apply_output_filter(beacon_fusion_result_t *current,
                                              const beacon_fusion_result_t *previous)
{
    uint8 previous_used[BEACON_FUSION_MAX_BEACONS] = {0U};
    beacon_fusion_beacon_t filtered[BEACON_FUSION_MAX_BEACONS];
    uint8 i;

    memset(filtered, 0, sizeof(filtered));
    for(i = 0U; i < current->beacon_count; i++)
    {
        uint8 match;

        if(current->beacon[i].valid == 0U)
        {
            continue;
        }

        filtered[i] = current->beacon[i];
        match = beacon_fusion_find_previous_match(&current->beacon[i], previous, previous_used);
        if(match < BEACON_FUSION_MAX_BEACONS)
        {
            float dx;
            float dy;

            previous_used[match] = 1U;
            filtered[i].x_body =
                (previous->beacon[match].x_body * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                (current->beacon[i].x_body * BEACON_FUSION_FILTER_ALPHA);
            filtered[i].y_body =
                (previous->beacon[match].y_body * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                (current->beacon[i].y_body * BEACON_FUSION_FILTER_ALPHA);
            filtered[i].control_x =
                (previous->beacon[match].control_x * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                (current->beacon[i].control_x * BEACON_FUSION_FILTER_ALPHA);
            filtered[i].control_y =
                (previous->beacon[match].control_y * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                (current->beacon[i].control_y * BEACON_FUSION_FILTER_ALPHA);
            filtered[i].stable_ticks = previous->beacon[match].stable_ticks;
            if(filtered[i].stable_ticks < 255U)
            {
                filtered[i].stable_ticks++;
            }
            filtered[i].confidence =
                (previous->beacon[match].confidence * 0.4f) +
                (current->beacon[i].confidence * 0.6f);

            dx = filtered[i].x_body;
            dy = filtered[i].y_body;
            filtered[i].range_proxy = sqrtf((dx * dx) + (dy * dy));
            filtered[i].bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(dx, dy)));
        }
        else
        {
            filtered[i].stable_ticks = 1U;
        }
    }

    for(i = 0U; i < current->beacon_count; i++)
    {
        uint8 j;

        if(filtered[i].valid != 0U)
        {
            continue;
        }

        for(j = 0U; j < previous->beacon_count; j++)
        {
            if((previous_used[j] != 0U) || (previous->beacon[j].valid == 0U))
            {
                continue;
            }

            filtered[i] = previous->beacon[j];
            previous_used[j] = 1U;
            filtered[i].confidence *= 0.95f;
            break;
        }
    }

    for(i = 0U; i < BEACON_FUSION_MAX_BEACONS; i++)
    {
        current->beacon[i] = filtered[i];
    }
}

static uint8 beacon_fusion_choose_best(const beacon_fusion_result_t *result)
{
    uint8 i;
    uint8 best = 0U;
    float best_score = -1000000.0f;

    for(i = 0U; i < result->beacon_count; i++)
    {
        float score;

        if(result->beacon[i].valid == 0U)
        {
            continue;
        }

        score = result->beacon[i].confidence -
                (result->beacon[i].range_proxy * 0.0002f) -
                (beacon_fusion_absf(result->beacon[i].bearing_deg) * 0.002f);
        if(score > best_score)
        {
            best_score = score;
            best = i;
        }
    }

    return best;
}

void beacon_fusion_init(void)
{
    memset(&g_beacon_fusion_result, 0, sizeof(g_beacon_fusion_result));
    g_beacon_fusion_result.best_index = BEACON_FUSION_MAX_BEACONS;
    s_fused_count_state = 0U;
    s_no_observation_ticks = 0U;
}

void beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT])
{
    beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS];
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS];
    beacon_fusion_result_t previous;
    beacon_fusion_result_t current;
    uint8 observation_count;
    uint8 group_count;
    uint8 target_count;
    uint8 i;

    previous = g_beacon_fusion_result;
    memset(&current, 0, sizeof(current));
    current.best_index = BEACON_FUSION_MAX_BEACONS;
    current.update_count = previous.update_count + 1U;

    memset(observation, 0, sizeof(observation));
    observation_count = beacon_fusion_collect_observations(camera, observation);
    group_count = beacon_fusion_build_groups(observation,
                                             observation_count,
                                             BEACON_FUSION_MAX_BEACONS,
                                             group);
    target_count = beacon_fusion_update_count_state(group_count);

    current.observation_count = observation_count;
    current.beacon_count = target_count;
    for(i = 0U; i < group_count; i++)
    {
        beacon_fusion_group_to_beacon(&group[i], &current.beacon[i]);
    }

    beacon_fusion_apply_output_filter(&current, &previous);
    if(current.beacon_count > 0U)
    {
        current.best_index = beacon_fusion_choose_best(&current);
    }

    g_beacon_fusion_result = current;
}

const beacon_fusion_result_t *beacon_fusion_get_result(void)
{
    return &g_beacon_fusion_result;
}
