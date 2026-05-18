#include "beacon_fusion.h"

#include <math.h>
#include <string.h>

#define BEACON_FUSION_PI                         (3.1415926f)
#define BEACON_FUSION_HALF_DIAGONAL_PX           (111.5168f)
#define BEACON_FUSION_MAX_IMAGE_ANGLE_RAD        (1.48353f)
#define BEACON_FUSION_MIN_RADIUS_PX              (1.0f)
#define BEACON_FUSION_SECONDARY_MIN_RADIUS_PX    (1.35f)
#define BEACON_FUSION_MIN_HEIGHT_MM              (100.0f)
#define BEACON_FUSION_MAX_HEIGHT_MM              (2500.0f)
#define BEACON_FUSION_MAX_ATTITUDE_DEG           (50.0f)
#define BEACON_FUSION_MIN_GROUND_RAY_Z           (0.02f)
#define BEACON_FUSION_FALLBACK_RANGE_MM          (900.0f)
#define BEACON_FUSION_CLUSTER_DISTANCE_MM        (900.0f)
#define BEACON_FUSION_CLUSTER_ANGLE_DEG          (18.0f)
#define BEACON_FUSION_MERGE_ANGLE_WEIGHT_MM      (25.0f)
#define BEACON_FUSION_TWO_EVIDENCE_TICKS         (15U)
#define BEACON_FUSION_NO_OBSERVATION_CLEAR_TICKS (30U)
#define BEACON_FUSION_FILTER_ALPHA               (0.35f)
#define BEACON_FUSION_STRONG_MIN_RADIUS_PX       (1.8f)
#define BEACON_FUSION_VERTICAL_DX_PX             (35.0f)
#define BEACON_FUSION_VERTICAL_DY_PX             (22.0f)
#define BEACON_FUSION_DIAGONAL_DX_PX             (75.0f)
#define BEACON_FUSION_DIAGONAL_DY_PX             (25.0f)
#define BEACON_FUSION_CROSS12_DX_MIN_PX          (45.0f)
#define BEACON_FUSION_CROSS12_DX_MAX_PX          (75.0f)
#define BEACON_FUSION_CROSS12_DY_MAX_PX          (14.0f)
#define BEACON_FUSION_CROSS02_DX_MIN_PX          (95.0f)
#define BEACON_FUSION_CROSS02_DY_MIN_PX          (35.0f)

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
    uint8 has_ground_point;
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
    uint8 ground_count;
    float sum_x;
    float sum_y;
    float sum_image_x;
    float sum_image_y;
    float sum_sin;
    float sum_cos;
    float sum_weight;
    float radius_max;
} beacon_fusion_group_t;

beacon_fusion_result_t g_beacon_fusion_result;

static uint8 s_fused_count_state;
static uint8 s_two_evidence_ticks;
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

static uint8 beacon_fusion_pose_valid(const beacon_fusion_pose_t *pose)
{
    if(pose == 0)
    {
        return 0U;
    }
    if(pose->valid == 0U)
    {
        return 0U;
    }
    if((beacon_fusion_float_valid(pose->height_mm) == 0U) ||
       (beacon_fusion_float_valid(pose->roll_deg) == 0U) ||
       (beacon_fusion_float_valid(pose->pitch_deg) == 0U))
    {
        return 0U;
    }
    if((pose->height_mm < BEACON_FUSION_MIN_HEIGHT_MM) ||
       (pose->height_mm > BEACON_FUSION_MAX_HEIGHT_MM))
    {
        return 0U;
    }
    if((beacon_fusion_absf(pose->roll_deg) > BEACON_FUSION_MAX_ATTITUDE_DEG) ||
       (beacon_fusion_absf(pose->pitch_deg) > BEACON_FUSION_MAX_ATTITUDE_DEG))
    {
        return 0U;
    }
    return 1U;
}

static beacon_fusion_vec3_t beacon_fusion_rotate_roll_pitch(const beacon_fusion_vec3_t *ray,
                                                            const beacon_fusion_pose_t *pose)
{
    beacon_fusion_vec3_t out;
    float roll_rad = beacon_fusion_deg_to_rad(pose->roll_deg);
    float pitch_rad = beacon_fusion_deg_to_rad(pose->pitch_deg);
    float sr = sinf(roll_rad);
    float cr = cosf(roll_rad);
    float sp = sinf(pitch_rad);
    float cp = cosf(pitch_rad);
    float x1;
    float y1;
    float z1;

    x1 = (cp * ray->x) + (sp * sr * ray->y) + (sp * cr * ray->z);
    y1 = (cr * ray->y) - (sr * ray->z);
    z1 = (-sp * ray->x) + (cp * sr * ray->y) + (cp * cr * ray->z);

    out.x = x1;
    out.y = y1;
    out.z = z1;
    return beacon_fusion_vec3_normalize(&out);
}

static beacon_fusion_vec3_t beacon_fusion_image_to_ray(uint8 camera_id, float image_x, float image_y)
{
    const beacon_fusion_camera_model_t *camera = &s_camera_model[camera_id];
    beacon_fusion_vec3_t ray;
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

static float beacon_fusion_target_weight(float radius, uint8 target_id, uint8 has_ground_point)
{
    float weight = beacon_fusion_clampf(radius / 4.0f, 0.25f, 2.0f);

    if(target_id > 0U)
    {
        weight *= 0.8f;
    }
    if(has_ground_point == 0U)
    {
        weight *= 0.6f;
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
    const beacon_fusion_pose_t *pose,
    uint8 pose_valid,
    beacon_fusion_observation_t *observation)
{
    beacon_fusion_vec3_t ray = beacon_fusion_image_to_ray(camera_id, target->x, target->y);
    beacon_fusion_vec3_t pose_ray = ray;
    float scale;

    memset(observation, 0, sizeof(*observation));
    observation->valid = 1U;
    observation->camera_id = camera_id;
    observation->target_id = target_id;
    observation->source_camera_mask = (uint8)(1U << camera_id);
    observation->image_x = target->x;
    observation->image_y = target->y;
    observation->radius = target->radius;

    if(pose_valid != 0U)
    {
        pose_ray = beacon_fusion_rotate_roll_pitch(&ray, pose);
    }

    observation->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(ray.x, ray.y)));
    if((pose_valid != 0U) && (pose_ray.z < -BEACON_FUSION_MIN_GROUND_RAY_Z))
    {
        scale = -pose->height_mm / pose_ray.z;
        observation->x_body = ray.x * scale;
        observation->y_body = ray.y * scale;
        observation->range_proxy = sqrtf((observation->x_body * observation->x_body) +
                                         (observation->y_body * observation->y_body));
        observation->has_ground_point = 1U;
    }
    else
    {
        float bearing_rad = beacon_fusion_deg_to_rad(observation->bearing_deg);

        observation->range_proxy = BEACON_FUSION_FALLBACK_RANGE_MM;
        observation->x_body = sinf(bearing_rad) * BEACON_FUSION_FALLBACK_RANGE_MM;
        observation->y_body = cosf(bearing_rad) * BEACON_FUSION_FALLBACK_RANGE_MM;
        observation->has_ground_point = 0U;
    }

    observation->weight = beacon_fusion_target_weight(target->radius,
                                                      target_id,
                                                      observation->has_ground_point);
}

static uint8 beacon_fusion_collect_observations(
    const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
    const beacon_fusion_pose_t *pose,
    beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS])
{
    uint8 pose_valid = beacon_fusion_pose_valid(pose);
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
                                                  pose,
                                                  pose_valid,
                                                  &observation[count]);
            count++;
        }
    }

    return count;
}

static uint8 beacon_fusion_has_strong_two_evidence(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 observation_count)
{
    uint8 i;
    uint8 j;

    for(i = 0U; i < observation_count; i++)
    {
        for(j = (uint8)(i + 1U); j < observation_count; j++)
        {
            float dx;
            float dy;
            float r_min;

            if(observation[i].camera_id != observation[j].camera_id)
            {
                continue;
            }

            dx = beacon_fusion_absf(observation[i].image_x - observation[j].image_x);
            dy = beacon_fusion_absf(observation[i].image_y - observation[j].image_y);
            r_min = (observation[i].radius < observation[j].radius) ?
                    observation[i].radius : observation[j].radius;

            if(r_min < BEACON_FUSION_STRONG_MIN_RADIUS_PX)
            {
                continue;
            }
            if((dx <= BEACON_FUSION_VERTICAL_DX_PX) && (dy >= BEACON_FUSION_VERTICAL_DY_PX))
            {
                return 1U;
            }
            if((dx >= BEACON_FUSION_DIAGONAL_DX_PX) && (dy >= BEACON_FUSION_DIAGONAL_DY_PX))
            {
                return 1U;
            }
        }
    }

    for(i = 0U; i < observation_count; i++)
    {
        for(j = (uint8)(i + 1U); j < observation_count; j++)
        {
            uint8 camera_min;
            uint8 camera_max;
            float dx;
            float dy;

            if(observation[i].camera_id == observation[j].camera_id)
            {
                continue;
            }

            camera_min = (observation[i].camera_id < observation[j].camera_id) ?
                         observation[i].camera_id : observation[j].camera_id;
            camera_max = (observation[i].camera_id < observation[j].camera_id) ?
                         observation[j].camera_id : observation[i].camera_id;
            dx = beacon_fusion_absf(observation[i].image_x - observation[j].image_x);
            dy = beacon_fusion_absf(observation[i].image_y - observation[j].image_y);

            if((camera_min == 1U) && (camera_max == 2U) &&
               (dx >= BEACON_FUSION_CROSS12_DX_MIN_PX) &&
               (dx <= BEACON_FUSION_CROSS12_DX_MAX_PX) &&
               (dy <= BEACON_FUSION_CROSS12_DY_MAX_PX))
            {
                return 1U;
            }
            if((camera_min == 0U) && (camera_max == 2U) &&
               (dx >= BEACON_FUSION_CROSS02_DX_MIN_PX) &&
               (dy >= BEACON_FUSION_CROSS02_DY_MIN_PX))
            {
                return 1U;
            }
        }
    }

    return 0U;
}

static uint8 beacon_fusion_update_count_state(uint8 observation_count, uint8 has_two_evidence)
{
    if(observation_count == 0U)
    {
        if(s_no_observation_ticks < 255U)
        {
            s_no_observation_ticks++;
        }
        s_two_evidence_ticks = 0U;
        if(s_no_observation_ticks >= BEACON_FUSION_NO_OBSERVATION_CLEAR_TICKS)
        {
            s_fused_count_state = 0U;
        }
        return s_fused_count_state;
    }

    s_no_observation_ticks = 0U;
    if(s_fused_count_state == 0U)
    {
        s_fused_count_state = (has_two_evidence != 0U) ? 2U : 1U;
        s_two_evidence_ticks = 0U;
    }
    else if(s_fused_count_state == 1U)
    {
        if(has_two_evidence != 0U)
        {
            if(s_two_evidence_ticks < 255U)
            {
                s_two_evidence_ticks++;
            }
            if(s_two_evidence_ticks >= BEACON_FUSION_TWO_EVIDENCE_TICKS)
            {
                s_fused_count_state = 2U;
                s_two_evidence_ticks = 0U;
            }
        }
        else
        {
            s_two_evidence_ticks = 0U;
        }
    }
    else
    {
        s_fused_count_state = 2U;
    }

    if(s_fused_count_state > BEACON_FUSION_MAX_BEACONS)
    {
        s_fused_count_state = BEACON_FUSION_MAX_BEACONS;
    }

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
                                           uint8 has_ground_point,
                                           uint8 observation_count)
{
    float bearing_rad = beacon_fusion_deg_to_rad(bearing_deg);

    group->valid = 1U;
    group->source_camera_mask |= source_camera_mask;
    group->observation_count += observation_count;
    group->ground_count += has_ground_point;
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
                                   observation->has_ground_point,
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

static uint8 beacon_fusion_observation_matches_group(const beacon_fusion_group_t *group,
                                                     const beacon_fusion_observation_t *observation)
{
    float dx = observation->x_body - beacon_fusion_group_x(group);
    float dy = observation->y_body - beacon_fusion_group_y(group);
    float distance = sqrtf((dx * dx) + (dy * dy));
    float angle = beacon_fusion_angle_diff_abs_deg(observation->bearing_deg,
                                                   beacon_fusion_group_bearing_deg(group));

    if((group->source_camera_mask & observation->source_camera_mask) != 0U)
    {
        float image_dx = beacon_fusion_absf(observation->image_x -
                                            beacon_fusion_group_image_x(group));
        float image_dy = beacon_fusion_absf(observation->image_y -
                                            beacon_fusion_group_image_y(group));

        if(((image_dx <= BEACON_FUSION_VERTICAL_DX_PX) &&
            (image_dy >= BEACON_FUSION_VERTICAL_DY_PX)) ||
           ((image_dx >= BEACON_FUSION_DIAGONAL_DX_PX) &&
            (image_dy >= BEACON_FUSION_CROSS12_DY_MAX_PX)) ||
           (image_dy >= BEACON_FUSION_DIAGONAL_DY_PX))
        {
            return 0U;
        }
    }

    if((distance <= BEACON_FUSION_CLUSTER_DISTANCE_MM) ||
       (angle <= BEACON_FUSION_CLUSTER_ANGLE_DEG))
    {
        return 1U;
    }
    return 0U;
}

static float beacon_fusion_group_group_score(const beacon_fusion_group_t *a,
                                             const beacon_fusion_group_t *b)
{
    float dx = beacon_fusion_group_x(a) - beacon_fusion_group_x(b);
    float dy = beacon_fusion_group_y(a) - beacon_fusion_group_y(b);
    float distance = sqrtf((dx * dx) + (dy * dy));
    float angle = beacon_fusion_angle_diff_abs_deg(beacon_fusion_group_bearing_deg(a),
                                                   beacon_fusion_group_bearing_deg(b));

    return distance + (angle * BEACON_FUSION_MERGE_ANGLE_WEIGHT_MM);
}

static void beacon_fusion_group_merge(beacon_fusion_group_t *dst,
                                      const beacon_fusion_group_t *src)
{
    dst->source_camera_mask |= src->source_camera_mask;
    dst->observation_count += src->observation_count;
    dst->ground_count += src->ground_count;
    dst->sum_x += src->sum_x;
    dst->sum_y += src->sum_y;
    dst->sum_image_x += src->sum_image_x;
    dst->sum_image_y += src->sum_image_y;
    dst->sum_sin += src->sum_sin;
    dst->sum_cos += src->sum_cos;
    dst->sum_weight += src->sum_weight;
    if(src->radius_max > dst->radius_max)
    {
        dst->radius_max = src->radius_max;
    }
}

static uint8 beacon_fusion_build_groups(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 observation_count,
    uint8 target_count,
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS])
{
    uint8 i;
    uint8 group_count = 0U;

    for(i = 0U; i < BEACON_FUSION_MAX_BEACONS; i++)
    {
        beacon_fusion_group_reset(&group[i]);
    }

    for(i = 0U; i < observation_count; i++)
    {
        uint8 j;
        uint8 best = BEACON_FUSION_MAX_BEACONS;
        float best_score = 100000000.0f;

        if(observation[i].valid == 0U)
        {
            continue;
        }

        for(j = 0U; j < group_count; j++)
        {
            float score;

            if(beacon_fusion_observation_matches_group(&group[j], &observation[i]) == 0U)
            {
                continue;
            }
            score = beacon_fusion_group_observation_score(&group[j], &observation[i]);
            if(score < best_score)
            {
                best_score = score;
                best = j;
            }
        }

        if(best < BEACON_FUSION_MAX_BEACONS)
        {
            beacon_fusion_group_add_observation(&group[best], &observation[i]);
        }
        else if(group_count < BEACON_FUSION_MAX_BEACONS)
        {
            beacon_fusion_group_add_observation(&group[group_count], &observation[i]);
            group_count++;
        }
    }

    while((target_count > 0U) && (group_count > target_count))
    {
        uint8 a;
        uint8 b;
        uint8 best_a = 0U;
        uint8 best_b = 1U;
        float best_score = 100000000.0f;

        for(a = 0U; a < group_count; a++)
        {
            for(b = (uint8)(a + 1U); b < group_count; b++)
            {
                float score = beacon_fusion_group_group_score(&group[a], &group[b]);

                if(score < best_score)
                {
                    best_score = score;
                    best_a = a;
                    best_b = b;
                }
            }
        }

        beacon_fusion_group_merge(&group[best_a], &group[best_b]);
        for(i = best_b; (uint8)(i + 1U) < group_count; i++)
        {
            group[i] = group[i + 1U];
        }
        group_count--;
    }

    return group_count;
}

static void beacon_fusion_group_to_beacon(const beacon_fusion_group_t *group,
                                          beacon_fusion_beacon_t *beacon)
{
    float ground_ratio;

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
    beacon->range_proxy = sqrtf((beacon->x_body * beacon->x_body) + (beacon->y_body * beacon->y_body));
    beacon->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(beacon->x_body,
                                                                                  beacon->y_body)));

    ground_ratio = (group->observation_count > 0U) ?
                   ((float)group->ground_count / (float)group->observation_count) : 0.0f;
    beacon->confidence = 0.25f +
                         (group->radius_max * 0.08f) +
                         ((float)group->observation_count * 0.08f) +
                         (ground_ratio * 0.2f);
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
    s_two_evidence_ticks = 0U;
    s_no_observation_ticks = 0U;
}

void beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
                                const beacon_fusion_pose_t *pose)
{
    beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS];
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS];
    beacon_fusion_result_t previous;
    beacon_fusion_result_t current;
    uint8 observation_count;
    uint8 group_count;
    uint8 target_count;
    uint8 has_two_evidence;
    uint8 i;

    previous = g_beacon_fusion_result;
    memset(&current, 0, sizeof(current));
    current.best_index = BEACON_FUSION_MAX_BEACONS;
    current.update_count = previous.update_count + 1U;

    memset(observation, 0, sizeof(observation));
    observation_count = beacon_fusion_collect_observations(camera, pose, observation);
    has_two_evidence = beacon_fusion_has_strong_two_evidence(observation, observation_count);
    target_count = beacon_fusion_update_count_state(observation_count, has_two_evidence);
    group_count = beacon_fusion_build_groups(observation, observation_count, target_count, group);

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
