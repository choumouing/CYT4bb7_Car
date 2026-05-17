#include "beacon_fusion.h"

#include <math.h>
#include <string.h>

#define BEACON_FUSION_PI                         (3.1415926f)
#define BEACON_FUSION_NEAR_X                     (0.0f)
#define BEACON_FUSION_NEAR_Y                     (16.0f)
#define BEACON_FUSION_MIN_RADIUS_PX              (1.0f)
#define BEACON_FUSION_SECONDARY_MIN_RADIUS_PX    (1.4f)
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
#define BEACON_FUSION_TWO_PROMOTE_TICKS          (15U)
#define BEACON_FUSION_NO_OBSERVATION_CLEAR_TICKS (30U)
#define BEACON_FUSION_FILTER_ALPHA               (0.35f)

typedef struct
{
    uint8 valid;
    uint8 camera_id;
    uint8 target_id;
    uint8 source_camera_mask;
    float image_x;
    float image_y;
    float radius;
    float range_proxy;
    float bearing_deg;
    float weight;
} beacon_fusion_observation_t;

typedef struct
{
    uint8 valid;
    uint8 source_camera_mask;
    uint8 observation_count;
    float sum_x;
    float sum_y;
    float sum_weight;
    float radius_max;
} beacon_fusion_group_t;

beacon_fusion_result_t g_beacon_fusion_result;

static uint8 s_fused_count_state;
static uint8 s_two_evidence_ticks;
static uint8 s_no_observation_ticks;

static const float s_camera_yaw_deg[BEACON_FUSION_CAMERA_COUNT] =
{
    120.0f,
    0.0f,
    -120.0f
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

static float beacon_fusion_deg_to_rad(float angle_deg)
{
    return angle_deg * (BEACON_FUSION_PI / 180.0f);
}

static float beacon_fusion_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / BEACON_FUSION_PI);
}

static float beacon_fusion_target_range_proxy(float image_x, float image_y)
{
    float dx = image_x - BEACON_FUSION_NEAR_X;
    float dy = image_y - BEACON_FUSION_NEAR_Y;

    return sqrtf((dx * dx) + (dy * dy));
}

static float beacon_fusion_target_bearing_deg(uint8 camera_id, float image_x)
{
    float local_deg;

    local_deg = (image_x / 94.0f) * 60.0f;
    return beacon_fusion_wrap_deg(s_camera_yaw_deg[camera_id] + local_deg);
}

static float beacon_fusion_target_weight(float radius, uint8 target_id)
{
    float weight = beacon_fusion_clampf(radius / 4.0f, 0.25f, 2.0f);

    if(target_id > 0U)
    {
        weight *= 0.75f;
    }
    return weight;
}

static uint8 beacon_fusion_target_valid(const beacon_fusion_camera_target_t *target, uint8 target_id)
{
    if(target->radius < BEACON_FUSION_MIN_RADIUS_PX)
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

            observation[count].valid = 1U;
            observation[count].camera_id = camera_id;
            observation[count].target_id = target_id;
            observation[count].source_camera_mask = (uint8)(1U << camera_id);
            observation[count].image_x = target->x;
            observation[count].image_y = target->y;
            observation[count].radius = target->radius;
            observation[count].range_proxy = beacon_fusion_target_range_proxy(target->x, target->y);
            observation[count].bearing_deg = beacon_fusion_target_bearing_deg(camera_id, target->x);
            observation[count].weight = beacon_fusion_target_weight(target->radius, target_id);
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
            if(s_two_evidence_ticks >= BEACON_FUSION_TWO_PROMOTE_TICKS)
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

    return s_fused_count_state;
}

static void beacon_fusion_group_reset(beacon_fusion_group_t *group)
{
    memset(group, 0, sizeof(*group));
}

static void beacon_fusion_group_add(beacon_fusion_group_t *group,
                                    const beacon_fusion_observation_t *observation)
{
    float bearing_rad = beacon_fusion_deg_to_rad(observation->bearing_deg);
    float x_body = sinf(bearing_rad) * observation->range_proxy;
    float y_body = cosf(bearing_rad) * observation->range_proxy;

    group->valid = 1U;
    group->source_camera_mask |= observation->source_camera_mask;
    group->observation_count++;
    group->sum_x += x_body * observation->weight;
    group->sum_y += y_body * observation->weight;
    group->sum_weight += observation->weight;
    if(observation->radius > group->radius_max)
    {
        group->radius_max = observation->radius;
    }
}

static uint8 beacon_fusion_find_nearest_observation(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 observation_count,
    const uint8 used[BEACON_FUSION_MAX_OBSERVATIONS])
{
    uint8 i;
    uint8 best = BEACON_FUSION_MAX_OBSERVATIONS;
    float best_range = 1000000.0f;

    for(i = 0U; i < observation_count; i++)
    {
        if((used[i] != 0U) || (observation[i].valid == 0U))
        {
            continue;
        }
        if(observation[i].range_proxy < best_range)
        {
            best_range = observation[i].range_proxy;
            best = i;
        }
    }

    return best;
}

static uint8 beacon_fusion_find_farthest_from_group(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 observation_count,
    const uint8 used[BEACON_FUSION_MAX_OBSERVATIONS],
    const beacon_fusion_group_t *group)
{
    uint8 i;
    uint8 best = BEACON_FUSION_MAX_OBSERVATIONS;
    float center_x = group->sum_x / group->sum_weight;
    float center_y = group->sum_y / group->sum_weight;
    float best_distance = -1.0f;

    for(i = 0U; i < observation_count; i++)
    {
        float bearing_rad;
        float x_body;
        float y_body;
        float dx;
        float dy;
        float distance;

        if((used[i] != 0U) || (observation[i].valid == 0U))
        {
            continue;
        }

        bearing_rad = beacon_fusion_deg_to_rad(observation[i].bearing_deg);
        x_body = sinf(bearing_rad) * observation[i].range_proxy;
        y_body = cosf(bearing_rad) * observation[i].range_proxy;
        dx = x_body - center_x;
        dy = y_body - center_y;
        distance = sqrtf((dx * dx) + (dy * dy));
        if(distance > best_distance)
        {
            best_distance = distance;
            best = i;
        }
    }

    return best;
}

static uint8 beacon_fusion_build_groups(
    const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 observation_count,
    uint8 target_count,
    beacon_fusion_group_t group[BEACON_FUSION_MAX_BEACONS])
{
    uint8 used[BEACON_FUSION_MAX_OBSERVATIONS] = {0U};
    uint8 group_count = 0U;
    uint8 best;

    for(best = 0U; best < BEACON_FUSION_MAX_BEACONS; best++)
    {
        beacon_fusion_group_reset(&group[best]);
    }

    if((target_count == 0U) || (observation_count == 0U))
    {
        return 0U;
    }

    best = beacon_fusion_find_nearest_observation(observation, observation_count, used);
    if(best < BEACON_FUSION_MAX_OBSERVATIONS)
    {
        beacon_fusion_group_add(&group[group_count], &observation[best]);
        used[best] = 1U;
        group_count++;
    }

    if((target_count > 1U) && (group_count < BEACON_FUSION_MAX_BEACONS))
    {
        best = beacon_fusion_find_farthest_from_group(observation, observation_count, used, &group[0]);
        if(best < BEACON_FUSION_MAX_OBSERVATIONS)
        {
            beacon_fusion_group_add(&group[group_count], &observation[best]);
            used[best] = 1U;
            group_count++;
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
    beacon->x_body = group->sum_x / group->sum_weight;
    beacon->y_body = group->sum_y / group->sum_weight;
    beacon->range_proxy = sqrtf((beacon->x_body * beacon->x_body) + (beacon->y_body * beacon->y_body));
    beacon->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(beacon->x_body,
                                                                                  beacon->y_body)));
    beacon->confidence = beacon_fusion_clampf(0.45f + (group->radius_max / 12.0f), 0.0f, 1.0f);
}

static void beacon_fusion_apply_output_filter(beacon_fusion_result_t *current,
                                              const beacon_fusion_result_t *previous)
{
    uint8 i;

    for(i = 0U; i < current->beacon_count; i++)
    {
        if((i < previous->beacon_count) && (previous->beacon[i].valid != 0U))
        {
            float dx;
            float dy;

            current->beacon[i].x_body =
                (previous->beacon[i].x_body * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                (current->beacon[i].x_body * BEACON_FUSION_FILTER_ALPHA);
            current->beacon[i].y_body =
                (previous->beacon[i].y_body * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                (current->beacon[i].y_body * BEACON_FUSION_FILTER_ALPHA);
            current->beacon[i].stable_ticks = previous->beacon[i].stable_ticks;
            if(current->beacon[i].stable_ticks < 255U)
            {
                current->beacon[i].stable_ticks++;
            }
            current->beacon[i].confidence =
                (previous->beacon[i].confidence * 0.4f) + (current->beacon[i].confidence * 0.6f);

            dx = current->beacon[i].x_body;
            dy = current->beacon[i].y_body;
            current->beacon[i].range_proxy = sqrtf((dx * dx) + (dy * dy));
            current->beacon[i].bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(dx, dy)));
        }
        else
        {
            current->beacon[i].stable_ticks = 1U;
        }
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
                (result->beacon[i].range_proxy * 0.01f) -
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

void beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT])
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
    observation_count = beacon_fusion_collect_observations(camera, observation);
    has_two_evidence = beacon_fusion_has_strong_two_evidence(observation, observation_count);
    target_count = beacon_fusion_update_count_state(observation_count, has_two_evidence);
    group_count = beacon_fusion_build_groups(observation, observation_count, target_count, group);

    current.observation_count = observation_count;
    current.beacon_count = target_count;
    for(i = 0U; (i < group_count) && (i < current.beacon_count); i++)
    {
        beacon_fusion_group_to_beacon(&group[i], &current.beacon[i]);
    }

    for(i = group_count; i < current.beacon_count; i++)
    {
        if((i < previous.beacon_count) && (previous.beacon[i].valid != 0U))
        {
            current.beacon[i] = previous.beacon[i];
        }
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
