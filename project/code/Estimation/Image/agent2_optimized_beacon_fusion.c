#include "agent2_optimized_beacon_fusion.h"

#include <math.h>
#include <string.h>

#define AGENT2_PI                         (3.1415926f)
#define AGENT2_IMAGE_WIDTH_PX             (188.0f)
#define AGENT2_IMAGE_HEIGHT_PX            (120.0f)
#define AGENT2_IMAGE_CENTER_X_PX          (94.0f)
#define AGENT2_ORIGIN_PROJ_Y_PX           (100.0f)
#define AGENT2_MIN_RADIUS_PX              (1.0f)
#define AGENT2_SECONDARY_MIN_RADIUS_PX    (1.35f)
#define AGENT2_RANGE_RADIUS_SCALE_MM      (5200.0f)
#define AGENT2_FALLBACK_RANGE_MM          (900.0f)

typedef struct
{
    uint8 valid;
    uint8 camera_id;
    uint8 source_camera_mask;
    float x;
    float y;
    float radius;
} agent2_observation_t;

typedef struct
{
    agent2_observation_t item[AGENT2_BEACON_CAMERA_TARGETS];
    uint8 count;
} agent2_camera_bucket_t;

agent2_beacon_result_t g_agent2_beacon_result;

static uint8 s_expected_count;

static float agent2_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float agent2_clampf(float value, float min_value, float max_value)
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

static float agent2_wrap_deg(float angle_deg)
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

static uint8 agent2_float_valid(float value)
{
    return (value == value) && (value > -1000000.0f) && (value < 1000000.0f);
}

static uint8 agent2_target_valid(const agent2_beacon_camera_target_t *target, uint8 target_id)
{
    if(target->valid == 0U)
    {
        return 0U;
    }
    if((agent2_float_valid(target->x) == 0U) ||
       (agent2_float_valid(target->y) == 0U) ||
       (agent2_float_valid(target->radius) == 0U))
    {
        return 0U;
    }
    if((agent2_absf(target->x) < 0.001f) &&
       (agent2_absf(target->y) < 0.001f) &&
       (agent2_absf(target->radius) < 0.001f))
    {
        return 0U;
    }
    if((target->x < 0.0f) || (target->x >= AGENT2_IMAGE_WIDTH_PX) ||
       (target->y < 0.0f) || (target->y >= AGENT2_IMAGE_HEIGHT_PX))
    {
        return 0U;
    }
    if(target->radius < AGENT2_MIN_RADIUS_PX)
    {
        return 0U;
    }
    if((target_id > 0U) && (target->radius < AGENT2_SECONDARY_MIN_RADIUS_PX))
    {
        return 0U;
    }
    return 1U;
}

static uint8 agent2_observation_before(const agent2_observation_t *left,
                                       const agent2_observation_t *right)
{
    if(left->radius > right->radius)
    {
        return 1U;
    }
    if(left->radius < right->radius)
    {
        return 0U;
    }
    if(left->y > right->y)
    {
        return 1U;
    }
    if(left->y < right->y)
    {
        return 0U;
    }
    return (left->x < right->x) ? 1U : 0U;
}

static void agent2_sort_bucket(agent2_camera_bucket_t *bucket)
{
    uint8 i;
    uint8 j;

    for(i = 0U; i < bucket->count; i++)
    {
        for(j = (uint8)(i + 1U); j < bucket->count; j++)
        {
            if(agent2_observation_before(&bucket->item[j], &bucket->item[i]) != 0U)
            {
                agent2_observation_t temp = bucket->item[i];
                bucket->item[i] = bucket->item[j];
                bucket->item[j] = temp;
            }
        }
    }
}

static void agent2_build_bucket(const agent2_beacon_camera_frame_t *frame,
                                uint8 camera_id,
                                agent2_camera_bucket_t *bucket)
{
    uint8 target_id;

    memset(bucket, 0, sizeof(*bucket));
    for(target_id = 0U; target_id < AGENT2_BEACON_CAMERA_TARGETS; target_id++)
    {
        const agent2_beacon_camera_target_t *target = &frame->target[target_id];

        if(agent2_target_valid(target, target_id) == 0U)
        {
            continue;
        }
        if(bucket->count >= AGENT2_BEACON_CAMERA_TARGETS)
        {
            break;
        }

        bucket->item[bucket->count].valid = 1U;
        bucket->item[bucket->count].camera_id = camera_id;
        bucket->item[bucket->count].source_camera_mask = (uint8)(1U << camera_id);
        bucket->item[bucket->count].x = target->x;
        bucket->item[bucket->count].y = target->y;
        bucket->item[bucket->count].radius = target->radius;
        bucket->count++;
    }
    agent2_sort_bucket(bucket);
}

static float agent2_estimate_bearing(float image_x, float image_y)
{
    float control_x = AGENT2_IMAGE_CENTER_X_PX - image_x;
    float control_y = image_y - AGENT2_ORIGIN_PROJ_Y_PX;

    if((agent2_absf(control_x) < 0.0001f) && (agent2_absf(control_y) < 0.0001f))
    {
        return 0.0f;
    }
    return agent2_wrap_deg(atan2f(control_x, control_y) * (180.0f / AGENT2_PI));
}

static float agent2_estimate_range(float radius, uint8 rank)
{
    if(radius > 0.001f)
    {
        return AGENT2_RANGE_RADIUS_SCALE_MM / radius;
    }
    return AGENT2_FALLBACK_RANGE_MM * (float)(rank + 1U);
}

static void agent2_fill_observed_beacon(agent2_beacon_t *beacon,
                                        const agent2_camera_bucket_t bucket[AGENT2_BEACON_CAMERA_COUNT],
                                        uint8 rank)
{
    float sum_weight = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_radius = 0.0f;
    uint8 camera_id;
    uint8 observation_count = 0U;
    uint8 source_mask = 0U;

    memset(beacon, 0, sizeof(*beacon));
    for(camera_id = 0U; camera_id < AGENT2_BEACON_CAMERA_COUNT; camera_id++)
    {
        if(rank >= bucket[camera_id].count)
        {
            continue;
        }
        {
            const agent2_observation_t *obs = &bucket[camera_id].item[rank];
            float weight = agent2_clampf(obs->radius, 0.1f, 20.0f);

            sum_weight += weight;
            sum_x += obs->x * weight;
            sum_y += obs->y * weight;
            sum_radius += obs->radius;
            source_mask |= obs->source_camera_mask;
            observation_count++;
        }
    }

    if(observation_count == 0U)
    {
        return;
    }

    beacon->valid = 1U;
    beacon->source_camera_mask = source_mask;
    beacon->observation_count = observation_count;
    beacon->image_x = sum_x / sum_weight;
    beacon->image_y = sum_y / sum_weight;
    beacon->radius = sum_radius / (float)observation_count;
    beacon->bearing_deg = agent2_estimate_bearing(beacon->image_x, beacon->image_y);
    beacon->range_proxy = agent2_estimate_range(beacon->radius, rank);
    beacon->confidence = agent2_clampf(0.25f + ((float)observation_count * 0.12f) +
                                       (beacon->radius * 0.08f),
                                       0.0f,
                                       1.0f);
}

void agent2_beacon_fusion_init(void)
{
    memset(&g_agent2_beacon_result, 0, sizeof(g_agent2_beacon_result));
    s_expected_count = 0U;
}

void agent2_beacon_fusion_set_expected_count(uint8 expected_count)
{
    if(expected_count > AGENT2_BEACON_MAX_BEACONS)
    {
        expected_count = AGENT2_BEACON_MAX_BEACONS;
    }
    s_expected_count = expected_count;
}

void agent2_beacon_fusion_update_100HZ(const agent2_beacon_camera_frame_t camera[AGENT2_BEACON_CAMERA_COUNT])
{
    agent2_camera_bucket_t bucket[AGENT2_BEACON_CAMERA_COUNT];
    agent2_beacon_result_t previous = g_agent2_beacon_result;
    uint8 camera_id;
    uint8 rank;

    memset(&g_agent2_beacon_result, 0, sizeof(g_agent2_beacon_result));
    g_agent2_beacon_result.configured_count = s_expected_count;
    g_agent2_beacon_result.beacon_count = s_expected_count;
    g_agent2_beacon_result.update_count = previous.update_count + 1U;

    for(camera_id = 0U; camera_id < AGENT2_BEACON_CAMERA_COUNT; camera_id++)
    {
        agent2_build_bucket(&camera[camera_id], camera_id, &bucket[camera_id]);
    }

    for(rank = 0U; rank < s_expected_count; rank++)
    {
        agent2_fill_observed_beacon(&g_agent2_beacon_result.beacon[rank], bucket, rank);
        if(g_agent2_beacon_result.beacon[rank].valid == 0U)
        {
            if((rank < previous.beacon_count) && (previous.beacon[rank].valid != 0U))
            {
                g_agent2_beacon_result.beacon[rank] = previous.beacon[rank];
                g_agent2_beacon_result.beacon[rank].confidence *= 0.9f;
            }
            else
            {
                g_agent2_beacon_result.beacon[rank].valid = 1U;
                g_agent2_beacon_result.beacon[rank].bearing_deg =
                    agent2_wrap_deg(((float)rank - (((float)s_expected_count - 1.0f) * 0.5f)) * 30.0f);
                g_agent2_beacon_result.beacon[rank].range_proxy = agent2_estimate_range(0.0f, rank);
                g_agent2_beacon_result.beacon[rank].confidence = 0.05f;
            }
        }
    }
}

const agent2_beacon_result_t *agent2_beacon_fusion_get_result(void)
{
    return &g_agent2_beacon_result;
}

