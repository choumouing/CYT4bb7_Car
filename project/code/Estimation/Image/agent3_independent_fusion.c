#include "agent3_independent_fusion.h"

#include <string.h>

typedef struct
{
    uint8 valid;
    uint8 camera_id;
    float x;
    float y;
    float radius;
} agent3_observation_t;

typedef struct
{
    agent3_observation_t item[AGENT3_FUSION_CAMERA_TARGETS];
    uint8 count;
} agent3_camera_bucket_t;

static agent3_fusion_result_t s_agent3_result;
static uint8 s_agent3_expected_count;

static uint8 agent3_target_valid(const agent3_fusion_target_t *target)
{
    if(target->valid == 0U)
    {
        return 0U;
    }
    if((target->x == 0.0f) && (target->y == 0.0f) && (target->radius == 0.0f))
    {
        return 0U;
    }
    return 1U;
}

static uint8 agent3_observation_before(const agent3_observation_t *left,
                                       const agent3_observation_t *right)
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

static void agent3_sort_bucket(agent3_camera_bucket_t *bucket)
{
    uint8 i;
    uint8 j;

    for(i = 0U; i < bucket->count; i++)
    {
        for(j = (uint8)(i + 1U); j < bucket->count; j++)
        {
            if(agent3_observation_before(&bucket->item[j], &bucket->item[i]) != 0U)
            {
                agent3_observation_t temp = bucket->item[i];
                bucket->item[i] = bucket->item[j];
                bucket->item[j] = temp;
            }
        }
    }
}

static void agent3_build_bucket(const agent3_fusion_camera_frame_t *frame,
                                uint8 camera_id,
                                agent3_camera_bucket_t *bucket)
{
    uint8 target_id;

    memset(bucket, 0, sizeof(*bucket));
    for(target_id = 0U; target_id < AGENT3_FUSION_CAMERA_TARGETS; target_id++)
    {
        const agent3_fusion_target_t *target = &frame->target[target_id];

        if(agent3_target_valid(target) == 0U)
        {
            continue;
        }
        if(bucket->count >= AGENT3_FUSION_CAMERA_TARGETS)
        {
            break;
        }
        bucket->item[bucket->count].valid = 1U;
        bucket->item[bucket->count].camera_id = camera_id;
        bucket->item[bucket->count].x = target->x;
        bucket->item[bucket->count].y = target->y;
        bucket->item[bucket->count].radius = target->radius;
        bucket->count++;
    }
    agent3_sort_bucket(bucket);
}

void agent3_fusion_init(uint8 expected_count)
{
    memset(&s_agent3_result, 0, sizeof(s_agent3_result));
    agent3_fusion_set_expected_count(expected_count);
}

void agent3_fusion_set_expected_count(uint8 expected_count)
{
    if(expected_count > AGENT3_FUSION_MAX_LIGHTS)
    {
        expected_count = AGENT3_FUSION_MAX_LIGHTS;
    }
    s_agent3_expected_count = expected_count;
    s_agent3_result.expected_count = expected_count;
}

void agent3_fusion_update(const agent3_fusion_camera_frame_t camera[AGENT3_FUSION_CAMERA_COUNT])
{
    agent3_camera_bucket_t bucket[AGENT3_FUSION_CAMERA_COUNT];
    uint32 next_update_count = s_agent3_result.update_count + 1U;
    uint8 camera_id;
    uint8 rank;

    memset(&s_agent3_result, 0, sizeof(s_agent3_result));
    s_agent3_result.expected_count = s_agent3_expected_count;
    s_agent3_result.light_count = s_agent3_expected_count;
    s_agent3_result.update_count = next_update_count;

    for(camera_id = 0U; camera_id < AGENT3_FUSION_CAMERA_COUNT; camera_id++)
    {
        agent3_build_bucket(&camera[camera_id], camera_id, &bucket[camera_id]);
    }

    for(rank = 0U; rank < s_agent3_expected_count; rank++)
    {
        agent3_fusion_light_t *light = &s_agent3_result.light[rank];
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        float sum_radius = 0.0f;
        uint8 evidence_count = 0U;

        light->valid = 1U;
        for(camera_id = 0U; camera_id < AGENT3_FUSION_CAMERA_COUNT; camera_id++)
        {
            if(rank >= bucket[camera_id].count)
            {
                continue;
            }
            sum_x += bucket[camera_id].item[rank].x;
            sum_y += bucket[camera_id].item[rank].y;
            sum_radius += bucket[camera_id].item[rank].radius;
            light->source_camera_mask |= (uint8)(1U << camera_id);
            evidence_count++;
        }

        light->evidence_count = evidence_count;
        if(evidence_count > 0U)
        {
            light->mean_x = sum_x / (float)evidence_count;
            light->mean_y = sum_y / (float)evidence_count;
            light->mean_radius = sum_radius / (float)evidence_count;
        }
    }
}

const agent3_fusion_result_t *agent3_fusion_get_result(void)
{
    return &s_agent3_result;
}

