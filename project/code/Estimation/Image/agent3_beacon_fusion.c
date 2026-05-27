#include "agent3_beacon_fusion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define AGENT3_IMAGE_WIDTH   (187.0f)
#define AGENT3_IMAGE_HEIGHT  (119.0f)
#define AGENT3_SIZE_SCALE    (6.0f)

typedef struct
{
    uint8_t camera_id;
    uint8_t slot_id;
    float x;
    float y;
    float size;
} agent3_observation_t;

static uint8_t agent3_target_is_valid(const agent3_camera_target_t *target)
{
    return target->valid != 0u;
}

static int agent3_observation_before(const agent3_observation_t *lhs,
                                     const agent3_observation_t *rhs)
{
    if (lhs->y > rhs->y) {
        return 1;
    }
    if (lhs->y < rhs->y) {
        return 0;
    }
    if (lhs->size > rhs->size) {
        return 1;
    }
    if (lhs->size < rhs->size) {
        return 0;
    }
    if (lhs->x < rhs->x) {
        return 1;
    }
    if (lhs->x > rhs->x) {
        return 0;
    }
    return lhs->slot_id < rhs->slot_id;
}

static void agent3_sort_observations(agent3_observation_t *items, uint8_t count)
{
    uint8_t i;

    for (i = 1u; i < count; ++i) {
        uint8_t j = i;
        const agent3_observation_t current = items[i];

        while (j > 0u && agent3_observation_before(&current, &items[j - 1u]) != 0) {
            items[j] = items[j - 1u];
            --j;
        }
        items[j] = current;
    }
}

static float agent3_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static void agent3_build_beacon(agent3_fused_beacon_t *beacon,
                                const agent3_observation_t members[AGENT3_CAMERA_COUNT],
                                uint8_t member_count)
{
    uint8_t i;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_size = 0.0f;
    float dispersion = 0.0f;

    memset(beacon, 0, sizeof(*beacon));

    if (member_count == 0u) {
        beacon->valid = 1u;
        beacon->synthetic = 1u;
        beacon->quality = 0.0f;
        return;
    }

    for (i = 0u; i < member_count; ++i) {
        mean_x += members[i].x;
        mean_y += members[i].y;
        mean_size += members[i].size;
        beacon->source_camera_mask = (uint8_t)(beacon->source_camera_mask | (uint8_t)(1u << members[i].camera_id));
    }

    mean_x /= (float)member_count;
    mean_y /= (float)member_count;
    mean_size /= (float)member_count;

    for (i = 0u; i < member_count; ++i) {
        dispersion += agent3_absf(members[i].x - mean_x) / AGENT3_IMAGE_WIDTH;
        dispersion += agent3_absf(members[i].y - mean_y) / AGENT3_IMAGE_HEIGHT;
        dispersion += agent3_absf(members[i].size - mean_size) / AGENT3_SIZE_SCALE;
    }
    dispersion /= (float)(member_count * 3u);

    beacon->valid = 1u;
    beacon->synthetic = 0u;
    beacon->observation_count = member_count;
    beacon->mean_x = mean_x;
    beacon->mean_y = mean_y;
    beacon->mean_size = mean_size;
    beacon->quality = ((float)member_count / (float)AGENT3_CAMERA_COUNT) * (1.0f - dispersion);
    if (beacon->quality < 0.0f) {
        beacon->quality = 0.0f;
    } else if (beacon->quality > 1.0f) {
        beacon->quality = 1.0f;
    }
}

void agent3_fusion_init_result(agent3_fusion_result_t *result)
{
    if (result != NULL) {
        memset(result, 0, sizeof(*result));
        result->status = AGENT3_FUSION_BAD_ARGUMENT;
    }
}

agent3_fusion_status_t agent3_fusion_update(
    const agent3_camera_frame_t camera[AGENT3_CAMERA_COUNT],
    uint8_t expected_count,
    agent3_fusion_result_t *result)
{
    agent3_observation_t sorted[AGENT3_CAMERA_COUNT][AGENT3_TARGETS_PER_CAMERA];
    uint8_t camera_id;
    uint8_t cluster_id;

    if (camera == NULL || result == NULL || expected_count > AGENT3_MAX_BEACONS) {
        if (result != NULL) {
            agent3_fusion_init_result(result);
        }
        return AGENT3_FUSION_BAD_ARGUMENT;
    }

    agent3_fusion_init_result(result);
    result->expected_count = expected_count;

    for (camera_id = 0u; camera_id < AGENT3_CAMERA_COUNT; ++camera_id) {
        uint8_t slot_id;

        for (slot_id = 0u; slot_id < AGENT3_TARGETS_PER_CAMERA; ++slot_id) {
            const agent3_camera_target_t *target = &camera[camera_id].target[slot_id];

            if (agent3_target_is_valid(target) != 0u) {
                const uint8_t index = result->camera_count[camera_id];
                sorted[camera_id][index].camera_id = camera_id;
                sorted[camera_id][index].slot_id = slot_id;
                sorted[camera_id][index].x = target->x;
                sorted[camera_id][index].y = target->y;
                sorted[camera_id][index].size = target->size;
                result->camera_count[camera_id] = (uint8_t)(index + 1u);
            }
        }

        if (result->camera_count[camera_id] > expected_count) {
            result->status = AGENT3_FUSION_DISCARD_OVER_COUNT;
            result->fused_count = 0u;
            return result->status;
        }

        agent3_sort_observations(sorted[camera_id], result->camera_count[camera_id]);
    }

    for (cluster_id = 0u; cluster_id < expected_count; ++cluster_id) {
        agent3_observation_t members[AGENT3_CAMERA_COUNT];
        uint8_t member_count = 0u;

        for (camera_id = 0u; camera_id < AGENT3_CAMERA_COUNT; ++camera_id) {
            if (cluster_id < result->camera_count[camera_id]) {
                members[member_count] = sorted[camera_id][cluster_id];
                ++member_count;
            }
        }

        agent3_build_beacon(&result->beacon[cluster_id], members, member_count);
    }

    result->status = AGENT3_FUSION_OK;
    result->fused_count = expected_count;
    return result->status;
}
