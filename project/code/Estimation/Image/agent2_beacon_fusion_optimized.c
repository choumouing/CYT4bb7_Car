#include "agent2_beacon_fusion_optimized.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define AGENT2_IMAGE_WIDTH_PX             (188.0f)
#define AGENT2_IMAGE_HEIGHT_PX            (120.0f)
#define AGENT2_IMAGE_CENTER_X_PX          (94.0f)
#define AGENT2_IMAGE_CENTER_Y_PX          (60.0f)
#define AGENT2_MIN_RADIUS_PX              (1.0f)
#define AGENT2_RADIUS_DISTANCE_SCALE      (900.0f)
#define AGENT2_MISSING_RANGE_PROXY        (5000.0f)
#define AGENT2_FILTER_ALPHA               (0.45f)
#define AGENT2_PI                         (3.1415926f)

typedef struct
{
    uint8_t camera_id;
    uint8_t slot_id;
    float x;
    float y;
    float radius;
    float bearing_deg;
    float range_proxy;
    float control_x;
    float control_y;
} agent2_observation_t;

typedef struct
{
    agent2_observation_t item[AGENT2_BEACON_FUSION_CAMERA_TARGETS];
    uint8_t count;
} agent2_camera_bucket_t;

agent2_beacon_fusion_result_t g_agent2_beacon_fusion_result;

static uint8_t s_expected_count;

static float agent2_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float agent2_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float agent2_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float agent2_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / AGENT2_PI);
}

static uint8_t agent2_float_valid(float value)
{
    return (value == value) && (value > -1000000.0f) && (value < 1000000.0f);
}

static uint8_t agent2_target_valid(const agent2_beacon_fusion_camera_target_t *target)
{
    if (target->valid == 0u) {
        return 0u;
    }
    if ((agent2_float_valid(target->x) == 0u) ||
        (agent2_float_valid(target->y) == 0u) ||
        (agent2_float_valid(target->radius) == 0u)) {
        return 0u;
    }
    if ((agent2_absf(target->x) < 0.001f) &&
        (agent2_absf(target->y) < 0.001f) &&
        (agent2_absf(target->radius) < 0.001f)) {
        return 0u;
    }
    if (target->radius < AGENT2_MIN_RADIUS_PX) {
        return 0u;
    }
    if ((target->x < 0.0f) || (target->x >= AGENT2_IMAGE_WIDTH_PX) ||
        (target->y < 0.0f) || (target->y >= AGENT2_IMAGE_HEIGHT_PX)) {
        return 0u;
    }
    return 1u;
}

static float agent2_bearing_from_image(uint8_t camera_id, float x)
{
    const float camera_center_deg[AGENT2_BEACON_FUSION_CAMERA_COUNT] = {
        -120.0f, 0.0f, 120.0f
    };
    float local_deg;

    if (camera_id >= AGENT2_BEACON_FUSION_CAMERA_COUNT) {
        camera_id = 1u;
    }

    local_deg = ((x - AGENT2_IMAGE_CENTER_X_PX) / AGENT2_IMAGE_CENTER_X_PX) * 60.0f;
    return agent2_wrap_deg(camera_center_deg[camera_id] + local_deg);
}

static float agent2_range_from_radius(float radius)
{
    const float safe_radius = agent2_clampf(radius, AGENT2_MIN_RADIUS_PX, 1000.0f);

    return AGENT2_RADIUS_DISTANCE_SCALE / safe_radius;
}

static int agent2_observation_before(const agent2_observation_t *lhs,
                                     const agent2_observation_t *rhs)
{
    if (lhs->y > rhs->y) {
        return 1;
    }
    if (lhs->y < rhs->y) {
        return 0;
    }
    if (lhs->radius > rhs->radius) {
        return 1;
    }
    if (lhs->radius < rhs->radius) {
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

static void agent2_sort_bucket(agent2_camera_bucket_t *bucket)
{
    uint8_t i;

    for (i = 1u; i < bucket->count; ++i) {
        uint8_t j = i;
        const agent2_observation_t current = bucket->item[i];

        while ((j > 0u) && (agent2_observation_before(&current, &bucket->item[j - 1u]) != 0)) {
            bucket->item[j] = bucket->item[j - 1u];
            --j;
        }
        bucket->item[j] = current;
    }
}

static void agent2_observation_from_target(uint8_t camera_id,
                                           uint8_t slot_id,
                                           const agent2_beacon_fusion_camera_target_t *target,
                                           agent2_observation_t *observation)
{
    memset(observation, 0, sizeof(*observation));
    observation->camera_id = camera_id;
    observation->slot_id = slot_id;
    observation->x = target->x;
    observation->y = target->y;
    observation->radius = target->radius;
    observation->bearing_deg = agent2_bearing_from_image(camera_id, target->x);
    observation->range_proxy = agent2_range_from_radius(target->radius);
    observation->control_x = AGENT2_IMAGE_CENTER_X_PX - target->x;
    observation->control_y = target->y - AGENT2_IMAGE_CENTER_Y_PX;
}

static agent2_beacon_fusion_status_t agent2_collect_observations(
    const agent2_beacon_fusion_camera_frame_t camera[AGENT2_BEACON_FUSION_CAMERA_COUNT],
    uint8_t expected_count,
    agent2_camera_bucket_t bucket[AGENT2_BEACON_FUSION_CAMERA_COUNT],
    agent2_beacon_fusion_result_t *result)
{
    uint8_t camera_id;

    memset(bucket, 0, sizeof(agent2_camera_bucket_t) * AGENT2_BEACON_FUSION_CAMERA_COUNT);
    for (camera_id = 0u; camera_id < AGENT2_BEACON_FUSION_CAMERA_COUNT; ++camera_id) {
        uint8_t slot_id;

        for (slot_id = 0u; slot_id < AGENT2_BEACON_FUSION_CAMERA_TARGETS; ++slot_id) {
            const agent2_beacon_fusion_camera_target_t *target = &camera[camera_id].target[slot_id];

            if (agent2_target_valid(target) == 0u) {
                continue;
            }
            if (bucket[camera_id].count >= AGENT2_BEACON_FUSION_CAMERA_TARGETS) {
                continue;
            }

            agent2_observation_from_target(camera_id,
                                           slot_id,
                                           target,
                                           &bucket[camera_id].item[bucket[camera_id].count]);
            bucket[camera_id].count++;
        }

        result->camera_count[camera_id] = bucket[camera_id].count;
        result->observation_count = (uint8_t)(result->observation_count + bucket[camera_id].count);
        if (bucket[camera_id].count > expected_count) {
            return AGENT2_BEACON_FUSION_DISCARD_OVER_COUNT;
        }
        agent2_sort_bucket(&bucket[camera_id]);
    }

    return AGENT2_BEACON_FUSION_OK;
}

static void agent2_build_observed_beacon(
    const agent2_observation_t member[AGENT2_BEACON_FUSION_CAMERA_COUNT],
    uint8_t member_count,
    agent2_beacon_fusion_beacon_t *beacon)
{
    uint8_t i;
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    float sum_range_weighted = 0.0f;
    float sum_control_x = 0.0f;
    float sum_control_y = 0.0f;
    float sum_weight = 0.0f;
    float radius_max = 0.0f;

    memset(beacon, 0, sizeof(*beacon));

    for (i = 0u; i < member_count; ++i) {
        const float bearing_rad = member[i].bearing_deg * (AGENT2_PI / 180.0f);
        const float weight = agent2_clampf(member[i].radius / 4.0f, 0.25f, 2.0f);

        sum_sin += sinf(bearing_rad) * weight;
        sum_cos += cosf(bearing_rad) * weight;
        sum_range_weighted += member[i].range_proxy * weight;
        sum_control_x += member[i].control_x * weight;
        sum_control_y += member[i].control_y * weight;
        sum_weight += weight;
        beacon->source_camera_mask = (uint8_t)(beacon->source_camera_mask | (uint8_t)(1u << member[i].camera_id));
        if (member[i].radius > radius_max) {
            radius_max = member[i].radius;
        }
    }

    if (sum_weight <= 0.0f) {
        beacon->valid = 1u;
        beacon->synthetic = 1u;
        beacon->range_proxy = AGENT2_MISSING_RANGE_PROXY;
        beacon->confidence = 0.0f;
        return;
    }

    beacon->valid = 1u;
    beacon->synthetic = 0u;
    beacon->observation_count = member_count;
    beacon->bearing_deg = agent2_wrap_deg(agent2_rad_to_deg(atan2f(sum_sin, sum_cos)));
    beacon->range_proxy = sum_range_weighted / sum_weight;
    beacon->control_x = sum_control_x / sum_weight;
    beacon->control_y = sum_control_y / sum_weight;
    beacon->x_body = sinf(beacon->bearing_deg * (AGENT2_PI / 180.0f)) * beacon->range_proxy;
    beacon->y_body = cosf(beacon->bearing_deg * (AGENT2_PI / 180.0f)) * beacon->range_proxy;
    beacon->confidence = 0.2f + ((float)member_count / (float)AGENT2_BEACON_FUSION_CAMERA_COUNT) * 0.55f;
    beacon->confidence += agent2_clampf(radius_max / 8.0f, 0.0f, 0.25f);
    beacon->confidence = agent2_clampf(beacon->confidence, 0.0f, 1.0f);
}

static void agent2_fill_missing_beacon(const agent2_beacon_fusion_result_t *previous,
                                       uint8_t index,
                                       agent2_beacon_fusion_beacon_t *beacon)
{
    memset(beacon, 0, sizeof(*beacon));
    if ((index < previous->beacon_count) && (previous->beacon[index].valid != 0u)) {
        *beacon = previous->beacon[index];
        beacon->synthetic = 1u;
        beacon->observation_count = 0u;
        beacon->source_camera_mask = 0u;
        beacon->confidence *= 0.90f;
        if (beacon->stable_ticks < 255u) {
            beacon->stable_ticks++;
        }
        return;
    }

    beacon->valid = 1u;
    beacon->synthetic = 1u;
    beacon->range_proxy = AGENT2_MISSING_RANGE_PROXY;
    beacon->confidence = 0.0f;
}

static void agent2_apply_filter(agent2_beacon_fusion_result_t *current,
                                const agent2_beacon_fusion_result_t *previous)
{
    uint8_t i;

    for (i = 0u; i < current->beacon_count; ++i) {
        agent2_beacon_fusion_beacon_t *beacon = &current->beacon[i];
        const agent2_beacon_fusion_beacon_t *old_beacon = &previous->beacon[i];

        if ((beacon->valid == 0u) || (beacon->synthetic != 0u) ||
            (i >= previous->beacon_count) || (old_beacon->valid == 0u)) {
            if (beacon->stable_ticks == 0u) {
                beacon->stable_ticks = 1u;
            }
            continue;
        }

        beacon->bearing_deg = agent2_wrap_deg((old_beacon->bearing_deg * (1.0f - AGENT2_FILTER_ALPHA)) +
                                              (beacon->bearing_deg * AGENT2_FILTER_ALPHA));
        beacon->range_proxy = (old_beacon->range_proxy * (1.0f - AGENT2_FILTER_ALPHA)) +
                              (beacon->range_proxy * AGENT2_FILTER_ALPHA);
        beacon->control_x = (old_beacon->control_x * (1.0f - AGENT2_FILTER_ALPHA)) +
                            (beacon->control_x * AGENT2_FILTER_ALPHA);
        beacon->control_y = (old_beacon->control_y * (1.0f - AGENT2_FILTER_ALPHA)) +
                            (beacon->control_y * AGENT2_FILTER_ALPHA);
        beacon->x_body = sinf(beacon->bearing_deg * (AGENT2_PI / 180.0f)) * beacon->range_proxy;
        beacon->y_body = cosf(beacon->bearing_deg * (AGENT2_PI / 180.0f)) * beacon->range_proxy;
        beacon->stable_ticks = old_beacon->stable_ticks;
        if (beacon->stable_ticks < 255u) {
            beacon->stable_ticks++;
        }
    }
}

static uint8_t agent2_choose_best(const agent2_beacon_fusion_result_t *result)
{
    uint8_t i;
    uint8_t best = AGENT2_BEACON_FUSION_MAX_BEACONS;
    float best_score = -1000000.0f;

    for (i = 0u; i < result->beacon_count; ++i) {
        const agent2_beacon_fusion_beacon_t *beacon = &result->beacon[i];
        float score;

        if (beacon->valid == 0u) {
            continue;
        }
        score = beacon->confidence - (beacon->range_proxy * 0.0002f);
        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }

    return best;
}

void agent2_beacon_fusion_init(void)
{
    memset(&g_agent2_beacon_fusion_result, 0, sizeof(g_agent2_beacon_fusion_result));
    g_agent2_beacon_fusion_result.best_index = AGENT2_BEACON_FUSION_MAX_BEACONS;
    g_agent2_beacon_fusion_result.status = AGENT2_BEACON_FUSION_BAD_ARGUMENT;
    s_expected_count = 0u;
}

void agent2_beacon_fusion_set_expected_count(uint8_t expected_count)
{
    if (expected_count > AGENT2_BEACON_FUSION_MAX_BEACONS) {
        expected_count = AGENT2_BEACON_FUSION_MAX_BEACONS;
    }
    s_expected_count = expected_count;
}

agent2_beacon_fusion_status_t agent2_beacon_fusion_update_100HZ(
    const agent2_beacon_fusion_camera_frame_t camera[AGENT2_BEACON_FUSION_CAMERA_COUNT])
{
    agent2_camera_bucket_t bucket[AGENT2_BEACON_FUSION_CAMERA_COUNT];
    agent2_beacon_fusion_result_t previous = g_agent2_beacon_fusion_result;
    agent2_beacon_fusion_result_t current;
    uint8_t expected_count = s_expected_count;
    uint8_t beacon_id;

    memset(&current, 0, sizeof(current));
    current.best_index = AGENT2_BEACON_FUSION_MAX_BEACONS;
    current.update_count = previous.update_count + 1u;
    current.expected_count = expected_count;

    if ((camera == NULL) || (expected_count > AGENT2_BEACON_FUSION_MAX_BEACONS)) {
        current.status = AGENT2_BEACON_FUSION_BAD_ARGUMENT;
        g_agent2_beacon_fusion_result = current;
        return current.status;
    }

    current.status = agent2_collect_observations(camera, expected_count, bucket, &current);
    if (current.status != AGENT2_BEACON_FUSION_OK) {
        g_agent2_beacon_fusion_result = current;
        return current.status;
    }

    current.beacon_count = expected_count;
    for (beacon_id = 0u; beacon_id < expected_count; ++beacon_id) {
        agent2_observation_t member[AGENT2_BEACON_FUSION_CAMERA_COUNT];
        uint8_t member_count = 0u;
        uint8_t camera_id;

        for (camera_id = 0u; camera_id < AGENT2_BEACON_FUSION_CAMERA_COUNT; ++camera_id) {
            if (beacon_id < bucket[camera_id].count) {
                member[member_count] = bucket[camera_id].item[beacon_id];
                member_count++;
            }
        }

        if (member_count == 0u) {
            agent2_fill_missing_beacon(&previous, beacon_id, &current.beacon[beacon_id]);
        } else {
            agent2_build_observed_beacon(member, member_count, &current.beacon[beacon_id]);
        }
    }

    agent2_apply_filter(&current, &previous);
    if (current.beacon_count > 0u) {
        current.best_index = agent2_choose_best(&current);
    }

    g_agent2_beacon_fusion_result = current;
    return current.status;
}

const agent2_beacon_fusion_result_t *agent2_beacon_fusion_get_result(void)
{
    return &g_agent2_beacon_fusion_result;
}
