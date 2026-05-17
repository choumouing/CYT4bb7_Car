#include "beacon_fusion.h"

#include <math.h>

#define BEACON_FUSION_PI                    (3.1415926f)
#define BEACON_FUSION_IMAGE_CENTER_X        (94.0f)
#define BEACON_FUSION_HALF_FOV_DEG          (70.0f)
#define BEACON_FUSION_MIN_RADIUS_PX         (1.0f)
#define BEACON_FUSION_SECONDARY_MIN_RADIUS  (1.4f)
#define BEACON_FUSION_MAX_RADIUS_PX         (12.0f)
#define BEACON_FUSION_GATING_BEARING_DEG    (65.0f)
#define BEACON_FUSION_GATING_RANGE_RATIO    (0.80f)
#define BEACON_FUSION_FILTER_ALPHA          (0.35f)
#define BEACON_FUSION_CONFIDENCE_SINGLE     (0.45f)
#define BEACON_FUSION_CONFIDENCE_MULTI      (0.25f)
#define BEACON_FUSION_RANGE_SCALE           (1.0f)

typedef struct
{
    uint8 valid;
    uint8 camera_id;
    uint8 target_id;
    uint8 source_camera_mask;
    float bearing_deg;
    float range_proxy;
    float radius;
    float weight;
} beacon_fusion_observation_t;

typedef struct
{
    uint8 valid;
    uint8 source_camera_mask;
    uint8 observation_count;
    float sum_sin;
    float sum_cos;
    float sum_range;
    float sum_weight;
    float radius_max;
} beacon_fusion_cluster_t;

beacon_fusion_result_t g_beacon_fusion_result;

static const float s_camera_yaw_deg[BEACON_FUSION_CAMERA_COUNT] =
{
    -120.0f,
    0.0f,
    120.0f
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

static float beacon_fusion_angle_delta_deg(float a_deg, float b_deg)
{
    return beacon_fusion_wrap_deg(a_deg - b_deg);
}

static float beacon_fusion_deg_to_rad(float angle_deg)
{
    return angle_deg * (BEACON_FUSION_PI / 180.0f);
}

static float beacon_fusion_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / BEACON_FUSION_PI);
}

static float beacon_fusion_target_bearing_deg(uint8 camera_id, float image_x)
{
    float local_deg;

    local_deg = (image_x / BEACON_FUSION_IMAGE_CENTER_X) * BEACON_FUSION_HALF_FOV_DEG;
    local_deg = beacon_fusion_clampf(local_deg,
                                     -BEACON_FUSION_HALF_FOV_DEG,
                                     BEACON_FUSION_HALF_FOV_DEG);
    return beacon_fusion_wrap_deg(s_camera_yaw_deg[camera_id] + local_deg);
}

static float beacon_fusion_target_range_proxy(float radius)
{
    float safe_radius;

    safe_radius = beacon_fusion_clampf(radius,
                                       BEACON_FUSION_MIN_RADIUS_PX,
                                       BEACON_FUSION_MAX_RADIUS_PX);
    return BEACON_FUSION_RANGE_SCALE / safe_radius;
}

static float beacon_fusion_target_weight(float radius, uint8 target_id)
{
    float weight;

    weight = beacon_fusion_clampf(radius / 4.0f, 0.25f, 2.0f);
    if(target_id > 0U)
    {
        weight *= 0.55f;
    }
    return weight;
}

static uint8 beacon_fusion_target_valid(const beacon_fusion_camera_target_t *target)
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
    if((target->valid == 0U) && (target->radius < BEACON_FUSION_SECONDARY_MIN_RADIUS))
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
    uint8 target_id;
    uint8 count = 0U;

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        for(target_id = 0U; target_id < BEACON_FUSION_CAMERA_TARGETS; target_id++)
        {
            const beacon_fusion_camera_target_t *target = &camera[camera_id].target[target_id];

            if(beacon_fusion_target_valid(target) == 0U)
            {
                continue;
            }
            if((target_id > 0U) && (target->radius < BEACON_FUSION_SECONDARY_MIN_RADIUS))
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
            observation[count].bearing_deg = beacon_fusion_target_bearing_deg(camera_id, target->x);
            observation[count].range_proxy = beacon_fusion_target_range_proxy(target->radius);
            observation[count].radius = target->radius;
            observation[count].weight = beacon_fusion_target_weight(target->radius, target_id);
            count++;
        }
    }

    return count;
}

static float beacon_fusion_cluster_bearing_deg(const beacon_fusion_cluster_t *cluster)
{
    if(cluster->sum_weight <= 0.0f)
    {
        return 0.0f;
    }
    return beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(cluster->sum_sin, cluster->sum_cos)));
}

static float beacon_fusion_cluster_range_proxy(const beacon_fusion_cluster_t *cluster)
{
    if(cluster->sum_weight <= 0.0f)
    {
        return 0.0f;
    }
    return cluster->sum_range / cluster->sum_weight;
}

static uint8 beacon_fusion_observation_matches_cluster(const beacon_fusion_observation_t *observation,
                                                       const beacon_fusion_cluster_t *cluster)
{
    float bearing_delta;
    float cluster_range;
    float range_delta;
    float range_gate;

    if(cluster->valid == 0U)
    {
        return 0U;
    }

    bearing_delta = beacon_fusion_absf(
        beacon_fusion_angle_delta_deg(observation->bearing_deg,
                                      beacon_fusion_cluster_bearing_deg(cluster)));
    if(bearing_delta > BEACON_FUSION_GATING_BEARING_DEG)
    {
        return 0U;
    }

    cluster_range = beacon_fusion_cluster_range_proxy(cluster);
    range_delta = beacon_fusion_absf(observation->range_proxy - cluster_range);
    range_gate = cluster_range * BEACON_FUSION_GATING_RANGE_RATIO;
    if(range_gate < 0.08f)
    {
        range_gate = 0.08f;
    }

    return (range_delta <= range_gate) ? 1U : 0U;
}

static void beacon_fusion_cluster_add(beacon_fusion_cluster_t *cluster,
                                      const beacon_fusion_observation_t *observation)
{
    float angle_rad;

    angle_rad = beacon_fusion_deg_to_rad(observation->bearing_deg);
    cluster->valid = 1U;
    cluster->source_camera_mask |= observation->source_camera_mask;
    cluster->observation_count++;
    cluster->sum_sin += sinf(angle_rad) * observation->weight;
    cluster->sum_cos += cosf(angle_rad) * observation->weight;
    cluster->sum_range += observation->range_proxy * observation->weight;
    cluster->sum_weight += observation->weight;
    if(observation->radius > cluster->radius_max)
    {
        cluster->radius_max = observation->radius;
    }
}

static uint8 beacon_fusion_build_clusters(const beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS],
                                          uint8 observation_count,
                                          beacon_fusion_cluster_t cluster[BEACON_FUSION_MAX_BEACONS])
{
    uint8 i;
    uint8 cluster_count = 0U;

    memset(cluster, 0, sizeof(beacon_fusion_cluster_t) * BEACON_FUSION_MAX_BEACONS);

    for(i = 0U; i < observation_count; i++)
    {
        uint8 j;
        uint8 best = BEACON_FUSION_MAX_BEACONS;
        float best_delta = 1000.0f;

        for(j = 0U; j < cluster_count; j++)
        {
            float delta;

            if(beacon_fusion_observation_matches_cluster(&observation[i], &cluster[j]) == 0U)
            {
                continue;
            }

            delta = beacon_fusion_absf(
                beacon_fusion_angle_delta_deg(observation[i].bearing_deg,
                                              beacon_fusion_cluster_bearing_deg(&cluster[j])));
            if(delta < best_delta)
            {
                best_delta = delta;
                best = j;
            }
        }

        if(best < BEACON_FUSION_MAX_BEACONS)
        {
            beacon_fusion_cluster_add(&cluster[best], &observation[i]);
        }
        else if(cluster_count < BEACON_FUSION_MAX_BEACONS)
        {
            beacon_fusion_cluster_add(&cluster[cluster_count], &observation[i]);
            cluster_count++;
        }
    }

    return cluster_count;
}

static uint8 beacon_fusion_count_cameras(uint8 camera_mask)
{
    uint8 count = 0U;

    if((camera_mask & 0x01U) != 0U)
    {
        count++;
    }
    if((camera_mask & 0x02U) != 0U)
    {
        count++;
    }
    if((camera_mask & 0x04U) != 0U)
    {
        count++;
    }
    return count;
}

static float beacon_fusion_cluster_confidence(const beacon_fusion_cluster_t *cluster)
{
    float confidence;
    uint8 camera_count;

    camera_count = beacon_fusion_count_cameras(cluster->source_camera_mask);
    confidence = BEACON_FUSION_CONFIDENCE_SINGLE;
    if(camera_count > 1U)
    {
        confidence += ((float)(camera_count - 1U) * BEACON_FUSION_CONFIDENCE_MULTI);
    }
    if(cluster->observation_count > camera_count)
    {
        confidence += 0.08f;
    }
    confidence += beacon_fusion_clampf(cluster->radius_max / 12.0f, 0.0f, 0.2f);

    return beacon_fusion_clampf(confidence, 0.0f, 1.0f);
}

static void beacon_fusion_cluster_to_beacon(const beacon_fusion_cluster_t *cluster,
                                            beacon_fusion_beacon_t *beacon)
{
    float bearing_rad;

    memset(beacon, 0, sizeof(*beacon));
    beacon->valid = 1U;
    beacon->source_camera_mask = cluster->source_camera_mask;
    beacon->observation_count = cluster->observation_count;
    beacon->bearing_deg = beacon_fusion_cluster_bearing_deg(cluster);
    beacon->range_proxy = beacon_fusion_cluster_range_proxy(cluster);
    bearing_rad = beacon_fusion_deg_to_rad(beacon->bearing_deg);
    beacon->x_body = sinf(bearing_rad) * beacon->range_proxy;
    beacon->y_body = cosf(bearing_rad) * beacon->range_proxy;
    beacon->confidence = beacon_fusion_cluster_confidence(cluster);
}

static uint8 beacon_fusion_find_previous_match(const beacon_fusion_beacon_t *candidate,
                                               const beacon_fusion_result_t *previous,
                                               const uint8 used[BEACON_FUSION_MAX_BEACONS])
{
    uint8 i;
    uint8 best = BEACON_FUSION_MAX_BEACONS;
    float best_delta = 1000.0f;

    for(i = 0U; i < previous->beacon_count; i++)
    {
        float delta;

        if((used[i] != 0U) || (previous->beacon[i].valid == 0U))
        {
            continue;
        }

        delta = beacon_fusion_absf(
            beacon_fusion_angle_delta_deg(candidate->bearing_deg,
                                          previous->beacon[i].bearing_deg));
        if((delta < BEACON_FUSION_GATING_BEARING_DEG) && (delta < best_delta))
        {
            best_delta = delta;
            best = i;
        }
    }

    return best;
}

static void beacon_fusion_apply_temporal_filter(beacon_fusion_result_t *current,
                                                const beacon_fusion_result_t *previous)
{
    uint8 i;
    uint8 used[BEACON_FUSION_MAX_BEACONS] = {0U};

    for(i = 0U; i < current->beacon_count; i++)
    {
        uint8 previous_index;

        previous_index = beacon_fusion_find_previous_match(&current->beacon[i], previous, used);
        if(previous_index >= BEACON_FUSION_MAX_BEACONS)
        {
            current->beacon[i].stable_ticks = 1U;
            continue;
        }

        used[previous_index] = 1U;
        current->beacon[i].bearing_deg =
            beacon_fusion_wrap_deg(previous->beacon[previous_index].bearing_deg +
                                   (beacon_fusion_angle_delta_deg(current->beacon[i].bearing_deg,
                                                                  previous->beacon[previous_index].bearing_deg) *
                                    BEACON_FUSION_FILTER_ALPHA));
        current->beacon[i].range_proxy =
            (previous->beacon[previous_index].range_proxy * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
            (current->beacon[i].range_proxy * BEACON_FUSION_FILTER_ALPHA);
        current->beacon[i].stable_ticks = previous->beacon[previous_index].stable_ticks;
        if(current->beacon[i].stable_ticks < 255U)
        {
            current->beacon[i].stable_ticks++;
        }
        current->beacon[i].confidence =
            (previous->beacon[previous_index].confidence * 0.4f) +
            (current->beacon[i].confidence * 0.6f);

        {
            float bearing_rad = beacon_fusion_deg_to_rad(current->beacon[i].bearing_deg);
            current->beacon[i].x_body = sinf(bearing_rad) * current->beacon[i].range_proxy;
            current->beacon[i].y_body = cosf(bearing_rad) * current->beacon[i].range_proxy;
        }
    }
}

static uint8 beacon_fusion_choose_best(const beacon_fusion_result_t *result)
{
    uint8 i;
    uint8 best = 0U;
    float best_score = -1.0f;

    for(i = 0U; i < result->beacon_count; i++)
    {
        float forward_bonus;
        float range_bonus;
        float score;

        if(result->beacon[i].valid == 0U)
        {
            continue;
        }

        forward_bonus = 1.0f - (beacon_fusion_absf(result->beacon[i].bearing_deg) / 180.0f);
        range_bonus = 1.0f / (1.0f + result->beacon[i].range_proxy);
        score = result->beacon[i].confidence + (forward_bonus * 0.25f) + (range_bonus * 0.35f);
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
}

void beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT])
{
    beacon_fusion_observation_t observation[BEACON_FUSION_MAX_OBSERVATIONS];
    beacon_fusion_cluster_t cluster[BEACON_FUSION_MAX_BEACONS];
    beacon_fusion_result_t previous;
    beacon_fusion_result_t current;
    uint8 observation_count;
    uint8 cluster_count;
    uint8 i;

    previous = g_beacon_fusion_result;
    memset(&current, 0, sizeof(current));
    current.best_index = BEACON_FUSION_MAX_BEACONS;
    current.update_count = previous.update_count + 1U;

    observation_count = beacon_fusion_collect_observations(camera, observation);
    cluster_count = beacon_fusion_build_clusters(observation, observation_count, cluster);
    current.observation_count = observation_count;

    for(i = 0U; (i < cluster_count) && (current.beacon_count < BEACON_FUSION_MAX_BEACONS); i++)
    {
        if(cluster[i].valid == 0U)
        {
            continue;
        }
        beacon_fusion_cluster_to_beacon(&cluster[i], &current.beacon[current.beacon_count]);
        current.beacon_count++;
    }

    beacon_fusion_apply_temporal_filter(&current, &previous);
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
