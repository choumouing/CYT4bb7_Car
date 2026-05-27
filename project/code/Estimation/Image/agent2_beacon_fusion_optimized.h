#ifndef AGENT2_BEACON_FUSION_OPTIMIZED_H
#define AGENT2_BEACON_FUSION_OPTIMIZED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AGENT2_BEACON_FUSION_CAMERA_COUNT       (3u)
#define AGENT2_BEACON_FUSION_CAMERA_TARGETS     (3u)
#define AGENT2_BEACON_FUSION_MAX_BEACONS        (5u)

typedef enum
{
    AGENT2_BEACON_FUSION_OK = 0,
    AGENT2_BEACON_FUSION_DISCARD_OVER_COUNT = 1,
    AGENT2_BEACON_FUSION_BAD_ARGUMENT = 2
} agent2_beacon_fusion_status_t;

typedef struct
{
    uint8_t valid;
    float x;
    float y;
    float radius;
} agent2_beacon_fusion_camera_target_t;

typedef struct
{
    agent2_beacon_fusion_camera_target_t target[AGENT2_BEACON_FUSION_CAMERA_TARGETS];
} agent2_beacon_fusion_camera_frame_t;

typedef struct
{
    uint8_t valid;
    uint8_t synthetic;
    uint8_t source_camera_mask;
    uint8_t observation_count;
    uint8_t stable_ticks;
    float bearing_deg;
    float range_proxy;
    float x_body;
    float y_body;
    float control_x;
    float control_y;
    float confidence;
} agent2_beacon_fusion_beacon_t;

typedef struct
{
    agent2_beacon_fusion_status_t status;
    uint8_t expected_count;
    uint8_t beacon_count;
    uint8_t best_index;
    uint8_t observation_count;
    uint8_t camera_count[AGENT2_BEACON_FUSION_CAMERA_COUNT];
    uint32_t update_count;
    agent2_beacon_fusion_beacon_t beacon[AGENT2_BEACON_FUSION_MAX_BEACONS];
} agent2_beacon_fusion_result_t;

extern agent2_beacon_fusion_result_t g_agent2_beacon_fusion_result;

void agent2_beacon_fusion_init(void);
void agent2_beacon_fusion_set_expected_count(uint8_t expected_count);
agent2_beacon_fusion_status_t agent2_beacon_fusion_update_100HZ(
    const agent2_beacon_fusion_camera_frame_t camera[AGENT2_BEACON_FUSION_CAMERA_COUNT]);
const agent2_beacon_fusion_result_t *agent2_beacon_fusion_get_result(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT2_BEACON_FUSION_OPTIMIZED_H */
