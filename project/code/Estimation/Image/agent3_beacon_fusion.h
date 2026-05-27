#ifndef AGENT3_BEACON_FUSION_H
#define AGENT3_BEACON_FUSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AGENT3_CAMERA_COUNT       (3u)
#define AGENT3_TARGETS_PER_CAMERA (3u)
#define AGENT3_MAX_BEACONS        (3u)

typedef enum
{
    AGENT3_FUSION_OK = 0,
    AGENT3_FUSION_DISCARD_OVER_COUNT = 1,
    AGENT3_FUSION_BAD_ARGUMENT = 2
} agent3_fusion_status_t;

typedef struct
{
    uint8_t valid;
    float x;
    float y;
    float size;
} agent3_camera_target_t;

typedef struct
{
    agent3_camera_target_t target[AGENT3_TARGETS_PER_CAMERA];
} agent3_camera_frame_t;

typedef struct
{
    uint8_t valid;
    uint8_t synthetic;
    uint8_t source_camera_mask;
    uint8_t observation_count;
    float mean_x;
    float mean_y;
    float mean_size;
    float quality;
} agent3_fused_beacon_t;

typedef struct
{
    agent3_fusion_status_t status;
    uint8_t expected_count;
    uint8_t fused_count;
    uint8_t camera_count[AGENT3_CAMERA_COUNT];
    agent3_fused_beacon_t beacon[AGENT3_MAX_BEACONS];
} agent3_fusion_result_t;

void agent3_fusion_init_result(agent3_fusion_result_t *result);

agent3_fusion_status_t agent3_fusion_update(
    const agent3_camera_frame_t camera[AGENT3_CAMERA_COUNT],
    uint8_t expected_count,
    agent3_fusion_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
