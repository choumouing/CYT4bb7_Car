#ifndef AGENT2_OPTIMIZED_BEACON_FUSION_H
#define AGENT2_OPTIMIZED_BEACON_FUSION_H

#include "zf_common_typedef.h"
#include "Protocols/CameraSpi/camera_spi_types.h"

#define AGENT2_BEACON_CAMERA_COUNT       (3U)
#define AGENT2_BEACON_CAMERA_TARGETS     (CAMERA_SPI_IMAGE_TARGET_COUNT)
#define AGENT2_BEACON_MAX_BEACONS        (5U)

typedef struct
{
    uint8 valid;
    float x;
    float y;
    float radius;
} agent2_beacon_camera_target_t;

typedef struct
{
    agent2_beacon_camera_target_t target[AGENT2_BEACON_CAMERA_TARGETS];
} agent2_beacon_camera_frame_t;

typedef struct
{
    uint8 valid;
    uint8 source_camera_mask;
    uint8 observation_count;
    float bearing_deg;
    float range_proxy;
    float image_x;
    float image_y;
    float radius;
    float confidence;
} agent2_beacon_t;

typedef struct
{
    uint8 configured_count;
    uint8 beacon_count;
    uint32 update_count;
    agent2_beacon_t beacon[AGENT2_BEACON_MAX_BEACONS];
} agent2_beacon_result_t;

extern agent2_beacon_result_t g_agent2_beacon_result;

void agent2_beacon_fusion_init(void);
void agent2_beacon_fusion_set_expected_count(uint8 expected_count);
void agent2_beacon_fusion_update_100HZ(const agent2_beacon_camera_frame_t camera[AGENT2_BEACON_CAMERA_COUNT]);
const agent2_beacon_result_t *agent2_beacon_fusion_get_result(void);

#endif /* AGENT2_OPTIMIZED_BEACON_FUSION_H */

