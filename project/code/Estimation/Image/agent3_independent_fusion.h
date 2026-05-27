#ifndef AGENT3_INDEPENDENT_FUSION_H
#define AGENT3_INDEPENDENT_FUSION_H

#include "zf_common_typedef.h"
#include "Protocols/CameraSpi/camera_spi_types.h"

#define AGENT3_FUSION_CAMERA_COUNT       (3U)
#define AGENT3_FUSION_CAMERA_TARGETS     (CAMERA_SPI_IMAGE_TARGET_COUNT)
#define AGENT3_FUSION_MAX_LIGHTS         (5U)

typedef struct
{
    uint8 valid;
    float x;
    float y;
    float radius;
} agent3_fusion_target_t;

typedef struct
{
    agent3_fusion_target_t target[AGENT3_FUSION_CAMERA_TARGETS];
} agent3_fusion_camera_frame_t;

typedef struct
{
    uint8 valid;
    uint8 evidence_count;
    uint8 source_camera_mask;
    float mean_x;
    float mean_y;
    float mean_radius;
} agent3_fusion_light_t;

typedef struct
{
    uint8 expected_count;
    uint8 light_count;
    uint32 update_count;
    agent3_fusion_light_t light[AGENT3_FUSION_MAX_LIGHTS];
} agent3_fusion_result_t;

void agent3_fusion_init(uint8 expected_count);
void agent3_fusion_set_expected_count(uint8 expected_count);
void agent3_fusion_update(const agent3_fusion_camera_frame_t camera[AGENT3_FUSION_CAMERA_COUNT]);
const agent3_fusion_result_t *agent3_fusion_get_result(void);

#endif /* AGENT3_INDEPENDENT_FUSION_H */

