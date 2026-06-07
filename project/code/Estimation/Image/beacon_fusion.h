#ifndef BEACON_FUSION_H
#define BEACON_FUSION_H

#include "zf_common_headfile.h"
#include "Protocols/CameraSpi/camera_spi_types.h"

#define BEACON_FUSION_CAMERA_COUNT   (3U)
#define BEACON_FUSION_CAMERA_TARGETS (CAMERA_SPI_IMAGE_TARGET_COUNT)

#define BEACON_FUSION_CAMERA_FRONT   (0U)
#define BEACON_FUSION_CAMERA_CENTER  (1U)
#define BEACON_FUSION_CAMERA_REAR    (2U)

typedef struct
{
    uint8 valid;
    float x;
    float y;
    float area;
} beacon_fusion_target_t;

typedef struct
{
    beacon_fusion_target_t target[BEACON_FUSION_CAMERA_TARGETS];
} beacon_fusion_camera_frame_t;

typedef struct
{
    uint8 active;
    uint8 valid;
    uint8 camera_id;
    uint16 frame_id;
    float image_x;
    float image_y;
    uint8 center_delta_valid;
    float center_delta_x;
    float center_delta_y;
    float area;
    uint8 missing_frame_count;
} beacon_fusion_state_t;

extern beacon_fusion_state_t g_beacon_fusion;

void beacon_fusion_init(void);
void beacon_fusion_set_auto_max_count(uint8 max_count);
void beacon_fusion_set_center_car_lamp(uint8 valid, float cx, float cy);
uint8 beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT]);

#endif /* BEACON_FUSION_H */
