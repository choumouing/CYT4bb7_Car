#include "zf_common_headfile.h"
#ifndef _BEACON_DETECTION_H_
#define _BEACON_DETECTION_H_



#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BEACON_BUMP_LOCATION_UNKNOWN = 0,
    BEACON_BUMP_LOCATION_FRONT,
    BEACON_BUMP_LOCATION_REAR,
    BEACON_BUMP_LOCATION_LEFT,
    BEACON_BUMP_LOCATION_RIGHT,
    BEACON_BUMP_LOCATION_LEFT_FRONT,
    BEACON_BUMP_LOCATION_RIGHT_FRONT,
    BEACON_BUMP_LOCATION_LEFT_REAR,
    BEACON_BUMP_LOCATION_RIGHT_REAR,
    BEACON_BUMP_LOCATION_DIAGONAL_LF_RR,
    BEACON_BUMP_LOCATION_DIAGONAL_RF_LR
} beacon_bump_location_t;

typedef enum
{
    BEACON_BUMP_CONFIDENCE_NONE = 0,
    BEACON_BUMP_CONFIDENCE_LOW,
    BEACON_BUMP_CONFIDENCE_HIGH
} beacon_bump_confidence_t;

#define BEACON_BUMP_WHEEL_LF_MASK (0x01U)
#define BEACON_BUMP_WHEEL_RF_MASK (0x02U)
#define BEACON_BUMP_WHEEL_LR_MASK (0x04U)
#define BEACON_BUMP_WHEEL_RR_MASK (0x08U)

typedef struct
{
    uint8_t bump_detected;
    uint8_t partial_bump;
    uint8_t wheel_mask;
    beacon_bump_location_t location;
    beacon_bump_confidence_t confidence;

    uint32_t event_count;
    uint16_t hold_ticks;

    float score;
    float speed_mps;
    float forward_velocity_mps;
    float strafe_velocity_mps;
    float gyro_xy_dps;
    float tilt_rate_dps;
    float tilt_deg;
    float accel_norm_error_g;
    float wheel_highpass_count;
} beacon_detection_state_t;

extern beacon_detection_state_t g_beacon_detection;

void beacon_detection_init(void);
void beacon_detection_reset(void);
void beacon_detection_update(void);
const beacon_detection_state_t *beacon_detection_get_state(void);

#ifdef __cplusplus
}
#endif

#endif
