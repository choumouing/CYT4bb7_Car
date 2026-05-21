#ifndef BEACON_EDGE_STATE_H
#define BEACON_EDGE_STATE_H

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BEACON_EDGE_LOCATION_UNKNOWN = 0,
    BEACON_EDGE_LOCATION_FRONT,
    BEACON_EDGE_LOCATION_RIGHT,
    BEACON_EDGE_LOCATION_LEFT,
    BEACON_EDGE_LOCATION_REAR
} beacon_edge_location_t;

typedef enum
{
    BEACON_EDGE_CONFIDENCE_NONE = 0,
    BEACON_EDGE_CONFIDENCE_LOW,
    BEACON_EDGE_CONFIDENCE_HIGH
} beacon_edge_confidence_t;

typedef struct
{
    uint8_t bump_detected;
    uint8_t on_beacon;
    uint8_t enter_event;
    uint8_t exit_event;

    beacon_edge_confidence_t confidence;
    beacon_edge_location_t location;

    uint16_t hold_ticks;
    uint32_t event_count;
    uint32_t enter_count;
    uint32_t exit_count;
    uint32_t event_tick_ms;
    uint32_t enter_tick_ms;
    uint32_t exit_tick_ms;

    float score;
    float speed_mps;
    float vel[2];
    float projected_gyro_dps;
    float gyro_xy_dps;
    float gyro_z_abs_dps;
    float tilt_deg;
    float accel_norm_error_g;
    float accel_horizontal_g;
    float accel_along_g;
    float wheel_highpass_count;
    float group_distance_m;
} beacon_edge_state_t;

extern beacon_edge_state_t g_beacon_edge_state;

void beacon_edge_state_reset(void);
void beacon_edge_state_update_100HZ(float left_front_count,
                                    float right_front_count,
                                    float left_rear_count,
                                    float right_rear_count);
void beacon_edge_state_update_1000HZ(float accel_x_g,
                                     float accel_y_g,
                                     float accel_z_g,
                                     float gyro_x_dps,
                                     float gyro_y_dps,
                                     float gyro_z_dps,
                                     float roll_deg,
                                     float pitch_deg);

void beacon_edge_state_update_from_project_100HZ(void);
void beacon_edge_state_update_from_project_1000HZ(void);
const beacon_edge_state_t *beacon_edge_state_get(void);

#ifdef __cplusplus
}
#endif

#endif
