#ifndef CARPLANFIX_H
#define CARPLANFIX_H

#include "zf_common_headfile.h"

#define CARPLANFIX_MIN_PLAN_SPEED_MPS       (0.05f)
#define CARPLANFIX_TARGET_RADIUS_M           (0.50f)
#define CARPLANFIX_TARGET_EXIT_CONFIRM_CYCLES (3U)

struct light_sequence_result;

typedef enum
{
    CARPLANFIX_STATUS_WAIT_SEQUENCE = 0U,
    CARPLANFIX_STATUS_TRACKING,
    CARPLANFIX_STATUS_COMPLETE,
    CARPLANFIX_STATUS_DISABLED
} carplanfix_status_e;

typedef enum
{
    CARPLANFIX_DISABLE_NONE = 0U,
    CARPLANFIX_DISABLE_SEQUENCE_FAILED,
    CARPLANFIX_DISABLE_ROUTE_NOT_FOUND,
    CARPLANFIX_DISABLE_UNEXPECTED_BEACON,
    CARPLANFIX_DISABLE_INVALID_ROUTE,
    CARPLANFIX_DISABLE_LEFT_TARGET_WITHOUT_EVENT
} carplanfix_disable_reason_e;

typedef struct
{
    uint8 status;
    uint8 disable_reason;
    uint8 correction_valid;
    uint8 near_beacon;
    uint8 target_zone_entered;
    uint8 target_event_pending;
    uint8 target_exit_confirm_count;
    uint8 sequence_id;
    uint8 identified_last_beacon_id;
    uint8 target_beacon_id;
    uint8 route_index;
    uint8 route_count;
    uint32 last_event_count;
    float target_speed_mps;
    float target_distance_m;
    float target_position[2];
    float corrected_forward_mps;
    float corrected_strafe_mps;
} carplanfix_state_t;

extern carplanfix_state_t g_carplanfix_state;

void carplanfix_reset(void);

/* 根据已识别灯序的预设路径修正速度方向；不修正时原样返回Air速度。 */
uint8 carplanfix_resolve(const struct light_sequence_result *light_sequence_result,
                         uint8 air_plan_valid,
                         float air_forward_mps,
                         float air_strafe_mps,
                         float *forward_mps,
                         float *strafe_mps);

#endif /* CARPLANFIX_H */
