#ifndef CARPLANFIX_H
#define CARPLANFIX_H

#include "zf_common_headfile.h"

#define CARPLANFIX_INVALID_BEACON_INDEX        (0xFFFFU)
#define CARPLANFIX_MIN_PLAN_SPEED_MPS          (0.05f)
#define CARPLANFIX_MAX_ANGLE_ERROR_DEG         (45.0f)
#define CARPLANFIX_NEAREST_REJECT_DISTANCE_M    (0.3f)
#define CARPLANFIX_APPLY_MIN_DISTANCE_M         (1.5f)

typedef struct
{
    uint8 matched;
    uint8 correction_valid;
    uint16 beacon_index;
    float angle_error_deg;
    float second_angle_error_deg;
    float target_speed_mps;
    float target_distance_m;
    float target_position[2];
    float corrected_forward_mps;
    float corrected_strafe_mps;
} carplanfix_state_t;

extern carplanfix_state_t g_carplanfix_state;

void carplanfix_reset(void);

/*
 * 根据无人机速度方向推断目标信标。附近0.3m内无信标、唯一候选且候选距离
 * 大于1.5m时才算猜测成功；
 * 猜测失败时原样返回无人机下发的前进和横移速度。
 */
uint8 carplanfix_resolve(float air_forward_mps,
                         float air_strafe_mps,
                         float air_yaw_target_deg,
                         float *forward_mps,
                         float *strafe_mps);

#endif /* CARPLANFIX_H */
