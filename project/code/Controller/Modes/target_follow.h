#include "zf_common_headfile.h"
#ifndef _TARGET_FOLLOW_H_
#define _TARGET_FOLLOW_H_



#define TARGET_FOLLOW_MAX_TARGETS              (8U)
#define TARGET_FOLLOW_INVALID_INDEX            (0xFFU)

#define TARGET_FOLLOW_MODE_IDLE                (0U)
#define TARGET_FOLLOW_MODE_FOLLOW_TAG          (1U)
#define TARGET_FOLLOW_MODE_GOTO_TARGET         (2U)
#define TARGET_FOLLOW_MODE_TARGET_REACHED      (3U)

#define TARGET_FOLLOW_TAG_GUARD_RADIUS_M       (1.00f)
#define TARGET_FOLLOW_TARGET_MATCH_RADIUS_M    (1.00f)
#define TARGET_FOLLOW_REACHED_RADIUS_M         (0.08f)
#define TARGET_FOLLOW_POSITION_DEADBAND_M      (0.02f)
#define TARGET_FOLLOW_OUTPUT_LIMIT             (400.0f)
#define TARGET_FOLLOW_POS_KP                   (120.0f)
#define TARGET_FOLLOW_POS_KI                   (0.0f)
#define TARGET_FOLLOW_POS_KD                   (30.0f)
#define TARGET_FOLLOW_POS_I_LIMIT              (0.0f)
#define TARGET_FOLLOW_UWB_TIMEOUT_MS           (160U)

typedef struct
{
    float strafe_m;
    float forward_m;
    uint8 reached;
    uint8 valid;
} target_follow_point_t;

typedef struct
{
    target_follow_point_t targets[TARGET_FOLLOW_MAX_TARGETS];
    float car_strafe_m;
    float car_forward_m;
    float tag_strafe_m;
    float tag_forward_m;
    float tag_relative_strafe_m;
    float tag_relative_forward_m;
    float car_tag_distance_m;
    float target_tag_distance_m;
    float target_car_distance_m;
    float target_error_strafe_m;
    float target_error_forward_m;
    float target_pid_strafe_output;
    float target_pid_forward_output;
    float forward_target;
    float strafe_target;
    uint8 target_count;
    uint8 active_index;
    uint8 candidate_index;
    uint8 mode;
    uint8 tag_online;
    uint8 car_in_tag_range;
    uint8 target_in_tag_range;
    uint8 output_valid;
} target_follow_state_t;

extern target_follow_state_t g_target_follow_state;

void target_follow_init(void);
void target_follow_reset(void);
void target_follow_restart_targets(void);
void target_follow_clear_targets(void);
uint8 target_follow_add_target(float strafe_m, float forward_m);
uint8 target_follow_set_target(uint8 index, float strafe_m, float forward_m);
void target_follow_update(uint32 now_ms);

#endif
