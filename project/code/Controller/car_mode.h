#ifndef CAR_MODE_H
#define CAR_MODE_H

typedef enum
{
    CAR_MODE_0 = 0,
    CAR_MODE_1,
    CAR_MODE_2,
    CAR_MODE_3,
    CAR_MODE_4,
    CAR_MODE_5,
    CAR_MODE_6,
    CAR_MODE_7,
    CAR_MODE_8
} car_mode_e;

#include "zf_common_headfile.h"

#define UWB_FOLLOW_PERIOD_MS              (40U)
#define UWB_FOLLOW_TIMEOUT_MS             (160U)

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
    float raw_x_cm;
    float raw_y_cm;
    float filt_x_cm;
    float filt_y_cm;
    float error_x_cm;
    float error_y_cm;
    float x_pid_p_term;
    float x_pid_i_term;
    float x_pid_d_term;
    float y_pid_p_term;
    float y_pid_i_term;
    float y_pid_d_term;
    float forward_target;
    float strafe_target;
    uint8 tag_online;
    uint8 output_valid;
} car_mode1_state_t;

typedef struct
{
    float strafe_m;
    float forward_m;
    uint8 reached;
    uint8 valid;
} car_mode2_point_t;

typedef struct
{
    car_mode2_point_t targets[TARGET_FOLLOW_MAX_TARGETS];
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
} car_mode2_state_t;

extern car_mode1_state_t g_car_mode1_state;
extern car_mode2_state_t g_car_mode2_state;

void car_mode_init(void);
void car_mode_reset(void);
car_mode_e car_mode_get(void);
void car_mode_update_25HZ(uint32 now_ms);

void car_mode0_init(void);
void car_mode0_reset(void);
void car_mode0_update_25HZ(uint32 now_ms);

void car_mode1_init(void);
void car_mode1_reset(void);
void car_mode1_update_25HZ(uint32 now_ms);

void car_mode2_init(void);
void car_mode2_reset(void);
void car_mode2_update_25HZ(uint32 now_ms);
void car_mode2_restart_targets(void);
void car_mode2_clear_targets(void);
uint8 car_mode2_add_target(float strafe_m, float forward_m);
uint8 car_mode2_set_target(uint8 index, float strafe_m, float forward_m);

void car_mode2_load_default_targets(void);

#endif /* CAR_MODE_H */
