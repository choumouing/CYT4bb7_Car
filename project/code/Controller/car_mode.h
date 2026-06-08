#ifndef CAR_MODE_H
#define CAR_MODE_H

#include "zf_common_headfile.h"

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

typedef struct
{
    float raw_forward_mps;
    float raw_strafe_mps;
    float velocity_forward_target_mps;
    float velocity_strafe_target_mps;
    float velocity_forward_feedback_mps;
    float velocity_strafe_feedback_mps;
    float forward_feedforward;
    float strafe_feedforward;
    float forward_pid_output;
    float strafe_pid_output;
    float forward_target;
    float strafe_target;
    float forward_pid_p_term;
    float forward_pid_i_term;
    float forward_pid_d_term;
    float strafe_pid_p_term;
    float strafe_pid_i_term;
    float strafe_pid_d_term;
    uint8 output_valid;
} car_mode1_state_t;

typedef struct
{
    uint8 target_valid;
    uint8 car_position_valid;
    uint8 car_position_in_center_window;
    uint8 distance_zone;
    float target_delta_x;
    float target_delta_y;
    float car_position_x;
    float car_position_y;
    float target_distance_px;
    float target_speed_mps;
    float forward_pid_output;
    float strafe_pid_output;
    float forward_ratio;
    float strafe_ratio;
    float target_forward_mps;
    float target_strafe_mps;
    float feedback_forward_mps;
    float feedback_strafe_mps;
    float limited_forward_mps;
    float limited_strafe_mps;
    float forward_command;
    float strafe_command;
    uint8 output_valid;
} car_mode2_state_t;

extern car_mode1_state_t g_car_mode1_state;
extern car_mode2_state_t g_car_mode2_state;

void car_mode_init(void);
void car_mode_reset(void);
car_mode_e car_mode_get(void);
void car_mode_update_25HZ(uint32 now_ms);
void car_mode_update_100HZ(uint32 now_ms);

void car_mode0_init(void);
void car_mode0_reset(void);
void car_mode0_update_25HZ(uint32 now_ms);

void car_mode1_init(void);
void car_mode1_reset(void);
void car_mode1_update_100HZ(uint32 now_ms);

void car_mode2_init(void);
void car_mode2_reset(void);
void car_mode2_update_25HZ(uint32 now_ms);
void car_mode2_update_100HZ(uint32 now_ms);

#endif /* CAR_MODE_H */
