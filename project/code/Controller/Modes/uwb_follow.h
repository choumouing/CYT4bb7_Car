#include "zf_common_headfile.h"
#ifndef UWB_FOLLOW_H
#define UWB_FOLLOW_H



#define UWB_FOLLOW_PERIOD_MS              (40U)
#define UWB_FOLLOW_TIMEOUT_MS             (160U)

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
    uint8_t tag_online;
    uint8_t output_valid;
} uwb_follow_state_t;

extern uwb_follow_state_t g_uwb_follow_state;

void uwb_follow_init(void);
void uwb_follow_reset(void);
void uwb_follow_update_25HZ(uint32 now_ms);

#endif
