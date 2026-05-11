#ifndef _WIRELESS_CONTROL_H_
#define _WIRELESS_CONTROL_H_

#include "zf_common_headfile.h"



#define MAX_CONTROL_SPEED                   (600)
#define MAX_ANGULAR_SPEED                   (2.0f)

#define WIRELESS_CONTROL_PERIOD_MS          (40U)

#define WIRELESS_CONTROL_CHANNEL_COUNT      (6U)
#define WIRELESS_CONTROL_WHEEL_COUNT        (4U)

typedef struct
{
    uint16_t raw_channel[WIRELESS_CONTROL_CHANNEL_COUNT];
    int16_t std_channel[WIRELESS_CONTROL_CHANNEL_COUNT];
    int16_t forward_speed;
    int16_t strafe_speed;
    float rotate_speed;
    int16_t wheel_target[WIRELESS_CONTROL_WHEEL_COUNT];
    int16_t wheel_pwm[WIRELESS_CONTROL_WHEEL_COUNT];
    uint8_t receiver_online;
    uint8_t control_enabled;
    uint8_t emergency_stop_active;
    uint8_t remote_mode_requested;
    uint8_t uwb_follow_requested;
    uint8_t mode_request_valid;
} wireless_control_state_t;

extern wireless_control_state_t g_wireless_control_state;

void wireless_control_init(void);
void wireless_control_update_25HZ(void);
const wireless_control_state_t *wireless_control_get_state(void);

#endif
