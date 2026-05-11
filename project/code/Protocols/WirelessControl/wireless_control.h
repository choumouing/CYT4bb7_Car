#include "zf_common_headfile.h"
#ifndef _WIRELESS_CONTROL_H_
#define _WIRELESS_CONTROL_H_



#define MAX_CONTROL_SPEED                   (600)
#define MAX_ANGULAR_SPEED                   (2.0f)

#define WIRELESS_CONTROL_PERIOD_MS          (40U)

#define WIRELESS_CONTROL_CHANNEL_COUNT      (6U)
#define WIRELESS_CONTROL_WHEEL_COUNT        (4U)

#define WIRELESS_CH1_LEFT_LIMIT             (322U)
#define WIRELESS_CH1_CENTER                 (1028U)
#define WIRELESS_CH1_RIGHT_LIMIT            (1807U)

#define WIRELESS_CH2_DOWN_LIMIT             (204U)
#define WIRELESS_CH2_CENTER                 (1025U)
#define WIRELESS_CH2_UP_LIMIT               (1805U)

#define WIRELESS_CH3_DOWN_LIMIT             (306U)
#define WIRELESS_CH3_UP_LIMIT               (1777U)

#define WIRELESS_CH4_LEFT_LIMIT             (306U)
#define WIRELESS_CH4_CENTER                 (1028U)
#define WIRELESS_CH4_RIGHT_LIMIT            (1754U)

#define WIRELESS_CH5_UP_VALUE               (204U)
#define WIRELESS_CH5_DOWN_VALUE             (1807U)

#define WIRELESS_CH6_UP_VALUE               (240U)
#define WIRELESS_CH6_DOWN_VALUE             (1807U)

#define WIRELESS_CENTER_DEADZONE            (50U)
#define WIRELESS_SWITCH_TOLERANCE           (50U)
#define WIRELESS_FRAME_TIMEOUT_MS           (100U)
#define WIRELESS_FRAME_TIMEOUT_CYCLES       ((WIRELESS_FRAME_TIMEOUT_MS + WIRELESS_CONTROL_PERIOD_MS - 1U) / WIRELESS_CONTROL_PERIOD_MS)

typedef struct
{
    uint16_t raw_channel[WIRELESS_CONTROL_CHANNEL_COUNT];
    uint16_t clamped_channel[WIRELESS_CONTROL_CHANNEL_COUNT];
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
