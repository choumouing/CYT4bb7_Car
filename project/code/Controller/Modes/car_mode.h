#include "zf_common_headfile.h"
#ifndef CAR_MODE_H
#define CAR_MODE_H



#define CAR_MODE_REMOTE        (0U)
#define CAR_MODE_UWB_FOLLOW    (1U)

typedef struct
{
    uint8_t mode;
    uint8_t last_mode;
    uint8_t mode_changed;
    uint8_t control_enabled;
    uint8_t emergency_stop_active;
    float forward_target;
    float strafe_target;
    float rotate_target;
} car_mode_state_t;

extern car_mode_state_t g_car_mode_state;

void car_mode_init(void);
void car_mode_reset(void);
uint8_t car_mode_get(void);
void car_mode_update_25HZ(uint32 now_ms);

#endif
