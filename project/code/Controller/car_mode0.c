#include "car_mode.h"


void car_mode0_init(void)
{
    car_mode0_reset();
}

void car_mode0_reset(void)
{
}

void car_mode0_update_25HZ(uint32 now_ms)
{
    const wireless_control_state_t *remote;

    (void)now_ms;

    remote = wireless_control_get_state();
    if(0U == car_start_sbus_is_running())
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        car_rotate_target = 0.0f;
        return;
    }

    car_forward_target = (float)remote->forward_speed;
    car_strafe_target = (float)remote->strafe_speed;
    car_rotate_target = remote->rotate_speed;
}
