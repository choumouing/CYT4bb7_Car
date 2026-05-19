/* Mode2: reserved stop mode.
 * Old target navigation was removed. CH6 high selects this placeholder.
 */
#include "car_mode.h"
#include "car_loop.h"

car_mode2_state_t g_car_mode2_state = {0};

void car_mode2_init(void)
{
    car_mode2_reset();
}

void car_mode2_reset(void)
{
    g_car_mode2_state.forward_target = 0.0f;
    g_car_mode2_state.strafe_target = 0.0f;
    g_car_mode2_state.output_valid = 0U;
}

void car_mode2_update_25HZ(uint32 now_ms)
{
    (void)now_ms;

    g_car_mode2_state.forward_target = 0.0f;
    g_car_mode2_state.strafe_target = 0.0f;
    g_car_mode2_state.output_valid = 1U;
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
}
