#include "car_mode2_config.h"


void car_mode2_load_default_targets(void)
{
    car_mode2_clear_targets();

    (void)car_mode2_add_target(2.00f, 2.00f);
    (void)car_mode2_add_target(-2.00f, -0.50f);
    (void)car_mode2_add_target(0.00f, -1.50f);

    car_mode2_restart_targets();
}
