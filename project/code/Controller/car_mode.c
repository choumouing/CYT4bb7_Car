#include "car_mode.h"


static car_mode_e s_car_mode = CAR_MODE_0;
static car_mode_e s_last_car_mode = CAR_MODE_0;
static uint8 s_last_control_enabled = 0U;

static void car_mode_reset_all(void)
{
    car_mode0_reset();
    car_mode1_reset();
    car_mode2_reset();
    control_cascade_reset();
}

static void car_mode_handle_transition_25HZ(car_mode_e mode, uint8 control_enabled)
{
    uint8 need_reset = 0U;

    if(s_last_car_mode != mode)
    {
        need_reset = 1U;
    }

    if(s_last_control_enabled != control_enabled)
    {
        need_reset = 1U;
    }

    if(0U != need_reset)
    {
        car_mode_reset_all();
    }

    s_last_car_mode = mode;
    s_last_control_enabled = control_enabled;
}

void car_mode_init(void)
{
    car_mode0_init();
    car_mode1_init();
    car_mode2_init();
    car_mode2_load_default_targets();
    car_mode_reset();
}

void car_mode_reset(void)
{
    s_car_mode = CAR_MODE_0;
    s_last_car_mode = CAR_MODE_0;
    s_last_control_enabled = 0U;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
    car_rotate_target = 0.0f;
    car_mode_reset_all();
}

car_mode_e car_mode_get(void)
{
    return s_car_mode;
}

void car_mode_update_25HZ(uint32 now_ms)
{
    s_car_mode = car_start_sbus_get_mode();

    car_control_enabled = car_start_sbus_is_running();
    car_emergency_stop_active = car_start_sbus_emergency_stop_active();
    car_mode_handle_transition_25HZ(s_car_mode, car_control_enabled);

    if(0U == car_control_enabled)
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        car_rotate_target = 0.0f;
        return;
    }

    switch(s_car_mode)
    {
    case CAR_MODE_0:
        car_mode0_update_25HZ(now_ms);
        break;

    case CAR_MODE_1:
        car_mode1_update_25HZ(now_ms);
        break;

    case CAR_MODE_2:
        car_mode2_update_25HZ(now_ms);
        break;

    case CAR_MODE_3:
    case CAR_MODE_4:
    case CAR_MODE_5:
    case CAR_MODE_6:
    case CAR_MODE_7:
    case CAR_MODE_8:
    default:
        car_mode0_update_25HZ(now_ms);
        break;
    }
}
