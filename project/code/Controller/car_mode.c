#include "car_mode.h"
#include "car_loop.h"

#define CAR_MODE_CH4_ENABLE_THRESHOLD (0.5f)
#define CAR_MODE_CH6_LOW_THRESHOLD    (-333.0f)
#define CAR_MODE_CH6_HIGH_THRESHOLD   (333.0f)

static car_mode_e s_car_mode = CAR_MODE_0;
static car_mode_e s_last_car_mode = CAR_MODE_0;
static uint8 s_last_control_enabled = 0U;

static car_mode_e car_mode_from_ch6(float ch6)
{
    if(ch6 < CAR_MODE_CH6_LOW_THRESHOLD)
    {
        return CAR_MODE_0;
    }

    if(ch6 > CAR_MODE_CH6_HIGH_THRESHOLD)
    {
        return CAR_MODE_2;
    }

    return CAR_MODE_1;
}

static void car_mode_reset_all(void)
{
    car_mode0_reset();
    car_mode1_reset();
    car_mode2_reset();
    Control_Reset();
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
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
    car_mode_reset_all();
}

car_mode_e car_mode_get(void)
{
    return s_car_mode;
}

void car_mode_update_25HZ(uint32 now_ms)
{
    car_control_enabled = (g_air_crsf_std_ch4 >= CAR_MODE_CH4_ENABLE_THRESHOLD) ? 1U : 0U;
    car_emergency_stop_active = (0U == car_control_enabled) ? 1U : 0U;
    s_car_mode = (0U != car_control_enabled) ? car_mode_from_ch6(g_air_crsf_std_ch6) : CAR_MODE_0;
    car_mode_handle_transition_25HZ(s_car_mode, car_control_enabled);

    if(0U == car_control_enabled)
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    switch(s_car_mode)
    {
    case CAR_MODE_0:
        car_mode0_update_25HZ(now_ms);
        break;

    case CAR_MODE_1:
        break;

    case CAR_MODE_2:
        car_mode2_update_25HZ(now_ms);
        break;

    default:
        car_mode0_update_25HZ(now_ms);
        break;
    }
}

void car_mode_update_100HZ(uint32 now_ms)
{
    if((0U == car_control_enabled) || (0U != car_emergency_stop_active))
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    switch(s_car_mode)
    {
    case CAR_MODE_1:
        car_mode1_update_100HZ(now_ms);
        break;

    case CAR_MODE_2:
        car_mode2_update_100HZ(now_ms);
        break;

    default:
        break;
    }
}
