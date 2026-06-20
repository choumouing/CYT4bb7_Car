#include "car_mode.h"

#define MODE4_DEG_TO_RAD        (0.017453292519943295f)
#define MODE4_HOLD_MS           (2000U)
#define MODE4_STEP_MS           (1000U)
#define MODE4_FULL_TURN_DEG     (360.0f)
#define MODE4_STEP_RATE_COUNT   ((uint32)(sizeof(s_mode4_step_rates_deg_s) / sizeof(s_mode4_step_rates_deg_s[0])))
#define MODE4_SMOOTH_RATE_COUNT ((uint32)(sizeof(s_mode4_smooth_rates_deg_s) / sizeof(s_mode4_smooth_rates_deg_s[0])))

static const float s_mode4_step_rates_deg_s[] = {30.0f, 90.0f};
static const float s_mode4_smooth_rates_deg_s[] = {10.0f, 45.0f};
static uint8 s_mode4_started = 0U;
static uint32 s_mode4_start_ms = 0U;
static float s_mode4_yaw_target_deg = 0.0f;

static uint32 car_mode4_step_phase_ms(float rate_deg_s)
{
    return (uint32)((MODE4_FULL_TURN_DEG / rate_deg_s) * 2.0f) * MODE4_STEP_MS;
}

static uint32 car_mode4_smooth_phase_ms(float rate_deg_s)
{
    return (uint32)((MODE4_FULL_TURN_DEG * 2.0f * 1000.0f) / rate_deg_s);
}

static float car_mode4_step_target(float rate_deg_s, uint32 phase_ms)
{
    uint32 step_index = phase_ms / MODE4_STEP_MS;
    uint32 steps_per_turn = (uint32)(MODE4_FULL_TURN_DEG / rate_deg_s);

    if(step_index < steps_per_turn)
    {
        return rate_deg_s * (float)(step_index + 1U);
    }

    return MODE4_FULL_TURN_DEG -
           rate_deg_s * (float)(step_index - steps_per_turn + 1U);
}

static float car_mode4_smooth_target(float rate_deg_s, uint32 phase_ms)
{
    float phase_s = (float)phase_ms * 0.001f;
    float one_turn_s = MODE4_FULL_TURN_DEG / rate_deg_s;

    if(phase_s < one_turn_s)
    {
        return rate_deg_s * phase_s;
    }

    return MODE4_FULL_TURN_DEG - rate_deg_s * (phase_s - one_turn_s);
}

void car_mode4_init(void)
{
    car_mode4_reset();
}

void car_mode4_reset(void)
{
    s_mode4_started = 0U;
    s_mode4_start_ms = 0U;
    s_mode4_yaw_target_deg = 0.0f;
}

void car_mode4_update_100HZ(uint32 now_ms)
{
    uint32 elapsed_ms;
    uint32 active_ms;
    uint32 cycle_ms;
    uint32 phase_ms;
    uint32 span_ms;
    uint32 i;

    if(0U == s_mode4_started)
    {
        s_mode4_started = 1U;
        s_mode4_start_ms = now_ms;
    }

    elapsed_ms = now_ms - s_mode4_start_ms;
    if(elapsed_ms < MODE4_HOLD_MS)
    {
        s_mode4_yaw_target_deg = 0.0f;
    }
    else
    {
        active_ms = elapsed_ms - MODE4_HOLD_MS;
        cycle_ms = 0U;
        for(i = 0U; i < MODE4_STEP_RATE_COUNT; i++)
        {
            cycle_ms += car_mode4_step_phase_ms(s_mode4_step_rates_deg_s[i]);
        }
        for(i = 0U; i < MODE4_SMOOTH_RATE_COUNT; i++)
        {
            cycle_ms += car_mode4_smooth_phase_ms(s_mode4_smooth_rates_deg_s[i]);
        }

        active_ms %= cycle_ms;
        phase_ms = active_ms;
        for(i = 0U; i < MODE4_STEP_RATE_COUNT; i++)
        {
            span_ms = car_mode4_step_phase_ms(s_mode4_step_rates_deg_s[i]);
            if(phase_ms < span_ms)
            {
                s_mode4_yaw_target_deg = car_mode4_step_target(s_mode4_step_rates_deg_s[i],
                                                               phase_ms);
                car_forward_target = 0.0f;
                car_strafe_target = 0.0f;
                return;
            }
            phase_ms -= span_ms;
        }
        for(i = 0U; i < MODE4_SMOOTH_RATE_COUNT; i++)
        {
            span_ms = car_mode4_smooth_phase_ms(s_mode4_smooth_rates_deg_s[i]);
            if(phase_ms < span_ms)
            {
                s_mode4_yaw_target_deg = car_mode4_smooth_target(s_mode4_smooth_rates_deg_s[i],
                                                                 phase_ms);
                car_forward_target = 0.0f;
                car_strafe_target = 0.0f;
                return;
            }
            phase_ms -= span_ms;
        }
    }

    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
}

float car_mode4_get_yaw_target_rad(void)
{
    return s_mode4_yaw_target_deg * MODE4_DEG_TO_RAD;
}
