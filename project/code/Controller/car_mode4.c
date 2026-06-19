#include "car_mode.h"

#define MODE4_DEG_TO_RAD        (0.017453292519943295f)
#define MODE4_HOLD_MS           (2000U)
#define MODE4_STEP_MS           (1000U)
#define MODE4_FULL_TURN_DEG     (360.0f)
#define MODE4_RATE_COUNT        ((uint32)(sizeof(s_mode4_rates_deg_s) / sizeof(s_mode4_rates_deg_s[0])))

static const float s_mode4_rates_deg_s[] = {30.0f, 60.0f, 90.0f};
static uint8 s_mode4_started = 0U;
static uint32 s_mode4_start_ms = 0U;
static float s_mode4_yaw_target_deg = 0.0f;

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
    uint32 rate_index;
    uint32 rate_phase_ms;
    uint32 step_index;
    uint32 steps_per_turn;
    uint32 steps_per_rate;
    uint32 i;
    float rate_deg_s;
    float target_deg;

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
        for(i = 0U; i < MODE4_RATE_COUNT; i++)
        {
            cycle_ms += (uint32)((MODE4_FULL_TURN_DEG / s_mode4_rates_deg_s[i]) * 2.0f) *
                        MODE4_STEP_MS;
        }

        active_ms %= cycle_ms;
        rate_phase_ms = active_ms;
        rate_index = 0U;
        for(i = 0U; i < MODE4_RATE_COUNT; i++)
        {
            steps_per_rate = (uint32)((MODE4_FULL_TURN_DEG / s_mode4_rates_deg_s[i]) * 2.0f);
            if(rate_phase_ms < (steps_per_rate * MODE4_STEP_MS))
            {
                rate_index = i;
                break;
            }
            rate_phase_ms -= steps_per_rate * MODE4_STEP_MS;
        }

        rate_deg_s = s_mode4_rates_deg_s[rate_index];
        steps_per_turn = (uint32)(MODE4_FULL_TURN_DEG / rate_deg_s);
        step_index = rate_phase_ms / MODE4_STEP_MS;

        if(step_index < steps_per_turn)
        {
            target_deg = rate_deg_s * (float)(step_index + 1U);
        }
        else
        {
            target_deg = MODE4_FULL_TURN_DEG -
                         rate_deg_s * (float)(step_index - steps_per_turn + 1U);
        }

        s_mode4_yaw_target_deg = target_deg;
    }

    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
}

float car_mode4_get_yaw_target_rad(void)
{
    return s_mode4_yaw_target_deg * MODE4_DEG_TO_RAD;
}
