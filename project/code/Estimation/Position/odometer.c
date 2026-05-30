#include "odometer.h"

#define ODOMETER_DEG_TO_RAD (0.017453292519943295f)
#define ODOMETER_PI (3.14159265358979323846f)
#define ODOMETER_TWO_PI (6.28318530717958647692f)

odometer_data_t g_odometer = {0};

static float s_yaw_zero_rad;
static uint8 s_yaw_ready;
static uint8 s_wheel_gate_hold_ticks;

static float odometer_normalize_angle(float angle)
{
    while (angle > ODOMETER_PI)
    {
        angle -= ODOMETER_TWO_PI;
    }

    while (angle < -ODOMETER_PI)
    {
        angle += ODOMETER_TWO_PI;
    }

    return angle;
}

static float odometer_mid2_average4(float a, float b, float c, float d)
{
    float min_value = a;
    float max_value = a;
    float sum = a + b + c + d;

    if (b < min_value)
    {
        min_value = b;
    }
    if (c < min_value)
    {
        min_value = c;
    }
    if (d < min_value)
    {
        min_value = d;
    }

    if (b > max_value)
    {
        max_value = b;
    }
    if (c > max_value)
    {
        max_value = c;
    }
    if (d > max_value)
    {
        max_value = d;
    }

    return 0.5f * (sum - min_value - max_value);
}

static float odometer_yaw_delta_rad(void)
{
    float yaw_now_rad = g_euler.yaw * ODOMETER_DEG_TO_RAD;

    if (0U == s_yaw_ready)
    {
        s_yaw_zero_rad = yaw_now_rad;
        s_yaw_ready = 1U;
    }

    return odometer_normalize_angle(yaw_now_rad - s_yaw_zero_rad);
}

static float odometer_norm2(float vx, float vy)
{
    return sqrtf((vx * vx) + (vy * vy));
}

static void odometer_body_to_horizontal(const float body[ODOMETER_AXIS_NUM],
                                        float horizontal[ODOMETER_AXIS_NUM])
{
    float yaw_delta_rad = odometer_yaw_delta_rad();
    float cos_yaw = cosf(yaw_delta_rad);
    float sin_yaw = sinf(yaw_delta_rad);

    horizontal[x] = (cos_yaw * body[x]) - (sin_yaw * body[y]);
    horizontal[y] = (sin_yaw * body[x]) + (cos_yaw * body[y]);
}

void odometer_init(void)
{
    odometer_reset();
}

void odometer_reset(void)
{
    float initial_position[ODOMETER_AXIS_NUM];

    g_odometer = (odometer_data_t){0};
    beacon_config_get_initial_position(initial_position);
    g_odometer.position[x] = initial_position[x];
    g_odometer.position[y] = initial_position[y];
    s_yaw_zero_rad = 0.0f;
    s_yaw_ready = 0U;
    s_wheel_gate_hold_ticks = 0U;
}

void odometer_update_100HZ(void)
{
    float body_vel[ODOMETER_AXIS_NUM];
    float horizontal_vel[ODOMETER_AXIS_NUM];
    float inertial_position[ODOMETER_AXIS_NUM];
    uint8 fix_applied = 0U;
    int16_t left_front_raw = encoder_get_left_front_count();
    int16_t right_front_raw = encoder_get_right_front_count();
    int16_t left_rear_raw = encoder_get_left_rear_count();
    int16_t right_rear_raw = encoder_get_right_rear_count();
    float left_front = encoder_get_left_front_filtered_count();
    float right_front = encoder_get_right_front_filtered_count();
    float left_rear = encoder_get_left_rear_filtered_count();
    float right_rear = encoder_get_right_rear_filtered_count();
    float wheel_q = fabsf(left_front + right_front - left_rear - right_rear);

    body_vel[x] =
        (left_front - right_front - left_rear + right_rear) *
        (0.25f / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S);
    body_vel[y] =
        (left_front + right_front + left_rear + right_rear) *
        (0.25f / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S);

    if (wheel_q > ODOMETER_WHEEL_Q_THRESH)
    {
        s_wheel_gate_hold_ticks = ODOMETER_WHEEL_GATE_HOLD_TICKS;
    }

    if (s_wheel_gate_hold_ticks > 0U)
    {
        float robust_body_vel[ODOMETER_AXIS_NUM];

        robust_body_vel[x] =
            odometer_mid2_average4(left_front, -right_front, -left_rear, right_rear) *
            (1.0f / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S);
        robust_body_vel[y] =
            odometer_mid2_average4(left_front, right_front, left_rear, right_rear) *
            (1.0f / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S);
        body_vel[x] += ODOMETER_WHEEL_ROBUST_BLEND * (robust_body_vel[x] - body_vel[x]);
        body_vel[y] += ODOMETER_WHEEL_ROBUST_BLEND * (robust_body_vel[y] - body_vel[y]);
        s_wheel_gate_hold_ticks--;
    }

    odometer_body_to_horizontal(body_vel, horizontal_vel);

    if (odometer_norm2(horizontal_vel[x], horizontal_vel[y]) < ODOMETER_STATIC_ENCODER_SPEED_MPS)
    {
        horizontal_vel[x] = 0.0f;
        horizontal_vel[y] = 0.0f;
    }

    g_odometer.vel[x] = horizontal_vel[x];
    g_odometer.vel[y] = horizontal_vel[y];
    g_odometer.position[x] += g_odometer.vel[x] * ODOMETER_UPDATE_DT_S;
    g_odometer.position[y] += g_odometer.vel[y] * ODOMETER_UPDATE_DT_S;
    inertial_position[x] = g_odometer.position[x];
    inertial_position[y] = g_odometer.position[y];

#if (ODOMETER_BEACON_FIXATOR_ENABLE != 0U)
    {
        float fixed_position[ODOMETER_AXIS_NUM];

        /* fixator 在本周期信标检测后产出方案，这里通常消费上一 100Hz 周期的修正。 */
        if(fixator_get_position_fix(fixed_position) != 0U)
        {
            g_odometer.position[x] = fixed_position[x];
            g_odometer.position[y] = fixed_position[y];
            fix_applied = 1U;
        }
    }
#endif

    wifi_core_Poll();

}
