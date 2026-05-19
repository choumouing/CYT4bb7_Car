#include "odometer.h"

#define ODOMETER_DEG_TO_RAD (0.017453292519943295f)
#define ODOMETER_PI (3.14159265358979323846f)
#define ODOMETER_TWO_PI (6.28318530717958647692f)

typedef struct
{
    float forward;
    float strafe;
} odometer_vec2_t;

typedef struct
{
    float yaw_zero_rad;
    uint8 yaw_ready;
} odometer_state_t;

odometer_data_t g_odometer = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
odometer_vec2_t body_velocity;
odometer_vec2_t horizontal_velocity;
static odometer_state_t s_odometer_state = {0.0f, 0U};

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

static float odometer_vec_norm(odometer_vec2_t value)
{
    return sqrtf((value.forward * value.forward) + (value.strafe * value.strafe));
}

static odometer_vec2_t odometer_get_encoder_count(void)
{
    odometer_vec2_t count;
    float left_front = encoder_get_left_front_filtered_count();
    float right_front = encoder_get_right_front_filtered_count();
    float left_rear = encoder_get_left_rear_filtered_count();
    float right_rear = encoder_get_right_rear_filtered_count();

    count.forward = (left_front + right_front + left_rear + right_rear) * 0.25f;
    count.strafe = (-left_front + right_front + left_rear - right_rear) * 0.25f;
    return count;
}

static odometer_vec2_t odometer_count_to_body_velocity(odometer_vec2_t count)
{
    odometer_vec2_t velocity;

    velocity.forward = count.forward / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S;
    velocity.strafe = count.strafe / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S;
    return velocity;
}

static odometer_vec2_t odometer_body_to_horizontal(odometer_vec2_t body_velocity)
{
    odometer_vec2_t horizontal;
    float yaw_now_rad = g_euler.yaw * ODOMETER_DEG_TO_RAD;
    float yaw_delta_rad;
    float cos_yaw;
    float sin_yaw;

    if (0U == s_odometer_state.yaw_ready)
    {
        s_odometer_state.yaw_zero_rad = yaw_now_rad;
        s_odometer_state.yaw_ready = 1U;
    }

    yaw_delta_rad = odometer_normalize_angle(yaw_now_rad - s_odometer_state.yaw_zero_rad);
    cos_yaw = cosf(yaw_delta_rad);
    sin_yaw = sinf(yaw_delta_rad);

    horizontal.forward = (cos_yaw * body_velocity.forward) - (sin_yaw * body_velocity.strafe);
    horizontal.strafe = (sin_yaw * body_velocity.forward) + (cos_yaw * body_velocity.strafe);
    return horizontal;
}

void odometer_init(void)
{
    odometer_reset();
}

void odometer_reset(void)
{
    g_odometer.forward_distance = 0.0f;
    g_odometer.strafe_distance = 0.0f;
    g_odometer.travel_distance = 0.0f;
    g_odometer.forward_velocity_mps = 0.0f;
    g_odometer.strafe_velocity_mps = 0.0f;
    s_odometer_state.yaw_zero_rad = 0.0f;
    s_odometer_state.yaw_ready = 0U;
}

void odometer_update_100HZ(void)
{
    odometer_vec2_t distance_delta;

    body_velocity = odometer_count_to_body_velocity(odometer_get_encoder_count());
    horizontal_velocity = odometer_body_to_horizontal(body_velocity);

    distance_delta.forward = horizontal_velocity.forward * ODOMETER_UPDATE_DT_S;
    distance_delta.strafe = horizontal_velocity.strafe * ODOMETER_UPDATE_DT_S;

    g_odometer.forward_velocity_mps = horizontal_velocity.forward;
    g_odometer.strafe_velocity_mps = horizontal_velocity.strafe;
    g_odometer.forward_distance += distance_delta.forward;
    g_odometer.strafe_distance += distance_delta.strafe;
    g_odometer.travel_distance += odometer_vec_norm(distance_delta);
}

void odometer_update_1000HZ(void)
{

    // 首先g_imufilter_1000hz.acc 为车体坐标系下面的加速度
    // 首先要根据欧拉角转换为水平坐标系下面的加速度(也要和yaw解耦,与车头无关)
    // 然后积分得到速度，积分得到位移
    wifi_justfloat(tick_1000us_cnt, g_imufilter_1000hz.accx, g_imufilter_1000hz.accy, g_imufilter_1000hz.accz,
                   g_euler.roll, g_euler.pitch, g_euler.yaw,
                   horizontal_velocity.forward, horizontal_velocity.strafe,
                   g_odometer.forward_distance, g_odometer.strafe_distance);
}