#include "odometer.h"
#include "Estimation/Attitude/Accel_Calibration.h"

#define ODOMETER_DEG_TO_RAD (0.017453292519943295f)
#define ODOMETER_PI (3.14159265358979323846f)
#define ODOMETER_TWO_PI (6.28318530717958647692f)

odometer_data_t g_odometer = {0};

static float s_yaw_zero_rad;
static float s_accel_bias[ODOMETER_AXIS_NUM];
static float s_imu_vel[ODOMETER_AXIS_NUM];
static float s_encoder_vel[ODOMETER_AXIS_NUM];
static uint8 s_yaw_ready;
static uint8 s_accel_bias_ready;

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

static float odometer_norm(const float value[ODOMETER_AXIS_NUM])
{
    return sqrtf((value[x] * value[x]) + (value[y] * value[y]));
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

static void odometer_body_to_horizontal(const float body[ODOMETER_AXIS_NUM],
                                        float horizontal[ODOMETER_AXIS_NUM])
{
    float yaw_delta_rad = odometer_yaw_delta_rad();
    float cos_yaw = cosf(yaw_delta_rad);
    float sin_yaw = sinf(yaw_delta_rad);

    horizontal[x] = (cos_yaw * body[x]) - (sin_yaw * body[y]);
    horizontal[y] = (sin_yaw * body[x]) + (cos_yaw * body[y]);
}

static void odometer_update_accel(void)
{
    float body_acc[ODOMETER_AXIS_NUM];
    float accel_y_right;

    AccelCalibration_GetHorizontalAccelMps2(&body_acc[x], &accel_y_right);
    body_acc[y] = -accel_y_right;
    odometer_body_to_horizontal(body_acc, g_odometer.acc);

    if (0U == s_accel_bias_ready)
    {
        s_accel_bias[x] = g_odometer.acc[x];
        s_accel_bias[y] = g_odometer.acc[y];
        s_accel_bias_ready = 1U;
    }

    g_odometer.acc[x] -= s_accel_bias[x];
    g_odometer.acc[y] -= s_accel_bias[y];
}

void odometer_init(void)
{
    odometer_reset();
}

void odometer_reset(void)
{
    g_odometer = (odometer_data_t){0};
    s_yaw_zero_rad = 0.0f;
    s_accel_bias[x] = 0.0f;
    s_accel_bias[y] = 0.0f;
    s_imu_vel[x] = 0.0f;
    s_imu_vel[y] = 0.0f;
    s_encoder_vel[x] = 0.0f;
    s_encoder_vel[y] = 0.0f;
    s_yaw_ready = 0U;
    s_accel_bias_ready = 0U;
}

void odometer_update_100HZ(void)
{
    float body_vel[ODOMETER_AXIS_NUM];
    float left_front = encoder_get_left_front_filtered_count();
    float right_front = encoder_get_right_front_filtered_count();
    float left_rear = encoder_get_left_rear_filtered_count();
    float right_rear = encoder_get_right_rear_filtered_count();

    body_vel[x] =
        (left_front + right_front + left_rear + right_rear) *
        (0.25f / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S);
    body_vel[y] =
        (-left_front + right_front + left_rear - right_rear) *
        (0.25f / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S);
    odometer_body_to_horizontal(body_vel, s_encoder_vel);
    odometer_update_accel();

    if ((odometer_norm(s_encoder_vel) < ODOMETER_STATIC_ENCODER_SPEED_MPS) &&
        (odometer_norm(g_odometer.acc) < ODOMETER_STATIC_ACCEL_MPS2))
    {
        s_accel_bias[x] += ODOMETER_ACCEL_BIAS_ALPHA_STATIC * g_odometer.acc[x];
        s_accel_bias[y] += ODOMETER_ACCEL_BIAS_ALPHA_STATIC * g_odometer.acc[y];
        s_imu_vel[x] = 0.0f;
        s_imu_vel[y] = 0.0f;
    }
    else
    {
        s_imu_vel[x] += ODOMETER_ENCODER_BLEND_ALPHA * (s_encoder_vel[x] - s_imu_vel[x]);
        s_imu_vel[y] += ODOMETER_ENCODER_BLEND_ALPHA * (s_encoder_vel[y] - s_imu_vel[y]);
    }

    g_odometer.vel[x] = s_imu_vel[x];
    g_odometer.vel[y] = s_imu_vel[y];
    g_odometer.position[x] += g_odometer.vel[x] * ODOMETER_UPDATE_DT_S;
    g_odometer.position[y] += g_odometer.vel[y] * ODOMETER_UPDATE_DT_S;
}

void odometer_update_1000HZ(void)
{
    odometer_update_accel();

    s_imu_vel[x] += g_odometer.acc[x] * ODOMETER_IMU_UPDATE_DT_S;
    s_imu_vel[y] += g_odometer.acc[y] * ODOMETER_IMU_UPDATE_DT_S;

    g_odometer.vel[x] = s_imu_vel[x];
    g_odometer.vel[y] = s_imu_vel[y];
    wifi_justfloat(tick_1000us_cnt,
    g_odometer.position[x], g_odometer.position[y],
    g_odometer.vel[x], g_odometer.vel[y],
    g_odometer.acc[x], g_odometer.acc[y],
    s_imu_vel[x], s_imu_vel[y],
    s_encoder_vel[x], s_encoder_vel[y]);
}
