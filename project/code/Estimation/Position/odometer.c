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
static float s_prev_encoder_vel[ODOMETER_AXIS_NUM];
static float s_body_vel[ODOMETER_AXIS_NUM];
static float s_body_acc_no_yaw[ODOMETER_AXIS_NUM];
static float s_raw_acc_no_yaw[ODOMETER_AXIS_NUM];
static float s_last_innovation_mag;
static float s_last_acc_diff_mag;
static float s_last_alpha;
static uint8 s_yaw_ready;
static uint8 s_accel_bias_ready;
static uint8 s_slip_detected;
static uint8 s_slip_hold_ticks;
static uint8 s_static_detected;

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

static void odometer_update_raw_accel_no_yaw(void)
{
    float raw_sensor_g[3];
    float raw_body_g[3];
    float raw_body_real[3];
    float raw_level[3];
    float gravity_body_g[3];
    float gravity_mps2 = AccelCalibration_GetGravityMps2();
    float sin_pitch = g_euler.sin_pitch;
    float cos_pitch = g_euler.cos_pitch;
    float sin_roll = g_euler.sin_roll;
    float cos_roll = g_euler.cos_roll;

    raw_sensor_g[0] = ICM42688.acc_x;
    raw_sensor_g[1] = ICM42688.acc_y;
    raw_sensor_g[2] = ICM42688.acc_z;
    AccelCalibration_ApplySensorCorrection(&raw_sensor_g[0], &raw_sensor_g[1], &raw_sensor_g[2]);
    AccelCalibration_RotateImuToBody(raw_sensor_g, raw_body_g);

    gravity_body_g[0] = -sin_pitch;
    gravity_body_g[1] = sin_roll * cos_pitch;
    gravity_body_g[2] = cos_roll * cos_pitch;

    raw_body_real[0] =
        (raw_body_g[0] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_body_g[0]) *
        gravity_mps2;
    raw_body_real[1] =
        (raw_body_g[1] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_body_g[1]) *
        gravity_mps2;
    raw_body_real[2] =
        (raw_body_g[2] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_body_g[2]) *
        gravity_mps2;

    raw_level[0] =
        (cos_pitch * raw_body_real[0]) +
        (sin_roll * sin_pitch * raw_body_real[1]) +
        (cos_roll * sin_pitch * raw_body_real[2]);
    raw_level[1] =
        (cos_roll * raw_body_real[1]) -
        (sin_roll * raw_body_real[2]);

    s_raw_acc_no_yaw[x] = raw_level[0];
    s_raw_acc_no_yaw[y] = -raw_level[1];
}

static void odometer_update_accel(void)
{
    float body_acc[ODOMETER_AXIS_NUM];
    float accel_y_right;

    AccelCalibration_GetBodyLevelAccelNoYawMps2(&body_acc[x], &accel_y_right);
    body_acc[y] = -accel_y_right;
    s_body_acc_no_yaw[x] = body_acc[x];
    s_body_acc_no_yaw[y] = body_acc[y];
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
    s_prev_encoder_vel[x] = 0.0f;
    s_prev_encoder_vel[y] = 0.0f;
    s_body_vel[x] = 0.0f;
    s_body_vel[y] = 0.0f;
    s_body_acc_no_yaw[x] = 0.0f;
    s_body_acc_no_yaw[y] = 0.0f;
    s_raw_acc_no_yaw[x] = 0.0f;
    s_raw_acc_no_yaw[y] = 0.0f;
    s_last_innovation_mag = 0.0f;
    s_last_acc_diff_mag = 0.0f;
    s_last_alpha = 0.0f;
    s_yaw_ready = 0U;
    s_accel_bias_ready = 0U;
    s_slip_detected = 0U;
    s_slip_hold_ticks = 0U;
    s_static_detected = 0U;
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
    s_body_vel[x] = body_vel[x];
    s_body_vel[y] = body_vel[y];
    odometer_body_to_horizontal(body_vel, s_encoder_vel);
    odometer_update_accel();

    if ((odometer_norm(s_encoder_vel) < ODOMETER_STATIC_ENCODER_SPEED_MPS) &&
        (odometer_norm(g_odometer.acc) < ODOMETER_STATIC_ACCEL_MPS2))
    {
        s_accel_bias[x] += ODOMETER_ACCEL_BIAS_ALPHA_STATIC * g_odometer.acc[x];
        s_accel_bias[y] += ODOMETER_ACCEL_BIAS_ALPHA_STATIC * g_odometer.acc[y];
        s_imu_vel[x] = 0.0f;
        s_imu_vel[y] = 0.0f;
        s_slip_detected = 0U;
        s_slip_hold_ticks = 0U;
        s_static_detected = 1U;
        s_last_innovation_mag = 0.0f;
        s_last_acc_diff_mag = 0.0f;
        s_last_alpha = 0.0f;
    }
    else
    {
        float innovation[ODOMETER_AXIS_NUM];
        float innovation_mag;
        float alpha;
        float enc_acc_x = (s_encoder_vel[x] - s_prev_encoder_vel[x]) / ODOMETER_UPDATE_DT_S;
        float enc_acc_y = (s_encoder_vel[y] - s_prev_encoder_vel[y]) / ODOMETER_UPDATE_DT_S;
        float diff_x = enc_acc_x - g_odometer.acc[x];
        float diff_y = enc_acc_y - g_odometer.acc[y];
        float diff_mag = sqrtf((diff_x * diff_x) + (diff_y * diff_y));

        innovation[x] = s_encoder_vel[x] - s_imu_vel[x];
        innovation[y] = s_encoder_vel[y] - s_imu_vel[y];
        innovation_mag = odometer_norm(innovation);
        s_last_innovation_mag = innovation_mag;
        s_last_acc_diff_mag = diff_mag;
        s_static_detected = 0U;

        if ((innovation_mag > ODOMETER_SLIP_INNOVATION_THRESH) &&
            (diff_mag > ODOMETER_SLIP_ACCEL_DIFF_THRESH))
        {
            s_slip_hold_ticks = ODOMETER_SLIP_HOLD_TICKS;
        }

        s_slip_detected = (s_slip_hold_ticks > 0U) ? 1U : 0U;
        alpha = (0U != s_slip_detected) ? ODOMETER_SLIP_BLEND_ALPHA : ODOMETER_ENCODER_BLEND_ALPHA;
        s_last_alpha = alpha;
        s_imu_vel[x] += alpha * (s_encoder_vel[x] - s_imu_vel[x]);
        s_imu_vel[y] += alpha * (s_encoder_vel[y] - s_imu_vel[y]);

        if (s_slip_hold_ticks > 0U)
        {
            s_slip_hold_ticks--;
        }
    }

    s_prev_encoder_vel[x] = s_encoder_vel[x];
    s_prev_encoder_vel[y] = s_encoder_vel[y];

    g_odometer.vel[x] = s_imu_vel[x];
    g_odometer.vel[y] = s_imu_vel[y];
    g_odometer.position[x] += g_odometer.vel[x] * ODOMETER_UPDATE_DT_S;
    g_odometer.position[y] += g_odometer.vel[y] * ODOMETER_UPDATE_DT_S;
}

void odometer_update_1000HZ(void)
{

    float left_front = encoder_get_left_front_filtered_count();
    float right_front = encoder_get_right_front_filtered_count();
    float left_rear = encoder_get_left_rear_filtered_count();
    float right_rear = encoder_get_right_rear_filtered_count();

    odometer_update_raw_accel_no_yaw();
    odometer_update_accel();

    s_imu_vel[x] += g_odometer.acc[x] * ODOMETER_IMU_UPDATE_DT_S;
    s_imu_vel[y] += g_odometer.acc[y] * ODOMETER_IMU_UPDATE_DT_S;

    g_odometer.vel[x] = s_imu_vel[x];
    g_odometer.vel[y] = s_imu_vel[y];
    wifi_justfloat(tick_1000us_cnt,
    ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z,
    g_imufilter_1000hz.accx, g_imufilter_1000hz.accy, g_imufilter_1000hz.accz,
    g_euler.roll, g_euler.pitch, g_euler.yaw,
    left_front, right_front, left_rear, right_rear,
    g_odometer.position[x], g_odometer.position[y],
    g_odometer.vel[x], g_odometer.vel[y],
    g_odometer.acc[x], g_odometer.acc[y],
    s_body_acc_no_yaw[x], s_body_acc_no_yaw[y],
    s_raw_acc_no_yaw[x], s_raw_acc_no_yaw[y],
    s_accel_bias[x], s_accel_bias[y],
    s_imu_vel[x], s_imu_vel[y],
    s_encoder_vel[x], s_encoder_vel[y],
    s_body_vel[x], s_body_vel[y],
    s_prev_encoder_vel[x], s_prev_encoder_vel[y],
    s_last_innovation_mag, s_last_acc_diff_mag,
    s_last_alpha, s_slip_detected,
    s_slip_hold_ticks, s_static_detected);
}
