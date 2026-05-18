#include "control.h"

#define CONTROL_DEG_TO_RAD (0.017453292519943295f)
#define CONTROL_PI         (3.14159265358979323846f)
#define CONTROL_TWO_PI     (6.28318530717958647692f)
#define CONTROL_WHEEL_FF_KS (220.0f)
#define CONTROL_WHEEL_FF_KV (5.8f)

/* PID 实例 */
PositionalPID wheel_left_front_pid;
PositionalPID wheel_right_front_pid;
PositionalPID wheel_left_rear_pid;
PositionalPID wheel_right_rear_pid;
PositionalPID yaw_angle_pid;
PositionalPID yaw_rate_pid;

/* 调试中间变量 */
float control_yaw_angle_current = 0.0f;
float control_yaw_angle_output = 0.0f;
float control_yaw_rate_target = 0.0f;
float control_yaw_rate_current = 0.0f;
float control_yaw_rate_raw = 0.0f;
float control_yaw_rate_output = 0.0f;

/* 航向保持状态 */
static float s_yaw_hold_target = 0.0f;
static uint8 s_last_rotate_active = 0U;
static uint8 s_yaw_hold_active = 0U;

static float control_normalize_angle_rad(float angle)
{
    while (angle > CONTROL_PI)
        angle -= CONTROL_TWO_PI;
    while (angle < -CONTROL_PI)
        angle += CONTROL_TWO_PI;
    return angle;
}

static float control_wheel_ff(float target)
{
    if(target > 0.0f) return CONTROL_WHEEL_FF_KS + CONTROL_WHEEL_FF_KV * target;
    if(target < 0.0f) return -CONTROL_WHEEL_FF_KS + CONTROL_WHEEL_FF_KV * target;
    return 0.0f;
}

static void control_pid_init_all(void)
{
    PositionalPID_Init(&wheel_left_front_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&wheel_right_front_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&wheel_left_rear_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&wheel_right_rear_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&yaw_angle_pid, 0.0f, yaw_angle_kp, yaw_angle_ki, yaw_angle_kd, yaw_angle_i_limit, yaw_angle_output_limit);
    PositionalPID_Init(&yaw_rate_pid, 0.0f, yaw_rate_kp, yaw_rate_ki, yaw_rate_kd, yaw_rate_i_limit, yaw_rate_output_limit);
}

static void control_wheel_pid_apply_params(PositionalPID *pid)
{
    pid->kp_2 = 0.0f;
    pid->kp_1 = wheel_kp;
    pid->ki = wheel_ki;
    pid->kd = wheel_kd;
    pid->i_limit = wheel_i_limit;
    pid->output_limit = wheel_output_limit;
}

static void control_pid_apply_all(void)
{
    control_wheel_pid_apply_params(&wheel_left_front_pid);
    control_wheel_pid_apply_params(&wheel_right_front_pid);
    control_wheel_pid_apply_params(&wheel_left_rear_pid);
    control_wheel_pid_apply_params(&wheel_right_rear_pid);

    yaw_angle_pid.kp_1 = yaw_angle_kp;     yaw_angle_pid.ki = yaw_angle_ki;
    yaw_angle_pid.kd = yaw_angle_kd;       yaw_angle_pid.i_limit = yaw_angle_i_limit;
    yaw_angle_pid.output_limit = yaw_angle_output_limit;

    yaw_rate_pid.kp_1 = yaw_rate_kp;       yaw_rate_pid.ki = yaw_rate_ki;
    yaw_rate_pid.kd = yaw_rate_kd;         yaw_rate_pid.i_limit = yaw_rate_i_limit;
    yaw_rate_pid.output_limit = yaw_rate_output_limit;
}

/* ========== 公共接口 ========== */

void Control_Init(void)
{
    control_pid_init_all();
}

void Control_Reset(void)
{
    control_pid_init_all();
    control_yaw_angle_current = 0.0f;
    control_yaw_angle_output = 0.0f;
    control_yaw_rate_target = 0.0f;
    control_yaw_rate_current = 0.0f;
    control_yaw_rate_raw = 0.0f;
    control_yaw_rate_output = 0.0f;
    Control_YawHoldReset();
}

void Control_Stop(void)
{
    Control_Reset();
    mecanum_motor_stop();
}

void Control_YawHoldReset(void)
{
    s_last_rotate_active = 0U;
    s_yaw_hold_active = 0U;
    s_yaw_hold_target = Control_GetYawAngle();
}

float Control_GetYawAngle(void)
{
    return control_normalize_angle_rad(-g_euler.yaw * CONTROL_DEG_TO_RAD);
}

/* 25Hz：航向角度环 + 航向保持 */
void Control_25Hz(float rotate_target)
{
    if (rotate_target != 0.0f)
    {
        s_yaw_hold_target = Control_GetYawAngle();
        s_yaw_hold_active = 0U;
    }
    else
    {
        if ((0U == s_yaw_hold_active) || (0U != s_last_rotate_active))
        {
            s_yaw_hold_target = Control_GetYawAngle();
            yaw_angle_pid.integral = 0.0f;
            control_yaw_angle_output = 0.0f;
            control_yaw_rate_target = 0.0f;
            s_yaw_hold_active = 1U;
        }

        /* 角度环：目标航向 → 角速度目标 */
        float yaw_error = control_normalize_angle_rad(s_yaw_hold_target - Control_GetYawAngle());
        control_yaw_angle_current = Control_GetYawAngle();
        yaw_angle_pid.kp_1 = yaw_angle_kp; yaw_angle_pid.ki = yaw_angle_ki;
        yaw_angle_pid.kd = yaw_angle_kd;   yaw_angle_pid.i_limit = yaw_angle_i_limit;
        yaw_angle_pid.output_limit = yaw_angle_output_limit;
        control_yaw_angle_output = PositionalPID_Update(&yaw_angle_pid, yaw_error, 0.0f);
        control_yaw_rate_target = control_yaw_angle_output;
    }

    s_last_rotate_active = (rotate_target != 0.0f) ? 1U : 0U;
}

/* 50Hz：航向角速度环 */
void Control_50Hz(float rotate_target)
{
    yaw_rate_pid.kp_1 = yaw_rate_kp; yaw_rate_pid.ki = yaw_rate_ki;
    yaw_rate_pid.kd = yaw_rate_kd;   yaw_rate_pid.i_limit = yaw_rate_i_limit;
    yaw_rate_pid.output_limit = yaw_rate_output_limit;

    control_yaw_rate_target = rotate_target;
    control_yaw_rate_raw = -ICM42688.gyro_z * CONTROL_DEG_TO_RAD;
    control_yaw_rate_current = -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD;
    control_yaw_rate_output = PositionalPID_Update(&yaw_rate_pid,
                                                   rotate_target,
                                                   control_yaw_rate_current);
}

/* 100Hz：麦克纳姆解算 + 四轮速度环 + 电机输出 */
void Control_100Hz(float forward, float strafe)
{
    float rot = control_yaw_rate_output;
    float lf = forward - strafe - rot;
    float rf = forward + strafe + rot;
    float lr = forward + strafe - rot;
    float rr = forward - strafe + rot;

    control_pid_apply_all();

    mecanum_motor_set_all(
        (int16_t)(control_wheel_ff(lf) + PositionalPID_Update(&wheel_left_front_pid, lf, encoder_get_left_front_filtered_count())),
        (int16_t)(control_wheel_ff(rf) + PositionalPID_Update(&wheel_right_front_pid, rf, encoder_get_right_front_filtered_count())),
        (int16_t)(control_wheel_ff(lr) + PositionalPID_Update(&wheel_left_rear_pid, lr, encoder_get_left_rear_filtered_count())),
        (int16_t)(control_wheel_ff(rr) + PositionalPID_Update(&wheel_right_rear_pid, rr, encoder_get_right_rear_filtered_count())));

    wifi_justfloat(lf, encoder_get_left_front_filtered_count(), control_wheel_ff(lf), wheel_left_front_pid.p_term, wheel_left_front_pid.i_term,
                   rf, encoder_get_right_front_filtered_count(), control_wheel_ff(rf), wheel_right_front_pid.p_term, wheel_right_front_pid.i_term,
                   lr, encoder_get_left_rear_filtered_count(), control_wheel_ff(lr), wheel_left_rear_pid.p_term, wheel_left_rear_pid.i_term,
                   rr, encoder_get_right_rear_filtered_count(), control_wheel_ff(rr), wheel_right_rear_pid.p_term, wheel_right_rear_pid.i_term,
                   g_euler.roll, g_euler.pitch, g_euler.yaw,
                   g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
                   g_imufilter_1000hz.accx, g_imufilter_1000hz.accy);



}
