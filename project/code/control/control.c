#include "control.h"
#include "../Attitude/IMU_Filtter.h"
#include "../imu/ICM42688.h"
#include "pid.h"
#include "../encoder/encoder_control.h"
#include "../menu/menu_config.h"
#include "../motor/motor.h"

#define CONTROL_DEG_TO_RAD (0.017453292519943295f)

IncrementPID wheel_left_front_pid;
IncrementPID wheel_right_front_pid;
IncrementPID wheel_left_rear_pid;
IncrementPID wheel_right_rear_pid;
PositionalPID yaw_rate_pid;

float control_yaw_rate_current = 0.0f;
float control_yaw_rate_raw = 0.0f;
float control_yaw_rate_output = 0.0f;

static void control_speed_pid_init(IncrementPID *pid)
{
    IncrementPID_Init(pid, wheel_kp, wheel_ki, wheel_kd, wheel_output_limit);
}

static void control_speed_pid_apply_params(IncrementPID *pid)
{
    pid->kp = wheel_kp;
    pid->ki = wheel_ki;
    pid->kd = wheel_kd;
    pid->output_limit = wheel_output_limit;
}

static void control_yaw_rate_pid_init(void)
{
    PositionalPID_Init(&yaw_rate_pid, 0.0f, yaw_rate_kp, yaw_rate_ki,
                       yaw_rate_kd, yaw_rate_i_limit, yaw_rate_output_limit);
}

static void control_yaw_rate_pid_apply_params(void)
{
    yaw_rate_pid.kp_2 = 0.0f;
    yaw_rate_pid.kp_1 = yaw_rate_kp;
    yaw_rate_pid.ki = yaw_rate_ki;
    yaw_rate_pid.kd = yaw_rate_kd;
    yaw_rate_pid.i_limit = yaw_rate_i_limit;
    yaw_rate_pid.output_limit = yaw_rate_output_limit;
}

void control_speed_loop_init(void)
{
    control_speed_loop_reset();
}

void control_cascade_init(void)
{
    control_yaw_rate_pid_init();
    control_speed_loop_init();
}

void control_speed_loop_reset(void)
{
    control_speed_pid_init(&wheel_left_front_pid);
    control_speed_pid_init(&wheel_right_front_pid);
    control_speed_pid_init(&wheel_left_rear_pid);
    control_speed_pid_init(&wheel_right_rear_pid);
}

void control_cascade_reset(void)
{
    control_yaw_rate_pid_init();
    control_speed_loop_reset();
    control_yaw_rate_current = 0.0f;
    control_yaw_rate_raw = 0.0f;
    control_yaw_rate_output = 0.0f;
}

float control_yaw_rate_loop_update(float yaw_rate_target)
{
    control_yaw_rate_pid_apply_params();

    control_yaw_rate_raw = -ICM42688.gyro_z * CONTROL_DEG_TO_RAD;
    control_yaw_rate_current = -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD;
    control_yaw_rate_output = PositionalPID_Update(&yaw_rate_pid,
                                                   yaw_rate_target,
                                                   control_yaw_rate_current);
    return control_yaw_rate_output;
}

void control_cascade_speed_loop_update(float forward_target, float strafe_target)
{
    float rotate_target = control_yaw_rate_output;

    float left_front_target = forward_target - strafe_target - rotate_target;
    float right_front_target = forward_target + strafe_target + rotate_target;
    float left_rear_target = forward_target + strafe_target - rotate_target;
    float right_rear_target = forward_target - strafe_target + rotate_target;

    control_speed_loop_update(left_front_target, right_front_target,
                              left_rear_target, right_rear_target);
}

void control_cascade_update(float forward_target, float strafe_target, float yaw_rate_target)
{
    control_yaw_rate_loop_update(yaw_rate_target);
    control_cascade_speed_loop_update(forward_target, strafe_target);
}

void control_speed_loop_update(float left_front_target, float right_front_target,
                               float left_rear_target, float right_rear_target)
{
    control_speed_pid_apply_params(&wheel_left_front_pid);
    control_speed_pid_apply_params(&wheel_right_front_pid);
    control_speed_pid_apply_params(&wheel_left_rear_pid);
    control_speed_pid_apply_params(&wheel_right_rear_pid);

    float left_front_pwm = IncreamPID_Update(&wheel_left_front_pid, left_front_target,
                                             encoder_get_left_front_filtered_count());
    float right_front_pwm = IncreamPID_Update(&wheel_right_front_pid, right_front_target,
                                              encoder_get_right_front_filtered_count());
    float left_rear_pwm = IncreamPID_Update(&wheel_left_rear_pid, left_rear_target,
                                            encoder_get_left_rear_filtered_count());
    float right_rear_pwm = IncreamPID_Update(&wheel_right_rear_pid, right_rear_target,
                                             encoder_get_right_rear_filtered_count());

    mecanum_motor_set_all((int16_t)left_front_pwm,
                          (int16_t)right_front_pwm,
                          (int16_t)left_rear_pwm,
                          (int16_t)right_rear_pwm);
}

void control_speed_loop_stop(void)
{
    control_speed_loop_reset();
    mecanum_motor_stop();
}

void control_cascade_stop(void)
{
    control_cascade_reset();
    mecanum_motor_stop();
}
